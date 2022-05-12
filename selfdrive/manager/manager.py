#!/usr/bin/env python3
import datetime
import os
import signal
import subprocess
import sys
import traceback
from typing import List, Tuple, Union

import cereal.messaging as messaging
import selfdrive.sentry as sentry
from common.basedir import BASEDIR
from common.params import Params, ParamKeyType
from common.text_window import TextWindow
from selfdrive.boardd.set_time import set_time
from selfdrive.hardware import HARDWARE, PC, EON
from selfdrive.manager.helpers import unblock_stdout
from selfdrive.manager.process import ensure_running
from selfdrive.manager.process_config import managed_processes
# from selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from selfdrive.swaglog import cloudlog, add_file_handler
from selfdrive.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
                              terms_version, training_version
import json

sys.path.append(os.path.join(BASEDIR, "pyextra"))

def get_car_list() -> str:
  attrs = ['FINGERPRINTS', 'FW_VERSIONS']
  cars = dict({"cars": []})
  models = []
  for car_folder in [x[0] for x in os.walk('/data/openpilot/selfdrive/car')]:
    try:
      car_name = car_folder.split('/')[-1]
      if car_name != "mock":
        for attr in attrs:
          values = __import__('selfdrive.car.%s.values' % car_name, fromlist=[attr])
          if hasattr(values, attr):
            attr_values = getattr(values, attr)
          else:
            continue
          if isinstance(attr_values, dict):
            for f, v in attr_values.items():
              if f not in models:
                models.append(f)
    except (ImportError, IOError, ValueError):
      pass
  models.sort()
  cars["cars"] = models
  return json.dumps(cars)

def manager_init() -> None:
  # update system time from panda
  set_time(cloudlog)

  # save boot log
  # if not EON:
  #   subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)

  default_params: List[Tuple[str, Union[str, bytes]]] = [
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("HasAcceptedTerms", "0"),
    ("OpenpilotEnabledToggle", "1"),

    ("IsMetric", "1"),
    ("Licence", ""),
    ("CarList", ""),
    ("CarSelected", ""),
    ("Locale", "zh-TW"),
    ("Timezone", "Asia/Taipei"),

    ("UseOldPanda", "0"),
    ("UseStockLong", "1"),
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  # if params.get_bool("RecordFrontLock"):
  #   params.put_bool("RecordFront", True)
  #
  # if not params.get_bool("DisableRadar_Allow"):
  #   params.delete("DisableRadar")

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # install default ssh key
  install_key = False
  if os.path.isfile("/EON"):
    os.system("setprop persist.neos.ssh 1")
    os.system("echo -n 1 > /data/params/d/SshEnabled")
    if not os.path.isfile("/data/params/d/GithubSshKeys"):
      install_key = True
    else:
      with open('/data/params/d/GithubSshKeys') as f:
        if f.read().strip() == "":
          install_key = True

    if install_key:
      os.system("echo -n openpilot > /data/params/d/GithubUsername")
      os.system("cp /data/data/com.termux/files/home/setup_keys /data/params/d/GithubSshKeys")

  # set language
  if EON:
    language = subprocess.check_output(["getprop", "persist.sys.locale"], encoding='utf8').strip()
    if language != "":
      params.put("Locale", language)
    subprocess.call(['setprop', 'persist.sys.timezone', '"Asia/Taipei"'])

  # gen car list
  params.put("CarList", get_car_list())

  # is this dashcam?
  # if os.getenv("PASSIVE") is not None:
  #   params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

  # if params.get("Passive") is None:
  #   raise Exception("Passive must be set to continue")

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  params.put("CompletedTrainingVersion", training_version)
  # set version params
  params.put("Version", get_version())
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_commit(default=""))
  params.put("GitBranch", get_short_branch(default=""))
  params.put("GitRemote", get_origin(default=""))

  dongle_id = HARDWARE.get_serial()
  params.put("HardwareSerial", dongle_id)

  # set dongle id
  # reg_res = register(show_spinner=True)
  # if reg_res:
  #   dongle_id = reg_res
  # else:
  #   serial = params.get("HardwareSerial")
  #   raise Exception(f"Registration failed for device {serial}")
  # os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  #
  # if not is_dirty():
  #   os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id, version=get_version(), dirty=is_dirty(),
                       device=HARDWARE.get_device_type())


def manager_prepare() -> None:
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: List[str] = []
  # if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
  #   ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll=['deviceState'])
  pm = messaging.PubMaster(['managerState'])

  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  while True:
    sm.update()

    started = sm['deviceState'].started
    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("%s%s\u001b[0m" % ("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState')
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", param)
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  prepare_only = os.getenv("PREPAREONLY") is not None

  manager_init()

  # Start UI early so prepare can happen in the background
  if not prepare_only:
    managed_processes['ui'].start()

  manager_prepare()

  if prepare_only:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  if os.path.isfile("/EON"):
    if not os.path.isfile("/system/fonts/NotoSansCJKtc-Regular.otf"):
      os.system("mount -o remount,rw /system")
      os.system("rm -fr /system/fonts/NotoSansTC*.otf")
      os.system("rm -fr /system/fonts/NotoSansSC*.otf")
      os.system("rm -fr /system/fonts/NotoSansKR*.otf")
      os.system("rm -fr /system/fonts/NotoSansJP*.otf")
      os.system("cp -rf /data/openpilot/selfdrive/assets/fonts/NotoSansCJKtc-* /system/fonts/")
      os.system("cp -rf /data/openpilot/selfdrive/assets/fonts/fonts.xml /system/etc/fonts.xml")
      os.system("chmod 644 /system/etc/fonts.xml")
      os.system("chmod 644 /system/fonts/NotoSansCJKtc-*")
      os.system("mount -o remount,r /system")

  unblock_stdout()

  try:
    main()
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
