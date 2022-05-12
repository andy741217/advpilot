#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/toggle.h"

QFrame *horizontal_line(QWidget *parent = nullptr);

class ElidedLabel : public QLabel {
  Q_OBJECT

public:
  explicit ElidedLabel(QWidget *parent = 0);
  explicit ElidedLabel(const QString &text, QWidget *parent = 0);

signals:
  void clicked();

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent* event) override;
  void mouseReleaseEvent(QMouseEvent *event) override { emit clicked(); }
  QString lastText_, elidedText_;
};


class AbstractControl : public QFrame {
  Q_OBJECT

public:
  void setDescription(const QString &desc) {
    if (description) description->setText(desc);
  }

  void setTitle(const QString &title) {
    title_label->setText(title);
  }

signals:
  void showDescription();

protected:
  AbstractControl(const QString &title, const QString &desc = "", const QString &icon = "", QWidget *parent = nullptr);
  void hideEvent(QHideEvent *e) override;

  QHBoxLayout *hlayout;
  QPushButton *title_label;
  QLabel *description = nullptr;
};

// widget to display a value
class LabelControl : public AbstractControl {
  Q_OBJECT

public:
  LabelControl(const QString &title, const QString &text = "", const QString &desc = "", QWidget *parent = nullptr) : AbstractControl(title, desc, "", parent) {
    label.setText(text);
    label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    hlayout->addWidget(&label);
  }
  void setText(const QString &text) { label.setText(text); }

private:
  ElidedLabel label;
};

// widget for a button with a label
class ButtonControl : public AbstractControl {
  Q_OBJECT

public:
  ButtonControl(const QString &title, const QString &text, const QString &desc = "", QWidget *parent = nullptr);
  inline void setText(const QString &text) { btn.setText(text); }
  inline QString text() const { return btn.text(); }

signals:
  void clicked();

public slots:
  void setEnabled(bool enabled) { btn.setEnabled(enabled); };

private:
  QPushButton btn;
};

class ToggleControl : public AbstractControl {
  Q_OBJECT

public:
  ToggleControl(const QString &title, const QString &desc = "", const QString &icon = "", const bool state = false, QWidget *parent = nullptr) : AbstractControl(title, desc, icon, parent) {
    toggle.setFixedSize(150, 100);
    if (state) {
      toggle.togglePosition();
    }
    hlayout->addWidget(&toggle);
    QObject::connect(&toggle, &Toggle::stateChanged, this, &ToggleControl::toggleFlipped);
  }

  void setEnabled(bool enabled) { toggle.setEnabled(enabled); toggle.update(); }

signals:
  void toggleFlipped(bool state);

protected:
  Toggle toggle;
};

// widget to toggle params
class ParamControl : public ToggleControl {
  Q_OBJECT

public:
  ParamControl(const QString &param, const QString &title, const QString &desc, const QString &icon, QWidget *parent = nullptr) : ToggleControl(title, desc, icon, false, parent) {
    key = param.toStdString();
    QObject::connect(this, &ToggleControl::toggleFlipped, [=](bool state) {
      params.putBool(key, state);
    });
  }

  void showEvent(QShowEvent *event) override {
    if (params.getBool(key) != toggle.on) {
      toggle.togglePosition();
    }
  };

private:
  std::string key;
  Params params;
};

class ListWidget : public QWidget {
  Q_OBJECT
 public:
  explicit ListWidget(QWidget *parent = 0) : QWidget(parent), outer_layout(this) {
    outer_layout.setMargin(0);
    outer_layout.setSpacing(0);
    outer_layout.addLayout(&inner_layout);
    inner_layout.setMargin(0);
    inner_layout.setSpacing(25); // default spacing is 25
    outer_layout.addStretch();
  }
  inline void addItem(QWidget *w) { inner_layout.addWidget(w); }
  inline void addItem(QLayout *layout) { inner_layout.addLayout(layout); }
  inline void setSpacing(int spacing) { inner_layout.setSpacing(spacing); }

private:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setPen(Qt::gray);
    for (int i = 0; i < inner_layout.count() - 1; ++i) {
      QRect r = inner_layout.itemAt(i)->geometry();
      int bottom = r.bottom() + inner_layout.spacing() / 2;
      p.drawLine(r.left() + 40, bottom, r.right() - 40, bottom);
    }
  }
  QVBoxLayout outer_layout;
  QVBoxLayout inner_layout;
};

// convenience class for wrapping layouts
class LayoutWidget : public QWidget {
  Q_OBJECT

public:
  LayoutWidget(QLayout *l, QWidget *parent = nullptr) : QWidget(parent) {
    setLayout(l);
  };
};

class ClickableWidget : public QWidget {
  Q_OBJECT

public:
  ClickableWidget(QWidget *parent = nullptr);

protected:
  void mouseReleaseEvent(QMouseEvent *event) override;
  void paintEvent(QPaintEvent *) override;

signals:
  void clicked();
};

#include <QComboBox>
#include <QLineEdit>
#include <QTextCodec>
#include <QTimer>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

class CarModelComboBox : public QComboBox {
public:
  CarModelComboBox(QWidget *parent = nullptr) : QComboBox(parent), _isPopupShown(false), _timer(new QTimer(this)) {
    auto lineEdit = new QLineEdit;
    setEditable(true);

    lineEdit->installEventFilter(this);
    lineEdit->setReadOnly(true);
    lineEdit->setAlignment(Qt::AlignCenter);
    setLineEdit(lineEdit);
    setStyleSheet(R"(
      QScrollBar:vertical {
        border: none;
        background: transparent;
        width: 150px;
        margin: 0;
      }
      QComboBox {
        border-radius: 30px;
        font-size: 48px;
        font-weight: 400;
        background: #585858;
      }
      QComboBox:editable {
        background: #393939;
      }
      QComboBox::drop-down:button {
        border-radius: 0px;
        background: #393939;
      }
      QComboBox QAbstractItemView {
        border-bottom-right-radius: 10px;
        border-bottom-left-radius: 10px;
        background: #585858;
        border: 0px solid #585858;
        padding: 4px 4px 4px 4px;
      }
      QComboBox::down-arrow {
          image: none;
      }
    )");

    QString list = QString::fromStdString((params.get("CarList")).c_str());
    QJsonDocument document = QJsonDocument::fromJson(list.toUtf8());
    QJsonObject object = document.object();

    QJsonArray models = object.value("cars").toArray();
    addItem("-");
    int model_size = models.size();
    for (int i = 0; i < model_size; i++) {
      addItem(models.at(i).toString());
    }
    QString search_str = QString::fromUtf8((params.get("CarSelected")).c_str());
    int index = this->findText(search_str);
    if (index != -1)
       this->setCurrentIndex(index);

    _timer->setSingleShot(true);
    _timer->setInterval(100);

    QObject::connect(this, QOverload<const QString &>::of(&QComboBox::currentTextChanged), [=](const QString &text) {
      params.put("CarSelected", text == "-"? "" : text.toStdString());
    });
  }

protected:
  bool eventFilter(QObject *object, QEvent *event) override {
    if (object == lineEdit() && event->type() == QEvent::MouseButtonPress) {
      if (!_timer->isActive()) {
        if (!_isPopupShown)
          showPopup();
        else if (_isPopupShown)
          hidePopup();
        return true;
      }
    }
    return false;
  }

  void showPopup() override {
    QComboBox::showPopup();
    _isPopupShown = true;
    _timer->start();
  }

  void hidePopup() override {
    QComboBox::hidePopup();
    _isPopupShown = false;
    _timer->start();
  }

private:
  bool _isPopupShown;
  QTimer *_timer;
  Params params;
};
