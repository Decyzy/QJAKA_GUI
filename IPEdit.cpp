//
// https://github.com/windywater/QtIPEdit
//

#include "IPEdit.h"

#include <QApplication>
#include <QClipboard>
#include <QDebug>
#include <QKeyEvent>
#include <QPainter>
#include <QRegExp>
#include <QRegExpValidator>
#include <QShortcutEvent>
#include <QStyle>
#include <QStyleOptionButton>

IPEdit::IPEdit(QWidget* parent) : QWidget(parent) { init(); }

IPEdit::~IPEdit() {}

void IPEdit::init() {
    m_layout = new QHBoxLayout(this);
    m_layout->setSpacing(3);
    m_layout->setContentsMargins(2, 0, 2, 0);

    m_edit1 = new QLineEdit(this);
    m_edit2 = new QLineEdit(this);
    m_edit3 = new QLineEdit(this);
    m_edit4 = new QLineEdit(this);

    initForEdit(m_edit1);
    initForEdit(m_edit2);
    initForEdit(m_edit3);
    initForEdit(m_edit4);

    m_dot1 = new QLabel(".", this);
    m_dot2 = new QLabel(".", this);
    m_dot3 = new QLabel(".", this);

    m_layout->addWidget(m_edit1, 1);
    m_layout->addWidget(m_dot1);
    m_layout->addWidget(m_edit2, 1);
    m_layout->addWidget(m_dot2);
    m_layout->addWidget(m_edit3, 1);
    m_layout->addWidget(m_dot3);
    m_layout->addWidget(m_edit4, 1);
}

void IPEdit::initForEdit(QLineEdit* edit) {
    edit->setFrame(false);
    edit->setAlignment(Qt::AlignCenter);
    edit->installEventFilter(this);

    QRegExpValidator* validator = new QRegExpValidator(QRegExp("^(2[0-4]\\d|25[0-5]|[01]?\\d\\d?)$"), this);
    edit->setValidator(validator);

    connect(edit, SIGNAL(textChanged(const QString&)), this, SLOT(editTextChanged(const QString&)));
}

QLineEdit* IPEdit::nextEdit(QLineEdit* curEdit) {
    if (curEdit == m_edit1)
        return m_edit2;
    else if (curEdit == m_edit2)
        return m_edit3;
    else if (curEdit == m_edit3)
        return m_edit4;
    else
        return NULL;
}

bool IPEdit::isEdit(QObject* object) {
    return (object == m_edit1 || object == m_edit2 || object == m_edit3 || object == m_edit4);
}

QString IPEdit::text() {
    QString sec1 = m_edit1->text().isEmpty() ? "0" : m_edit1->text();
    QString sec2 = m_edit2->text().isEmpty() ? "0" : m_edit2->text();
    QString sec3 = m_edit3->text().isEmpty() ? "0" : m_edit3->text();
    QString sec4 = m_edit4->text().isEmpty() ? "0" : m_edit4->text();
    return QString("%1.%2.%3.%4").arg(sec1).arg(sec2).arg(sec3).arg(sec4);
}

void IPEdit::setText(const QString& text) {
    QRegExp reg("^((2[0-4]\\d|25[0-5]|[01]?\\d\\d?)\\.){3}(2[0-4]\\d|25[0-5]|[01]?\\d\\d?)$");
    if (!reg.exactMatch(text)) return;

    QStringList ips = text.split(".");
    m_edit1->setText(ips.at(0));
    m_edit2->setText(ips.at(1));
    m_edit3->setText(ips.at(2));
    m_edit4->setText(ips.at(3));
}

void IPEdit::paintEvent(QPaintEvent* event) {
    QPainter painter(this);

    QStyleOptionFrame option;
    option.initFrom(this);
    option.lineWidth = 1;
    style()->drawPrimitive(QStyle::PE_PanelLineEdit, &option, &painter, this);
    QWidget::paintEvent(event);
}

bool IPEdit::eventFilter(QObject* object, QEvent* event) {
    if (isEdit(object)) {
        if (event->type() == QEvent::KeyPress) {
            QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
            if (keyEvent->key() == Qt::Key_Period) {
                QLineEdit* next = nextEdit(qobject_cast<QLineEdit*>(object));
                if (next) {
                    next->setFocus();
                    next->selectAll();
                }
            } else if (keyEvent->matches(QKeySequence::Paste)) {
                QString clip = QApplication::clipboard()->text(QClipboard::Clipboard);
                if (clip.split(".").size() == 4) {
                    setText(clip);
                    return true;
                }
            }
        }
    }

    return QWidget::eventFilter(object, event);
}

void IPEdit::editTextChanged(const QString& text) {
    QLineEdit* curEdit = qobject_cast<QLineEdit*>(sender());
    if (text.size() == 3) {
        QLineEdit* next = nextEdit(curEdit);
        if (next) {
            next->setFocus();
            next->selectAll();
        }
    }
}
