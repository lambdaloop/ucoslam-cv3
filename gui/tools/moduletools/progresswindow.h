#ifndef PROGRESSWINDOW_H
#define PROGRESSWINDOW_H

#include <QWidget>
#include <string>
#include "exports.h"
namespace Ui {
class progresswindow;
}

class APP_MODULESET_TOOLS_API progresswindow : public QWidget
{
    Q_OBJECT

public:
    explicit progresswindow(std::string name,QWidget *parent = 0);
    ~progresswindow();

public slots:
    void notify_action_progress(int,std::string);
private slots:

private:
    Ui::progresswindow *ui;
    std::string _name;
    bool _isExpanded;
};

#endif // PROGRESSWINDOW_H
