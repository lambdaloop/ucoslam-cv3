#ifndef CAMERACONTROLS_H
#define CAMERACONTROLS_H

#include <QWidget>
#include "genericimagereader.h"
#include <memory>
namespace Ui {
class cameracontrols;
}

class cameracontrols : public QWidget
{
    Q_OBJECT

public:
    explicit cameracontrols(QWidget *parent = nullptr);
    ~cameracontrols();

    void set(std::shared_ptr<GenericImageReader> reader);
private slots:
    void on_pushButton_clicked();

    void on_comboBox_currentIndexChanged(int index);

signals:
    void resolutionChanged(QString resolution);
    void reconnect(int idex,QString resolution);
private:
    Ui::cameracontrols *ui;
    std::shared_ptr<GenericImageReader> _reader;
};

#endif // CAMERACONTROLS_H
