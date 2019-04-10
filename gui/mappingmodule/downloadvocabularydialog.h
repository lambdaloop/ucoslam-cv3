#ifndef DOWNLOADVOCABULARYDIALOG_H
#define DOWNLOADVOCABULARYDIALOG_H

#include <QDialog>
#include <QUrl>
#include "downloadmanager.h"
namespace Ui {
class DownloadVocabularyDialog;
}

class DownloadVocabularyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DownloadVocabularyDialog(QUrl downloadUrl,QString filepath,QWidget *parent = nullptr);
    ~DownloadVocabularyDialog();
    bool succeeded()const{return resultSucceed;}
    QString errorMsg()const{return _errMsg;}

private slots:
    void on_pushButton_clicked();
    void on_downloadProgress(qint64 bytesReceived, qint64 bytesTotal);
    void on_downloadFinished(bool status, QString msg);
public :
    void	showEvent(QShowEvent * event);
private:
    Ui::DownloadVocabularyDialog *ui;
    DownloadManager _DwManager;
    bool resultSucceed=false;
    QString _errMsg;
};

#endif // DOWNLOADVOCABULARYDIALOG_H
