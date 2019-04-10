#include "downloadvocabularydialog.h"
#include "ui_downloadvocabularydialog.h"

DownloadVocabularyDialog::DownloadVocabularyDialog(QUrl downloadUrl,QString filepath,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DownloadVocabularyDialog)
{
    ui->setupUi(this);
    _DwManager.setParams(downloadUrl,filepath);
    ui->progressBar->setValue(0);
    connect(&_DwManager,SIGNAL(downloadProgress(qint64,qint64)),this,SLOT(on_downloadProgress(qint64,qint64)));
    connect(&_DwManager,&DownloadManager::finished,this,&DownloadVocabularyDialog::on_downloadFinished);
}

DownloadVocabularyDialog::~DownloadVocabularyDialog()
{
    delete ui;
}
void DownloadVocabularyDialog::on_downloadFinished(bool status, QString msg){

    resultSucceed=status;
    _errMsg=msg;
    accept();
}

void DownloadVocabularyDialog::on_downloadProgress(qint64 bytesReceived, qint64 bytesTotal)
{
    ui->progressBar->setMaximum(bytesTotal);
    ui->progressBar->setValue(bytesReceived);
}

void DownloadVocabularyDialog::showEvent(QShowEvent * event){
    QTimer::singleShot(0, &_DwManager, SLOT(execute()));
}

void DownloadVocabularyDialog::on_pushButton_clicked()
{
    QDialog::reject();
}
