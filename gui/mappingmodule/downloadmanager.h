#ifndef DownloadManager_H
#define DownloadManager_H
#include <QtCore>
#include <QtNetwork>
#include <QProgressBar>
#include <QProgressDialog>
#include <cstdio>
#include <iostream>
class QSslError;

using namespace std;
class DownloadManager: public QObject
{
    Q_OBJECT
    QNetworkAccessManager manager;
    QVector<QNetworkReply *> currentDownloads;
    QString _saveFileName;
    QUrl _inUrl;

public:
    DownloadManager();
    void setParams(QUrl inUrl,QString fileName);
public slots:
    void execute();
signals:
    void downloadProgress(qint64 bytesReceived, qint64 bytesTotal);
    void finished(bool status, QString msg);

private:
    void doDownload(const QUrl &url);
     bool saveToDisk(const QString &filename, QIODevice *data);
    static bool isHttpRedirect(QNetworkReply *reply);


private slots:
    void on_downloadProgress(qint64 bytesReceived, qint64 bytesTotal);
    void downloadFinished(QNetworkReply *reply);
    void sslErrors(const QList<QSslError> &errors);


};
#endif
