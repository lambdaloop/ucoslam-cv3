#include "downloadmanager.h"
DownloadManager::DownloadManager()
{
    connect(&manager, SIGNAL(finished(QNetworkReply*)),
            SLOT(downloadFinished(QNetworkReply*)));
}
void DownloadManager::setParams(QUrl inUrl,QString fileName){
    _inUrl=inUrl;
    _saveFileName=fileName;

}
void DownloadManager::on_downloadProgress(qint64 bytesReceived, qint64 bytesTotal){
    // calculate the download speed
    cerr<<"DownloadManager::on_downloadProgress "<<bytesReceived<<" "<<bytesTotal<<endl;
    emit downloadProgress(bytesReceived,bytesTotal);
}

void DownloadManager::doDownload(const QUrl &url)
{
    QNetworkRequest request(url);
    QNetworkReply *reply = manager.get(request);

    connect(reply, SIGNAL(downloadProgress(qint64,qint64)),
            SLOT(on_downloadProgress(qint64,qint64)));


//#if QT_CONFIG(ssl)
//    connect(reply, SIGNAL(sslErrors(QList<QSslError>)),
//            SLOT(sslErrors(QList<QSslError>)));
//#endif

    currentDownloads.append(reply);
}


bool DownloadManager::saveToDisk(const QString &filename, QIODevice *data)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        fprintf(stderr, "Could not open %s for writing: %s\n",
                qPrintable(filename),
                qPrintable(file.errorString()));
        return false;
    }

    file.write(data->readAll());
    file.close();

    return true;
}

bool DownloadManager::isHttpRedirect(QNetworkReply *reply)
{
    int statusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    return statusCode == 301 || statusCode == 302 || statusCode == 303
           || statusCode == 305 || statusCode == 307 || statusCode == 308;
}

void DownloadManager::execute()
{
        doDownload(_inUrl);
}

void DownloadManager::sslErrors(const QList<QSslError> &sslErrors)
{
//#if QT_CONFIG(ssl)
//    for (const QSslError &error : sslErrors)
//        fprintf(stderr, "SSL error: %s\n", qPrintable(error.errorString()));
//#else
//    Q_UNUSED(sslErrors);
//#endif
}

void DownloadManager::downloadFinished(QNetworkReply *reply)
{
    QUrl url = reply->url();
    if (reply->error()) {
        emit finished(false,tr("Download failed")+QString(reply->errorString()));
        std::cout<<"Download failed "<<reply->errorString().toStdString()<<std::endl;
    } else {
        if (isHttpRedirect(reply)) {
            emit finished(false,tr("Request was redirected"));
            fputs("Request was redirected.\n", stderr);
        } else {
            if (saveToDisk(_saveFileName, reply)) {
                emit finished(true,tr("Successfully saved to:")+_saveFileName);
                printf("Download of %s succeeded (saved to %s)\n",
                       url.toEncoded().constData(), qPrintable(_saveFileName));
            }
            else
                emit finished(false,tr("Could not save to file:")+_saveFileName);

        }
    }

    currentDownloads.removeAll(reply);
    reply->deleteLater();

//    if (currentDownloads.isEmpty()) {
//        // all downloads finished
//        QCoreApplication::instance()->quit();
//    }
}
