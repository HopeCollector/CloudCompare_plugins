#pragma once
#include <ccPointCloud.h>
#include <QByteArray>
#include <QDataStream>
#include <QThread>

class qUDPPointCloud :
    public ccPointCloud
{
    struct point 
    {
        float x, y, z, i;
    };
public:
    qUDPPointCloud(QString name = QString()) throw();
    ~qUDPPointCloud();
    void addPoints(const QByteArray& data);
    void addPoints(const QString);
};

