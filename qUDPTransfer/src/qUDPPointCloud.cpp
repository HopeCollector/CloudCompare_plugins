#include "qUDPPointCloud.h"

qUDPPointCloud::qUDPPointCloud(QString name) throw()
	:ccPointCloud(name)
{
}

qUDPPointCloud::~qUDPPointCloud()
{
}

void qUDPPointCloud::addPoints(const QByteArray& data)
{
	const quint16 POINTSIZE = sizeof(point);
	auto points = reinterpret_cast<const point*>(data.data());
	quint64 len = data.size() / (POINTSIZE);

	for (size_t i = 0; i < len; i++)
	{
		if (points[i].x != points[i].x
			|| points[i].y != points[i].y
			|| points[i].z != points[i].z)
		{
			continue;
		}
		else
		{
			m_points.push_back(CCVector3::fromArray(reinterpret_cast<const float*>(points+i)));
		}
	}
	prepareDisplayForRefresh();
	m_bbox.setValidity(false);
}

void qUDPPointCloud::addPoints(const QString fileName)
{
	QFile in(fileName);
	if (!in.open(QIODevice::ReadOnly))
	{
		return;
	}
	QDataStream data(&in);
	point p;
	char* buffer = reinterpret_cast<char*>(&p);
	while(!in.atEnd())
	{
		data.readRawData(buffer, sizeof(p));
		m_points.push_back(CCVector3::fromArray(
			reinterpret_cast<float*>(buffer)));
	}
	in.close();
	redrawDisplay();
	m_bbox.setValidity(false);
}
