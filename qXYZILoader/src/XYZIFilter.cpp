//##########################################################################
//#                                                                        #
//#                           XYZILoaderIOPlugin                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "XYZIFilter.h"

#include <ccPointCloud.h>
#include <ccProgressDialog.h>

#include <QString>
#include <QFileInfo>


XYZIFilter::XYZIFilter()
	: FileIOFilter( {
					"XYZI Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "xyzi" },
					"foo",
					QStringList{ "XYZI file (*.xyzi)" },
					QStringList(),
					Import
					} )
{
}

CC_FILE_ERROR XYZIFilter::loadFile( const QString &fileName, ccHObject &container, LoadParameters &parameters )
{	
	Q_UNUSED( container );
	Q_UNUSED( parameters );
	
	QFile file( fileName );
	QFileInfo fileinfo(fileName);
	
	if ( !file.open( QIODevice::ReadOnly ) )
	{
		return CC_FERR_READING;
	}

	auto cloud = new ccPointCloud(fileName);
	container.addChild(cloud);
	quint64 pointCnt = fileinfo.size() / 4 / sizeof(float);
	cloud->reserve(pointCnt);

	ccProgressDialog dlg(true);
	dlg.setMethodTitle(QString::fromLocal8Bit("导入文件"));
	dlg.setInfo(QString::fromLocal8Bit("正在导入点云文件..."));
	dlg.start();
	CCCoreLib::NormalizedProgress nprogress(&dlg, pointCnt);

	float point[4];
	char* buffer = reinterpret_cast<char*>(point);
	while (file.read(buffer, 4 * sizeof(float))) {
		if (std::isfinite(point[0]) &&
			std::isfinite(point[1]) &&
			std::isfinite(point[2])) {
			cloud->addPoint(CCVector3::fromArray(point));
		}

		if (!nprogress.oneStep()) {
			container.removeChild(cloud);
			//delete cloud;
			return CC_FERR_CANCELED_BY_USER;
		}
	}

	return CC_FERR_NO_ERROR;
}

bool XYZIFilter::canSave( CC_CLASS_ENUM type, bool &multiple, bool &exclusive ) const
{
	Q_UNUSED( type );
	Q_UNUSED( multiple );
	Q_UNUSED( exclusive );

	// ... can we save this?
	return false;
}
