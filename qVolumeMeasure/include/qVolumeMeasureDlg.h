#pragma once
#include "ui_qVolumeMeasure.h"

#include <ccMainAppInterface.h>
#include <ccGLWindow.h>
#include <ccPointCloud.h>
#include <ccGlFilter.h>
#include <cc2DLabel.h>
#include <ccRasterGrid.h>
#include <ccProgressDialog.h>
#include <ccOctreeProxy.h>
#include <PointProjectionTools.h>
#include <ccMesh.h>
#include <ccGLMatrix.h>

#include <QDateTime>
#include <QDialog>
#include <QMainWindow>
#include <QAxObject>
#include <QAxWidget>
#include <QTextStream>
#include <QFileDialog>
#include <QTemporaryFile>
#include <QMessageBox>

class qVolumeMeasureDlg : public QDialog, public Ui::VolumeMeasureDlg
{
	Q_OBJECT

public:
	explicit qVolumeMeasureDlg(ccMainAppInterface* app = nullptr);
	~qVolumeMeasureDlg();
	void setCloud(ccPointCloud* cloud);

private:

private slots:
	void onClearPushButtonClick();
	void onCalPushButtonClick();
	void onGenReportPushButtonClick();
	void onOkPushButtonClick();
	void handlePickedItem(ccHObject*, unsigned, int, int, const CCVector3&, const CCVector3d&);
	void dispToConsole(const QString&);

private:
	struct {
		qreal volumeUnder;
		qreal volumeAbove;
		qreal density;
	} report;

private:
	ccPointCloud* m_cloud;
	ccMesh* m_rasterMesh;
	ccMainAppInterface* m_app;
	ccGLWindow* m_glWindow;
	ccGLWindow* m_preGlWindow;
	cc2DLabel m_label;
	std::vector<unsigned> m_pointsIdx;
	QDir currentPath;
	qreal gridStep;
	quint32 gridWidth, gridHeight;

};