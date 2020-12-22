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
	bool setCloud(ccPointCloud* cloud);

private:

private slots:
	void onClearPushButtonClick();
	void onCalPushButtonClick();
	void onGenReportPushButtonClick();
	void onOkPushButtonClick();
	void onSwitchPushButtonClick();
	void handlePickedItem(ccHObject*, unsigned, int, int, const CCVector3&, const CCVector3d&);
	void dispToConsole(const QString&);

private:
	struct {
		qreal volumeUnder;
		qreal volumeAbove;
		qreal density;
	} report;

	struct CloudBackup
	{
		ccPointCloud* ref;
		RGBAColorsTableType* colors;
		bool hadColors;
		int displayedSFIndex;
		ccGenericGLDisplay* originDisplay;
		bool colorsWereDisplayed;
		bool sfWasDisplayed;
		bool wasVisible;
		bool wasEnabled;
		bool wasSelected;
		bool hadOctree;
		bool ownCloud;

		//! Default constructor
		CloudBackup()
			: ref(0)
			, colors(0)
			, hadColors(false)
			, displayedSFIndex(-1)
			, originDisplay(0)
			, colorsWereDisplayed(false)
			, sfWasDisplayed(false)
			, wasVisible(false)
			, wasEnabled(false)
			, wasSelected(false)
			, hadOctree(false)
			, ownCloud(false)
		{}

		//! Destructor
		~CloudBackup() { restore(); clear(); }

		//! Backups the given cloud
		void backup(ccPointCloud* cloud);

		//! Backups the colors (not done by default)
		bool backupColors();

		//! Restores the cloud
		void restore();

		//! Clears the structure
		void clear();
	};

private:
	ccMainAppInterface* m_app;

	ccGLWindow* m_glWindow;
	ccMesh* m_rasterMesh;
	std::vector<unsigned> m_pointsIdx;
	CloudBackup m_cloud;
	cc2DLabel m_label;

	QDir currentPath;

	qreal gridStep;
	quint32 gridWidth, gridHeight;

};

class WordAppType : public QAxWidget
{
public:
	WordAppType(const QString& c, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags())
		:QAxWidget(c, parent, f) {}
	~WordAppType()
	{
		this->dynamicCall("Quit()");
	}
};