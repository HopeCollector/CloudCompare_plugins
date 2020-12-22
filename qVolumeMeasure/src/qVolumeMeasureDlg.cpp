#include "qVolumeMeasureDlg.h"

qVolumeMeasureDlg::qVolumeMeasureDlg(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr,  Qt::Dialog | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_app(app)
	, m_glWindow(nullptr)
	, m_rasterMesh(nullptr)
	, m_cloud()
	, m_label(cc2DLabel("Ground"))
	, m_pointsIdx(std::vector<unsigned>())
	, currentPath(QDir("./"))
	, gridStep(0.15)
	, gridWidth(0)
	, gridHeight(0)
{
	setupUi(this);
	setModal(true);

	{
		QWidget* glWidget = nullptr;
		m_app->createGLWindow(m_glWindow, glWidget);
		assert(m_glWindow && glWidget);

		m_glWindow->setPerspectiveState(false, true);
		m_glWindow->displayOverlayEntities(true);
		m_glWindow->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_glWindow->setPickingMode(ccGLWindow::POINT_PICKING);

		//add window to the input frame (if any)
		viewFrame->setLayout(new QHBoxLayout());
		viewFrame->layout()->addWidget(glWidget);
	}

	{
		connect(clearPushButton, &QPushButton::clicked, this, &qVolumeMeasureDlg::onClearPushButtonClick);
		connect(okPushButton, &QPushButton::clicked, this, &qVolumeMeasureDlg::onOkPushButtonClick);
		connect(calPushButton, &QPushButton::clicked, this, &qVolumeMeasureDlg::onCalPushButtonClick);
		connect(genReportPushButton, &QPushButton::clicked, this, &qVolumeMeasureDlg::onGenReportPushButtonClick);
		connect(switchPcMeshPushButton, &QPushButton::clicked, this, &qVolumeMeasureDlg::onSwitchPushButtonClick);
		connect(densityLineEdit, &QLineEdit::textChanged, this, [&](const QString& s) {report.density = s.toDouble();});
		connect(m_glWindow, &ccGLWindow::itemPicked, this, &qVolumeMeasureDlg::handlePickedItem);
	}

	{
		okPushButton->setEnabled(false);
		clearPushButton->setEnabled(false);
		calPushButton->setEnabled(false);
		genReportPushButton->setEnabled(false);
		switchPcMeshPushButton->setEnabled(false);
	}

	{
		m_glWindow->addToOwnDB(&m_label);
		m_label.setVisible(true);
		m_label.setDisplayedIn2D(false);
	}

	{
		report.volumeAbove = 0;
		report.volumeUnder = 0;
		report.density = 0;
	}
}

qVolumeMeasureDlg::~qVolumeMeasureDlg()
{
	if (m_glWindow)
	{
		m_glWindow->getOwnDB()->removeAllChildren();
		if (m_app)
		{
			m_app->destroyGLWindow(m_glWindow);
			m_glWindow = nullptr;
		}
	}

	if (m_cloud.originDisplay)
		static_cast<ccGLWindow*>(m_cloud.originDisplay)->zoomGlobal();
}

bool qVolumeMeasureDlg::setCloud(ccPointCloud* cloud)
{
	m_cloud.backup(cloud);
	if (!m_cloud.backupColors())
	{
		//failed to backup the cloud colors
		QMessageBox::warning(this,
			QString::fromLocal8Bit("警告"),
			QString::fromLocal8Bit("运行内存不足"));
		return false;
	}

	if (!cloud->getOctree())
	{
		ccProgressDialog pDlg(true, this);
		ccOctree::Shared octree = cloud->computeOctree(&pDlg);
		if (!octree)
		{
			QMessageBox::warning(this,
				QString::fromLocal8Bit("警告"),
				QString::fromLocal8Bit("运行内存不足"));
			return false;
		}
	}

	ccRasterGrid::ComputeGridSize(2, cloud->getOwnBB(), gridStep, gridWidth, gridHeight);
	m_glWindow->addToOwnDB(cloud);
	m_glWindow->zoomGlobal();
	m_glWindow->setView(CC_TOP_VIEW);
	m_glWindow->redraw();


	QMessageBox::information(this
		, QString::fromLocal8Bit("温馨提示")
		, QString::fromLocal8Bit("请从地面选取三个点（左键点击点云即可），帮助软件确定地面位置。选取成功后点击确认按钮保存"));
	return true;
}

void qVolumeMeasureDlg::onClearPushButtonClick()
{
	if (m_rasterMesh) {
		m_rasterMesh->removeAllChildren();
		m_glWindow->removeFromOwnDB(m_rasterMesh);
		m_rasterMesh = nullptr;
	}

	clearPushButton->setEnabled(false);
	okPushButton->setEnabled(false);
	calPushButton->setEnabled(false);
	genReportPushButton->setEnabled(false);
	switchPcMeshPushButton->setEnabled(false);

	m_label.clear();
	m_label.setVisible(true);
	m_cloud.ref->setEnabled(true);
	m_pointsIdx.clear();
	m_glWindow->redraw();
}

void qVolumeMeasureDlg::onCalPushButtonClick()
{
	calPushButton->setEnabled(false);

	// gen progress dlg
	auto pDlg = std::make_unique<ccProgressDialog>(false, this);
	{
		pDlg->setMethodTitle(QString::fromLocal8Bit("计算体积......"));
		pDlg->start();
		pDlg->show();
		QCoreApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nProgress(pDlg.get(), 3+gridHeight*gridWidth, 3+gridHeight*gridWidth);

	// get ground height
	pDlg->setInfo(QString::fromLocal8Bit("计算地面高度..."));
	qreal groundHeight = 0;
	ccGLMatrix rotateMatrix;
	{
		auto zAix = CCVector3(0, 0, 1);
		auto line1 = *(m_cloud.ref->getPoint(m_pointsIdx[0])) - *(m_cloud.ref->getPoint(m_pointsIdx[1]));
		auto line2 = *(m_cloud.ref->getPoint(m_pointsIdx[0])) - *(m_cloud.ref->getPoint(m_pointsIdx[2]));
		auto directon = line1.cross(line2);
		directon.normalize();
		if (directon.angle_rad(zAix) > 1.57) // 如果 direction 的方向是反的
			directon *= -1; // 给他翻转过来
		rotateMatrix = ccGLMatrix::FromToRotation(directon,zAix);
		m_cloud.ref->rotateGL(rotateMatrix);
		m_cloud.ref->applyGLTransformation_recursive();

		// 重新计算旋转后的地面高度，这次三个点的高度是相同的
		groundHeight = m_cloud.ref->getPoint(m_pointsIdx.front())->z;
	}
	nProgress.oneStep();

	// init ceil raster
	pDlg->setInfo(QString::fromLocal8Bit("初始化栅格..."));
	ccRasterGrid ceilRaster;
	{
		auto box = m_cloud.ref->getOwnBB();
		auto minConer = CCVector3d::fromArray(box.minCorner().u);

		auto gridTotalSize = gridWidth * gridHeight;
		if (gridTotalSize == 1)
		{
			dispToConsole(QString::fromLocal8Bit("点云体积太小，针对当前点云的计算结果误差较大"));
		}
		else if (gridTotalSize > 10000000)
		{
			dispToConsole(QString::fromLocal8Bit("点云体积太大，针对当前点云的计算用时较长"));
		}
		if (!ceilRaster.init(gridWidth, gridHeight, gridStep, minConer))
		{
			QMessageBox::warning(this,
				QString::fromLocal8Bit("警告"),
				QString::fromLocal8Bit("运行内存不足"));
			assert(false);
		}
		if (!ceilRaster.fillWith(m_cloud.ref, 2,
			ccRasterGrid::ProjectionType::PROJ_AVERAGE_VALUE,
			true,
			ccRasterGrid::INVALID_PROJECTION_TYPE))
		{
			QMessageBox::warning(this,
				QString::fromLocal8Bit("警告"),
				QString::fromLocal8Bit("运行内存不足"));
			assert(false);
		}
		ceilRaster.fillEmptyCells(ccRasterGrid::INTERPOLATE);
		dispToConsole(
			QString::fromLocal8Bit("栅格规模: %1 x %2")
			.arg(ceilRaster.width).arg(ceilRaster.height)
		);
	}
	nProgress.oneStep();

	pDlg->setInfo(QString::fromLocal8Bit("计算体积，单元数量: %1 x %2").arg(gridWidth).arg(gridHeight));
	{
		qreal& volumeAbove = report.volumeAbove = 0;
		qreal& volumeUnder = report.volumeUnder = 0;
		for (size_t i = 0; i < gridHeight; i++)
		{
			for (size_t j = 0; j < gridWidth; j++)
			{
				if (std::isfinite(ceilRaster.rows[i][j].h))
				{
					auto h = ceilRaster.rows[i][j].h - groundHeight;
					if (h > 0)
						volumeAbove += h;
					else
						volumeUnder -= h;
				}
			}
			nProgress.oneStep();
		}
		
		volumeAbove *= static_cast<qreal>(gridStep * gridStep);
		volumeUnder *= static_cast<qreal>(gridStep * gridStep);

		dispToConsole(QString::fromLocal8Bit(
			"\n======================"
			"\n= 地上体积：%1 立方米"
			"\n= 地下体积：%2 立方米"
			"\n======================\n"
		).arg(volumeAbove, 0, 'f', 3).arg(volumeUnder, 0, 'f', 3));
	}

	pDlg->setInfo(QString::fromLocal8Bit("生成 mesh ..."));
	{
		auto rasterCloud = ceilRaster.convertToCloud({ ccRasterGrid::PER_CELL_HEIGHT },
			false, // interpolate scalar field
			false, // interpolate colors
			false, // resample input cloud xy
			false, // resample input cloud z
			m_cloud.ref, 2, m_cloud.ref->getOwnBB(),
			true, // fill Empty Cells
			groundHeight, 
			false); // export to original cs

		if (rasterCloud) {
			int activeSFIndex = rasterCloud->getScalarFieldIndexByName("Height grid values");
			rasterCloud->showSF(activeSFIndex >= 0);
			if (activeSFIndex < 0 && rasterCloud->getNumberOfScalarFields() != 0)
			{
				//if no SF is displayed, we should at least set a valid one (for later)
				activeSFIndex = static_cast<int>(rasterCloud->getNumberOfScalarFields()) - 1;
			}
			rasterCloud->setCurrentDisplayedScalarField(activeSFIndex);

			std::string errorStr;
			auto baseMesh = CCCoreLib::PointProjectionTools::computeTriangulation(rasterCloud,
				CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED,
				CCCoreLib::PointProjectionTools::IGNORE_MAX_EDGE_LENGTH,
				2, errorStr);
			if (baseMesh)
			{
				m_rasterMesh = new ccMesh(baseMesh, rasterCloud);
				delete baseMesh;
			}
			if (m_rasterMesh)
			{
				rasterCloud->setEnabled(false);
				rasterCloud->setVisible(true);
				rasterCloud->setName("vertices");

				m_rasterMesh->addChild(rasterCloud);
				m_rasterMesh->setDisplay_recursive(m_glWindow);
				m_rasterMesh->setName(m_cloud.ref->getName() + ".mesh");
				m_rasterMesh->showSF(true);
				m_rasterMesh->showColors(true);

				rotateMatrix.invert();
				m_cloud.ref->rotateGL(rotateMatrix);
				m_cloud.ref->applyGLTransformation_recursive();
				m_cloud.ref->setEnabled(false);

				m_glWindow->addToOwnDB(m_rasterMesh);
				m_glWindow->zoomGlobal();
				m_glWindow->redraw();
			}
		}
		else {
			dispToConsole(QString::fromLocal8Bit("运行内存不足，生成失败"));
		}
	}

	genReportPushButton->setEnabled(true);
	switchPcMeshPushButton->setEnabled(true);
	nProgress.oneStep();
	return;
}

void qVolumeMeasureDlg::onGenReportPushButtonClick()
{
	// check density
	{
		auto density = densityLineEdit->text().toFloat();
		if (density < 0 || density > 23) {
			QMessageBox::warning(this,
				QString::fromLocal8Bit("警告"),
				QString::fromLocal8Bit("请输入合理的密度值 0.0 ~ 23.0"));
			return;
		}
	}

	// get file name
	QString fileName;
	QString imgName;
	QTemporaryFile img("XXXXXX.jpg", this);
	{
		QString defaultName = QString::fromLocal8Bit("测量报告")
			+ QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss")
			+ ".doc";

		fileName = QFileDialog::getSaveFileName(
			this,
			QString::fromLocal8Bit("保存文件"),
			currentPath.absoluteFilePath(defaultName),
			"Doc (*.doc)"
		);
		
		if (fileName.isEmpty()) return;

		img.open();
		imgName = img.fileName();
	}

	// gen progress dialog
	auto pDlg = std::make_unique<ccProgressDialog>(false, this);
	{
		pDlg->setMethodTitle(QString::fromLocal8Bit("生成报告......"));
		pDlg->start();
		pDlg->show();
		QCoreApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nProgress(pDlg.get(), 9);

	// create screen short
	{
		pDlg->setInfo(QString::fromLocal8Bit("生成点云截图 ..."));
		m_glWindow->renderToFile(img.fileName());
		nProgress.oneStep();
	}

	// create doc file
	{
		pDlg->setInfo(QString::fromLocal8Bit("生成 Word 文档 ..."));
		QScopedPointer<WordAppType> wordApp(new WordAppType("Word.Application", this, Qt::MSWindowsOwnDC));
		/*if (!m_word.get())
			m_word = std::make_unique<QAxWidget>("Word.Application", this, Qt::MSWindowsOwnDC);*/
		wordApp->setProperty("Visible", false);
		auto documents = wordApp->querySubObject("Documents");
		documents->dynamicCall("Add(QString)", currentPath.absoluteFilePath("template.dot"));
		auto document = wordApp->querySubObject("ActiveDocument");
		nProgress.oneStep();

		auto bookmarkTime = document->querySubObject("Bookmarks(QVariant)", "GenerateTime");
		if (!bookmarkTime->isNull()) {
			bookmarkTime->dynamicCall("Select(void)");
			bookmarkTime->querySubObject("Range")
				->setProperty("Text", QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss"));
			nProgress.oneStep();
		}

		auto bookmarkVolume = document->querySubObject("Bookmarks(Volume)");
		if (!bookmarkVolume->isNull()) {
			bookmarkVolume->dynamicCall("Select(void)");
			bookmarkVolume->querySubObject("Range")
				->setProperty("Text", QString::number(report.volumeAbove, 'f', 3));
			nProgress.oneStep();
		}

		auto bookmarkDensity = document->querySubObject("Bookmarks(QVariant)", "Density");
		if (!bookmarkDensity->isNull()) {
			bookmarkDensity->dynamicCall("Select(void)");
			bookmarkDensity->querySubObject("Range")
				->setProperty("Text",QString::number(report.density, 'f', 3));
			nProgress.oneStep();
		}

		auto bookmarkWeight = document->querySubObject("Bookmarks(QVariant)", "Weight");
		if (!bookmarkWeight->isNull()) {
			bookmarkWeight->dynamicCall("Select(void)");
			bookmarkWeight->querySubObject("Range")
				->setProperty("Text", QString::number(report.volumeAbove* report.density, 'f', 3));
			nProgress.oneStep();
		}

		auto bookmarkPicture = document->querySubObject("Bookmarks(QVariant)", "Picture");
		if (!bookmarkPicture->isNull()) {
			bookmarkPicture->dynamicCall("Select(void)");
			auto tmp = document->querySubObject("InlineShapes");
			tmp->dynamicCall("AddPicture(const QString&)", imgName);
			nProgress.oneStep();
		}

		auto bookmarkDetail = document->querySubObject("Bookmarks(QVariant)", "Detail");
		if (!bookmarkDetail->isNull()) {
			bookmarkDetail->dynamicCall("Select(void)");
			bookmarkDetail->querySubObject("Range")
				->setProperty("Text", QString::fromLocal8Bit(
					"地上体积：%1 立方米\n"
					"地下体积：%2 立方米\n"
				).arg(report.volumeAbove, 0, 'f', 3).arg(report.volumeUnder, 0, 'f', 3));
			nProgress.oneStep();
		}

		pDlg->setInfo(QString::fromLocal8Bit("保存报告 ..."));
		document->dynamicCall("SaveAs(const QString&)", fileName);
		document->dynamicCall("Close(boolean)", true);
		dispToConsole(QString::fromLocal8Bit("报告已保存到：%1").arg(fileName));
	}

	nProgress.oneStep();
	return;
}

void qVolumeMeasureDlg::onOkPushButtonClick()
{
	clearPushButton->setEnabled(false);
	calPushButton->setEnabled(true);
	okPushButton->setEnabled(false);

	m_label.setVisible(false);

	m_cloud.ref->showSF(false);
	m_cloud.ref->setColor(ccColor::white);

	m_glWindow->redraw();

	dispToConsole(QString::fromLocal8Bit("基准地面保存成功！"));
}

void qVolumeMeasureDlg::onSwitchPushButtonClick()
{
	clearPushButton->setEnabled(true);

	if (m_cloud.ref->isEnabled())
	{
		m_cloud.ref->setEnabled(false);
		m_rasterMesh->setEnabled(true);
	}
	else
	{
		m_cloud.ref->setEnabled(true);
		m_rasterMesh->setEnabled(false);
	}
	m_glWindow->redraw();
}

void qVolumeMeasureDlg::handlePickedItem(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& p, const CCVector3d& uwv)
{
	if (m_pointsIdx.size() < 3 && entity)
	{
		m_label.addPickedPoint(static_cast<ccGenericPointCloud*>(entity), itemIdx);
		m_pointsIdx.push_back(itemIdx);
		m_glWindow->redraw();
		clearPushButton->setEnabled(m_pointsIdx.size() > 0);
		okPushButton->setEnabled(m_pointsIdx.size() == 3);

		dispToConsole(QString::fromLocal8Bit("点 %4 (%1, %2, %3)")
		.arg(p.x).arg(p.y).arg(p.z)
		.arg(m_pointsIdx.size()));
	}
	else
	{
		dispToConsole(QString::fromLocal8Bit("无效点，已舍弃"));
	}
}

void qVolumeMeasureDlg::dispToConsole(const QString& msg)
{
	textBrowser->append(msg);
}


void qVolumeMeasureDlg::CloudBackup::backup(ccPointCloud* cloud)
{
	//save state
	assert(!colors);
	wasVisible = cloud->isVisible();
	wasEnabled = cloud->isEnabled();
	wasSelected = cloud->isSelected();
	hadColors = cloud->hasColors();
	displayedSFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
	originDisplay = cloud->getDisplay();
	colorsWereDisplayed = cloud->colorsShown();
	sfWasDisplayed = cloud->sfShown();
	hadOctree = (cloud->getOctree() != nullptr);
	ref = cloud;
}

bool qVolumeMeasureDlg::CloudBackup::backupColors()
{
	if (!ref)
	{
		assert(false);
		return false;
	}

	//we backup the colors (as we are going to change them)
	if (ref->hasColors())
	{
		colors = new RGBAColorsTableType;
		if (!colors->resizeSafe(ref->size()))
		{
			//not enough memory
			colors->release();
			colors = nullptr;
			return false;
		}

		//copy the existing colors
		for (unsigned i = 0; i < ref->size(); ++i)
		{
			colors->setValue(i, ref->getPointColor(i));
		}
	}

	return true;
}

void qVolumeMeasureDlg::CloudBackup::restore()
{
	if (!ref)
	{
		//nothing to do
		return;
	}

	if (!hadOctree)
	{
		//we can only delete the octree if it has not already been added to the DB tree!!!
		if (!ref->getParent())
		{
			ref->deleteOctree();
		}
	}

	if (hadColors)
	{
		//restore original colors
		if (colors)
		{
			assert(ref->hasColors());
			for (unsigned i = 0; i < ref->size(); ++i)
			{
				ref->setPointColor(i, colors->getValue(i));
			}
		}
	}
	else
	{
		ref->unallocateColors();
	}

	ref->setEnabled(wasEnabled);
	ref->setVisible(wasVisible);
	ref->setSelected(wasSelected);
	ref->showColors(colorsWereDisplayed);
	ref->showSF(sfWasDisplayed);
	ref->setCurrentDisplayedScalarField(displayedSFIndex);
	ref->setDisplay(originDisplay);
}

void qVolumeMeasureDlg::CloudBackup::clear()
{
	if (colors)
	{
		colors->release();
		colors = nullptr;
	}

	if (ref)
	{
		if (ownCloud)
		{
			//the dialog takes care of its own clouds!
			delete ref;
		}
		ref = nullptr;
	}
}