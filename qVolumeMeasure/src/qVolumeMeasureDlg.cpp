#include "qVolumeMeasureDlg.h"

qVolumeMeasureDlg::qVolumeMeasureDlg(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_app(app)
	, m_glWindow(nullptr)
	, m_preGlWindow(nullptr)
	, m_cloud(nullptr)
	, m_rasterMesh(nullptr)
	, m_label(cc2DLabel("Ground"))
	, m_pointsIdx(std::vector<unsigned>())
	, currentPath(QDir("./"))
	, gridStep(1.0)
	, gridWidth(0)
	, gridHeight(0)
{
	setupUi(this);

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
		connect(densityLineEdit, &QLineEdit::textChanged, this, [&](const QString& s) {report.density = s.toDouble();});
		connect(m_glWindow, &ccGLWindow::itemPicked, this, &qVolumeMeasureDlg::handlePickedItem);
	}

	{
		okPushButton->setEnabled(false);
		clearPushButton->setEnabled(false);
		calPushButton->setEnabled(false);
		genReportPushButton->setEnabled(false);
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
	{
		if (m_cloud)
			m_cloud->setDisplay(m_preGlWindow);
		if (m_rasterMesh) {
			m_app->addToDB(m_rasterMesh);
			m_rasterMesh->setDisplay_recursive(m_preGlWindow);
		}
			
		m_preGlWindow->redraw();
		m_cloud = nullptr;
		m_rasterMesh = nullptr;
		m_preGlWindow = nullptr;
	}

	if (m_glWindow)
	{
		m_glWindow->getOwnDB()->removeAllChildren();
		if (m_app)
		{
			m_app->destroyGLWindow(m_glWindow);
			m_glWindow = nullptr;
		}
	}
}

void qVolumeMeasureDlg::setCloud(ccPointCloud* cloud)
{
	if (!cloud->getOctree())
	{
		ccProgressDialog pDlg(true, this);
		ccOctree::Shared octree = cloud->computeOctree(&pDlg);
		if (!octree)
		{
			ccLog::Error("Failed to compute octree!");
			return;
		}
		else
		{
			m_app->addToDB(cloud->getOctreeProxy());
		}
	}

	m_cloud = cloud;
	ccRasterGrid::ComputeGridSize(2, cloud->getOwnBB(), gridStep, gridWidth, gridHeight);
	m_preGlWindow = dynamic_cast<ccGLWindow*>(m_cloud->getDisplay());
	m_glWindow->addToOwnDB(cloud);
	m_glWindow->setViewportParameters(m_preGlWindow->getViewportParameters());
	m_glWindow->redraw();


	QMessageBox::information(this
		, QString::fromLocal8Bit("温馨提示")
		, QString::fromLocal8Bit("请从地面选取三个点（左键点击点云即可），帮助软件确定地面位置。选取成功后点击确认按钮保存"));
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

	m_label.clear();
	m_label.setVisible(true);
	m_cloud->setVisible(true);
	m_pointsIdx.clear();
	m_glWindow->redraw();
}

void qVolumeMeasureDlg::onCalPushButtonClick()
{
	calPushButton->setEnabled(false);
	// get ground height
	qreal groundHeight = 0;
	{
		auto zAix = CCVector3(0, 0, 1);
		auto line1 = *(m_cloud->getPoint(m_pointsIdx[0])) - *(m_cloud->getPoint(m_pointsIdx[1]));
		auto line2 = *(m_cloud->getPoint(m_pointsIdx[0])) - *(m_cloud->getPoint(m_pointsIdx[2]));
		auto directon = line1.cross(line2);
		if (directon.angle_rad(zAix) > 3.14) directon *= -1;
		directon.normalize();
		auto rotateMatrix = ccGLMatrix::FromToRotation(directon,zAix);
		m_cloud->rotateGL(rotateMatrix);
		m_cloud->applyGLTransformation_recursive();

		for (auto idx : m_pointsIdx)
		{
			groundHeight += m_cloud->getPoint(idx)->z;
		}
		groundHeight /= 3.0;
	}

	auto pDlg = std::make_unique<ccProgressDialog>(true, this);

	// init ceil raster
	ccRasterGrid ceilRaster, grid;
	{
		auto box = m_cloud->getOwnBB();
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
		if (!ceilRaster.init(gridWidth, gridHeight, gridStep, minConer)
			|| !grid.init(gridWidth, gridHeight, gridStep, minConer))
		{
			dispToConsole(QString::fromLocal8Bit("运行内存不足"));
			return;
		}
		if (!ceilRaster.fillWith(m_cloud, 2,
			ccRasterGrid::ProjectionType::PROJ_AVERAGE_VALUE,
			true,
			ccRasterGrid::INVALID_PROJECTION_TYPE,
			pDlg.get()))
		{
			dispToConsole(QString::fromLocal8Bit("运行内存不足"));
			return;
		}
		ceilRaster.fillEmptyCells(ccRasterGrid::INTERPOLATE);
		dispToConsole(
			QString::fromLocal8Bit("grid: size: %1 x %2 / heights: [%3 ; %4]")
			.arg(ceilRaster.width).arg(ceilRaster.height)
			.arg(ceilRaster.minHeight).arg(ceilRaster.maxHeight)
		);
	}

	{
		if (pDlg)
		{
			pDlg->setMethodTitle(QString::fromLocal8Bit("Volume computation"));
			pDlg->setInfo(QString::fromLocal8Bit("Cells: %1 x %2").arg(grid.width).arg(grid.height));
			pDlg->start();
			pDlg->show();
			QCoreApplication::processEvents();
		}
		CCCoreLib::NormalizedProgress nProgress(pDlg.get(), grid.width * grid.height);

		qreal& volumeAbove = report.volumeAbove = 0;
		qreal& volumeUnder = report.volumeUnder = 0;
		for (size_t i = 0; i < grid.height; i++)
		{
			for (size_t j = 0; j < grid.width; j++)
			{
				auto cell = grid.rows[i][j];
				cell.minHeight = groundHeight;
				cell.maxHeight = ceilRaster.rows[i][j].h;

				if (std::isfinite(cell.minHeight) && std::isfinite(cell.maxHeight))
				{
					cell.h = cell.maxHeight - cell.minHeight;
					cell.nbPoints = 1;
					if (cell.h > 0)
						volumeAbove += cell.h;
					else
						volumeUnder -= cell.h;
				}

				if (pDlg && !nProgress.oneStep())
				{
					dispToConsole(QString::fromLocal8Bit("用户手动取消计算进程，已终止计算"));
					return;
				}
			}
		}
		volumeAbove *= static_cast<qreal>(grid.gridStep * grid.gridStep);
		volumeUnder *= static_cast<qreal>(grid.gridStep * grid.gridStep);

		dispToConsole(QString::fromLocal8Bit(
			"\n====================\n"
			"= 地上体积：%1 立方米\n"
			"= 地下体积：%2 立方米\n"
			"======================\n"
		).arg(volumeAbove, 0, 'f', 3).arg(volumeUnder, 0, 'f', 3));
	}

	{
		auto rasterCloud = ceilRaster.convertToCloud({ ccRasterGrid::PER_CELL_HEIGHT },
			false, // interpolate scalar field
			false, // interpolate colors
			false, // resample input cloud xy
			false, // resample input cloud z
			m_cloud, 2, m_cloud->getOwnBB(),
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
				if (m_rasterMesh) {
					auto idx = m_cloud->getParent()->getChildIndex(m_rasterMesh);
					if (idx > 0) {
						m_cloud->getParent()->removeChild(idx);
						//delete m_rasterMesh;
					}
				}
				m_rasterMesh = new ccMesh(baseMesh, rasterCloud);
				delete baseMesh;
			}
			if (m_rasterMesh)
			{
				if (m_cloud->getParent()) {
					m_cloud->getParent()->addChild(m_rasterMesh);
				}
					
				rasterCloud->setEnabled(false);
				rasterCloud->setVisible(true);
				rasterCloud->setName("vertices");
				m_rasterMesh->addChild(rasterCloud);
				m_rasterMesh->setDisplay_recursive(m_glWindow);
				m_rasterMesh->setName(m_cloud->getName() + ".mesh");
				m_rasterMesh->showSF(true);
				m_rasterMesh->showColors(true);
				m_cloud->setVisible(false);

				m_glWindow->addToOwnDB(m_rasterMesh);
				m_glWindow->redraw();
			}
		}
		else return;


	}

	genReportPushButton->setEnabled(true);
	return;
}

void qVolumeMeasureDlg::onGenReportPushButtonClick()
{
	// get file path and file name
	QString fileName;
	QString imgName;
	QTemporaryFile img("XXXXXX.jpg", this);
	{
		fileName = QFileDialog::getExistingDirectory(this, QString::fromLocal8Bit("保存文件"),
			currentPath.absolutePath(),
			QFileDialog::ShowDirsOnly
			| QFileDialog::DontResolveSymlinks)
			+ QDir::separator()
			+ QString::fromLocal8Bit("测试报告")
			+ QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss") 
			+ ".docx";

		img.open();
		imgName = img.fileName();
	}

	auto pDlg = std::make_unique<ccProgressDialog>(true, this);
	{
		pDlg->setMethodTitle(QString::fromLocal8Bit("生成报告......"));
		pDlg->start();
		pDlg->show();
		QCoreApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nProgress(pDlg.get(), 10);

	// create screen short
	pDlg->setInfo(QString::fromLocal8Bit("生成点云截图 ..."));

	{
		auto view = m_glWindow->getViewportParameters();

		m_glWindow->setView(CC_ISO_VIEW_1);
		m_glWindow->renderToFile(img.fileName());

		m_glWindow->setViewportParameters(view);
	}
	if (!nProgress.oneStep())
	{
		dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
		return;
	}

	// create doc file
	pDlg->setInfo(QString::fromLocal8Bit("生成 Word 文档 ..."));
	{
		QAxWidget word("Word.Application", this, Qt::MSWindowsOwnDC);
		word.setProperty("Visible", false);

		auto documents = word.querySubObject("Documents");
		documents->dynamicCall("Add(QString)", currentPath.absoluteFilePath("template.dotx"));
		auto document = word.querySubObject("ActiveDocument");
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkPointCloudName = document->querySubObject("Bookmarks(QVariant)", "PointCloudName");
		if (!bookmarkPointCloudName->isNull()) {
			bookmarkPointCloudName->dynamicCall("Select(void)");
			bookmarkPointCloudName->querySubObject("Range")
				->setProperty("Text", m_cloud->getName());
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkTime = document->querySubObject("Bookmarks(QVariant)", "GenerateTime");
		if (!bookmarkTime->isNull()) {
			bookmarkTime->dynamicCall("Select(void)");
			bookmarkTime->querySubObject("Range")
				->setProperty("Text", QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss"));
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkVolume = document->querySubObject("Bookmarks(Volume)");
		if (!bookmarkVolume->isNull()) {
			bookmarkVolume->dynamicCall("Select(void)");
			bookmarkVolume->querySubObject("Range")
				->setProperty("Text", QString::number(report.volumeAbove, 'f', 3));
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkDensity = document->querySubObject("Bookmarks(QVariant)", "Density");
		if (!bookmarkDensity->isNull()) {
			bookmarkDensity->dynamicCall("Select(void)");
			bookmarkDensity->querySubObject("Range")
				->setProperty("Text",QString::number(report.density, 'f', 3));
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkWeight = document->querySubObject("Bookmarks(QVariant)", "Weight");
		if (!bookmarkWeight->isNull()) {
			bookmarkWeight->dynamicCall("Select(void)");
			bookmarkWeight->querySubObject("Range")
				->setProperty("Text", QString::number(report.volumeAbove* report.density, 'f', 3));
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkPicture = document->querySubObject("Bookmarks(QVariant)", "Picture");
		if (!bookmarkPicture->isNull()) {
			bookmarkPicture->dynamicCall("Select(void)");
			auto tmp = document->querySubObject("InlineShapes");
			tmp->dynamicCall("AddPicture(const QString&)", imgName);
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		auto bookmarkDetail = document->querySubObject("Bookmarks(QVariant)", "Detail");
		if (!bookmarkDetail->isNull()) {
			bookmarkDetail->dynamicCall("Select(void)");
			bookmarkDetail->querySubObject("Range")
				->setProperty("Text", QString::fromLocal8Bit(
					"地上体积：%1 立方米\n"
					"地下体积：%2 立方米\n"
				).arg(report.volumeAbove, 0, 'f', 3).arg(report.volumeUnder, 0, 'f', 3));
		}
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}

		document->dynamicCall("SaveAs(const QString&)", fileName);
		document->dynamicCall("Close(boolean)", true);
		word.dynamicCall("Quit()");
		if (!nProgress.oneStep())
		{
			dispToConsole(QString::fromLocal8Bit("终止生成报告！"));
			return;
		}
	}

	dispToConsole(QString::fromLocal8Bit("报告已保存到：%1").arg(fileName));
}

void qVolumeMeasureDlg::onOkPushButtonClick()
{
	clearPushButton->setEnabled(false);
	calPushButton->setEnabled(true);
	okPushButton->setEnabled(false);

	m_label.setVisible(false);

	m_glWindow->redraw();

	dispToConsole(QString::fromLocal8Bit("基准地面保存成功！"));
}

void qVolumeMeasureDlg::handlePickedItem(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& p, const CCVector3d& uwv)
{
	if (m_pointsIdx.size() <= 3 && entity)
	{
		m_label.addPickedPoint(static_cast<ccGenericPointCloud*>(entity), itemIdx);
		m_pointsIdx.push_back(itemIdx);
		m_glWindow->redraw();
		clearPushButton->setEnabled(m_pointsIdx.size() > 0);
		okPushButton->setEnabled(m_pointsIdx.size() == 3);

		dispToConsole(QString("Point %4 (%1, %2, %3)")
		.arg(p.x).arg(p.y).arg(p.z)
		.arg(m_pointsIdx.size()));
	}
	else
	{
		dispToConsole("Invalid Point");
	}
}

void qVolumeMeasureDlg::dispToConsole(const QString& msg)
{
	textBrowser->append(msg);
}
