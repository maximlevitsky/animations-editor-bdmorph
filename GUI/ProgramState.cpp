
#include <QMessageBox>
#include <QApplication>
#include <algorithm>

#include "OffScreenRenderer.h"
#include "ProgramState.h"
#include <fstream>

/***********************************************************************************************************/
ProgramState::ProgramState() :
	currentModel(NULL),
	mode(PROGRAM_MODE_NONE),
	currentAnimationTime(0),
	animationRepeat(false),
	videoModel(NULL),
	outlineModel(NULL)
{
	renderSettings.pinMode = false;
	renderSettings.showVF = false;
	renderSettings.showVForig = false;
	renderSettings.showSelection = false;
	renderSettings.wireframeTransparency = 0;
	renderSettings.alpha = 0.5;
	renderSettings.showBDmorphEdge = false;
	renderSettings.showBDmorphOrigMesh = false;
	renderSettings.targetFPS = 30;

	animationTimer = new QTimer(this);
	animationTimer->setSingleShot(true);
	connect_(animationTimer, timeout (), this, onAnimationTimer());
	clearStatusBar();
}

/***********************************************************************************************************/
ProgramState::~ProgramState()
{
	delete videoModel;
	delete outlineModel;
}

/***********************************************************************************************************/
void ProgramState::initialize()
{
	mode = PROGRAM_MODE_NONE;
	emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED);
}

/***********************************************************************************************************/
bool ProgramState::createProject(std::string file)
{
	if (isBusy())
		return false;

	if (file == "")
	{
		unloadAll();
		outlineModel = new OutlineModel();
		currentModel = outlineModel;
		renderSettings.showSelection = false;
		mode = PROGRAM_MODE_OUTLINE;
		renderSettings.textureFile = "";
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED);
	}

	else if (ends_with(file, ".png") || ends_with(file, ".jpg") || ends_with(file, ".jpeg") || ends_with(file, ".bmp") || file == "")
	{
		QPixmap newtexture;
		if (!loadTextureFile(file, newtexture) && file != "") {
			QMessageBox::warning(NULL, "Error", "Can't load picture");
			return false;
		}

		/* Now we can commit to loading*/
		unloadAll();
		outlineModel = new OutlineModel();
		currentModel = outlineModel;
		mode = PROGRAM_MODE_OUTLINE;
		renderSettings.showSelection = false;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED);

		renderSettings.texture.swap(newtexture);
		renderSettings.textureFile = file;
		updateTexture();
	}

	else if (ends_with(file, ".poly"))
	{
		OutlineModel *newOutlineModel = new OutlineModel();
		if (!newOutlineModel->loadFromFile(file))
		{
			QMessageBox::warning(NULL, "Error", "Can't load .poly  file");
			delete newOutlineModel;
			return false;
		}

		unloadAll();

		/* We start project from outline  */
		outlineModel = newOutlineModel;
		currentModel = outlineModel;
		renderSettings.showSelection = false;
		mode = PROGRAM_MODE_OUTLINE;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED);
		tryToGuessLoadTexture(file);
	}

	else if (ends_with(file, ".obj") || ends_with(file, ".off"))
	{
		VideoModel *newVideoModel = new VideoModel();
		if (!newVideoModel->createFromFile(file)) {
			QMessageBox::warning(NULL, "Error", "Can't load mesh");
			delete newVideoModel;
			return false;
		}

		unloadAll();
		videoModel = newVideoModel;

		mode = PROGRAM_MODE_DEFORMATIONS;
		currentModel = videoModel->getKeyframeByIndex(0);
		statusbarState.vertexCount = videoModel->getNumVertices();
		statusbarState.facesCount = videoModel->getNumFaces();
	    renderSettings.wireframeTransparency = 0;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|KEYFRAME_LIST_EDITED|RENDER_SETTINGS_CHANGED);
		tryToGuessLoadTexture(file);
		switchToKeyframe(1);
	}
	else {
		QMessageBox::warning(NULL, "Error", "Unknown file type selected");
		return false;
	}

	return true;
}

/***********************************************************************************************************/
bool ProgramState::createProjectFromOutline(int triangleCount)
{
	if (mode != PROGRAM_MODE_OUTLINE) return false;
	if (!outlineModel) return false;

	VideoModel* newVideoModel  = new VideoModel(outlineModel, triangleCount);
	if (!newVideoModel->keyframesCount())
	{
    	QMessageBox::warning(NULL, "Error","Problem on creating the mesh, check if outline is valid");
    	delete newVideoModel;
    	return false;
	}

	unloadVideoModel();
	videoModel = newVideoModel;
    currentModel = videoModel->getKeyframeByIndex(0);
    statusbarState.vertexCount = currentModel->getNumVertices();
    statusbarState.facesCount = currentModel->getNumFaces();
    mode = PROGRAM_MODE_DEFORMATIONS;

    if (renderSettings.textureFile == "")
    	renderSettings.wireframeTransparency = 0.5;
    else
    	renderSettings.wireframeTransparency = 0;

    emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|RENDER_SETTINGS_CHANGED|KEYFRAME_LIST_EDITED);
    switchToKeyframe(1);
    return true;
}

/***********************************************************************************************************/

bool ProgramState::loadProject(std::string filename)
{
	if (isBusy())
		return false;

    if (ends_with(filename, ".vproject"))
    {
    	VideoModel *newVideoModel = new VideoModel();
    	if (!newVideoModel->loadFromFile(filename)) {
    		QMessageBox::warning(NULL, "Error", "Can't load this video project");
    		delete videoModel;
    		return false;
    	}

    	unloadAll();
		videoModel = newVideoModel;
		mode = PROGRAM_MODE_DEFORMATIONS;
		currentModel = videoModel->getKeyframeByIndex(0);
		statusbarState.vertexCount = videoModel->getNumVertices();
		statusbarState.facesCount = videoModel->getNumFaces();
	    currentAnimationTime = 0;
	    renderSettings.wireframeTransparency = 0;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|KEYFRAME_LIST_EDITED|RENDER_SETTINGS_CHANGED|KEYFRAME_LIST_EDITED);
    } else {
    	QMessageBox::warning(NULL, "Error", "Unknown file type selected");
    	return false;
    }

	tryToGuessLoadTexture(filename);
    return true;
}

/***********************************************************************************************************/
bool ProgramState::loadTexture(std::string newtextureFile)
{
	if (isBusy())
		return false;

	QPixmap newtex;
	if (!loadTextureFile(newtextureFile, newtex) && newtextureFile != "") {
		QMessageBox::warning(NULL, "Error", "Can't load texture file");
		renderSettings.wireframeTransparency = 0.5;
		emit programStateUpdated(RENDER_SETTINGS_CHANGED);
		return false;
	}

	if (newtextureFile == "")
		renderSettings.wireframeTransparency = 0.5;
	else
		renderSettings.wireframeTransparency = 0;

	renderSettings.textureFile = newtextureFile;
	renderSettings.texture.swap(newtex);
	updateTexture();

	emit programStateUpdated(RENDER_SETTINGS_CHANGED);
	return true;
}

/***********************************************************************************************************/

bool ProgramState::loadKeyframe(std::string filename)
{
	if (mode != PROGRAM_MODE_DEFORMATIONS) return false;

	VideoKeyFrame* currentKeyframe = dynamic_cast<VideoKeyFrame*>(currentModel);
	if (!currentKeyframe) return false;

    MeshModel *newModel = new MeshModel();
    if (!newModel->loadFromFile(filename)) {
    	QMessageBox::warning(NULL, "Error", "Can't load mesh");
    	delete newModel;
    	return false;
    }

    if (newModel->getNumFaces() != currentKeyframe->getNumFaces() || newModel->getNumVertices() != currentKeyframe->getNumVertices() ) {
    	QMessageBox::warning(NULL, "Error", "Incompatible model selected, can only load deformations of current mesh");
    	delete newModel;
    	return false;
    }

    currentKeyframe->vertices.swap(newModel->vertices);
    currentKeyframe->updateMeshInfo();
    currentKeyframe->width = videoModel->width;
    currentKeyframe->height = videoModel->height;
    currentKeyframe->moveMesh(videoModel->center);
    delete newModel;
    emit programStateUpdated(MODEL_EDITED);
    return true;
}

/***********************************************************************************************************/

bool ProgramState::saveToFile(std::string filename)
{
	if (isBusy())
		return false;

	bool result;

    if (ends_with(filename, ".obj"))
    	result = currentModel && currentModel->saveToFile(filename);
    else if (ends_with(filename, ".vproject"))
    	result = videoModel && videoModel->saveToFile(filename);
    else if (ends_with(filename, ".poly"))
    	result = outlineModel && outlineModel->saveToFile(filename);
    else
    	result = false;

    if (!result)
    	QMessageBox::warning(NULL, "Error", "Error on file save");
    return result;
}

/***********************************************************************************************************/

bool ProgramState::saveScreenshot(std::string filename)
{
	if (isBusy())
		return false;

	if (!currentModel) return false;
	QImage img;

	OffScreenRenderer *imageRenderer = new OffScreenRenderer(NULL, NULL, 1024,768);
	imageRenderer->setTexture(renderSettings.texture);
	imageRenderer->renderToQImage(currentModel, img,0,0.9);
	delete imageRenderer;

	bool result = img.save(QString::fromStdString(filename));
	if (!result) {
		QMessageBox::warning(NULL, "Error", "Error on screenshot save");
		return false;
	}

	return true;
}

/***********************************************************************************************************/
bool ProgramState::saveVideo(std::string filename)
{
	if (isBusy())
		return false;

	FFMpegEncoder* videoEncoder = new FFMpegEncoder(30);

	if (!videoEncoder->createFile(filename, 1024,768)) {
		delete videoEncoder;
		videoEncoder = NULL;
		QMessageBox::warning(NULL, "Error", "Error initializing video encoding");
		return false;
	}

	switchToKeyframe(0);
	mode = PROGRAM_MODE_BUSY;
	maxAnimationTime = videoModel->getTotalTime();
	statusbarState.statusbarMessage = "Creating video...";
	emit programStateUpdated(MODE_CHANGED|STATUSBAR_UPDATED);

	uint8_t* imagebuffer = (uint8_t*)malloc(1024*768*4);

	OffScreenRenderer *imageRenderer = new OffScreenRenderer(NULL, NULL, 1024,768);
	imageRenderer->setTexture(renderSettings.texture);
	imageRenderer->setupTransform(videoModel,true,0,0.9);

	for (double position = 0; position < maxAnimationTime ; position += videoEncoder->getFrameTimeMsec())
	{
		double duration;
		MeshModel* pFrame = videoModel->interpolateFrame(position, &duration);
		statusbarState.FPS = 1000.0 / duration;

		TimeMeasurment t;
		imageRenderer->renderToBufferBGRA(pFrame,imagebuffer);
		printf("Took %f msec to render the image\n", t.measure_msec());


		if (!videoEncoder->encodeImageBGRA(imagebuffer)) {
			QMessageBox::critical(NULL, "Error", "Failure during encoding");
			delete videoEncoder;
			videoEncoder = NULL;
			return false;

		}

		setProgress((int)((position * 100) /maxAnimationTime));
		QApplication::processEvents();
	}

	videoEncoder->close();
	delete videoEncoder;
	delete imageRenderer;
	free(imagebuffer);

	mode = PROGRAM_MODE_DEFORMATIONS;
	statusbarState.progressValue = 0;
	statusbarState.statusbarMessage.clear();
	emit programStateUpdated(MODE_CHANGED|STATUSBAR_UPDATED);
	return true;
}


/***********************************************************************************************************/
void ProgramState::editOutline()
{
	if (isBusy()) return;
	if (mode == PROGRAM_MODE_OUTLINE) return;

	 if (QMessageBox::question(NULL,
		"Warning","This will make you loose all changes in the animation project",
		QMessageBox::Cancel|QMessageBox::Ok,QMessageBox::Cancel) != QMessageBox::Ok )
		return;

	if (!outlineModel && currentModel)
		outlineModel = new OutlineModel(currentModel);

	clearStatusBar();
    if (outlineModel)
    {
    	unloadVideoModel();
    	currentModel = outlineModel;
    	mode = PROGRAM_MODE_OUTLINE;
    	updateTexture();
    	emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED);
	}
}

/***********************************************************************************************************/
void ProgramState::switchToKeyframe(int newIndex)
{
	VideoKeyFrame* newFrame = videoModel->getKeyframeByIndex(newIndex);
	if (newFrame != currentModel)
	{
		if (mode != PROGRAM_MODE_DEFORMATIONS && mode != PROGRAM_MODE_ANIMATION_RUNNING)
		{
			mode = PROGRAM_MODE_DEFORMATIONS;
			emit programStateUpdated(MODE_CHANGED);
		}

		currentModel = newFrame;
		currentAnimationTime = videoModel->getKeyFrameTimeMsec(newFrame);
		emit programStateUpdated(CURRENT_MODEL_CHANGED);
	}
}
/***********************************************************************************************************/
void ProgramState::cloneKeyframe(int index)
{
	VideoKeyFrame* parent;

	if (mode != PROGRAM_MODE_DEFORMATIONS) return;

	if (index == -1)
		parent = dynamic_cast<VideoKeyFrame*>(currentModel);
	else
		parent  = videoModel->getKeyframeByIndex(index);

	if (!parent) return;

	currentModel = videoModel->forkFrame(parent);
	emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED);
}

/***********************************************************************************************************/

void ProgramState::createKeyframeFromPFrame()
{
	if (mode != PROGRAM_MODE_ANIMATION_PAUSED) return;

	if (currentModel == videoModel->pFrame && videoModel->pFrame && mode == PROGRAM_MODE_ANIMATION_PAUSED)
	{
		currentModel = videoModel->insertFrame(currentAnimationTime, videoModel->pFrame);
		mode = PROGRAM_MODE_DEFORMATIONS;
		emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED|MODE_CHANGED);
	}

}

/***********************************************************************************************************/
void ProgramState::deleteKeyFrame(int index)
{
	VideoKeyFrame* victim;
	if (mode != PROGRAM_MODE_DEFORMATIONS) return;

	/* Can't delete last keyframe */
	if (videoModel->keyframesCount() <= 1) return;

	if (index == -1)
		victim = dynamic_cast<VideoKeyFrame*>(currentModel);
	else
		victim  = videoModel->getKeyframeByIndex(index);

	if (!victim) return;

	if (victim == currentModel)
	{
		/* Switch away from current model */
		currentModel = NULL;
		emit programStateUpdated(CURRENT_MODEL_CHANGED);
		currentModel = videoModel->deleteFrame(victim);
		emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED);
	}
}
/***********************************************************************************************************/
int ProgramState::getCurrentKeyframeId()
{
	if (!videoModel) return -1;
	VideoKeyFrame* currentKeyframe;

	if (currentModel == videoModel->pFrame && videoModel->pFrame)
		currentKeyframe = dynamic_cast<VideoKeyFrame*>(videoModel->pFrame->modela);
	else
		currentKeyframe = dynamic_cast<VideoKeyFrame*>(currentModel);

	if (!currentKeyframe) return -1;
	return videoModel->getKeyFrameIndex(currentKeyframe);
}

/***********************************************************************************************************/
int ProgramState::getKeyframeTime(int id)
{
	if (!videoModel) return -1;
	VideoKeyFrame* keyframe = videoModel->getKeyframeByIndex(id);
	return videoModel->getKeyFrameTimeMsec(keyframe);
}

/***********************************************************************************************************/
void ProgramState::setKeyframeTime(int id, int newTime)
{
	if (!videoModel) return;
	newTime = std::max(1,newTime);
	VideoKeyFrame* keyframe = videoModel->getKeyframeByIndex(id);
	keyframe->duration = newTime;
	emit programStateUpdated(KEYFRAME_LIST_EDITED);
}

/***********************************************************************************************************/
int ProgramState::getKeyframeCount()
{
	if (!videoModel) return -1;
	return videoModel->keyframesCount();
}

/***********************************************************************************************************/
void ProgramState::informModelEdited()
{
	emit programStateUpdated(MODEL_EDITED);
}

/***********************************************************************************************************/
void ProgramState::setRenderSettings(const RenderSettings &newsettings)
{
	renderSettings = newsettings;

	if (newsettings.showSelection == false) {
		statusbarState.selectedFace = -1;
		statusbarState.selectedVertex = -1;
	}

	emit programStateUpdated(RENDER_SETTINGS_CHANGED);
}
/***********************************************************************************************************/

void ProgramState::setProgress(int value)
{
	if (value > 100) value = 100;
	if (value < 0) value = 0;
	statusbarState.progressValue = value;
	emit programStateUpdated(STATUSBAR_UPDATED);
}

/***********************************************************************************************************/
enum ProgramState::PROGRAM_MODE ProgramState::getCurrentMode()
{
	return mode;
}

void ProgramState::setCurrentMode(enum ProgramState::PROGRAM_MODE newmode)
{
	mode = newmode;
	emit programStateUpdated(MODE_CHANGED);
}

/***********************************************************************************************************/
void ProgramState::startStopAnimations()
{
	if (!videoModel) return;

	if (mode != PROGRAM_MODE_ANIMATION_RUNNING)
	{
		mode = PROGRAM_MODE_ANIMATION_RUNNING;
		maxAnimationTime = videoModel->getTotalTime();
		emit programStateUpdated(MODE_CHANGED);
		animationReferenceTimer.start();
		animationTimer->start(0);
	} else
	{
		animationTimer->stop();
		mode = PROGRAM_MODE_ANIMATION_PAUSED;
		emit programStateUpdated(MODE_CHANGED);
	}
}

/***********************************************************************************************************/

void ProgramState::resetTransform()
{
	emit programStateUpdated(TRANSFORM_RESET);
}

/***********************************************************************************************************/

void ProgramState::onAnimationTimer()
{
	currentAnimationTime += animationReferenceTimer.nsecsElapsed() / (1000*1000);

	if (currentAnimationTime >= maxAnimationTime) {
		if (animationRepeat)
			currentAnimationTime = 0;
		else {
			startStopAnimations();
			return;
		}
	}

	animationReferenceTimer.start();
	interpolateFrame(currentAnimationTime);
	int sleepTime = (1000/renderSettings.targetFPS) - animationReferenceTimer.nsecsElapsed() / (1000*1000);
	printf("Sleep time %d\n", sleepTime);
	if (sleepTime < 0) sleepTime = 0;
	animationTimer->start(sleepTime);
}

/***********************************************************************************************************/

void ProgramState::interpolateFrame(int time)
{
	if (!videoModel) return;
	VideoKeyFrame* prevFrame = videoModel->getLastKeyframeBeforeTime(time);
	if (!prevFrame) return;


	double duration;
	currentModel = videoModel->interpolateFrame(time, &duration);
	currentAnimationTime = time;
	emit programStateUpdated(ANIMATION_POSITION_CHANGED);

	statusbarState.FPS = 1000.0/duration;
	emit programStateUpdated(STATUSBAR_UPDATED);
}

/***********************************************************************************************************/

void ProgramState::setAnimationPosition(int newPosition)
{
	interpolateFrame(newPosition);

	if (mode != PROGRAM_MODE_ANIMATION_PAUSED && mode != PROGRAM_MODE_ANIMATION_RUNNING)
	{
		mode = PROGRAM_MODE_ANIMATION_PAUSED;
		emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED);
		emit programStateUpdated(CURRENT_MODEL_CHANGED);
	}
}


/***********************************************************************************************************/

void ProgramState::setAnimationRepeat(bool enabled)
{
	animationRepeat = enabled;
}

/***********************************************************************************************************/

void ProgramState::clearStatusBar()
{
	statusbarState.FPS=-1;
	statusbarState.vertexCount=0;
	statusbarState.facesCount=0;
	statusbarState.progressValue=0;
	statusbarState.selectedVertex=-1;
	statusbarState.selectedFace=-1;
	emit programStateUpdated(STATUSBAR_UPDATED);
}
/***********************************************************************************************************/

void ProgramState::unloadAll()
{
	clearStatusBar();
	mode = PROGRAM_MODE_NONE;
	currentModel = NULL;
	emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED);

	/* Delete everything */
	delete videoModel;
	videoModel = NULL;
	delete outlineModel;
	outlineModel = NULL;
	loadTextureFile("", renderSettings.texture);
	renderSettings.textureFile = "";
	updateTexture();
}
/***********************************************************************************************************/

void ProgramState::unloadVideoModel()
{
	clearStatusBar();
	mode = PROGRAM_MODE_NONE;
	currentModel = NULL;
	emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED);
	delete videoModel;
	videoModel = NULL;
}
/***********************************************************************************************************/

void ProgramState::unloadOutlineModel()
{
	clearStatusBar();
	mode = PROGRAM_MODE_NONE;
	currentModel = NULL;
	emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED);
	delete outlineModel;
	outlineModel = NULL;
}

/***********************************************************************************************************/

bool ProgramState::loadTextureFile(std::string file, QPixmap &out)
{
	/* If we are asked to remove the texture or fail to load it, use default blue texture */
	if (file == "" || out.load(QString::fromStdString(file)) == false)
	{
		out = QPixmap(16,16);
		out.fill(QColor(200,200,255));
		return false;
	}
	return true;
}

/***********************************************************************************************************/

void ProgramState::updateTexture()
{
	if (mode == PROGRAM_MODE_OUTLINE)
	{
		int x = renderSettings.texture.width();
		int y = renderSettings.texture.height();

		if (y > x)
			outlineModel->setScale((double)x/y,1);
		else
			outlineModel->setScale(1,(double)y/x);
	}

	emit programStateUpdated(TEXTURE_CHANGED);
}

/***********************************************************************************************************/

void ProgramState::updateGUI()
{
	emit programStateUpdated(PANEL_VISIBLITIY_CHANGED);
}

/***********************************************************************************************************/

void ProgramState::tryToGuessLoadTexture(std::string file)
{
	/* now try to load texture with same name */
	int lastindex = file.find_last_of(".");
	std::string rawname = file.substr(0, lastindex);
	rawname += ".png";

	QPixmap tex;

	if (loadTextureFile(rawname, tex)) {
		renderSettings.textureFile = rawname;
		renderSettings.texture.swap(tex);
		updateTexture();
		renderSettings.wireframeTransparency = 0;
	} else
		renderSettings.wireframeTransparency = 0.5;

	emit programStateUpdated(RENDER_SETTINGS_CHANGED);
}

/***********************************************************************************************************/

void ProgramState::autoCreateOutline()
{
	if (!outlineModel || mode != PROGRAM_MODE_OUTLINE)
		return;

	currentModel = NULL;
	emit programStateUpdated(CURRENT_MODEL_CHANGED);

	delete outlineModel;
	outlineModel = new OutlineModel(renderSettings.texture.toImage());
	updateTexture();

	currentModel = outlineModel;
	emit programStateUpdated(MODEL_EDITED|TRANSFORM_RESET|CURRENT_MODEL_CHANGED);
}

/***********************************************************************************************************/
void ProgramState::setSelectedVertexAndFace(int selectedVertex,
		int selectedFace)
{
	statusbarState.selectedVertex = selectedVertex;
	statusbarState.selectedFace = selectedFace;
	emit programStateUpdated(STATUSBAR_UPDATED);
}

/***********************************************************************************************************/

void ProgramState::setFPS(double newFPS) {
	statusbarState.FPS = newFPS;
	emit programStateUpdated(STATUSBAR_UPDATED);

}

/***********************************************************************************************************/

void ProgramState::showStatusBarMessage(const QString& message)
{
	statusbarState.statusbarMessage = message;
	emit programStateUpdated(STATUSBAR_UPDATED);
}

/***********************************************************************************************************/

void ProgramState::runLog(std::string filename)
{
	KVFModel *kvfModel = dynamic_cast<KVFModel *>(currentModel);
	if (!kvfModel)
		return;

	std::ifstream infile(filename);
    printf("STARTING log replay\n");
    TimeMeasurment t;

    int numSteps;
    infile >> numSteps;
    showStatusBarMessage("Replaying log...");

    ProgramState::PROGRAM_MODE savedmode = getCurrentMode();
    setCurrentMode(ProgramState::PROGRAM_MODE_BUSY);

    for (int step = 0; step < numSteps; step++)
    {

    	kvfModel->historyLoadFromFile(infile);
    	informModelEdited();
    	setFPS(1000.0 / (kvfModel->lastVFCalcTime+kvfModel->lastVFApplyTime));
    	setProgress((step*100)/numSteps);
		QApplication::processEvents();

    }

    showStatusBarMessage("");
    setProgress(0);
    setCurrentMode(savedmode);

    printf("DONE WITH log replay (took %f msec)\n", t.measure_msec());
    kvfModel->historySnapshot();
}

/***********************************************************************************************************/
void ProgramState::saveLog(std::string filename)
{
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(currentModel);
	if (!kvfModel) return;

    std::ofstream outfile(filename);
    kvfModel->historySaveToFile(outfile);
}

/***********************************************************************************************************/

