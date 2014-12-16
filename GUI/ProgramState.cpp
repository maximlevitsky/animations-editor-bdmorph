#include "ProgramState.h"
#include <QMessageBox>
#include <algorithm>
#include <QApplication>

/***********************************************************************************************************/
ProgramState::ProgramState() :
	showVF(false),
	showVForig(false),
	showSelection(false),
	wireframeTransparency(0),
	multitouchMode(false),
	pinMode(false),
	progressValue(-1),
	selectedFace(-1),
	selectedVertex(-1),
	currentModel(NULL),
	textureRef(0),
	mode(PROGRAM_MODE_NONE),
	thumbnailRenderer(NULL),
	currentAnimationTime(0),
	animationRepeat(false),
	videoModel(NULL),
	outlineModel(NULL),
	showBDmorphEdge(false),
	showBDmorphOrigMesh(false),
	targetFPS(30),
	videoEncoder(NULL)
{
	animationTimer = new QTimer(this);
	animationTimer->setSingleShot(true);
	connect_(animationTimer, timeout (), this, onAnimationTimer());
	imagebuffer = (uint8_t*)malloc(1024*768*4);
}

/***********************************************************************************************************/
ProgramState::~ProgramState()
{
	delete videoModel;
	delete outlineModel;
	free(imagebuffer);
}
/***********************************************************************************************************/
bool ProgramState::createProject(std::string file)
{
	if (file == "")
	{
		unloadAll();
		outlineModel = new OutlineModel();
		currentModel = outlineModel;
		mode = PROGRAM_MODE_OUTLINE;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
	}

	else if (ends_with(file, ".png") || ends_with(file, ".jpg") || file == "")
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
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);

		texture.swap(newtexture);
		textureFile = file;
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
		mode = PROGRAM_MODE_OUTLINE;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
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
		currentModel = videoModel->getKeyframeByIndex(1);
		vertexCount = videoModel->numVertices;
		facesCount = videoModel->numFaces;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED,NULL);
	}
	else {
		QMessageBox::warning(NULL, "Error", "Unknown file type selected");
		return false;
	}

	tryToGuessLoadTexture(file);
	return true;
}

/***********************************************************************************************************/
bool ProgramState::loadProject(std::string filename)
{
    if (ends_with(filename, ".vobj"))
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
		vertexCount = videoModel->numVertices;
		facesCount = videoModel->numFaces;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED, NULL);
    } else {
    	QMessageBox::warning(NULL, "Error", "Unknown file type selected");
    	return false;
    }

	tryToGuessLoadTexture(filename);
    return true;
}

/***********************************************************************************************************/
bool ProgramState::saveToFile(std::string filename)
{
	bool result;

    if (ends_with(filename, ".obj"))
    	result = currentModel && currentModel->saveToFile(filename);
    else if (ends_with(filename, ".vobj"))
    	result = videoModel && videoModel->saveToFile(filename);
    else if (ends_with(filename, ".poly"))
    	result = outlineModel && outlineModel->saveToFile(filename);
    else
    	result = false;

    if (!result)
    	QMessageBox::warning(NULL, "Error", "Error on file save");
    return result;
}

bool ProgramState::saveScreenshot(std::string filename)
{
	if (!currentModel) return false;
	QImage img;
	imageRenderer->renderToImage(currentModel, img,0,0.9);

	bool result = img.save(QString::fromStdString(filename));
	if (!result) {
		QMessageBox::warning(NULL, "Error", "Error on screenshot save");
		return false;
	}
	return true;
}

/***********************************************************************************************************/
bool ProgramState::createVideo(QString file)
{
	videoEncoder = new QVideoEncoder();
	if (!videoEncoder->createFile(file, 1024,768)) {
		delete videoEncoder;
		videoEncoder = NULL;
		QMessageBox::warning(NULL, "Error", "Error initializing video encoding");
		return false;
	}

	switchToKeyframe(0);
	mode = PROGRAM_MODE_BUSY;
	emit programStateUpdated(MODE_CHANGED, NULL);

	maxAnimationTime = videoModel->getTotalTime();
	imageRenderer->setupTransform(videoModel,true,0,0.7);
	videoEncodingTime = 0;

	statusbarMessage = "Creating video...";

	for (; videoEncodingTime < maxAnimationTime ; videoEncodingTime += 16)
	{
		TimeMeasurment t;
		MeshModel* pFrame = videoModel->interpolateFrame(videoEncodingTime, &FPS);
		printf("Took %f msec to BDMORPH the image\n", t.measure_msec());
		imageRenderer->renderBGRA(pFrame,imagebuffer);
		printf("Took %f msec to render the image\n", t.measure_msec());


		if (!videoEncoder->encodeImage(imagebuffer, videoEncodingTime)) {
			QMessageBox::critical(NULL, "Error", "Failure during encoding");
			delete videoEncoder;
			videoEncoder = NULL;
			return false;

		}
		progressValue = (videoEncodingTime * 100) /maxAnimationTime;
		updateStatistics();
		QApplication::processEvents();
	}

	videoEncoder->close();
	mode = PROGRAM_MODE_DEFORMATIONS;
	progressValue = 0;
	statusbarMessage.clear();
	emit programStateUpdated(MODE_CHANGED|STATUSBAR_UPDATED, NULL);
	return true;
}

/***********************************************************************************************************/

bool ProgramState::setTexture(std::string newtextureFile)
{
	QPixmap newtex;
	if (!loadTextureFile(newtextureFile, newtex) && newtextureFile != "") {
		QMessageBox::warning(NULL, "Error", "Can't load texture file");
		return false;
	}

	textureFile = newtextureFile;
	texture.swap(newtex);
	updateTexture();
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

    if (newModel->numFaces != currentKeyframe->numFaces || newModel->numVertices != currentKeyframe->numVertices ) {
    	QMessageBox::warning(NULL, "Error", "Incomatable model selected, can only load deformations of current mesh");
    	delete newModel;
    	return false;
    }

    currentKeyframe->vertices.swap(newModel->vertices);
    currentKeyframe->updateMeshInfo();
    currentKeyframe->minPoint = videoModel->minPoint;
    currentKeyframe->maxPoint = videoModel->maxPoint;
    currentKeyframe->moveMesh(videoModel->center);
    delete newModel;
    emit programStateUpdated(KEYFRAME_EDITED, NULL);
    return true;
}

/***********************************************************************************************************/
bool ProgramState::createMeshFromOutline(int triangleCount)
{
	if (mode != PROGRAM_MODE_OUTLINE) return false;
	if (!outlineModel) return false;

	VideoModel* newVideoModel  = new VideoModel();


	if (!newVideoModel->createFromOutline(outlineModel, triangleCount))
	{
    	QMessageBox::warning(NULL, "Error","Problem on creating the mesh, check if outline is valid");
    	delete newVideoModel;
    	return false;
	}

	unloadVideoModel();
	videoModel = newVideoModel;
    currentModel = videoModel->getKeyframeByIndex(1);
    vertexCount = currentModel->numVertices;
    facesCount = currentModel->numFaces;
    mode = PROGRAM_MODE_DEFORMATIONS;
    wireframeTransparency = 0.5;
    emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|EDIT_SETTINGS_CHANGED,NULL);
    return true;
}
/***********************************************************************************************************/
void ProgramState::editOutline()
{
	if (mode == PROGRAM_MODE_OUTLINE) return;

	 if (QMessageBox::question(NULL,
		"Warning","This will make you loose all changes in animation project",
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
    	emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
	}
}

/***********************************************************************************************************/
void ProgramState::switchToKeyframe(int newIndex)
{
	VideoKeyFrame* newFrame = videoModel->getKeyframeByIndex(newIndex);
	if (newFrame != currentModel)
	{
		currentModel = newFrame;
		mode = PROGRAM_MODE_DEFORMATIONS;
		currentAnimationTime = videoModel->getKeyFrameTimeMsec(newFrame);
		emit programStateUpdated(CURRENT_MODEL_CHANGED|MODE_CHANGED,NULL);
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
	emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED,NULL);
}

void ProgramState::createKeyframeFromPFrame()
{
	if (mode != PROGRAM_MODE_ANIMATION) return;

	if (currentModel == videoModel->pFrame && videoModel->pFrame && mode == PROGRAM_MODE_ANIMATION)
	{
		VideoKeyFrame* currentKeyframe = dynamic_cast<VideoKeyFrame*>(videoModel->pFrame->modela);
		int keyframeStartTime =videoModel->getKeyFrameTimeMsec(currentKeyframe);
		int newDuration = currentAnimationTime - keyframeStartTime;
		if (newDuration < 1) newDuration = 1;
		int nextDuration = currentKeyframe->duration - newDuration;
		if (nextDuration < 1) nextDuration = 1;

		currentKeyframe->duration = newDuration;
		currentModel = videoModel->forkFrame(currentKeyframe, currentModel);
		currentKeyframe = dynamic_cast<VideoKeyFrame*>(currentModel);
		currentKeyframe->duration = nextDuration;

		mode = PROGRAM_MODE_DEFORMATIONS;
		emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED|MODE_CHANGED|KEYFRAME_EDITED,NULL);
	}

}


/***********************************************************************************************************/
void ProgramState::deleteKeyFrame(int index)
{
	VideoKeyFrame* victim;
	if (mode != PROGRAM_MODE_DEFORMATIONS) return;

	/* Can't delete last keyframe */
	if (videoModel->count() <= 1) return;

	if (index == -1)
		victim = dynamic_cast<VideoKeyFrame*>(currentModel);
	else
		victim  = videoModel->getKeyframeByIndex(index);

	if (!victim) return;

	if (victim == currentModel)
	{
		/* Switch away from current model */
		currentModel = NULL;
		emit programStateUpdated(CURRENT_MODEL_CHANGED,NULL);
		currentModel = videoModel->deleteFrame(victim);
		emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED,NULL);
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
	emit programStateUpdated(KEYFRAME_EDITED, NULL);
}

/***********************************************************************************************************/
int ProgramState::getKeyframeCount()
{
	if (!videoModel) return -1;
	return videoModel->count();
}

/***********************************************************************************************************/
void ProgramState::onUpdateModel()
{
	emit programStateUpdated(KEYFRAME_EDITED,NULL);
}
/***********************************************************************************************************/
void ProgramState::updateStatistics()
{
	emit programStateUpdated(STATUSBAR_UPDATED,NULL);
}
/***********************************************************************************************************/
void ProgramState::updateSettings()
{
	emit programStateUpdated(EDIT_SETTINGS_CHANGED,NULL);
}
/***********************************************************************************************************/

void ProgramState::setProgress(int value)
{
	if (value > 100) value = 100;
	if (value < 0) value = 0;
	progressValue = value;
	emit programStateUpdated(STATUSBAR_UPDATED,NULL);
}

/***********************************************************************************************************/
enum ProgramState::PROGRAM_MODE ProgramState::getCurrentMode()
{
	return mode;
}

/***********************************************************************************************************/
void ProgramState::startAnimations(int time)
{
	if (!videoModel) return;
	mode = PROGRAM_MODE_BUSY;
	currentAnimationTime = time;
	maxAnimationTime = videoModel->getTotalTime();
	animationReferenceTimer.start();

	/* TODO: kick animation thread here*/
	emit programStateUpdated(MODE_CHANGED,NULL);
	animationTimer->start(0);
}

/***********************************************************************************************************/
void ProgramState::stopAnimations()
{
	animationTimer->stop();
	mode = PROGRAM_MODE_ANIMATION;
	emit programStateUpdated(MODE_CHANGED,NULL);
}


/***********************************************************************************************************/
void ProgramState::resetTransform()
{
	emit programStateUpdated(TRANSFORM_RESET,NULL);
}

void ProgramState::onAnimationTimer()
{
	currentAnimationTime += animationReferenceTimer.nsecsElapsed() / (1000*1000);

	if (currentAnimationTime >= maxAnimationTime) {
		if (animationRepeat)
			currentAnimationTime = 0;
		else {
			stopAnimations();
			return;
		}
	}

	animationReferenceTimer.start();
	interpolateFrame(currentAnimationTime);
	int sleepTime = (1000/targetFPS) - animationReferenceTimer.nsecsElapsed() / (1000*1000);
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
	MeshModel* pFrame = videoModel->interpolateFrame(time, &duration);

	if (pFrame != currentModel)
	{
		currentModel = pFrame;
		emit programStateUpdated(CURRENT_MODEL_CHANGED, NULL);

	}

	currentAnimationTime = time;
	emit programStateUpdated(ANIMATION_STEPPED, NULL);

	if (mode != PROGRAM_MODE_ANIMATION && mode != PROGRAM_MODE_BUSY)
	{
		mode = PROGRAM_MODE_ANIMATION;
		emit programStateUpdated(MODE_CHANGED, NULL);
	}


	FPS = duration;
	emit programStateUpdated(STATUSBAR_UPDATED, NULL);
}

/***********************************************************************************************************/
void ProgramState::setAnimationRepeat(bool enabled)
{
	animationRepeat = enabled;
	/* TODO: tell about repeat to animation thread */
}

/***********************************************************************************************************/

void ProgramState::clearStatusBar()
{
	FPS=0;
	vertexCount=0;
	facesCount=0;
	progressValue=0;
	selectedVertex=-1;
	selectedFace=-1;
	emit programStateUpdated(STATUSBAR_UPDATED, NULL);
}
/***********************************************************************************************************/


void ProgramState::unloadAll()
{
	clearStatusBar();
	mode = PROGRAM_MODE_NONE;
	currentModel = NULL;
	emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);

	/* Delete everything */
	delete videoModel;
	videoModel = NULL;
	delete outlineModel;
	outlineModel = NULL;
	loadTextureFile("", texture);
	updateTexture();
}
/***********************************************************************************************************/

void ProgramState::unloadVideoModel()
{
	clearStatusBar();
	mode = PROGRAM_MODE_NONE;
	currentModel = NULL;
	emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
	delete videoModel;
	videoModel = NULL;
}
/***********************************************************************************************************/

void ProgramState::unloadOutlineModel()
{
	clearStatusBar();
	mode = PROGRAM_MODE_NONE;
	currentModel = NULL;
	emit programStateUpdated(MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
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
	if (mode == PROGRAM_MODE_OUTLINE) {
		int x = texture.width();
		int y = texture.height();

		if (y > x)
			outlineModel->setScale((double)x/y,1);
		else
			outlineModel->setScale(1,(double)y/x);
	}


	/* Bind it and in darkness grind it....*/
	textureRef = thumbnailRenderer->bindTexture(texture,GL_TEXTURE_2D);
	thumbnailRenderer->makeCurrent();
	glBindTexture(GL_TEXTURE_2D, textureRef);
	imageRenderer->makeCurrent();
	glBindTexture(GL_TEXTURE_2D, textureRef);
	emit programStateUpdated(TEXTURE_CHANGED, NULL);
}

/***********************************************************************************************************/

void ProgramState::updateGUI()
{
	emit programStateUpdated(PANEL_VISIBLITIY_CHANGED, NULL);
}

/***********************************************************************************************************/


void ProgramState::tryToGuessLoadTexture(std::string file)
{
	/* now try to load texture with same name */
	int lastindex = file.find_last_of(".");
	std::string rawname = file.substr(0, lastindex);
	rawname += ".png";
	if (loadTextureFile(rawname, texture)) {
		textureFile = rawname;
		updateTexture();
	}
}

/***********************************************************************************************************/


