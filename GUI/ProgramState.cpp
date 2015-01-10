
#include <QMessageBox>
#include <QApplication>
#include <algorithm>

#include "OffScreenRenderer.h"
#include "ProgramState.h"

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
	mode(PROGRAM_MODE_NONE),
	currentAnimationTime(0),
	animationRepeat(false),
	videoModel(NULL),
	outlineModel(NULL),
	showBDmorphEdge(false),
	showBDmorphOrigMesh(false),
	targetFPS(30)
{
	animationTimer = new QTimer(this);
	animationTimer->setSingleShot(true);
	connect_(animationTimer, timeout (), this, onAnimationTimer());
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
	emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
}

/***********************************************************************************************************/
bool ProgramState::createProject(std::string file)
{
	if (mode == PROGRAM_MODE_BUSY)
		return false;

	if (file == "")
	{
		unloadAll();
		outlineModel = new OutlineModel();
		currentModel = outlineModel;
		mode = PROGRAM_MODE_OUTLINE;
		textureFile = "";
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
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
		currentModel = videoModel->getKeyframeByIndex(1);
		vertexCount = videoModel->getNumVertices();
		facesCount = videoModel->getNumFaces();
	    currentAnimationTime = videoModel->getKeyframeByIndex(0)->duration;
	    wireframeTransparency = 0;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|KEYFRAME_LIST_EDITED|EDIT_SETTINGS_CHANGED,NULL);
		tryToGuessLoadTexture(file);
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
    vertexCount = currentModel->getNumVertices();
    facesCount = currentModel->getNumFaces();
    mode = PROGRAM_MODE_DEFORMATIONS;

    if (textureFile == "")
    	wireframeTransparency = 0.5;
    else
    	wireframeTransparency = 0;

    currentAnimationTime = videoModel->getKeyframeByIndex(0)->duration;
    emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|EDIT_SETTINGS_CHANGED|KEYFRAME_LIST_EDITED,NULL);
    return true;
}

/***********************************************************************************************************/

bool ProgramState::loadProject(std::string filename)
{
	if (mode == PROGRAM_MODE_BUSY)
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
		vertexCount = videoModel->getNumVertices();
		facesCount = videoModel->getNumFaces();
	    currentAnimationTime = 0;
	    wireframeTransparency = 0;
		emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED|STATUSBAR_UPDATED|KEYFRAME_LIST_EDITED|EDIT_SETTINGS_CHANGED, NULL);
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
	if (mode == PROGRAM_MODE_BUSY)
		return false;

	QPixmap newtex;
	if (!loadTextureFile(newtextureFile, newtex) && newtextureFile != "") {
		QMessageBox::warning(NULL, "Error", "Can't load texture file");
		wireframeTransparency = 0.5;
		emit programStateUpdated(EDIT_SETTINGS_CHANGED,NULL);
		return false;
	}

	if (newtextureFile == "")
		wireframeTransparency = 0.5;
	else
		wireframeTransparency = 0;

	textureFile = newtextureFile;
	texture.swap(newtex);
	updateTexture();

	emit programStateUpdated(EDIT_SETTINGS_CHANGED,NULL);
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
    emit programStateUpdated(KEYFRAME_EDITED, NULL);
    return true;
}

/***********************************************************************************************************/

bool ProgramState::saveToFile(std::string filename)
{
	if (mode == PROGRAM_MODE_BUSY)
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
	if (mode == PROGRAM_MODE_BUSY)
		return false;

	if (!currentModel) return false;
	QImage img;

	OffScreenRenderer *imageRenderer = new OffScreenRenderer(NULL, NULL, 1024,768);
	imageRenderer->setTexture(texture);
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
	if (mode == PROGRAM_MODE_BUSY)
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
	statusbarMessage = "Creating video...";
	emit programStateUpdated(MODE_CHANGED|STATUSBAR_UPDATED, NULL);

	uint8_t* imagebuffer = (uint8_t*)malloc(1024*768*4);

	OffScreenRenderer *imageRenderer = new OffScreenRenderer(NULL, NULL, 1024,768);
	imageRenderer->setTexture(texture);
	imageRenderer->setupTransform(videoModel,true,0,0.9);

	for (double position = 0; position < maxAnimationTime ; position += videoEncoder->getFrameTimeMsec())
	{
		double duration;
		MeshModel* pFrame = videoModel->interpolateFrame(position, &duration);
		FPS = 1000.0 / duration;

		TimeMeasurment t;
		imageRenderer->renderToBufferBGRA(pFrame,imagebuffer);
		printf("Took %f msec to render the image\n", t.measure_msec());


		if (!videoEncoder->encodeImageBGRA(imagebuffer)) {
			QMessageBox::critical(NULL, "Error", "Failure during encoding");
			delete videoEncoder;
			videoEncoder = NULL;
			return false;

		}
		progressValue = (int)((position * 100) /maxAnimationTime);
		updateStatistics();
		QApplication::processEvents();
	}

	videoEncoder->close();
	delete videoEncoder;
	delete imageRenderer;
	free(imagebuffer);

	mode = PROGRAM_MODE_DEFORMATIONS;
	progressValue = 0;
	statusbarMessage.clear();
	emit programStateUpdated(MODE_CHANGED|STATUSBAR_UPDATED, NULL);
	return true;
}


/***********************************************************************************************************/
void ProgramState::editOutline()
{
	if (mode == PROGRAM_MODE_BUSY) return;
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
    	updateTexture();
    	emit programStateUpdated(TRANSFORM_RESET|MODE_CHANGED|CURRENT_MODEL_CHANGED,NULL);
	}
}

/***********************************************************************************************************/
void ProgramState::switchToKeyframe(int newIndex)
{
	VideoKeyFrame* newFrame = videoModel->getKeyframeByIndex(newIndex);
	if (newFrame != currentModel)
	{

		if (mode != PROGRAM_MODE_DEFORMATIONS) {
			mode = PROGRAM_MODE_DEFORMATIONS;
			emit programStateUpdated(MODE_CHANGED,NULL);
		}

		currentModel = newFrame;
		currentAnimationTime = videoModel->getKeyFrameTimeMsec(newFrame);
		emit programStateUpdated(CURRENT_MODEL_CHANGED,NULL);
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

/***********************************************************************************************************/

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
		emit programStateUpdated(CURRENT_MODEL_CHANGED|KEYFRAME_LIST_EDITED|MODE_CHANGED,NULL);
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
	emit programStateUpdated(KEYFRAME_LIST_EDITED, NULL);
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
void ProgramState::startStopAnimations()
{
	if (!videoModel) return;

	if (mode != PROGRAM_MODE_BUSY)
	{
		mode = PROGRAM_MODE_BUSY;
		maxAnimationTime = videoModel->getTotalTime();
		emit programStateUpdated(MODE_CHANGED,NULL);
		animationReferenceTimer.start();
		animationTimer->start(0);
	} else
	{
		animationTimer->stop();
		mode = PROGRAM_MODE_ANIMATION;
		emit programStateUpdated(MODE_CHANGED,NULL);
	}
}

/***********************************************************************************************************/

void ProgramState::resetTransform()
{
	emit programStateUpdated(TRANSFORM_RESET,NULL);
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


	FPS = 1000.0/duration;
	emit programStateUpdated(STATUSBAR_UPDATED, NULL);
}

/***********************************************************************************************************/

void ProgramState::setAnimationRepeat(bool enabled)
{
	animationRepeat = enabled;
}

/***********************************************************************************************************/

void ProgramState::clearStatusBar()
{
	FPS=-1;
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
	if (mode == PROGRAM_MODE_OUTLINE)
	{
		int x = texture.width();
		int y = texture.height();

		if (y > x)
			outlineModel->setScale((double)x/y,1);
		else
			outlineModel->setScale(1,(double)y/x);
	}

	/* Bind it and in darkness grind it....*/
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

	QPixmap tex;

	if (loadTextureFile(rawname, tex)) {
		textureFile = rawname;
		texture.swap(tex);
		updateTexture();
		wireframeTransparency = 0;
	} else
		wireframeTransparency = 0.5;

	emit programStateUpdated(EDIT_SETTINGS_CHANGED,NULL);
}

/***********************************************************************************************************/

void ProgramState::autoCreateOutline()
{
	if (!outlineModel || mode != PROGRAM_MODE_OUTLINE)
		return;

	QImage image = texture.scaled(350,350,Qt::KeepAspectRatio).toImage();
	image = image.mirrored(false,true);

	currentModel = NULL;
	emit programStateUpdated(CURRENT_MODEL_CHANGED,NULL);

	delete outlineModel;
	outlineModel = new OutlineModel;

	const int stroke = 2;
	int Vmap[350+4*2][350+4*2] = {{0}}; //added stroke 2 in each side of the map
	for (int i=stroke; i<image.width()+stroke; i++)
	{
		for (int j=stroke; j<image.height()+stroke; j++)
		{
			if ( (image.hasAlphaChannel() && qAlpha(image.pixel(i-stroke,j-stroke)) > 200) ||
				 (!image.hasAlphaChannel() && qGray(image.pixel(i-stroke,j-stroke)) > 200))
			{
				for (int k=i-stroke; k<=i+stroke; k++) {
					for (int l=j-stroke; l<=j+stroke; l++) {
						Vmap[k][l] = -1;
					}
				}
			}
		}
	}

	int count = 0;
	for (int i=0; i<image.width()+2*stroke; i++) {
		for (int j=0; j<image.height()+2*stroke; j++) {
			if (Vmap[i][j] == -1)
			{
				if (Vmap[std::max(0,i-1)][j] == 0 ||
					Vmap[i][std::max(0,j-1)] == 0 ||
					Vmap[std::min(image.width()+2*stroke-1,i+1)][j] == 0 ||
					Vmap[i][std::min(image.height()+2*stroke-1,j+1)] == 0 ||
					Vmap[std::max(0,i-1)][std::max(0,j-1)] == 0 ||
					Vmap[std::max(0,i-1)][std::min(image.height()+2*stroke-1,j+1)] == 0 ||
					Vmap[std::min(image.width()+2*stroke-1,i+1)][std::max(0,j-1)] == 0 ||
					Vmap[std::min(image.width()+2*stroke-1,i+1)][std::min(image.height()+2*stroke-1,j+1)] == 0)
				{
						Vmap[i][j] = count++;

						Point2 p;
						p.x = (double)i / (image.width()+stroke);
						p.y = (double)j / (image.height()+stroke);

						outlineModel->vertices.push_back(p);
				}
			}
		}
	}

	for (int i=0; i<image.width()+2*stroke; i++)
	{
		for (int j=0; j<image.height()+2*stroke; j++) {
			if (j+1 < image.height()+2*stroke && Vmap[i][j] > 0 && Vmap[i][j+1] > 0)
			{
				outlineModel->edges.insert(Edge(Vmap[i][j],Vmap[i][j+1]));
			}
			if (i+1 < image.width()+2*stroke && Vmap[i][j] > 0 && Vmap[i+1][j] > 0)
			{
				outlineModel->edges.insert(Edge(Vmap[i][j],Vmap[i+1][j]));
			}
		}
	}

	outlineModel->updateMeshInfo();
	updateTexture();

	currentModel = outlineModel;
	emit programStateUpdated(KEYFRAME_EDITED|TRANSFORM_RESET|CURRENT_MODEL_CHANGED,NULL);

}

/***********************************************************************************************************/
