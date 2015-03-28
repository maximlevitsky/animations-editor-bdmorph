#ifndef PROGRAMSTATE_H_
#define PROGRAMSTATE_H_

#include "VideoModel.h"
#include "OutlineModel.h"
#include "ffmpeg_encoder.h"

#include <string>
#include <qobject.h>
#include <QTimer>
#include <QPixmap>

struct StatusBarState {
	double FPS;
	int vertexCount;
	int facesCount;
	int progressValue;
	int selectedVertex;
	int selectedFace;
	QString statusbarMessage;
};

struct RenderSettings
{
	/* KVF render settings */
	bool showVF;
	bool showVForig;
	bool showSelection;
    double wireframeTransparency;

    /* bdmorph settings*/
    bool showBDmorphEdge;
    bool showBDmorphOrigMesh;
    int targetFPS;

	bool pinMode;
    double alpha;

	std::string textureFile;
	QPixmap texture;

};


class ProgramState: public QObject
{
	Q_OBJECT
public:
	ProgramState();
	virtual ~ProgramState();

	enum PROGRAM_MODE
	{
		PROGRAM_MODE_NONE,

		/* outline editor */
		PROGRAM_MODE_OUTLINE,

		/* use KVF to edit current frame */
		PROGRAM_MODE_DEFORMATIONS,

		/* animation running or paused */
		PROGRAM_MODE_ANIMATION_PAUSED,
		PROGRAM_MODE_ANIMATION_RUNNING,

		/* busy (saving video or replaying log or whatever)*/
		PROGRAM_MODE_BUSY,
	};

	/* Models */
	MeshModel *currentModel;
	VideoModel *videoModel;
	OutlineModel *outlineModel;

	/* current mode management */
	void setCurrentMode(enum PROGRAM_MODE mode);

	enum PROGRAM_MODE getCurrentMode();

	/* is some model loaded*/
	bool isModelLoaded() {  return mode != PROGRAM_MODE_NONE; }

	/* are we in busy state where most of GUI should be disabled?
	 *  (animation is running, or we are saving to file or we are replaying the log)
	 */
	bool isBusy() { return mode == PROGRAM_MODE_ANIMATION_RUNNING || mode == PROGRAM_MODE_BUSY; }

	/* do we edit something ?*/
	bool isOutlineEditor() { return mode == PROGRAM_MODE_OUTLINE; }
	bool isDeformationEditor() {  return mode == PROGRAM_MODE_DEFORMATIONS; }
	bool isEditing() { return isOutlineEditor() || isDeformationEditor(); }

	/* do we show animations*/
	bool isAnimations() { return mode == PROGRAM_MODE_ANIMATION_RUNNING ||  mode == PROGRAM_MODE_ANIMATION_PAUSED; }

	/* Are we in full edit mode ( as opposed to noting or outline)*/
	bool isFullMode() { return mode != PROGRAM_MODE_OUTLINE && mode != PROGRAM_MODE_NONE; }


	/* render setting management */
	void setRenderSettings(const RenderSettings &newsettings);
	const RenderSettings& getRenderSettings() { return renderSettings;};
	const QPixmap& getTexture() const { return renderSettings.texture; }
	void resetTransform();

	/* editor calls this to inform that current model got updated */
	void informModelEdited();
	void updateGUI();

	/* statusbar */
	void setSelectedVertexAndFace(int selectedVertex, int selectedFace);
	void setFPS(double newFPS);
	void showStatusBarMessage(const QString& message);
	void setProgress(int value);
	StatusBarState getStatusbarData() { return statusbarState; }


	void runLog(std::string filename);
	void saveLog(std::string filename);

public:
	void initialize();

	/* Load/store parts of the state */
    bool createProject(std::string file);
	bool loadProject(std::string filename);
	bool saveToFile(std::string filename);
	bool saveScreenshot(std::string filename);
	bool saveVideo(std::string file);
	bool createProjectFromOutline(int triangleCount);
	void editOutline();
	void autoCreateOutline();
	bool loadTexture(std::string textureFile);
	bool loadKeyframe(std::string file);

	/* Animation panel uses this to edit list of keyframes*/
	void switchToKeyframe(int newIndex);
	void cloneKeyframe(int id = -1);
	void deleteKeyFrame(int id = -1);
	void setKeyframeTime(int id, int newTime);
	void createKeyframeFromPFrame();

	/* Gets information on keyframes*/
	int getCurrentKeyframeId();
	int getKeyframeCount();
	int  getKeyframeTime(int id);

	/* Animations controls */
	void startStopAnimations();
	void setAnimationRepeat(bool enabled);

	/* Calculate and show on screen an interpolated frame */
	int getAnimationPosition() { return currentAnimationTime; }
	void setAnimationPosition(int newPosition);

	enum UPDATE_FLAGS
	{
		/* Mode of operation (outline/kvf/pframe/video) got changed */
		MODE_CHANGED 			= 0x1,

		/* new keyframe created/keyframe deleted/keyframe got time change */
		KEYFRAME_LIST_EDITED	= 0x2,

		/* currentModel is now different model */
		CURRENT_MODEL_CHANGED 	= 0x4,

		/*  current model is the same but it got edited (used changed the points using KVF)  */
		MODEL_EDITED 		    = 0x8,

		/* New animation frame was interpolated */
		ANIMATION_POSITION_CHANGED		= 0x10,

		/* Settings actuall for edit window got changed */
		RENDER_SETTINGS_CHANGED   = 0x20,

		/* textureref got changed (this is also part of render settings actually) */
		TEXTURE_CHANGED 		= 0x40,

		/* Information in the state to be displayed on statusbar got changed */
		STATUSBAR_UPDATED 		= 0x80,

		/* Request for scale/move reset */
		TRANSFORM_RESET 		= 0x100,

		PANEL_VISIBLITIY_CHANGED = 0x200,
	};

signals:
	/* Informs all the users that parts of the state changed */
	void programStateUpdated(int flags);
private slots:
	void onAnimationTimer();
private:
	void clearStatusBar();
	void unloadAll();
	void unloadVideoModel();
	void unloadOutlineModel();
	bool loadTextureFile(std::string file, QPixmap &out);
	void updateTexture();
	void tryToGuessLoadTexture(std::string file);
	void interpolateFrame(int time);


	/* Current program mode */
	ProgramState::PROGRAM_MODE mode;

	/* Animation settings */
	int currentAnimationTime;
	bool animationRepeat;
	QTimer *animationTimer;
	QElapsedTimer animationReferenceTimer;
	int maxAnimationTime;

	/* Render  settings*/
	RenderSettings renderSettings;

	/* Statusbar settings */
	StatusBarState statusbarState;
};
#endif /* PROGRAMSTATE_H_ */
