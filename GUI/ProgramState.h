#ifndef PROGRAMSTATE_H_
#define PROGRAMSTATE_H_

#include <string>
#include <qobject.h>
#include "VideoModel.h"
#include "OffScreenRenderer.h"
#include "OutlineModel.h"
#include <QTimer>
#include <QElapsedTimer>
#include <QVideoEncoder.h>

class ProgramState: public QObject
{
	Q_OBJECT
public:
	ProgramState();
	virtual ~ProgramState();

	enum UPDATE_FLAGS
	{
		/*  user edited the keyframe in editor  */
		KEYFRAME_EDITED 		= 0x01,
		/* New animation frame was interpolated */
		ANIMATION_STEPPED		= 0x02,
		/* textureref got changed */
		TEXTURE_CHANGED 		= 0x04,
		/* currentModel is now different model */
		CURRENT_MODEL_CHANGED 	= 0x08,
		/* Information in the state to be displayed on statusbar got changed */
		STATUSBAR_UPDATED 		= 0x10,
		/* Mode of operation (outline/kvf/pframe/video) got changed */
		MODE_CHANGED 			= 0x20,
		/* Settings actuall for edit window got changed */
		EDIT_SETTINGS_CHANGED   = 0x40,
		/* Request for scale/move reset */
		TRANSFORM_RESET 		= 0x80,

		KEYFRAME_LIST_EDITED	= 0x100,

		PANEL_VISIBLITIY_CHANGED = 0x200,
	};

	enum PROGRAM_MODE
	{
		PROGRAM_MODE_NONE,
		PROGRAM_MODE_DEFORMATIONS,
		PROGRAM_MODE_OUTLINE,
		PROGRAM_MODE_ANIMATION,
		PROGRAM_MODE_BUSY,
	};

	/* Current program mode */
	ProgramState::PROGRAM_MODE mode;
	/* Current model */
	MeshModel *currentModel;

	/* Models */
	VideoModel *videoModel;
	OutlineModel *outlineModel;

	/* statusbar statistics*/
	double FPS;
	int vertexCount;
	int facesCount;
	int progressValue;
	int selectedVertex;
	int selectedFace;
	QString statusbarMessage;

	/* Editor settings*/
	bool pinMode;
	bool showVF;
	bool showVForig;
	bool showSelection;
    double wireframeTransparency;
    bool multitouchMode;
    bool showBDmorphEdge;
    bool showBDmorphOrigMesh;

    /* Texture and texture file */
	std::string textureFile;
	GLuint textureRef;
	QPixmap texture;

	/* Animation settings */
	int currentAnimationTime;
	bool animationRepeat;
	int targetFPS;

	/* Misc */
	OffScreenRenderer *thumbnailRenderer;
	OffScreenRenderer *imageRenderer;

public:
	/* Load/store parts of the state */
	void initialize();

    bool createProject(std::string file);
	bool loadProject(std::string filename);
	bool saveToFile(std::string filename);
	bool saveScreenshot(std::string filename);
	bool createMeshFromOutline(int triangleCount);
	void editOutline();
	bool setTexture(std::string textureFile);
	bool loadKeyframe(std::string file);

	/*  Editor calls this when it edited the current model
	 *  Animation code calls this when p-frame got redone
	 *  Side panel calls this when it changed KVF model settings
	 *  */
	void onUpdateModel();
	void switchToKeyframe(int newIndex);
	void updateStatistics();
	void updateSettings();

	/* Animation panel uses this to edit list of keyframes*/
	void cloneKeyframe(int id = -1);
	void deleteKeyFrame(int id = -1);
	void setKeyframeTime(int id, int newTime);
	void createKeyframeFromPFrame();


	/* Gets information on keyframes*/
	int getCurrentKeyframeId();
	int getKeyframeCount();
	int  getKeyframeTime(int id);
	enum PROGRAM_MODE getCurrentMode();

	/* Animations controls */
	void startAnimations(int time);
	void stopAnimations();
	void interpolateFrame(int time);
	void setAnimationRepeat(bool enabled);

	void resetTransform();
	void setProgress(int value);
	void updateGUI();

	bool createVideo(QString file);
signals:
	/* Informs all the users that parts of the state changed */
	void programStateUpdated(int flags, void *param);

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

	QTimer *animationTimer;
	QElapsedTimer animationReferenceTimer;
	int maxAnimationTime;

	/* Video encoding task*/
	QVideoEncoder* videoEncoder;
	uint8_t* imagebuffer;
	int videoEncodingTime;

};

/***********************************************************************************************************/
#endif /* PROGRAMSTATE_H_ */
