
#include <QDockWidget>
#include <QFileDialog>
#include <QtOpenGL>
#include <QProgressBar>
#include <QString>
#include <QDialog>
#include <fstream>

#include "MainWindow.h"
#include "EditorWindow.h"
#include "SidePanel.h"
#include "ThumbnailRenderer.h"
#include "AnimationPanel.h"
#include "utils.h"
#include "VideoModel.h"
#include "BDMORPH.h"
#include "OutlineModel.h"
#include "ui_About.h"

/*****************************************************************************************************/
MainWindow::MainWindow()
{
	setupUi(this);

	/* setup GL view */
	editorWindow = new EditorWindow(this);
	setCentralWidget(editorWindow);

	/* setup side panel */
	sidePanel = new SidePanel(this);
	addDockWidget(Qt::RightDockWidgetArea, sidePanel);

	/* setup animation panel */
	animationPanel = new AnimationPanel(this);
	animationPanel->setTitleBarWidget(new QWidget(this));
	addDockWidget(Qt::BottomDockWidgetArea, animationPanel);

	programstate = new ProgramState();
	programstate->thumbnailRenderer = new ThumbnailRenderer(NULL, editorWindow);

	/* --------------------------------------------------------------------------------*/
	/* Make everyone listen to program state */
	connect_(programstate, programStateUpdated(int, void *), editorWindow, programStateUpdated(int, void *));
	connect_(programstate, programStateUpdated(int, void *), sidePanel, programStateUpdated(int, void *));
	connect_(programstate, programStateUpdated(int, void *), animationPanel, programStateUpdated(int, void *));
	connect_(programstate, programStateUpdated(int, void *), this, programStateUpdated(int, void *));

	editorWindow->programStateCreated(programstate);
	sidePanel->programStateCreated(programstate);
	animationPanel->programStateCreated(programstate);

	connect_(actionNew_model, triggered(), 					sidePanel, onCreateModel());
	connect_(actionLoad_mesh, triggered(), 					sidePanel, onLoadModel());
	connect_(actionSave_model, triggered(), 				sidePanel, onSaveModel());

	connect_(actionUndo, triggered(), 						sidePanel, onUndoModel());
	connect_(actionRedo, triggered(), 						sidePanel, onRedoModel());
	connect_(actionReset_model, triggered(), 				sidePanel, onResetPoints());
	connect_(actionReset_pins, triggered(), 				sidePanel, onClearPins());
	connect_(actionReplay_log, triggered(), 				sidePanel, onRunLog());
	connect_(actionSave_log, triggered(), 					sidePanel, onSaveLog());
	connect_(action_show_VF, toggled(bool), 				sidePanel, onDrawVFModeChanged(bool));
	connect_(action_show_orig_VF, toggled(bool), 			sidePanel, onDrawOrigVFModeChanged(bool));
	connect_(actionReapply_VF,triggered(), 					sidePanel, onReuseVF());
	connect_(actionLoad_texture, triggered(), 				sidePanel, onChooseTexture());
	connect_(actionReset_texture, triggered(), 				sidePanel, onResetTexture());

	connect_(actionNew_keyframe, triggered(), 				animationPanel, onCloneKeyFramePressed());
	connect_(actionDelete_keyframe, triggered(), 			animationPanel, onDeleteKeyframe());
	connect_(actionPlay, triggered(), 						animationPanel, onPlayPauseButtonPressed());

	connect_(actionSave_screenshot, triggered(), 			this, onSaveScreenShot());
	connect_(actionSide_panel, toggled(bool), 				sidePanel, onShowHide(bool));
	connect_(actionAnimation_panel, toggled(bool), 			animationPanel, onShowHide(bool));
	connect_(actionAbout, triggered(),						this, onAbout());
	connect_(actionExit, triggered(), 						this, close());
	connect_(actionTest_animations, triggered(), 			this, onInterpolationTest());

	/* --------------------------------------------------------------------------------*/
	animationPanel->btnRepeat->setAction(actionLoop);
	sidePanel->chkPinMode->setAction(actionPin_edit_mode);
	actionSide_panel->setChecked(true);
	actionAnimation_panel->setChecked(true);

	/* setup statusbar*/
	statusBar()->showMessage(tr("Ready"));
	lblVertexCount = new QLabel(this);
	lblVertexCount->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblFacesCount = new QLabel(this);
	lblFacesCount->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblFPS = new QLabel(this);
	lblFPS->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblSelectedFace = new QLabel(this);
	lblSelectedFace->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblSelectedVertex = new QLabel(this);
	lblSelectedVertex->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	progressIndicator = new QProgressBar;

	statusBar()->addPermanentWidget(lblFacesCount);
	statusBar()->addPermanentWidget(lblVertexCount);
	statusBar()->addPermanentWidget(lblSelectedVertex);
	statusBar()->addPermanentWidget(lblSelectedFace);
	statusBar()->addPermanentWidget(lblFPS);
	statusBar()->addPermanentWidget(progressIndicator);

	lblFPS->hide();
	lblFacesCount->hide();
	lblVertexCount->hide();
	lblSelectedFace->hide();
	lblSelectedVertex->hide();
	progressIndicator->hide();
	programstate->createProject("");
	show();
	programstate->resetTransform();
}

/*****************************************************************************************************/
MainWindow::~MainWindow()
{
	delete editorWindow;
	delete animationPanel;
	delete sidePanel;
	delete programstate;
}

/*****************************************************************************************************/
void MainWindow::programStateUpdated(int flags, void *param)
{
	QString str;
	if (flags & ProgramState::STATUSBAR_UPDATED)
	{
		if (programstate->vertexCount > 0)
		{
			str.sprintf("Vertexes: %d", programstate->vertexCount);
			lblVertexCount->setText(str);
			lblVertexCount->show();
		} else
			lblVertexCount->hide();

		if (programstate->facesCount > 0)
		{
			str.sprintf("Faces: %d", programstate->facesCount);
			lblFacesCount->setText(str);
			lblFacesCount->show();
		} else
			lblFacesCount->hide();

		if (programstate->selectedFace >= 0)  {
			lblSelectedFace->show();
			str.sprintf("Face: %d", programstate->selectedFace);
			lblSelectedFace->setText(str);
		} else
			lblSelectedFace->hide();

		if (programstate->selectedVertex >= 0)  {
			lblSelectedVertex->show();
			str.sprintf("Vertex: %d", programstate->selectedVertex);
			lblSelectedVertex->setText(str);
		} else
			lblSelectedVertex->hide();

		if (programstate->FPS >0 )
		{
			str.sprintf("%4.2f FPS", 1000.0/programstate->FPS);
			lblFPS->setText(str);
			lblFPS->show();
		} else {
			lblFPS->hide();
		}

		if (programstate->progressValue != 0) {
			progressIndicator->setValue(programstate->progressValue);
			progressIndicator->show();
		} else
			progressIndicator->hide();
	}

	if (flags & ProgramState::MODE_CHANGED)
	{
		auto mode = programstate->getCurrentMode();
		bool hasKeyframes = mode != ProgramState::PROGRAM_MODE_NONE && mode != ProgramState::PROGRAM_MODE_OUTLINE;

		animationPanel->setVisible(hasKeyframes);
		actionAnimation_panel->setEnabled(hasKeyframes);
		actionAnimation_panel->setChecked(hasKeyframes);

		actionNew_keyframe->setEnabled(hasKeyframes);
		actionDelete_keyframe->setEnabled(hasKeyframes);
		actionLoop->setEnabled(hasKeyframes);
		actionLoad_keyframe->setEnabled(hasKeyframes);
		actionPlay->setEnabled(hasKeyframes);
		actionPin_edit_mode->setEnabled(hasKeyframes);
		actionReset_pins->setEnabled(hasKeyframes);
		actionSave_video->setEnabled(hasKeyframes);
	}

	if (flags & ProgramState::PANEL_VISIBLITIY_CHANGED) {
		actionSide_panel->setChecked(sidePanel->isVisible());
	}
}
/*****************************************************************************************************/
void MainWindow::onSaveScreenShot()
{
	/*TODO */
}
void MainWindow::onSaveVideo()
{
	/*TODO */
}


void MainWindow::onAbout()
{
	QDialog *dialog = new QDialog(this);
	Ui_aboutDialog t;
	t.setupUi(dialog);
	dialog->exec();
}

/*****************************************************************************************************/

void MainWindow::onInterpolationTest()
{
	programstate->interpolateFrame(programstate->currentAnimationTime+10);
}

/*****************************************************************************************************/
