
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
#include "AnimationPanel.h"
#include "utils.h"
#include "VideoModel.h"
#include "BDMORPH.h"
#include "OutlineModel.h"
#include "ui_About.h"

#include <fcntl.h>

#ifdef _WIN32
#include <io.h>
#endif

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

	/* --------------------------------------------------------------------------------*/
	/* Make everyone listen to program state */
	connect_(programstate, programStateUpdated(int), editorWindow, programStateUpdated(int));
	connect_(programstate, programStateUpdated(int), sidePanel, programStateUpdated(int));
	connect_(programstate, programStateUpdated(int), animationPanel, programStateUpdated(int));
	connect_(programstate, programStateUpdated(int), this, programStateUpdated(int));

	editorWindow->programStateCreated(programstate);
	sidePanel->programStateCreated(programstate);
	animationPanel->programStateCreated(programstate);

	connect_(actionNew_model, triggered(), 					sidePanel, onImportProject());
	connect_(actionLoad_mesh, triggered(), 					sidePanel, onLoadProject());
	connect_(actionSave_model, triggered(), 				sidePanel, onSaveProject());

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

	connect_(actionSave_video, triggered(),					this, onSaveVideo());
	connect_(actionDebug_Console, toggled(bool),            this, onToggleDebugConsole(bool));

	connect_(actionNew, triggered(), 						this, onNewProject());

	/* --------------------------------------------------------------------------------*/
	animationPanel->btnRepeat->setAction(actionLoop);
	sidePanel->chkPinMode->setAction(actionPin_edit_mode);
	actionSide_panel->setChecked(true);
	actionAnimation_panel->setChecked(true);

	/* setup statusbar*/
	statusBar()->showMessage(tr("Ready"));

	lblMode = new QLabel(this);
	lblMode->setFrameStyle(QFrame::Panel | QFrame::Sunken);

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
	statusBar()->addPermanentWidget(lblMode);
	statusBar()->addPermanentWidget(progressIndicator);

	setContextMenuPolicy(Qt::NoContextMenu);

	lblMode->hide();
	lblFPS->hide();
	lblFacesCount->hide();
	lblVertexCount->hide();
	lblSelectedFace->hide();
	lblSelectedVertex->hide();
	progressIndicator->hide();
	programstate->initialize();
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
void MainWindow::programStateUpdated(int flags)
{
	QString str;
	if (flags & ProgramState::STATUSBAR_UPDATED)
	{
		StatusBarState state = programstate->getStatusbarData();

		if (state.vertexCount > 0)
		{
			str.sprintf("Vertexes: %d", state.vertexCount);
			lblVertexCount->setText(str);
			lblVertexCount->show();
		} else
			lblVertexCount->hide();

		if (state.facesCount > 0)
		{
			str.sprintf("Faces: %d", state.facesCount);
			lblFacesCount->setText(str);
			lblFacesCount->show();
		} else
			lblFacesCount->hide();

		if (state.selectedFace >= 0)  {
			lblSelectedFace->show();
			str.sprintf("Face: %5d", state.selectedFace);
			lblSelectedFace->setText(str);
		} else
			lblSelectedFace->hide();

		if (state.selectedVertex >= 0)  {
			lblSelectedVertex->show();
			str.sprintf("Vertex: %5d", state.selectedVertex);
			lblSelectedVertex->setText(str);
		} else
			lblSelectedVertex->hide();

		if (state.FPS >=0 )
		{
			str.sprintf("%06.2f FPS", state.FPS);
			lblFPS->setText(str);
			lblFPS->show();
		} else {
			lblFPS->hide();
		}

		if (state.progressValue != 0) {
			progressIndicator->setValue(state.progressValue);
			progressIndicator->show();
		} else
			progressIndicator->hide();

		if (state.statusbarMessage != "")
			statusBar()->showMessage(state.statusbarMessage);
		else
			statusBar()->showMessage("Ready");
	}

	if (flags & ProgramState::MODE_CHANGED)
	{

		if (!programstate->isFullMode())
		{
			animationPanel->setVisible(false);
			actionAnimation_panel->setEnabled(false);
			actionAnimation_panel->setChecked(false);

		} else if(actionAnimation_panel->isEnabled() == false) 
		{
			animationPanel->setVisible(true);
			actionAnimation_panel->setEnabled(true);
			actionAnimation_panel->setChecked(true);
		}

		actionNew_keyframe->setEnabled(programstate->isDeformationEditor());
		actionDelete_keyframe->setEnabled(programstate->isDeformationEditor());
		actionReset_pins->setEnabled(programstate->isDeformationEditor());
		actionPin_edit_mode->setEnabled(programstate->isDeformationEditor());

		actionLoop->setEnabled(programstate->isFullMode() && (!programstate->isBusy() || programstate->isAnimations()));
		actionPlay->setEnabled(programstate->isFullMode() && (!programstate->isBusy() || programstate->isAnimations()));
		actionSave_video->setEnabled(programstate->isFullMode() && !programstate->isBusy());

		actionNew->setEnabled(!programstate->isBusy());
		actionNew_model->setEnabled(!programstate->isBusy());
		actionLoad_mesh->setEnabled(!programstate->isBusy());

		actionSave_model->setEnabled(!programstate->isBusy() && programstate->isModelLoaded());
		actionLoad_texture->setEnabled(!programstate->isBusy() && programstate->isModelLoaded());
		actionReset_texture->setEnabled(!programstate->isBusy() && programstate->isModelLoaded());
		actionSave_screenshot->setEnabled(!programstate->isBusy() && programstate->isModelLoaded());

		actionUndo->setEnabled(programstate->isEditing());
		actionRedo->setEnabled(programstate->isEditing());
		actionReset_model->setEnabled(programstate->isEditing());

		actionReapply_VF->setEnabled(programstate->isDeformationEditor());
		actionReplay_log->setEnabled(programstate->isDeformationEditor());
		actionSave_log->setEnabled(programstate->isDeformationEditor());

		action_show_VF->setEnabled(programstate->isDeformationEditor());
		action_show_orig_VF->setEnabled(programstate->isDeformationEditor());
		actionTest_animations->setEnabled(programstate->isFullMode());

		switch (programstate->getCurrentMode()) {
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_ANIMATION_PAUSED:
			lblMode->show();
			lblMode->setText("Interpolated frame");
			break;
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_NONE:
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_BUSY:
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_ANIMATION_RUNNING:
			lblMode->hide();
			break;
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_DEFORMATIONS:
			lblMode->show();
			lblMode->setText("Keyframe");
			break;
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_OUTLINE:
			lblMode->show();
			lblMode->setText("Outline");
		}


	}

	if (flags & ProgramState::PANEL_VISIBLITIY_CHANGED) {
		actionSide_panel->setChecked(sidePanel->isVisible());
	}
}
/*****************************************************************************************************/
void MainWindow::onSaveScreenShot()
{
	if ( !programstate) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("Image file (*.png *.jpg)"));
    if ( filename == "") return;
    programstate->saveScreenshot(filename.toStdString());
}

/*****************************************************************************************************/
void MainWindow::onSaveVideo()
{
	if ( !programstate) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("Video file (*.avi *.mp4)"));
    if ( filename == "") return;
    programstate->saveVideo(filename.toStdString());
}

/*****************************************************************************************************/

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
	programstate->setAnimationPosition(programstate->getAnimationPosition()+10);
}

/*****************************************************************************************************/

void MainWindow::onNewProject()
{
	programstate->createProject("");
}

/*****************************************************************************************************/

void MainWindow::onToggleDebugConsole(bool on)
{
#ifdef _WIN32

	// This shit is made exclusive for Windows(R) (TM)

	if (on)
	{
			// allocate a console for this app
		AllocConsole();

		// redirect unbuffered STDOUT to the console
		HANDLE consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
		int fileDescriptor = _open_osfhandle((intptr_t)consoleHandle, _O_TEXT);
		FILE *fp = _fdopen( fileDescriptor, "w" );
		*stdout = *fp;
		//setvbuf( stdout, NULL, _IONBF, 0 );

		// give the console window a nicer title
		SetConsoleTitle("Debug Output");

		// give the console window a bigger buffer size
		CONSOLE_SCREEN_BUFFER_INFO csbi;
		if ( GetConsoleScreenBufferInfo(consoleHandle, &csbi) )
		{
			COORD bufferSize;
			bufferSize.X = csbi.dwSize.X;
			bufferSize.Y = 9999;
			SetConsoleScreenBufferSize(consoleHandle, bufferSize);
		}
	}
	else {
		FreeConsole();
	}
#endif
}

/*****************************************************************************************************/
