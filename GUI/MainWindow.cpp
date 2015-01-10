
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
	connect_(programstate, programStateUpdated(int, void *), editorWindow, programStateUpdated(int, void *));
	connect_(programstate, programStateUpdated(int, void *), sidePanel, programStateUpdated(int, void *));
	connect_(programstate, programStateUpdated(int, void *), animationPanel, programStateUpdated(int, void *));
	connect_(programstate, programStateUpdated(int, void *), this, programStateUpdated(int, void *));

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
			str.sprintf("Face: %5d", programstate->selectedFace);
			lblSelectedFace->setText(str);
		} else
			lblSelectedFace->hide();

		if (programstate->selectedVertex >= 0)  {
			lblSelectedVertex->show();
			str.sprintf("Vertex: %5d", programstate->selectedVertex);
			lblSelectedVertex->setText(str);
		} else
			lblSelectedVertex->hide();

		if (programstate->FPS >=0 )
		{
			str.sprintf("%06.2f FPS", programstate->FPS);
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

		if (programstate->statusbarMessage != "")
			statusBar()->showMessage(programstate->statusbarMessage);
		else
			statusBar()->showMessage("Ready");
	}

	if (flags & ProgramState::MODE_CHANGED)
	{
		auto mode = programstate->getCurrentMode();
		bool hasKeyframes = mode != ProgramState::PROGRAM_MODE_NONE && mode != ProgramState::PROGRAM_MODE_OUTLINE;

		bool busy = mode == ProgramState::PROGRAM_MODE_BUSY;
		bool deformation = mode == ProgramState::PROGRAM_MODE_DEFORMATIONS;
		bool editmode = mode == ProgramState::PROGRAM_MODE_DEFORMATIONS || mode == ProgramState::PROGRAM_MODE_OUTLINE;
		bool nothing = mode == ProgramState::PROGRAM_MODE_NONE;

		if (mode == ProgramState::PROGRAM_MODE_NONE || mode == ProgramState::PROGRAM_MODE_OUTLINE) 
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

		actionNew_keyframe->setEnabled(deformation);
		actionDelete_keyframe->setEnabled(deformation);
		actionReset_pins->setEnabled(deformation);
		actionPin_edit_mode->setEnabled(deformation);

		actionLoop->setEnabled(hasKeyframes);
		actionPlay->setEnabled(hasKeyframes);
		actionSave_video->setEnabled(hasKeyframes && !busy);


		actionNew->setEnabled(!busy);
		actionNew_model->setEnabled(!busy);
		actionLoad_mesh->setEnabled(!busy);
		actionSave_model->setEnabled(!busy && !nothing);
		actionLoad_texture->setEnabled(!busy && !nothing);
		actionReset_texture->setEnabled(!busy && !nothing);
		actionSave_screenshot->setEnabled(!busy && !nothing);

		actionUndo->setEnabled(editmode);
		actionRedo->setEnabled(editmode);
		actionReset_model->setEnabled(editmode);


		switch (programstate->mode) {
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_ANIMATION:
			lblMode->show();
			lblMode->setText("Interpolated frame");
			break;
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_NONE:
		case ProgramState::PROGRAM_MODE::PROGRAM_MODE_BUSY:
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
	programstate->interpolateFrame(programstate->currentAnimationTime+10);
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
