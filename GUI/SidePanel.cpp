
#include <QDockWidget>
#include "SidePanel.h"
#include "KVFModel.h"
#include "BDMORPH.h"
#include "OutlineModel.h"
#include "utils.h"
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <QApplication>

SidePanel::SidePanel(QWidget* parent) : QDockWidget(parent), programstate(NULL)
{
	setupUi(this);
	connect_(btnCreateMesh, clicked(), 							this, onMeshCreateButtonPressed());
	connect_(btnResetMesh, clicked(), 							this, onResetPoints());
	connect_(btnResetPins, clicked(),							this, onClearPins());
	connect_(btnResetTransform, clicked(),						this, onResetTransform());
	connect_(btnUndo, clicked(),								this, onUndoModel());
	connect_(btnRedo, clicked(),								this, onRedoModel());
	connect_(chkPinMode, clicked(bool), 						this, onPinModeChanged(bool));
	connect_(chkShowSelection, clicked(bool), 					this, onShowSelectionChanged(bool));
	connect_(sliderWireframeTransparency, valueChanged(int), 	this, onChangeWireframe(int));
	connect_(sliderAlpha, valueChanged(int), 					this, onChangeAlpha(int));

	connect_(btnNewModel, clicked(), 							this, onImportProject());
	connect_(btnLoadModel, clicked(),							this, onLoadProject());
	connect_(btnSaveModel, clicked(), 							this, onSaveProject());
	connect_(btnResetTexture, clicked(),						this, onResetTexture());
	connect_(btnLoadTexture, clicked(),							this, onChooseTexture());
	connect_(btnEditOutline, clicked(),							this, onEditOutlinePressed());

	connect_(chkShowStartModel, clicked(bool),					this, onBdmorphOrigModel(bool));
	connect_(chkShowAnchorEdge, clicked(bool),					this, onShowBdmorphEdgeClicked(bool));

	connect_(btnConvertToKeyframe, clicked(),					this, onBdmorphConvertToKeyframe());
	connect_(spinFPSBox, valueChanged (int), 					this, onTargetFPSChanged(int));
	connect_(btnCreateOutline, clicked(),						this, onAutoOutlineCreate());

	sliderMeshDensity->setRange(1,100000);
	sliderMeshDensity->setTickPosition(QSlider::NoTicks);
	sliderMeshDensity->setSliderPosition(1000);
	sliderAlpha->setValue(5);
	sliderAlpha->setMaximum(10000);
	sliderAlpha->setTickPosition(QSlider::NoTicks); /* I don't want ticks, these are awful I heard*/
}

/******************************************************************************************************************************/
void SidePanel::programStateUpdated(int flags, void *param)
{
	if (flags & ProgramState::ANIMATION_STEPPED)
	{
		QString bdmorphTime;
		bdmorphTime.sprintf("Current time: %s (t = %f)", printTime(programstate->currentAnimationTime).c_str(), programstate->videoModel->pFrame->last_t);

		lblCurrentBDMORPHTime->setText(bdmorphTime);
	}

	if (flags & ProgramState::MODE_CHANGED)
	{
		bool animations = programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_ANIMATION;
		bool outline = programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_OUTLINE;
		bool deformations = programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_DEFORMATIONS;
		bool video = programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_BUSY;
		bool nothing = programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_NONE;

		frameOutline->setVisible(false);
		frameKVF->setVisible(false);
		frameBDMorph->setVisible(false);
		frameEdit->setVisible(false);
		frameWireframe->setVisible(false);

		frameLoad->setEnabled(!video);
		btnConvertToKeyframe->setEnabled(animations);
		frameOutline->setVisible(outline);
		frameKVF->setVisible(deformations);
		frameBDMorph->setVisible(animations || video);
		frameEdit->setVisible(outline || deformations);
		frameWireframe->setVisible(!nothing);
		btnCreateMesh->setEnabled(outline);
		btnEditOutline->setEnabled(!outline && !nothing);

		btnLoadTexture->setEnabled(!nothing);
		btnResetTexture->setEnabled(!nothing);
		btnSaveModel->setEnabled(!nothing);
	}

	if (flags & (ProgramState::EDIT_SETTINGS_CHANGED|ProgramState::CURRENT_MODEL_CHANGED))
	{
		KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);

		if (kvfModel)  {
			sliderAlpha->setValue(kvfModel->getAlpha() * 1000);
			QString alphaLabel;
			alphaLabel.sprintf("Control points weight: (alpha=%f)", kvfModel->getAlpha() );
			lblControlPoints->setText(alphaLabel);
		}
		sliderWireframeTransparency->setValue(programstate->wireframeTransparency*100);
	}
}

/******************************************************************************************************************************/
void SidePanel::onImportProject()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose base for new model (picture/outline/mesh"),
    		QString(), QLatin1String("Picture (*.png *.jpg);;Outline (*.poly);;Mesh (*.obj *.off)"));
    if (filename == NULL)
		return;
    programstate->createProject(filename.toStdString());
}

/*****************************************************************************************************/
void SidePanel::onLoadProject()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose video model to load"),
    		QString(), QLatin1String("*.vproject"));
    if (filename == "") return;
    programstate->loadProject(filename.toStdString());
}

/*****************************************************************************************************/
void SidePanel::onSaveProject()
{
	if ( !programstate) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("Mesh (*.obj);;Outline (*.poly);;Video Project (*.vproject)"));
    if ( filename == "") return;
    programstate->saveToFile(filename.toStdString());
}

/*****************************************************************************************************/
void SidePanel::onChooseTexture()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose image"), QString(), QLatin1String("Texture (*.png *.jpg *.bmp)"));
    if (filename == NULL)
		return;

    programstate->setTexture(filename.toStdString());
}

/*****************************************************************************************************/
void SidePanel::onResetTexture()
{
	programstate->setTexture("");
}

/*****************************************************************************************************/
void SidePanel::onMeshCreateButtonPressed()
{
	int density = sliderMeshDensity->value();
	programstate->createMeshFromOutline(density);
}

/******************************************************************************************************************************/

void SidePanel::onEditOutlinePressed()
{
	programstate->editOutline();
}

/******************************************************************************************************************************/
void  SidePanel::onResetPoints()
{
	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;
	model->historyReset();
	programstate->onUpdateModel();
	programstate->updateSettings();
}
/******************************************************************************************************************************/
void SidePanel::onUndoModel()
{
	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;
	model->historyUndo();
	programstate->onUpdateModel();
	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onRedoModel()
{
	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;
	model->historyRedo();
	programstate->onUpdateModel();
	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onSaveLog()
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;

	QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    std::ofstream outfile(filename.toAscii());
    kvfModel->historySaveToFile(outfile);
}

/******************************************************************************************************************************/
void SidePanel::onRunLog()
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;

	QString filename = QFileDialog::getOpenFileName(this, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    std::ifstream infile(filename.toAscii());

    printf("STARTING log replay\n");
    TimeMeasurment t;

    int numSteps;
    infile >> numSteps;

    programstate->statusbarMessage = "Replaying log...";

    for (int step = 0; step < numSteps; step++)
    {
    	kvfModel->historyLoadFromFile(infile);
    	programstate->onUpdateModel();
    	programstate->updateSettings();
    	programstate->FPS = 1000.0 / (kvfModel->lastVFCalcTime+kvfModel->lastVFApplyTime);
    	programstate->setProgress((step*100)/numSteps);
		QApplication::processEvents();

    }

    programstate->statusbarMessage.clear();
    programstate->setProgress(0);

    printf("DONE WITH log replay (took %f msec)\n", t.measure_msec());
    kvfModel->historySnapshot();
}
/******************************************************************************************************************************/

void SidePanel::onClearPins()
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;
	kvfModel->clearPins();
	programstate->onUpdateModel();
	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onReuseVF()
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;
	kvfModel->applyVF();
	programstate->onUpdateModel();
}

/******************************************************************************************************************************/
void SidePanel::onChangeAlpha(int i)
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;
	kvfModel->setAlpha((((double) (i)) / 1000));
	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onDrawVFModeChanged(bool m)
{
	if (!programstate) return;
	programstate->showVF = m;
	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onDrawOrigVFModeChanged(bool m)
{
	if (!programstate) return;
	programstate->showVForig = m;
	programstate->updateSettings();
}

void SidePanel::onShowSelectionChanged(bool m)
{
	programstate->showSelection = m;

	if (!m) {
		programstate->selectedFace = -1;
		programstate->selectedVertex = -1;
	}

	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onPinModeChanged(bool m)
{
	if (!programstate) return;
	programstate->pinMode = m;
	programstate->updateSettings();
}
/******************************************************************************************************************************/
void SidePanel::onChangeWireframe(int i)
{
	if (!programstate) return;
	programstate->wireframeTransparency = (double)i / 100.0;
	programstate->updateSettings();
}

/******************************************************************************************************************************/
void SidePanel::onResetTransform()
{
	if (!programstate) return;
	programstate->resetTransform();
}

/******************************************************************************************************************************/

void SidePanel::onShowHide(bool checked)
{
	setVisible(checked);
}

/******************************************************************************************************************************/
void SidePanel::onShowBdmorphEdgeClicked(bool checked)
{
	programstate->showBDmorphEdge = checked;
	programstate->updateSettings();
}


/******************************************************************************************************************************/
void SidePanel::onBdmorphOrigModel(bool checked)
{
	programstate->showBDmorphOrigMesh = checked;
	programstate->updateSettings();
}


/******************************************************************************************************************************/
void SidePanel::onBdmorphConvertToKeyframe()
{
	programstate->createKeyframeFromPFrame();
}


/******************************************************************************************************************************/
void SidePanel::onTargetFPSChanged(int newValue)
{
	programstate->targetFPS = newValue;
	programstate->updateSettings();
}


/******************************************************************************************************************************/
void SidePanel::closeEvent (QCloseEvent *event)
{
	hide();
	programstate->updateGUI();
}

void SidePanel::onAutoOutlineCreate()
{
	QMessageBox::information(this, "Message", "Not implemented yet");
}

