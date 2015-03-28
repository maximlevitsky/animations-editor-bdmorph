
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

	sliderMeshDensity->setRange(0,100000);
	sliderMeshDensity->setTickPosition(QSlider::NoTicks);
	sliderMeshDensity->setSliderPosition(1000);
	sliderAlpha->setValue(5);
	sliderAlpha->setMaximum(10000);
	sliderAlpha->setMinimum(0);
	sliderAlpha->setTickPosition(QSlider::NoTicks); /* I don't want ticks, these are awful I heard*/
}

/******************************************************************************************************************************/
void SidePanel::programStateUpdated(int flags)
{
	if (flags & ProgramState::ANIMATION_POSITION_CHANGED)
	{
		QString bdmorphTime;
		bdmorphTime.sprintf("Current time: %s (t = %f)",
				printTime(programstate->getAnimationPosition()).c_str(),
				programstate->videoModel->pFrame->current_t);

		lblCurrentBDMORPHTime->setText(bdmorphTime);
	}

	if (flags & ProgramState::MODE_CHANGED)
	{
		bool animations_paused = programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_ANIMATION_PAUSED;

		frameOutline->setVisible(false);
		frameKVF->setVisible(false);
		frameBDMorph->setVisible(false);
		frameEdit->setVisible(false);
		frameWireframe->setVisible(false);

		frameLoad->setEnabled(!programstate->isBusy());
		btnLoadTexture->setEnabled(programstate->isModelLoaded());
		btnResetTexture->setEnabled(programstate->isModelLoaded());
		btnSaveModel->setEnabled(programstate->isModelLoaded());

		btnConvertToKeyframe->setEnabled(animations_paused);
		frameOutline->setVisible(programstate->isOutlineEditor());
		frameKVF->setVisible(programstate->isDeformationEditor());
		frameBDMorph->setVisible(programstate->isAnimations());
		frameEdit->setVisible(programstate->isEditing());
		frameWireframe->setVisible(programstate->isFullMode());
		btnEditOutline->setEnabled(programstate->isFullMode());
		btnCreateMesh->setEnabled(!programstate->isFullMode());

	}

	if (flags & (ProgramState::RENDER_SETTINGS_CHANGED|ProgramState::CURRENT_MODEL_CHANGED))
	{
		KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
		RenderSettings settings = programstate->getRenderSettings();

		if (kvfModel)  {
			sliderAlpha->setValue(settings.alpha * 1000);
			QString alphaLabel;
			alphaLabel.sprintf("Control points weight: (alpha=%f)", settings.alpha );
			lblControlPoints->setText(alphaLabel);
		}
		sliderWireframeTransparency->setValue(settings.wireframeTransparency*100);
	}
}

/******************************************************************************************************************************/
void SidePanel::onImportProject()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose base for new model (picture/outline/mesh"),
    		QString(), QLatin1String("Mesh (*.obj *.off);;Picture (*.png *.jpg);;Outline (*.poly)"));
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

	QString outputFormat;

	if (programstate->videoModel)
		outputFormat = "Video Project (*.vproject);;Mesh (*.obj)";

	else if (programstate->outlineModel)
		outputFormat="Outline (*.poly)";

    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), outputFormat);
    if ( filename == "") return;

    programstate->saveToFile(filename.toStdString());
}

/*****************************************************************************************************/
void SidePanel::onChooseTexture()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose image"), QString(), QString("Texture (*.png *.jpg *.bmp)"));
    if (filename == NULL)
		return;

    programstate->loadTexture(filename.toStdString());
}

/*****************************************************************************************************/
void SidePanel::onResetTexture()
{
	programstate->loadTexture("");
}

/*****************************************************************************************************/
void SidePanel::onMeshCreateButtonPressed()
{
	int density = sliderMeshDensity->value();
	programstate->createProjectFromOutline(density);
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
	programstate->informModelEdited();
}
/******************************************************************************************************************************/
void SidePanel::onUndoModel()
{
	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;
	model->historyUndo();
	programstate->informModelEdited();
}

/******************************************************************************************************************************/
void SidePanel::onRedoModel()
{
	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;
	model->historyRedo();
	programstate->informModelEdited();
}

/******************************************************************************************************************************/
void SidePanel::onSaveLog()
{
	if (!programstate) return;
	QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    programstate->saveLog(filename.toStdString());
}

/******************************************************************************************************************************/
void SidePanel::onRunLog()
{
	if (!programstate) return;
	QString filename = QFileDialog::getOpenFileName(this, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    programstate->runLog(filename.toStdString());
}
/******************************************************************************************************************************/

void SidePanel::onClearPins()
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;
	kvfModel->clearPins();
	programstate->informModelEdited();
}

/******************************************************************************************************************************/
void SidePanel::onReuseVF()
{
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!kvfModel) return;
	kvfModel->applyVF();
	programstate->informModelEdited();
}

/******************************************************************************************************************************/
void SidePanel::onChangeAlpha(int i)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.alpha = (((double) (i)) / 1000);
	programstate->setRenderSettings(settings);
}

/******************************************************************************************************************************/
void SidePanel::onDrawVFModeChanged(bool m)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.showVF = m;
	programstate->setRenderSettings(settings);
}

/******************************************************************************************************************************/
void SidePanel::onDrawOrigVFModeChanged(bool m)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.showVForig = m;
	programstate->setRenderSettings(settings);
}

/******************************************************************************************************************************/

void SidePanel::onShowSelectionChanged(bool m)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.showSelection = m;
	programstate->setRenderSettings(settings);
}

/******************************************************************************************************************************/
void SidePanel::onPinModeChanged(bool m)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.pinMode = m;
	programstate->setRenderSettings(settings);
}
/******************************************************************************************************************************/
void SidePanel::onChangeWireframe(int i)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.wireframeTransparency = (double)i / 100.0;
	programstate->setRenderSettings(settings);
}

/******************************************************************************************************************************/
void SidePanel::onShowBdmorphEdgeClicked(bool checked)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.showBDmorphEdge = checked;
	programstate->setRenderSettings(settings);
}


/******************************************************************************************************************************/
void SidePanel::onBdmorphOrigModel(bool checked)
{
	if (!programstate) return;
	RenderSettings settings = programstate->getRenderSettings();
	settings.showBDmorphOrigMesh = checked;
	programstate->setRenderSettings(settings);
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
void SidePanel::onBdmorphConvertToKeyframe()
{
	programstate->createKeyframeFromPFrame();
}

/******************************************************************************************************************************/
void SidePanel::onTargetFPSChanged(int newValue)
{
	RenderSettings settings = programstate->getRenderSettings();
	settings.targetFPS = newValue;
	programstate->setRenderSettings(settings);
}

/******************************************************************************************************************************/
void SidePanel::closeEvent (QCloseEvent *event)
{
	hide();
	programstate->updateGUI();
}
/******************************************************************************************************************************/
void SidePanel::onAutoOutlineCreate()
{
	programstate->autoCreateOutline();
}
