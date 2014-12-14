
#include <QDockWidget>
#include "SidePanel.h"
#include "KVFModel.h"
#include "BDMORPH.h"
#include "OutlineModel.h"
#include "Utils.h"

SidePanel::SidePanel(QWidget* parent) : QDockWidget(parent), currentisBDMORPH(false)
{
	setupUi(this);
	connect_(btnCreateMesh, clicked(), this, onMeshCreateButtonPressed());
	sliderMeshDensity->setRange(1,100000);
	sliderMeshDensity->setTickPosition(QSlider::NoTicks);
	sliderMeshDensity->setSliderPosition(1000);
}
/******************************************************************************************************************************/

void SidePanel::onAnimationStarted()
{
	this->frameLoad->setEnabled(false);
	btnConvertToKeyframe->setEnabled(false);
}

/******************************************************************************************************************************/

void SidePanel::onAnimationStopped()
{
	this->frameLoad->setEnabled(true);
	btnConvertToKeyframe->setEnabled(true);
}

/******************************************************************************************************************************/

void SidePanel::onFrameSwitched(MeshModel* model)
{
	bool isKVFModel = dynamic_cast<KVFModel*>(model) != NULL;
	bool isBDMORPH = dynamic_cast<BDMORPHModel*>(model) != NULL;
	bool isOutlineModel = dynamic_cast<OutlineModel*>(model) != NULL;

	if (isBDMORPH && currentisBDMORPH)
		return;
	currentisBDMORPH = isBDMORPH;

	setUpdatesEnabled(false);
	frameOutline->setVisible(isOutlineModel);
	frameKVF->setVisible(isKVFModel);
	frameBDMorph->setVisible(isBDMORPH);
	frameEdit->setVisible(isOutlineModel||isKVFModel);
	frameWireframe->setVisible(model != NULL);
	setUpdatesEnabled(true);
	/* TODO */
}

/******************************************************************************************************************************/

void SidePanel::onVideoModelLoaded(VideoModel* model)
{
	videoModel = model;
}

/******************************************************************************************************************************/

void SidePanel::onMeshCreateButtonPressed()
{
	int density = sliderMeshDensity->value();
	/* TODO: think about desnsity */
	emit meshCreationRequest(density);
}
