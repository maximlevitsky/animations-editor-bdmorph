
#include <QDockWidget>
#include "ui_SidePanel.h"
class MeshModel;

class SidePanel : public QDockWidget, public Ui_sidePanel
{
	Q_OBJECT
public:
	SidePanel(QWidget* parent);

public slots:
	void onAnimationStarted();
	void onAnimationStopped();
	void onFrameSwitched(MeshModel* model);
	void onMeshCreateButtonPressed();
signals:
	void meshCreationRequest(int requestedDensity);
};
