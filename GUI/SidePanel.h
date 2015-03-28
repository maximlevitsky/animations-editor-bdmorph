
#include <QDockWidget>
#include "ui_SidePanel.h"
#include "ProgramState.h"
class MeshModel;
class VideoModel;

class SidePanel : public QDockWidget, public Ui_sidePanel
{
	Q_OBJECT
public:
	SidePanel(QWidget* parent);
	void programStateCreated(ProgramState* state) { programstate = state; }

public slots:
	void programStateUpdated(int flags);
	void onShowHide(bool show);

	void onImportProject();
	void onLoadProject();
	void onSaveProject();
	void onChooseTexture();
	void onResetTexture();
	void onMeshCreateButtonPressed();
	void onEditOutlinePressed();
	void onAutoOutlineCreate();

	void onUndoModel();
	void onRedoModel();
	void onReuseVF();
    void onResetPoints();

	void onDrawVFModeChanged(bool m);
	void onDrawOrigVFModeChanged(bool m);
	void onPinModeChanged(bool m);
	void onShowSelectionChanged(bool m);

	void onChangeAlpha(int i);
	void onChangeWireframe(int i);

	void onClearPins();
    void onSaveLog();
    void onRunLog();
    void onResetTransform();


	void onShowBdmorphEdgeClicked(bool checked);
	void onBdmorphOrigModel(bool checked);
	void onBdmorphConvertToKeyframe();
	void onTargetFPSChanged(int newValue);

private:
	ProgramState* programstate;
	void closeEvent (QCloseEvent *event);
};
