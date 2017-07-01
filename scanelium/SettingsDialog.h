#include <qdialog.h>
#include "ui_SettingsDialog.h"

class SettingsDialog : public QDialog, public Ui::Dialog {
    Q_OBJECT

public:
    SettingsDialog( QWidget * parent = 0, float focal = 535.002f, int snapshot_rate = 2000);

	float focal;
	int snapshot_rate;

public slots:
	void accept_click();

	void freqSliderChanged(int value);

private:
	Ui::Dialog ui;

};