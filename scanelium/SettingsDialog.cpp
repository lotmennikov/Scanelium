#include "SettingsDialog.h"

SettingsDialog::SettingsDialog( QWidget * parent, float focal_, int snapshot_) : QDialog(parent) {

    ui.setupUi(this);

	this->focal = focal_;
	this->snapshot_rate = snapshot_;

	ui.focalLengthLabel->setText(QString::fromLocal8Bit("Фокусное расстояние"));
	ui.focalLengthEdit->setText(QString("%1").arg(focal));
	ui.freqLabel->setText(QString::fromLocal8Bit("Частота сохранения цветных изображений:  1 снимок/ %1 с").arg(snapshot_rate/1000.0));
	ui.freqSlider->setValue(snapshot_rate/100);

	connect(ui.freqSlider, &QSlider::valueChanged, this, &SettingsDialog::freqSliderChanged);
}


void SettingsDialog::accept_click() {
	bool converted = false;
	float focal_ = ui.focalLengthEdit->text().toFloat(&converted);
	if (converted && focal_ > 400.0f && focal_ < 700.0f) {
		this->focal = focal_;
		this->accept();
		this->close();
	} else {
		ui.focalLengthEdit->setFocus();
	}

}

void SettingsDialog::freqSliderChanged(int value) {
	this->snapshot_rate = value * 100;
	ui.freqLabel->setText(QString::fromLocal8Bit("Частота сохранения цветных изображений: 1 снимок/ %1 с").arg(snapshot_rate/1000.0));
}
