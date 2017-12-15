#include "SettingsDialog.h"

#define MAX_FOCAL 700
#define MIN_FOCAL 400

SettingsDialog::SettingsDialog( QWidget * parent, float focal_, int snapshot_) : QDialog(parent) {

    ui.setupUi(this);

	this->focal = focal_;
	this->snapshot_rate = snapshot_;

	ui.focalLengthLabel->setText(QString::fromLocal8Bit("Фокусное расстояние"));
	ui.focalEdit->setText(QString("%1").arg(focal));
	ui.freqLabel->setText(QString::fromLocal8Bit("Частота сохранения цветных изображений:  1 снимок/ %1 с").arg(snapshot_rate/1000.0));
	ui.freqSlider->setValue(snapshot_rate/100.0f);
	ui.focalSlider->setValue(focal_);

	connect(ui.freqSlider, &QSlider::valueChanged, this, &SettingsDialog::freqSliderChanged);
	connect(ui.focalEdit, &QLineEdit::textChanged, this, &SettingsDialog::focalEditChanged);
	connect(ui.focalSlider, &QSlider::valueChanged, this, &SettingsDialog::focalSliderChanged);
}

void SettingsDialog::focalEditChanged(QString text) {
	bool ok = false;
	float foc = text.toFloat(&ok);
	if (ok && foc >= MIN_FOCAL && foc <= MAX_FOCAL) {
		this->focal = foc;
		ui.focalSlider->setValue(foc);

		emit focalChanged(foc, foc);
	}
}

void SettingsDialog::focalSliderChanged(int value) {
	this->focal = (float)value;
	ui.focalEdit->setText(QString("%1").arg(value));

	emit focalChanged(this->focal, this->focal);
}


void SettingsDialog::accept_click() {
	bool converted = false;
	float focal_ = ui.focalEdit->text().toFloat(&converted);
	if (converted && focal_ >= MIN_FOCAL  && focal_ <= MAX_FOCAL) {
		this->focal = focal_;
		this->accept();
		this->close();
	} else {
		ui.focalEdit->setFocus();
	}
}

void SettingsDialog::freqSliderChanged(int value) {
	this->snapshot_rate = value * 100;

	if (this->snapshot_rate > 0) {
		ui.freqLabel->setText(QString::fromLocal8Bit("Частота сохранения цветных изображений: 1 снимок/ %1 с").arg(snapshot_rate / 1000.0));
	}
	else {
		ui.freqLabel->setText(QString::fromLocal8Bit("Частота сохранения цветных изображений: все подряд"));
	}
	emit snapshotChanged(this->snapshot_rate);
}
