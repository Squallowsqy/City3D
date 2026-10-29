#include "dlg_lidar_hd.h"

#include "../../model/point_set_serializer_las.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>


bool ask_lidar_load_options(QWidget* parent, const QStringList& files, LasLoadOptions& options) {
	bool ign = false;
	for (int i = 0; i < files.size(); ++i) {
		if (LasLoadOptions::file_looks_like_ign_lidar_hd(files[i].toStdString()))
			ign = true;
	}

	QDialog dialog(parent);
	dialog.setWindowTitle(QString::fromUtf8("Charger un nuage LiDAR"));
	dialog.setMinimumWidth(520);

	QVBoxLayout* layout = new QVBoxLayout(&dialog);

	QString intro = QString::fromUtf8(
		"Les dalles LiDAR HD de l'IGN sont des fichiers LAS 1.4 (souvent .copc.laz), "
		"en Lambert-93, à environ 10 points/m². Une dalle d'1 km contient en général "
		"10 à 20 millions de points.\n\n"
		"Pour reconstruire les bâtiments, gardez la classe 6. Le sol (classe 2) est "
		"tout de même lu pour poser les empreintes à la bonne altitude, sans le stocker. "
		"Chargez ensuite un GeoJSON d'empreintes dans le même système (Lambert-93, "
		"EPSG:2154) : BD TOPO ou cadastre. Plusieurs dalles peuvent être ouvertes ensemble.");
	if (files.size() > 1)
		intro += QString::fromUtf8("\n\n%1 fichiers sélectionnés.").arg(files.size());
	else if (!files.isEmpty())
		intro += QString::fromUtf8("\n\nFichier : %1").arg(files.front());

	QLabel* intro_label = new QLabel(intro, &dialog);
	intro_label->setWordWrap(true);
	layout->addWidget(intro_label);

	QComboBox* preset = new QComboBox(&dialog);
	preset->addItem(QString::fromUtf8("LiDAR HD — bâtiments (classe 6), recommandé"), 0);
	preset->addItem(QString::fromUtf8("LiDAR HD — bâtiments et sol (classes 6 et 2)"), 1);
	preset->addItem(QString::fromUtf8("Toutes les classes, sans points virtuels (66)"), 2);
	preset->addItem(QString::fromUtf8("Fichier brut, aucun filtre"), 3);
	preset->setCurrentIndex(ign ? 0 : 3);

	QDoubleSpinBox* voxel = new QDoubleSpinBox(&dialog);
	voxel->setRange(0.0, 5.0);
	voxel->setDecimals(2);
	voxel->setSingleStep(0.05);
	voxel->setSuffix(QString::fromUtf8(" m"));
	voxel->setSpecialValueText(QString::fromUtf8("aucun"));
	voxel->setValue(0.0);
	voxel->setToolTip(QString::fromUtf8(
		"Garde un point par voxel. 0,30 m allège une dalle dense sans effacer les toits. "
		"0 conserve tous les points."));

	QCheckBox* overlap = new QCheckBox(
		QString::fromUtf8("Ignorer les points marqués en recouvrement"), &dialog);
	overlap->setChecked(files.size() > 1);
	overlap->setToolTip(QString::fromUtf8(
		"Utile quand plusieurs dalles se chevauchent et que le bit overlap du LAS est renseigné."));

	QFormLayout* form = new QFormLayout;
	form->addRow(QString::fromUtf8("Contenu"), preset);
	form->addRow(QString::fromUtf8("Échantillonnage"), voxel);
	form->addRow(QString(), overlap);
	layout->addLayout(form);

	QLabel* hint = new QLabel(QString::fromUtf8(
		"Les bâtiments sont dessinés en rouge, le sol en brun, la végétation en vert. "
		"Si un .copc.laz refuse de s'ouvrir, convertissez-le : "
		"pdal translate dalle.copc.laz dalle.laz"), &dialog);
	hint->setWordWrap(true);
	layout->addWidget(hint);

	QDialogButtonBox* buttons = new QDialogButtonBox(
		QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
	QObject::connect(buttons, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(buttons, SIGNAL(rejected()), &dialog, SLOT(reject()));
	layout->addWidget(buttons);

	if (dialog.exec() != QDialog::Accepted)
		return false;

	switch (preset->currentData().toInt()) {
	case 0: options = LasLoadOptions::ign_buildings(); break;
	case 1: options = LasLoadOptions::ign_buildings_and_ground(); break;
	case 2: options = LasLoadOptions::ign_all_classes(); break;
	default: options = LasLoadOptions::legacy(); break;
	}
	// Class 2 is only stored when the preset asks for it, but its altitude is always kept.
	options.record_ground = true;
	options.voxel_size = voxel->value();
	options.drop_overlap = overlap->isChecked();
	if (ign && preset->currentData().toInt() == 3)
		options.color_by_classification = true;
	return true;
}
