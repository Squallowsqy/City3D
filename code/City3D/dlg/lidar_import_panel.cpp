#include "lidar_import_panel.h"

#include "../../model/point_set.h"

#include <QFileInfo>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QResizeEvent>
#include <QStringList>

#include <algorithm>


namespace {

QString grouped(unsigned long long value) {
	QString text = QString::number(value);
	for (int i = text.size() - 3; i > 0; i -= 3)
		text.insert(i, QChar(0x202F));
	return text;
}

QString meters(double value) {
	return QString::number(value, 'f', value >= 1000.0 || value <= -1000.0 ? 1 : 2);
}

}


LidarImportPanel::LidarImportPanel(QWidget* parent)
	: QWidget(parent)
	, downloadButton_(0)
	, phase_(LasPhaseReading)
	, file_index_(0)
	, file_count_(0)
	, percent_(0)
	, version_major_(0)
	, version_minor_(0)
	, point_format_(0)
	, copc_(false)
	, read_count_(0)
	, kept_total_(0)
	, has_ground_(false)
	, ground_z_(0.0)
	, has_origin_(false)
	, origin_x_(0.0)
	, origin_y_(0.0)
	, origin_z_(0.0)
	, has_cloud_(false)
	, cloud_points_(0)
	, extent_x_(0.0)
	, extent_y_(0.0)
	, z_min_(0.0)
	, z_max_(0.0)
	, footprints_(0)
	, buildings_(0)
	, roofs_(0)
	, faces_(0) {
	setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
	setMinimumWidth(340);
	downloadButton_ = new QPushButton(QString::fromUtf8("Dalles LiDAR HD…"), this);
	connect(downloadButton_, &QPushButton::clicked, this, &LidarImportPanel::downloadRequested);
	relayout();
}

void LidarImportPanel::relayout() {
	const int height = contentHeight();
	setMinimumHeight(height);
	setMaximumHeight(height);
	placeDownloadButton();
	updateGeometry();
	update();
}

void LidarImportPanel::placeDownloadButton() {
	if (!downloadButton_)
		return;
	downloadButton_->setGeometry(16, height() - 40, std::max(120, width() - 32), 28);
	downloadButton_->raise();
}

void LidarImportPanel::resizeEvent(QResizeEvent* event) {
	QWidget::resizeEvent(event);
	placeDownloadButton();
}

void LidarImportPanel::begin(const QStringList& files) {
	phase_ = LasPhaseReading;
	file_count_ = files.size();
	file_index_ = files.isEmpty() ? 0 : 1;
	file_name_ = files.isEmpty() ? QString() : QFileInfo(files.front()).fileName();
	source_.clear();
	message_ = QString::fromUtf8("Ouverture des dalles…");
	percent_ = 0;
	read_count_ = 0;
	kept_total_ = 0;
	classes_.clear();
	has_cloud_ = false;
	has_origin_ = false;
	has_ground_ = false;
	version_major_ = 0;
	version_minor_ = 0;
	point_format_ = 0;
	copc_ = false;
	buildings_ = 0;
	roofs_ = 0;
	faces_ = 0;
	footprints_ = 0;
	relayout();
}

void LidarImportPanel::setSnapshot(const LasImportSnapshot& snapshot) {
	phase_ = snapshot.phase;
	file_index_ = snapshot.file_index;
	file_count_ = snapshot.file_count;
	file_name_ = QFileInfo(QString::fromStdString(snapshot.file_name)).fileName();
	source_ = QString::fromStdString(snapshot.source);
	message_ = QString::fromStdString(snapshot.message);
	version_major_ = snapshot.version_major;
	version_minor_ = snapshot.version_minor;
	point_format_ = snapshot.point_format;
	copc_ = snapshot.copc;
	read_count_ = snapshot.read_count;
	kept_total_ = snapshot.kept_total;
	has_ground_ = snapshot.has_ground;
	ground_z_ = snapshot.ground_z;
	has_origin_ = snapshot.has_origin;
	origin_x_ = snapshot.origin_x;
	origin_y_ = snapshot.origin_y;
	origin_z_ = snapshot.origin_z;

	if (snapshot.phase == LasPhaseDone || snapshot.phase == LasPhaseFailed)
		percent_ = snapshot.phase == LasPhaseDone ? 100 : percent_;
	else if (snapshot.announced > 0)
		percent_ = std::max(0, std::min(100, static_cast<int>((100.0 * snapshot.file_read) / snapshot.announced)));
	else if (snapshot.phase == LasPhaseFileDone)
		percent_ = 100;

	classes_.clear();
	for (std::size_t i = 0; i < snapshot.classes.size(); ++i) {
		const LasClassTally& tally = snapshot.classes[i];
		ClassRow row;
		row.code = tally.code;
		row.seen = tally.seen;
		row.kept = tally.kept;
		las_class_rgb(tally.code, row.r, row.g, row.b);
		const char* label = las_class_label(tally.code);
		row.label = (label && label[0])
			? QString::fromUtf8(label)
			: QString::fromUtf8("classe %1").arg(tally.code);
		classes_.push_back(row);
	}
	std::sort(classes_.begin(), classes_.end(), [](const ClassRow& a, const ClassRow& b) {
		if (a.seen != b.seen)
			return a.seen > b.seen;
		return a.code < b.code;
	});
	if (classes_.size() > 10)
		classes_.resize(10);

	relayout();
}

void LidarImportPanel::setCloud(const PointSet* cloud) {
	has_cloud_ = cloud != 0 && cloud->num_points() > 0;
	if (!has_cloud_) {
		relayout();
		return;
	}
	cloud_points_ = cloud->num_points();
	const Box3d& box = cloud->bbox();
	extent_x_ = box.x_max() - box.x_min();
	extent_y_ = box.y_max() - box.y_min();
	z_min_ = box.z_min();
	z_max_ = box.z_max();
	has_ground_ = cloud->has_ground_z();
	if (has_ground_)
		ground_z_ = cloud->ground_z();
	has_origin_ = true;
	origin_x_ = cloud->offset().x;
	origin_y_ = cloud->offset().y;
	origin_z_ = cloud->offset().z;
	if (phase_ != LasPhaseFailed)
		phase_ = LasPhaseDone;
	percent_ = 100;
	if (kept_total_ <= 0)
		kept_total_ = static_cast<long long>(cloud_points_);
	relayout();
}

void LidarImportPanel::setPipeline(int footprints, int buildings, int roofs, int faces) {
	footprints_ = footprints;
	buildings_ = buildings;
	roofs_ = roofs;
	faces_ = faces;
	relayout();
}

void LidarImportPanel::showStatus(const QString& status) {
	message_ = status;
	update();
}

int LidarImportPanel::contentHeight() const {
	int height = 168;
	if (!classes_.empty())
		height += 28 + static_cast<int>(classes_.size()) * 26;
	if (has_cloud_ || phase_ == LasPhaseDone || phase_ == LasPhaseFailed || has_origin_)
		height += 168;
	if (file_count_ == 0 && !has_cloud_)
		height = 148;
	return height + 44;
}

QSize LidarImportPanel::sizeHint() const {
	return QSize(380, contentHeight());
}

void LidarImportPanel::paintEvent(QPaintEvent*) {
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);

	const QRect card = rect().adjusted(2, 2, -2, -2);
	painter.setPen(QColor("#e4ddd2"));
	painter.setBrush(QColor("#fbf8f3"));
	painter.drawRoundedRect(card, 12, 12);

	const int left = card.left() + 14;
	const int right = card.right() - 14;
	int y = card.top() + 16;

	QFont title = font();
	title.setPixelSize(15);
	title.setBold(true);
	painter.setFont(title);
	painter.setPen(QColor("#1c1915"));
	painter.drawText(left, y, right - left, 20, Qt::AlignLeft | Qt::AlignVCenter,
		QString::fromUtf8("Import LiDAR"));

	QFont small = font();
	small.setPixelSize(11);
	small.setBold(false);
	painter.setFont(small);
	painter.setPen(QColor("#6b645b"));
	QString meta;
	if (version_major_ > 0)
		meta = QString::fromUtf8("LAS %1.%2 · format %3").arg(version_major_).arg(version_minor_).arg(point_format_);
	if (copc_)
		meta += QString::fromUtf8(meta.isEmpty() ? "COPC" : " · COPC");
	painter.drawText(left, y, right - left, 20, Qt::AlignRight | Qt::AlignVCenter, meta);
	y += 28;

	const QStringList steps = QStringList()
		<< QString::fromUtf8("Lecture")
		<< QString::fromUtf8("Classes")
		<< QString::fromUtf8("Nuage")
		<< QString::fromUtf8("Maillage");
	const bool reading = phase_ == LasPhaseReading || phase_ == LasPhaseFileDone;
	const bool classes_on = !classes_.empty();
	const bool cloud_on = has_cloud_ || phase_ == LasPhaseDone;
	const bool mesh_on = faces_ > 0;
	const bool step_on[4] = {
		file_count_ > 0 || has_cloud_,
		classes_on || cloud_on,
		cloud_on,
		mesh_on || buildings_ > 0
	};
	const bool step_done[4] = {
		cloud_on || (phase_ == LasPhaseFileDone && file_index_ >= file_count_ && file_count_ > 0),
		cloud_on,
		cloud_on,
		mesh_on
	};
	int step_w = (right - left - 18) / 4;
	for (int i = 0; i < 4; ++i) {
		const QRect pill(left + i * (step_w + 6), y, step_w, 22);
		QColor fill = QColor("#efeae2");
		QColor ink = QColor("#6b645b");
		if (step_done[i]) {
			fill = QColor("#e7f0df");
			ink = QColor("#3f6212");
		}
		else if (step_on[i] && (i == 0 ? reading : true) && !step_done[i]) {
			fill = QColor("#f3e2d6");
			ink = QColor("#9a3412");
		}
		painter.setPen(Qt::NoPen);
		painter.setBrush(fill);
		painter.drawRoundedRect(pill, 11, 11);
		painter.setPen(ink);
		painter.setFont(small);
		painter.drawText(pill, Qt::AlignCenter, steps[i]);
	}
	y += 34;

	if (file_count_ == 0 && !has_cloud_) {
		painter.setPen(QColor("#3f3a34"));
		QFont body = font();
		body.setPixelSize(12);
		painter.setFont(body);
		const QString hint = QString::fromUtf8(
			"Ouvrez des dalles, ou téléchargez-les depuis la grille IGN.\n"
			"La lecture, les classes gardées et le nuage s'affichent ici.");
		painter.drawText(QRect(left, y, right - left, 72), Qt::AlignLeft | Qt::TextWordWrap, hint);
		return;
	}

	QFont body = font();
	body.setPixelSize(12);
	body.setBold(true);
	painter.setFont(body);
	painter.setPen(QColor("#1c1915"));
	QString headline = file_name_.isEmpty() ? QString::fromUtf8("Nuage") : file_name_;
	if (file_count_ > 1)
		headline += QString::fromUtf8("  ·  %1/%2").arg(std::max(file_index_, 1)).arg(file_count_);
	painter.drawText(QRect(left, y, right - left, 18), Qt::AlignLeft | Qt::AlignVCenter, headline);
	y += 22;

	const QRect bar(left, y, right - left, 8);
	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor("#efeae2"));
	painter.drawRoundedRect(bar, 4, 4);
	if (percent_ > 0) {
		QRect done = bar;
		done.setWidth(std::max(8, bar.width() * percent_ / 100));
		painter.setBrush(phase_ == LasPhaseFailed ? QColor("#b91c1c") : QColor("#9a3412"));
		painter.drawRoundedRect(done, 4, 4);
	}
	y += 16;

	painter.setFont(small);
	painter.setPen(phase_ == LasPhaseFailed ? QColor("#b91c1c") : QColor("#6b645b"));
	QString status = message_;
	if (status.isEmpty()) {
		status = QString::fromUtf8("%1 lus  ·  %2 gardés")
			.arg(grouped(static_cast<unsigned long long>(std::max(read_count_, 0LL))))
			.arg(grouped(static_cast<unsigned long long>(std::max(kept_total_, 0LL))));
	}
	painter.drawText(QRect(left, y, right - left, 32), Qt::AlignLeft | Qt::TextWordWrap, status);
	y += 36;

	if (!classes_.empty()) {
		painter.setPen(QColor("#1c1915"));
		QFont section = small;
		section.setBold(true);
		section.setPixelSize(11);
		painter.setFont(section);
		painter.drawText(left, y, right - left, 16, Qt::AlignLeft | Qt::AlignVCenter,
			QString::fromUtf8("Classes"));
		y += 20;

		unsigned long long max_seen = 1;
		for (std::size_t i = 0; i < classes_.size(); ++i)
			max_seen = std::max(max_seen, classes_[i].seen);

		painter.setFont(small);
		for (std::size_t i = 0; i < classes_.size(); ++i) {
			const ClassRow& row = classes_[i];
			painter.setPen(Qt::NoPen);
			painter.setBrush(QColor(row.r, row.g, row.b));
			painter.drawRoundedRect(QRect(left, y + 4, 10, 10), 2, 2);

			painter.setPen(QColor("#1c1915"));
			painter.drawText(QRect(left + 16, y, 118, 16), Qt::AlignLeft | Qt::AlignVCenter, row.label);

			const int bar_x = left + 136;
			const int bar_w = std::max(40, right - bar_x);
			painter.setBrush(QColor("#efeae2"));
			painter.setPen(Qt::NoPen);
			painter.drawRoundedRect(QRect(bar_x, y + 5, bar_w, 6), 3, 3);
			const int kept_w = static_cast<int>((bar_w * row.kept) / max_seen);
			const int seen_w = static_cast<int>((bar_w * row.seen) / max_seen);
			painter.setBrush(QColor(row.r, row.g, row.b, 70));
			painter.drawRoundedRect(QRect(bar_x, y + 5, std::max(seen_w, row.seen ? 2 : 0), 6), 3, 3);
			painter.setBrush(QColor(row.r, row.g, row.b));
			if (kept_w > 0)
				painter.drawRoundedRect(QRect(bar_x, y + 5, kept_w, 6), 3, 3);

			y += 16;
			painter.setPen(QColor("#6b645b"));
			const QString counts = row.kept == row.seen
				? grouped(row.kept)
				: QString::fromUtf8("%1 gardés · %2 lus").arg(grouped(row.kept)).arg(grouped(row.seen));
			painter.drawText(QRect(left + 16, y, right - left - 16, 12), Qt::AlignLeft | Qt::AlignVCenter, counts);
			y += 10;
		}
		y += 6;
	}

	if (!(has_cloud_ || phase_ == LasPhaseDone || has_origin_ || phase_ == LasPhaseFailed))
		return;

	painter.setPen(QColor("#1c1915"));
	QFont section = small;
	section.setBold(true);
	painter.setFont(section);
	painter.drawText(left, y, right - left, 16, Qt::AlignLeft | Qt::AlignVCenter,
		QString::fromUtf8("Résultat"));
	y += 20;

	QFont line = small;
	line.setBold(false);
	painter.setFont(line);
	const QStringList rows = QStringList()
		<< QString::fromUtf8("Points gardés")
		<< QString::fromUtf8("Origine Lambert")
		<< QString::fromUtf8("Sol (classe 2)")
		<< QString::fromUtf8("Emprise")
		<< QString::fromUtf8("Empreintes / bâtiments")
		<< QString::fromUtf8("Toits / faces");
	QStringList values;
	values << (has_cloud_ ? grouped(cloud_points_) : grouped(static_cast<unsigned long long>(std::max(kept_total_, 0LL))));
	values << (has_origin_
		? QString::fromUtf8("%1, %2").arg(meters(origin_x_)).arg(meters(origin_y_))
		: QString::fromUtf8("—"));
	values << (has_ground_
		? QString::fromUtf8("Z local %1 m").arg(QString::number(ground_z_, 'f', 2))
		: QString::fromUtf8("non vu"));
	values << (has_cloud_
		? QString::fromUtf8("%1 × %2 m").arg(meters(extent_x_)).arg(meters(extent_y_))
		: QString::fromUtf8("—"));
	values << QString::fromUtf8("%1 / %2").arg(footprints_).arg(buildings_);
	values << QString::fromUtf8("%1 / %2").arg(roofs_).arg(faces_);

	for (int i = 0; i < rows.size(); ++i) {
		painter.setPen(QColor("#6b645b"));
		painter.drawText(QRect(left, y, 150, 16), Qt::AlignLeft | Qt::AlignVCenter, rows[i]);
		painter.setPen(QColor("#1c1915"));
		painter.drawText(QRect(left + 150, y, right - left - 150, 16), Qt::AlignRight | Qt::AlignVCenter, values[i]);
		y += 18;
	}
}
