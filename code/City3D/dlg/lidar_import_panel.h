#ifndef LIDAR_IMPORT_PANEL_H
#define LIDAR_IMPORT_PANEL_H

#include "../../model/point_set_serializer_las.h"

#include <QStringList>
#include <QWidget>
#include <vector>

class PointSet;
class QResizeEvent;

// Side panel: live import (progress, classes kept or dropped) and the reconstruction result.
class LidarImportPanel : public QWidget {
	Q_OBJECT

public:
	explicit LidarImportPanel(QWidget* parent = 0);

	void begin(const QStringList& files);
	void setSnapshot(const LasImportSnapshot& snapshot);
	void setCloud(const PointSet* cloud);
	void setPipeline(int footprints, int buildings, int roofs, int faces);
	void showStatus(const QString& status);

	QSize sizeHint() const;
	QSize minimumSizeHint() const { return sizeHint(); }

signals:
	void downloadRequested();

protected:
	void paintEvent(QPaintEvent* event);
	void resizeEvent(QResizeEvent* event);

private:
	struct ClassRow {
		int code;
		int r, g, b;
		QString label;
		unsigned long long seen;
		unsigned long long kept;
	};

	int contentHeight() const;
	void relayout();
	void placeDownloadButton();

	class QPushButton* downloadButton_;

	LasImportPhase phase_;
	QString file_name_;
	QString source_;
	QString message_;
	int file_index_;
	int file_count_;
	int percent_;
	int version_major_;
	int version_minor_;
	int point_format_;
	bool copc_;
	long long read_count_;
	long long kept_total_;
	bool has_ground_;
	double ground_z_;
	bool has_origin_;
	double origin_x_;
	double origin_y_;
	double origin_z_;
	bool has_cloud_;
	unsigned cloud_points_;
	double extent_x_;
	double extent_y_;
	double z_min_;
	double z_max_;
	int footprints_;
	int buildings_;
	int roofs_;
	int faces_;
	std::vector<ClassRow> classes_;
};

#endif
