#ifndef DLG_IGN_GRID_H
#define DLG_IGN_GRID_H

#include <QDialog>
#include <QHash>
#include <QSet>
#include <QVector>

class QLabel;
class QLineEdit;
class QListWidget;
class QNetworkAccessManager;
class QNetworkReply;
class QProgressBar;
class QPushButton;
class QTimer;
class QFile;

// One 1 km LiDAR HD tile from the IGN assembly grid.
struct IgnDalle {
	QString name;
	QString url;
	qint64 points;
	double xmin;
	double ymin;
	double xmax;
	double ymax;
};

// Plan IGN basemap with the LiDAR HD 1 km grid painted on top.
class IgnGridMap : public QWidget {
	Q_OBJECT

public:
	explicit IgnGridMap(QWidget* parent = 0);

	void setCenterLonLat(double lon, double lat, int zoom);
	void setDalles(const QVector<IgnDalle>& dalles, const QSet<QString>& selected);
	void viewLambertBounds(double& xmin, double& ymin, double& xmax, double& ymax) const;
	bool viewIsDetailedEnough() const;

signals:
	void viewChanged();
	void dalleClicked(const QString& name);

protected:
	void paintEvent(QPaintEvent* event);
	void mousePressEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
	void resizeEvent(QResizeEvent* event);

private:
	double resolution() const;
	void screenToMercator(const QPointF& screen, double& mx, double& my) const;
	QPointF lambertToScreen(double x, double y) const;
	void requestVisibleTiles();
	void pumpTileQueue();

	int zoom_;
	double centerMx_;
	double centerMy_;
	bool dragging_;
	QPoint pressPos_;
	QPoint lastPos_;
	QVector<IgnDalle> dalles_;
	QSet<QString> selected_;
	QNetworkAccessManager* tiles_;
	QHash<QString, QImage> cache_;
	QStringList cacheOrder_;
	QSet<QString> inFlight_;
	QStringList pending_;
};

// Search a place, pick tiles on the IGN grid, download the .copc.laz files.
class DlgIgnGrid : public QDialog {
	Q_OBJECT

public:
	explicit DlgIgnGrid(QWidget* parent = 0);

	bool shouldLoad() const { return shouldLoad_; }
	QStringList downloadedFiles() const { return downloaded_; }

private slots:
	void searchPlace();
	void onGeocodeFinished();
	void scheduleQuery();
	void queryTiles();
	void onWfsFinished();
	void onDalleClicked(const QString& name);
	void onListItemChanged();
	void chooseFolder();
	void clearSelection();
	void onDownloadButton();
	void startDownload();
	void onDownloadReady();
	void onDownloadFinished();

private:
	void applyPlace(double lon, double lat, const QString& label);
	void refreshList();
	void refreshDownloadButton();
	void setStatus(const QString& text);
	void downloadNext();
	void finishDownload(bool propose);

	IgnGridMap* map_;
	QLineEdit* search_;
	QListWidget* list_;
	QLineEdit* folder_;
	QLabel* status_;
	QProgressBar* progress_;
	QPushButton* downloadButton_;
	QPushButton* searchButton_;
	QNetworkAccessManager* net_;
	QNetworkReply* geocodeReply_;
	QNetworkReply* wfsReply_;
	QNetworkReply* downloadReply_;
	QTimer* queryTimer_;
	QFile* downloadFile_;
	QString downloadPart_;
	QString downloadFinal_;
	QStringList downloadQueue_;
	QStringList downloaded_;
	int downloadIndex_;
	bool cancelDownload_;
	bool downloading_;
	bool shouldLoad_;
	bool updatingList_;
	QHash<QString, IgnDalle> catalog_;
	QSet<QString> selected_;
	QVector<IgnDalle> visible_;
};

#endif
