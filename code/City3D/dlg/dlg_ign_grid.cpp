#include "dlg_ign_grid.h"

#include <QAction>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QStandardPaths>
#include <QTimer>
#include <QUrlQuery>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace {

const double kPi = 3.14159265358979323846;
const double kWebMercatorOrigin = 20037508.342789244;
const double kLambertN = 0.7256077650532670;
const double kLambertC = 11754255.426096;
const double kLambertXs = 700000.0;
const double kLambertYs = 12655612.049876;
const double kLambertLon0 = 3.0 * kPi / 180.0;
const double kGrs80E = 0.0818191910428158;
const int kMaxSelection = 12;
const int kWarnSelection = 4;
const char* kWfs = "https://data.geopf.fr/wfs/ows";
const char* kGeocode = "https://data.geopf.fr/geocodage/search/";

double isometricLatitude(double phi) {
	const double s = std::sin(phi);
	return std::log(std::tan(kPi / 4.0 + phi / 2.0) * std::pow((1.0 - kGrs80E * s) / (1.0 + kGrs80E * s), kGrs80E / 2.0));
}

void lambertToLonLat(double x, double y, double& lonDeg, double& latDeg) {
	const double radius = std::hypot(x - kLambertXs, y - kLambertYs);
	const double gamma = std::atan2(x - kLambertXs, kLambertYs - y);
	const double lon = kLambertLon0 + gamma / kLambertN;
	const double latIso = -std::log(std::abs(radius / kLambertC)) / kLambertN;
	double phi = 2.0 * std::atan(std::exp(latIso)) - kPi / 2.0;
	for (int i = 0; i < 8; ++i) {
		const double s = std::sin(phi);
		phi = 2.0 * std::atan(std::pow((1.0 + kGrs80E * s) / (1.0 - kGrs80E * s), kGrs80E / 2.0) * std::exp(latIso)) - kPi / 2.0;
	}
	lonDeg = lon * 180.0 / kPi;
	latDeg = phi * 180.0 / kPi;
}

void lonLatToLambert(double lonDeg, double latDeg, double& x, double& y) {
	const double phi = latDeg * kPi / 180.0;
	const double lam = lonDeg * kPi / 180.0;
	const double radius = kLambertC * std::exp(-kLambertN * isometricLatitude(phi));
	const double gamma = kLambertN * (lam - kLambertLon0);
	x = kLambertXs + radius * std::sin(gamma);
	y = kLambertYs - radius * std::cos(gamma);
}

void lonLatToMercator(double lonDeg, double latDeg, double& mx, double& my) {
	const double lat = std::max(-85.0, std::min(85.0, latDeg)) * kPi / 180.0;
	mx = lonDeg * kPi / 180.0 * 6378137.0;
	my = std::log(std::tan(kPi / 4.0 + lat / 2.0)) * 6378137.0;
}

void mercatorToLonLat(double mx, double my, double& lonDeg, double& latDeg) {
	lonDeg = (mx / 6378137.0) * 180.0 / kPi;
	latDeg = (2.0 * std::atan(std::exp(my / 6378137.0)) - kPi / 2.0) * 180.0 / kPi;
}

QString tileLabel(const QString& name) {
	const QStringList parts = name.split('_');
	if (parts.size() >= 4)
		return parts[2] + QString::fromUtf8(" × ") + parts[3];
	return name;
}

QString millions(qint64 points) {
	if (points <= 0)
		return QString::fromUtf8("taille inconnue");
	return QString::fromUtf8("%1 M pts").arg(points / 1000000.0, 0, 'f', 1);
}

bool ignCloudUrl(const QString& text, QUrl* url) {
	const QUrl parsed(text);
	if (!parsed.isValid() || parsed.scheme() != QLatin1String("https"))
		return false;
	if (parsed.host().compare(QLatin1String("data.geopf.fr"), Qt::CaseInsensitive) != 0)
		return false;
	const QString path = parsed.path().toLower();
	if (!path.endsWith(QLatin1String(".laz")) && !path.endsWith(QLatin1String(".las")))
		return false;
	*url = parsed;
	return true;
}

QString safeFileName(const QUrl& url) {
	QString name = QFileInfo(url.path()).fileName();
	if (name.contains(QLatin1String("..")) || name.contains('/') || name.contains('\\') || name.isEmpty())
		return QString();
	return name;
}

void firstRing(const QJsonValue& value, QVector<double>& xs, QVector<double>& ys) {
	if (!value.isArray())
		return;
	const QJsonArray array = value.toArray();
	if (array.isEmpty() || !array.at(0).isArray())
		return;
	const QJsonArray head = array.at(0).toArray();
	if (head.size() >= 2 && !head.at(0).isArray()) {
		for (int i = 0; i < array.size(); ++i) {
			const QJsonArray point = array.at(i).toArray();
			if (point.size() >= 2) {
				xs.push_back(point.at(0).toDouble());
				ys.push_back(point.at(1).toDouble());
			}
		}
		return;
	}
	firstRing(array.at(0), xs, ys);
}

QNetworkRequest ignRequest(const QUrl& url) {
	QNetworkRequest request(url);
	request.setHeader(QNetworkRequest::UserAgentHeader, QString::fromUtf8("City3D"));
	request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
	request.setTransferTimeout(300000);
	return request;
}

}


IgnGridMap::IgnGridMap(QWidget* parent)
	: QWidget(parent)
	, zoom_(6)
	, centerMx_(0)
	, centerMy_(0)
	, dragging_(false)
	, tiles_(new QNetworkAccessManager(this)) {
	setMinimumSize(480, 380);
	setMouseTracking(false);
	setCursor(Qt::CrossCursor);
	double mx = 0;
	double my = 0;
	lonLatToMercator(2.5, 46.6, mx, my);
	centerMx_ = mx;
	centerMy_ = my;
	setAutoFillBackground(true);
}

void IgnGridMap::setCenterLonLat(double lon, double lat, int zoom) {
	lonLatToMercator(lon, lat, centerMx_, centerMy_);
	zoom_ = std::max(5, std::min(18, zoom));
	update();
	emit viewChanged();
}

void IgnGridMap::setDalles(const QVector<IgnDalle>& dalles, const QSet<QString>& selected) {
	dalles_ = dalles;
	selected_ = selected;
	update();
}

double IgnGridMap::resolution() const {
	return (2.0 * kWebMercatorOrigin) / (256.0 * std::pow(2.0, zoom_));
}

void IgnGridMap::screenToMercator(const QPointF& screen, double& mx, double& my) const {
	const double res = resolution();
	mx = centerMx_ + (screen.x() - width() / 2.0) * res;
	my = centerMy_ - (screen.y() - height() / 2.0) * res;
}

QPointF IgnGridMap::lambertToScreen(double x, double y) const {
	double lon = 0;
	double lat = 0;
	double mx = 0;
	double my = 0;
	lambertToLonLat(x, y, lon, lat);
	lonLatToMercator(lon, lat, mx, my);
	const double res = resolution();
	return QPointF((mx - centerMx_) / res + width() / 2.0, (centerMy_ - my) / res + height() / 2.0);
}

void IgnGridMap::viewLambertBounds(double& xmin, double& ymin, double& xmax, double& ymax) const {
	const QPointF corners[4] = {
		QPointF(0, 0), QPointF(width(), 0), QPointF(0, height()), QPointF(width(), height())
	};
	bool any = false;
	xmin = ymin = xmax = ymax = 0;
	for (int i = 0; i < 4; ++i) {
		double mx = 0;
		double my = 0;
		double lon = 0;
		double lat = 0;
		double x = 0;
		double y = 0;
		screenToMercator(corners[i], mx, my);
		mercatorToLonLat(mx, my, lon, lat);
		lonLatToLambert(lon, lat, x, y);
		if (!any) {
			xmin = xmax = x;
			ymin = ymax = y;
			any = true;
		}
		else {
			xmin = std::min(xmin, x);
			xmax = std::max(xmax, x);
			ymin = std::min(ymin, y);
			ymax = std::max(ymax, y);
		}
	}
}

bool IgnGridMap::viewIsDetailedEnough() const {
	double xmin, ymin, xmax, ymax;
	viewLambertBounds(xmin, ymin, xmax, ymax);
	return (xmax - xmin) < 14000.0 && (ymax - ymin) < 14000.0;
}

void IgnGridMap::requestVisibleTiles() {
	if (width() < 2 || height() < 2)
		return;
	const double res = resolution();
	const double tileMeters = res * 256.0;
	const double left = centerMx_ - (width() / 2.0) * res;
	const double top = centerMy_ + (height() / 2.0) * res;
	const int limit = 1 << zoom_;
	const int col0 = std::max(0, (int)std::floor((left + kWebMercatorOrigin) / tileMeters));
	const int col1 = std::min(limit - 1, (int)std::floor((left + width() * res + kWebMercatorOrigin) / tileMeters));
	const int row0 = std::max(0, (int)std::floor((kWebMercatorOrigin - top) / tileMeters));
	const int row1 = std::min(limit - 1, (int)std::floor((kWebMercatorOrigin - (top - height() * res)) / tileMeters));
	for (int row = row0; row <= row1; ++row) {
		for (int col = col0; col <= col1; ++col) {
			const QString key = QString::number(zoom_) + "/" + QString::number(col) + "/" + QString::number(row);
			if (cache_.contains(key) || inFlight_.contains(key) || pending_.contains(key))
				continue;
			pending_.append(key);
		}
	}
	pumpTileQueue();
}

void IgnGridMap::pumpTileQueue() {
	while (inFlight_.size() < 6 && !pending_.isEmpty()) {
		const QString key = pending_.takeFirst();
		if (cache_.contains(key) || inFlight_.contains(key))
			continue;
		const QStringList parts = key.split('/');
		if (parts.size() != 3)
			continue;
		QUrlQuery query;
		query.addQueryItem("SERVICE", "WMTS");
		query.addQueryItem("REQUEST", "GetTile");
		query.addQueryItem("VERSION", "1.0.0");
		query.addQueryItem("LAYER", "GEOGRAPHICALGRIDSYSTEMS.PLANIGNV2");
		query.addQueryItem("STYLE", "normal");
		query.addQueryItem("TILEMATRIXSET", "PM");
		query.addQueryItem("TILEMATRIX", parts[0]);
		query.addQueryItem("TILEROW", parts[2]);
		query.addQueryItem("TILECOL", parts[1]);
		query.addQueryItem("FORMAT", "image/png");
		QUrl url(QString::fromUtf8("https://data.geopf.fr/wmts"));
		url.setQuery(query);
		QNetworkReply* reply = tiles_->get(ignRequest(url));
		reply->setProperty("tileKey", key);
		inFlight_.insert(key);
		connect(reply, &QNetworkReply::finished, this, [this, reply]() {
			const QString done = reply->property("tileKey").toString();
			inFlight_.remove(done);
			if (reply->error() == QNetworkReply::NoError) {
				QImage image;
				image.loadFromData(reply->readAll());
				if (!image.isNull()) {
					cache_.insert(done, image);
					cacheOrder_.append(done);
					while (cacheOrder_.size() > 140) {
						cache_.remove(cacheOrder_.takeFirst());
					}
					update();
				}
			}
			reply->deleteLater();
			pumpTileQueue();
		});
	}
}

void IgnGridMap::paintEvent(QPaintEvent*) {
	QPainter painter(this);
	painter.fillRect(rect(), QColor("#d9d3c8"));
	requestVisibleTiles();

	const double res = resolution();
	const double tileMeters = res * 256.0;
	const double left = centerMx_ - (width() / 2.0) * res;
	const double top = centerMy_ + (height() / 2.0) * res;
	const int limit = 1 << std::min(zoom_, 22);
	const int col0 = std::max(0, (int)std::floor((left + kWebMercatorOrigin) / tileMeters) - 1);
	const int col1 = std::min(limit - 1, (int)std::floor((left + width() * res + kWebMercatorOrigin) / tileMeters) + 1);
	const int row0 = std::max(0, (int)std::floor((kWebMercatorOrigin - top) / tileMeters) - 1);
	const int row1 = std::min(limit - 1, (int)std::floor((kWebMercatorOrigin - (top - height() * res)) / tileMeters) + 1);
	for (int row = row0; row <= row1; ++row) {
		for (int col = col0; col <= col1; ++col) {
			const QString key = QString::number(zoom_) + "/" + QString::number(col) + "/" + QString::number(row);
			const QHash<QString, QImage>::const_iterator it = cache_.constFind(key);
			if (it == cache_.constEnd())
				continue;
			const double sx = (col * tileMeters - kWebMercatorOrigin - left) / res;
			const double sy = (top - (kWebMercatorOrigin - row * tileMeters)) / res;
			painter.drawImage(QRectF(sx, sy, 256.0, 256.0), it.value());
		}
	}

	QFont label = font();
	label.setPixelSize(11);
	label.setBold(true);
	painter.setFont(label);
	for (int i = 0; i < dalles_.size(); ++i) {
		const IgnDalle& dalle = dalles_[i];
		const QPointF a = lambertToScreen(dalle.xmin, dalle.ymax);
		const QPointF b = lambertToScreen(dalle.xmax, dalle.ymax);
		const QPointF c = lambertToScreen(dalle.xmax, dalle.ymin);
		const QPointF d = lambertToScreen(dalle.xmin, dalle.ymin);
		QPolygonF polygon;
		polygon << a << b << c << d;
		const bool on = selected_.contains(dalle.name);
		painter.setPen(QPen(on ? QColor("#9a3412") : QColor("#1d4ed8"), on ? 2.0 : 1.2));
		painter.setBrush(on ? QColor(194, 65, 12, 80) : QColor(37, 99, 235, 36));
		painter.drawPolygon(polygon);
		if (zoom_ >= 14) {
			painter.setPen(QColor("#1c1915"));
			painter.drawText(polygon.boundingRect(), Qt::AlignCenter, tileLabel(dalle.name));
		}
	}
}

void IgnGridMap::mousePressEvent(QMouseEvent* event) {
	if (event->button() != Qt::LeftButton)
		return;
	dragging_ = true;
	pressPos_ = event->pos();
	lastPos_ = event->pos();
	setCursor(Qt::ClosedHandCursor);
}

void IgnGridMap::mouseMoveEvent(QMouseEvent* event) {
	if (!dragging_)
		return;
	const QPoint delta = event->pos() - lastPos_;
	lastPos_ = event->pos();
	const double res = resolution();
	centerMx_ -= delta.x() * res;
	centerMy_ += delta.y() * res;
	update();
}

void IgnGridMap::mouseReleaseEvent(QMouseEvent* event) {
	if (!dragging_ || event->button() != Qt::LeftButton)
		return;
	dragging_ = false;
	setCursor(Qt::CrossCursor);
	if ((event->pos() - pressPos_).manhattanLength() <= 4) {
		QString hit;
		for (int i = dalles_.size() - 1; i >= 0; --i) {
			const IgnDalle& dalle = dalles_[i];
			QPolygonF polygon;
			polygon << lambertToScreen(dalle.xmin, dalle.ymax)
				<< lambertToScreen(dalle.xmax, dalle.ymax)
				<< lambertToScreen(dalle.xmax, dalle.ymin)
				<< lambertToScreen(dalle.xmin, dalle.ymin);
			if (polygon.containsPoint(event->pos(), Qt::OddEvenFill)) {
				hit = dalle.name;
				break;
			}
		}
		if (!hit.isEmpty())
			emit dalleClicked(hit);
	}
	else {
		emit viewChanged();
	}
}

void IgnGridMap::wheelEvent(QWheelEvent* event) {
	const int steps = event->angleDelta().y() / 120;
	if (steps == 0)
		return;
	const int next = std::max(5, std::min(18, zoom_ + (steps > 0 ? 1 : -1)));
	if (next == zoom_)
		return;
	double mx = 0;
	double my = 0;
	screenToMercator(event->position(), mx, my);
	const double oldRes = resolution();
	zoom_ = next;
	const double newRes = resolution();
	centerMx_ = mx - (event->position().x() - width() / 2.0) * newRes;
	centerMy_ = my + (event->position().y() - height() / 2.0) * newRes;
	Q_UNUSED(oldRes);
	update();
	emit viewChanged();
}

void IgnGridMap::resizeEvent(QResizeEvent* event) {
	QWidget::resizeEvent(event);
	emit viewChanged();
}


DlgIgnGrid::DlgIgnGrid(QWidget* parent)
	: QDialog(parent)
	, map_(0)
	, search_(0)
	, list_(0)
	, folder_(0)
	, status_(0)
	, progress_(0)
	, downloadButton_(0)
	, searchButton_(0)
	, net_(new QNetworkAccessManager(this))
	, geocodeReply_(0)
	, wfsReply_(0)
	, downloadReply_(0)
	, queryTimer_(new QTimer(this))
	, downloadFile_(0)
	, downloadIndex_(0)
	, cancelDownload_(false)
	, downloading_(false)
	, shouldLoad_(false)
	, updatingList_(false) {
	setWindowTitle(QString::fromUtf8("Dalles LiDAR HD — grille IGN"));
	resize(980, 680);

	QLabel* intro = new QLabel(QString::fromUtf8(
		"Même découpage que le <a href=\"https://cartes.gouv.fr/telechargement/IGNF_NUAGES-DE-POINTS-LIDAR-HD\">téléchargement à la carte IGN</a> : "
		"dalles de 1 km, nuages .copc.laz. Cherchez une commune, cliquez les dalles, puis téléchargez. "
		"Chaque dalle pèse souvent 150 à 250 Mo."), this);
	intro->setTextFormat(Qt::RichText);
	intro->setOpenExternalLinks(true);
	intro->setWordWrap(true);

	search_ = new QLineEdit(this);
	search_->setPlaceholderText(QString::fromUtf8("Commune, adresse…"));
	searchButton_ = new QPushButton(QString::fromUtf8("Chercher"), this);
	connect(searchButton_, &QPushButton::clicked, this, &DlgIgnGrid::searchPlace);
	connect(search_, &QLineEdit::returnPressed, this, &DlgIgnGrid::searchPlace);

	map_ = new IgnGridMap(this);
	connect(map_, &IgnGridMap::viewChanged, this, &DlgIgnGrid::scheduleQuery);
	connect(map_, &IgnGridMap::dalleClicked, this, &DlgIgnGrid::onDalleClicked);

	list_ = new QListWidget(this);
	list_->setMinimumWidth(250);
	connect(list_, &QListWidget::itemChanged, this, &DlgIgnGrid::onListItemChanged);

	QPushButton* clearButton = new QPushButton(QString::fromUtf8("Effacer la sélection"), this);
	connect(clearButton, &QPushButton::clicked, this, &DlgIgnGrid::clearSelection);

	QSettings settings("LiangliangNan", "City3D");
	QString folder = settings.value("lidarHdDirectory").toString();
	if (folder.isEmpty()) {
		folder = QStandardPaths::writableLocation(QStandardPaths::DownloadLocation);
		if (folder.isEmpty())
			folder = QDir::homePath();
		folder += "/LiDAR-HD";
	}
	folder_ = new QLineEdit(folder, this);
	QPushButton* folderButton = new QPushButton(QString::fromUtf8("Dossier…"), this);
	connect(folderButton, &QPushButton::clicked, this, &DlgIgnGrid::chooseFolder);

	status_ = new QLabel(QString::fromUtf8("Zoomez ou cherchez un lieu pour afficher les dalles."), this);
	status_->setWordWrap(true);
	progress_ = new QProgressBar(this);
	progress_->setVisible(false);
	progress_->setRange(0, 100);

	downloadButton_ = new QPushButton(QString::fromUtf8("Télécharger"), this);
	downloadButton_->setEnabled(false);
	connect(downloadButton_, &QPushButton::clicked, this, &DlgIgnGrid::onDownloadButton);
	QPushButton* closeButton = new QPushButton(QString::fromUtf8("Fermer"), this);
	connect(closeButton, &QPushButton::clicked, this, &QDialog::reject);

	QHBoxLayout* searchRow = new QHBoxLayout();
	searchRow->addWidget(search_, 1);
	searchRow->addWidget(searchButton_);

	QVBoxLayout* side = new QVBoxLayout();
	side->addWidget(list_, 1);
	side->addWidget(clearButton);
	QHBoxLayout* folderRow = new QHBoxLayout();
	folderRow->addWidget(folder_, 1);
	folderRow->addWidget(folderButton);
	side->addLayout(folderRow);
	side->addWidget(status_);
	side->addWidget(progress_);
	QHBoxLayout* actions = new QHBoxLayout();
	actions->addWidget(downloadButton_, 1);
	actions->addWidget(closeButton);
	side->addLayout(actions);

	QHBoxLayout* body = new QHBoxLayout();
	body->addWidget(map_, 1);
	body->addLayout(side);

	QVBoxLayout* root = new QVBoxLayout(this);
	root->addWidget(intro);
	root->addLayout(searchRow);
	root->addLayout(body, 1);

	queryTimer_->setSingleShot(true);
	queryTimer_->setInterval(700);
	connect(queryTimer_, &QTimer::timeout, this, &DlgIgnGrid::queryTiles);
}

void DlgIgnGrid::setStatus(const QString& text) {
	status_->setText(text);
}

void DlgIgnGrid::scheduleQuery() {
	queryTimer_->start();
}

void DlgIgnGrid::searchPlace() {
	const QString text = search_->text().trimmed();
	if (text.isEmpty() || geocodeReply_)
		return;
	QUrlQuery query;
	query.addQueryItem("q", text);
	query.addQueryItem("limit", "6");
	QUrl url(QString::fromLatin1(kGeocode));
	url.setQuery(query);
	searchButton_->setEnabled(false);
	setStatus(QString::fromUtf8("Recherche de « %1 »…").arg(text));
	geocodeReply_ = net_->get(ignRequest(url));
	connect(geocodeReply_, &QNetworkReply::finished, this, &DlgIgnGrid::onGeocodeFinished);
}

void DlgIgnGrid::onGeocodeFinished() {
	QNetworkReply* reply = geocodeReply_;
	geocodeReply_ = 0;
	searchButton_->setEnabled(true);
	if (!reply)
		return;
	const QByteArray body = reply->readAll();
	const bool failed = reply->error() != QNetworkReply::NoError;
	const QString error = reply->errorString();
	reply->deleteLater();
	if (failed) {
		setStatus(QString::fromUtf8("Géocodage impossible : %1").arg(error));
		return;
	}
	const QJsonArray features = QJsonDocument::fromJson(body).object().value("features").toArray();
	if (features.isEmpty()) {
		setStatus(QString::fromUtf8("Aucun lieu trouvé."));
		return;
	}
	int index = 0;
	if (features.size() > 1) {
		QMenu menu(this);
		for (int i = 0; i < features.size(); ++i) {
			const QString label = features.at(i).toObject().value("properties").toObject().value("label").toString();
			QAction* action = menu.addAction(label.isEmpty() ? QString::fromUtf8("Résultat %1").arg(i + 1) : label);
			action->setData(i);
		}
		QAction* chosen = menu.exec(search_->mapToGlobal(QPoint(0, search_->height())));
		if (!chosen)
			return;
		index = chosen->data().toInt();
	}
	const QJsonObject feature = features.at(index).toObject();
	const QJsonArray coordinates = feature.value("geometry").toObject().value("coordinates").toArray();
	if (coordinates.size() < 2) {
		setStatus(QString::fromUtf8("Réponse de géocodage illisible."));
		return;
	}
	const QString label = feature.value("properties").toObject().value("label").toString();
	applyPlace(coordinates.at(0).toDouble(), coordinates.at(1).toDouble(), label);
}

void DlgIgnGrid::applyPlace(double lon, double lat, const QString& label) {
	map_->setCenterLonLat(lon, lat, 14);
	setStatus(label.isEmpty()
		? QString::fromUtf8("Chargement des dalles…")
		: QString::fromUtf8("%1 — chargement des dalles…").arg(label));
}

void DlgIgnGrid::queryTiles() {
	if (wfsReply_ || downloadReply_ || downloading_) {
		queryTimer_->start();
		return;
	}
	if (!map_->viewIsDetailedEnough()) {
		visible_.clear();
		map_->setDalles(visible_, selected_);
		refreshList();
		setStatus(QString::fromUtf8("Zoomez pour afficher la grille (moins de 14 km de côté)."));
		return;
	}
	double xmin, ymin, xmax, ymax;
	map_->viewLambertBounds(xmin, ymin, xmax, ymax);
	xmin -= 200.0;
	ymin -= 200.0;
	xmax += 200.0;
	ymax += 200.0;
	QUrlQuery query;
	query.addQueryItem("SERVICE", "WFS");
	query.addQueryItem("VERSION", "2.0.0");
	query.addQueryItem("REQUEST", "GetFeature");
	query.addQueryItem("TYPENAMES", "IGNF_LIDAR-HD_METADONNEE:metadata");
	query.addQueryItem("SRSNAME", "EPSG:2154");
	query.addQueryItem("OUTPUTFORMAT", "application/json");
	query.addQueryItem("COUNT", "80");
	query.addQueryItem("BBOX", QString::number(xmin, 'f', 1) + "," + QString::number(ymin, 'f', 1) + ","
		+ QString::number(xmax, 'f', 1) + "," + QString::number(ymax, 'f', 1) + ",EPSG:2154");
	QUrl url(QString::fromLatin1(kWfs));
	url.setQuery(query);
	wfsReply_ = net_->get(ignRequest(url));
	connect(wfsReply_, &QNetworkReply::finished, this, &DlgIgnGrid::onWfsFinished);
}

void DlgIgnGrid::onWfsFinished() {
	QNetworkReply* reply = wfsReply_;
	wfsReply_ = 0;
	if (!reply)
		return;
	const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
	const QByteArray body = reply->readAll();
	const bool failed = reply->error() != QNetworkReply::NoError;
	const QString error = reply->errorString();
	reply->deleteLater();
	if (failed) {
		setStatus(status == 429
			? QString::fromUtf8("L'IGN limite le rythme des requêtes. Patientez une seconde, puis déplacez la carte.")
			: QString::fromUtf8("Grille indisponible : %1").arg(error));
		if (queryTimer_->isActive())
			queryTimer_->start();
		return;
	}
	const QJsonArray features = QJsonDocument::fromJson(body).object().value("features").toArray();
	visible_.clear();
	for (int i = 0; i < features.size(); ++i) {
		const QJsonObject feature = features.at(i).toObject();
		const QJsonObject properties = feature.value("properties").toObject();
		QUrl cloud;
		if (!ignCloudUrl(properties.value("url_npl").toString(), &cloud))
			continue;
		const QString name = safeFileName(cloud);
		if (name.isEmpty())
			continue;
		QVector<double> xs;
		QVector<double> ys;
		firstRing(feature.value("geometry").toObject().value("coordinates"), xs, ys);
		if (xs.isEmpty())
			continue;
		IgnDalle dalle;
		dalle.name = name;
		dalle.url = cloud.toString();
		dalle.points = properties.value("nombre_points").toVariant().toLongLong();
		dalle.xmin = *std::min_element(xs.begin(), xs.end());
		dalle.xmax = *std::max_element(xs.begin(), xs.end());
		dalle.ymin = *std::min_element(ys.begin(), ys.end());
		dalle.ymax = *std::max_element(ys.begin(), ys.end());
		catalog_.insert(name, dalle);
		visible_.push_back(dalle);
	}
	std::sort(visible_.begin(), visible_.end(), [](const IgnDalle& a, const IgnDalle& b) {
		return a.name < b.name;
	});
	map_->setDalles(visible_, selected_);
	refreshList();
	if (visible_.isEmpty())
		setStatus(QString::fromUtf8("Aucune dalle LiDAR HD publiée sur cette emprise."));
	else if (features.size() >= 80)
		setStatus(QString::fromUtf8("%1 dalles affichées. Zoomez si la liste est incomplète.").arg(visible_.size()));
	else
		setStatus(QString::fromUtf8("%1 dalles dans la vue. Cliquez pour les sélectionner.").arg(visible_.size()));
	if (queryTimer_->isActive())
		queryTimer_->start();
}

void DlgIgnGrid::onDalleClicked(const QString& name) {
	if (selected_.contains(name))
		selected_.remove(name);
	else if (selected_.size() >= kMaxSelection) {
		QMessageBox::information(this, windowTitle(),
			QString::fromUtf8("Douze dalles au maximum en une fois."));
		return;
	}
	else
		selected_.insert(name);
	map_->setDalles(visible_, selected_);
	refreshList();
}

void DlgIgnGrid::onListItemChanged() {
	if (updatingList_)
		return;
	QSet<QString> next;
	for (int i = 0; i < list_->count(); ++i) {
		QListWidgetItem* item = list_->item(i);
		if (item->checkState() == Qt::Checked)
			next.insert(item->data(Qt::UserRole).toString());
	}
	if (next.size() > kMaxSelection) {
		QMessageBox::information(this, windowTitle(),
			QString::fromUtf8("Douze dalles au maximum en une fois."));
		refreshList();
		return;
	}
	selected_ = next;
	map_->setDalles(visible_, selected_);
	refreshDownloadButton();
}

void DlgIgnGrid::clearSelection() {
	selected_.clear();
	map_->setDalles(visible_, selected_);
	refreshList();
}

void DlgIgnGrid::refreshList() {
	updatingList_ = true;
	list_->clear();
	QSet<QString> shown;
	for (int i = 0; i < visible_.size(); ++i) {
		const IgnDalle& dalle = visible_[i];
		QListWidgetItem* item = new QListWidgetItem(tileLabel(dalle.name) + QString::fromUtf8("   ·   ") + millions(dalle.points), list_);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(selected_.contains(dalle.name) ? Qt::Checked : Qt::Unchecked);
		item->setData(Qt::UserRole, dalle.name);
		shown.insert(dalle.name);
	}
	for (QSet<QString>::const_iterator it = selected_.constBegin(); it != selected_.constEnd(); ++it) {
		if (shown.contains(*it) || !catalog_.contains(*it))
			continue;
		const IgnDalle dalle = catalog_.value(*it);
		QListWidgetItem* item = new QListWidgetItem(tileLabel(dalle.name) + QString::fromUtf8("   ·   hors vue"), list_);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
		item->setData(Qt::UserRole, dalle.name);
	}
	updatingList_ = false;
	refreshDownloadButton();
}

void DlgIgnGrid::refreshDownloadButton() {
	const int count = selected_.size();
	downloadButton_->setEnabled(count > 0 && !downloadReply_);
	downloadButton_->setText(count == 0
		? QString::fromUtf8("Télécharger")
		: QString::fromUtf8("Télécharger %1 dalle%2").arg(count).arg(count > 1 ? "s" : ""));
}

void DlgIgnGrid::chooseFolder() {
	const QString chosen = QFileDialog::getExistingDirectory(this, QString::fromUtf8("Dossier des dalles"), folder_->text());
	if (!chosen.isEmpty())
		folder_->setText(chosen);
}

void DlgIgnGrid::onDownloadButton() {
	if (downloading_) {
		cancelDownload_ = true;
		if (downloadReply_)
			downloadReply_->abort();
		return;
	}
	startDownload();
}

void DlgIgnGrid::startDownload() {
	if (selected_.isEmpty() || downloading_)
		return;
	downloading_ = true;
	if (selected_.size() > kWarnSelection) {
		const QMessageBox::StandardButton answer = QMessageBox::question(this, windowTitle(),
			QString::fromUtf8("%1 dalles, souvent plus de %2 Mo. Continuer ?")
				.arg(selected_.size())
				.arg(selected_.size() * 150));
		if (answer != QMessageBox::Yes) {
			downloading_ = false;
			return;
		}
	}
	const QDir directory(folder_->text());
	if (!directory.exists() && !QDir().mkpath(directory.absolutePath())) {
		QMessageBox::warning(this, windowTitle(), QString::fromUtf8("Impossible de créer le dossier."));
		downloading_ = false;
		return;
	}
	QSettings settings("LiangliangNan", "City3D");
	settings.setValue("lidarHdDirectory", directory.absolutePath());

	downloadQueue_.clear();
	downloaded_.clear();
	for (QSet<QString>::const_iterator it = selected_.constBegin(); it != selected_.constEnd(); ++it) {
		if (!catalog_.contains(*it))
			continue;
		const QString path = directory.absoluteFilePath(*it);
		if (QFileInfo(path).size() > 10 * 1024 * 1024) {
			downloaded_.append(path);
			continue;
		}
		downloadQueue_.append(*it);
	}
	downloadIndex_ = 0;
	cancelDownload_ = false;
	shouldLoad_ = false;
	if (downloadQueue_.isEmpty()) {
		setStatus(QString::fromUtf8("Ces dalles sont déjà dans le dossier."));
		finishDownload(true);
		return;
	}
	progress_->setVisible(true);
	progress_->setValue(0);
	downloadButton_->setText(QString::fromUtf8("Annuler"));
	searchButton_->setEnabled(false);
	downloadNext();
}

void DlgIgnGrid::downloadNext() {
	if (cancelDownload_ || downloadIndex_ >= downloadQueue_.size()) {
		finishDownload(!cancelDownload_);
		return;
	}
	const QString name = downloadQueue_.at(downloadIndex_);
	const IgnDalle dalle = catalog_.value(name);
	QUrl url;
	if (!ignCloudUrl(dalle.url, &url)) {
		setStatus(QString::fromUtf8("Adresse de téléchargement refusée pour %1.").arg(name));
		++downloadIndex_;
		downloadNext();
		return;
	}
	downloadFinal_ = QDir(folder_->text()).absoluteFilePath(name);
	downloadPart_ = downloadFinal_ + ".part";
	QFile::remove(downloadPart_);
	downloadFile_ = new QFile(downloadPart_, this);
	if (!downloadFile_->open(QIODevice::WriteOnly)) {
		setStatus(QString::fromUtf8("Écriture impossible : %1").arg(downloadFinal_));
		delete downloadFile_;
		downloadFile_ = 0;
		finishDownload(false);
		return;
	}
	setStatus(QString::fromUtf8("Téléchargement %1/%2 — %3")
		.arg(downloadIndex_ + 1)
		.arg(downloadQueue_.size())
		.arg(name));
	downloadReply_ = net_->get(ignRequest(url));
	connect(downloadReply_, &QNetworkReply::readyRead, this, &DlgIgnGrid::onDownloadReady);
	connect(downloadReply_, &QNetworkReply::downloadProgress, this, [this](qint64 received, qint64 total) {
		if (total > 0)
			progress_->setValue((int)(received * 100 / total));
	});
	connect(downloadReply_, &QNetworkReply::finished, this, &DlgIgnGrid::onDownloadFinished);
}

void DlgIgnGrid::onDownloadReady() {
	if (downloadReply_ && downloadFile_)
		downloadFile_->write(downloadReply_->readAll());
}

void DlgIgnGrid::onDownloadFinished() {
	QNetworkReply* reply = downloadReply_;
	downloadReply_ = 0;
	if (downloadFile_) {
		if (reply)
			downloadFile_->write(reply->readAll());
		downloadFile_->close();
		delete downloadFile_;
		downloadFile_ = 0;
	}
	const bool failed = !reply || reply->error() != QNetworkReply::NoError;
	const bool cancelled = reply && reply->error() == QNetworkReply::OperationCanceledError;
	const QString error = reply ? reply->errorString() : QString();
	if (reply)
		reply->deleteLater();
	if (failed) {
		QFile::remove(downloadPart_);
		if (cancelled || cancelDownload_) {
			setStatus(QString::fromUtf8("Téléchargement annulé."));
			finishDownload(false);
			return;
		}
		setStatus(QString::fromUtf8("Échec de %1 : %2").arg(QFileInfo(downloadFinal_).fileName(), error));
		finishDownload(!downloaded_.isEmpty());
		return;
	}
	QFile::remove(downloadFinal_);
	if (!QFile::rename(downloadPart_, downloadFinal_)) {
		setStatus(QString::fromUtf8("Impossible de finaliser %1.").arg(downloadFinal_));
		finishDownload(false);
		return;
	}
	downloaded_.append(downloadFinal_);
	++downloadIndex_;
	downloadNext();
}

void DlgIgnGrid::finishDownload(bool propose) {
	if (downloadReply_) {
		downloadReply_->abort();
		return;
	}
	downloading_ = false;
	progress_->setVisible(false);
	searchButton_->setEnabled(true);
	downloadButton_->setEnabled(true);
	refreshDownloadButton();
	if (!propose || downloaded_.isEmpty())
		return;

	const int count = downloaded_.size();
	const QString text = count == 1
		? QString::fromUtf8("La dalle est téléchargée.\nLa charger dans City3D ?")
		: QString::fromUtf8("%1 dalles sont téléchargées.\nLes charger dans City3D ?").arg(count);
	QMessageBox box(QMessageBox::Question, windowTitle(), text, QMessageBox::NoButton, this);
	QPushButton* loadButton = box.addButton(QString::fromUtf8("Charger"), QMessageBox::AcceptRole);
	box.addButton(QString::fromUtf8("Plus tard"), QMessageBox::RejectRole);
	box.setDefaultButton(loadButton);
	box.exec();
	if (box.clickedButton() == loadButton) {
		shouldLoad_ = true;
		accept();
		return;
	}
	setStatus(QString::fromUtf8("%1 dalle(s) dans %2.").arg(count).arg(folder_->text()));
}
