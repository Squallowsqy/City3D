#include "selection_tool.h"

#include "paint_canvas.h"

#include "../model/map.h"
#include "../model/map_geometry.h"
#include "../model/point_set.h"
#include "../model/vertex_group.h"

#include <QLineF>
#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

namespace {

vec3 facetCentroid(const Map::Facet* f) {
	vec3 sum(0, 0, 0);
	int count = 0;
	Map::Halfedge* h = f->halfedge();
	do {
		sum = sum + h->vertex()->point();
		++count;
		h = h->next();
	} while (h != f->halfedge());
	if (count > 0)
		sum = sum / double(count);
	return sum;
}

bool pointInPolygon(const QPolygonF& poly, const QPointF& p) {
	return poly.containsPoint(p, Qt::OddEvenFill);
}

}


SelectionTool::SelectionTool()
	: mode_(None)
	, drawing_(false) {
}

void SelectionTool::setMode(Mode mode) {
	mode_ = mode;
	drawing_ = false;
	polygon_.clear();
	origin_ = QPoint();
	current_ = QPoint();
}

void SelectionTool::clear() {
	clearGeometry();
	selected_footprints_.clear();
	selected_points_.clear();
}

void SelectionTool::clearGeometry() {
	drawing_ = false;
	polygon_.clear();
	origin_ = QPoint();
	current_ = QPoint();
}

bool SelectionTool::pointInSelection(const QPointF& screen) const {
	if (mode_ == Rectangle || (mode_ == None && origin_ != current_)) {
		QRectF rect = QRectF(origin_, current_).normalized();
		if (rect.width() < 2 || rect.height() < 2)
			return false;
		return rect.contains(screen);
	}
	if (mode_ == Circle) {
		const QPointF c(origin_);
		const double radius = QLineF(c, QPointF(current_)).length();
		if (radius < 2.0)
			return false;
		return QLineF(c, screen).length() <= radius;
	}
	if (mode_ == Polygon) {
		if (polygon_.size() < 3)
			return false;
		return pointInPolygon(polygon_, screen);
	}
	return false;
}

bool SelectionTool::mousePress(PaintCanvas* canvas, QMouseEvent* e) {
	if (!isActive() || e->button() != Qt::LeftButton)
		return false;

	if (mode_ == Polygon) {
		polygon_ << e->pos();
		current_ = e->pos();
		drawing_ = true;
		canvas->update();
		return true;
	}

	drawing_ = true;
	origin_ = e->pos();
	current_ = e->pos();
	canvas->update();
	return true;
}

bool SelectionTool::mouseMove(PaintCanvas* canvas, QMouseEvent* e) {
	if (!isActive() || !drawing_)
		return false;
	current_ = e->pos();
	canvas->update();
	return true;
}

bool SelectionTool::mouseRelease(PaintCanvas* canvas, QMouseEvent* e) {
	if (!isActive() || e->button() != Qt::LeftButton)
		return false;

	if (mode_ == Polygon) {
		current_ = e->pos();
		canvas->update();
		return true;
	}

	current_ = e->pos();
	drawing_ = false;
	finishShape(canvas);
	return true;
}

bool SelectionTool::mouseDoubleClick(PaintCanvas* canvas, QMouseEvent* e) {
	if (!isActive() || mode_ != Polygon || e->button() != Qt::LeftButton)
		return false;
	drawing_ = false;
	if (polygon_.size() >= 3)
		finishShape(canvas);
	else
		clearGeometry();
	canvas->update();
	return true;
}

void SelectionTool::finishShape(PaintCanvas* canvas) {
	if (!canvas)
		return;
	applyToScene(canvas, canvas->pointSet(), canvas->footPrint());
	clearGeometry();
	canvas->update();
}

void SelectionTool::applyToScene(PaintCanvas* canvas, PointSet* cloud, Map* footprints) {
	selected_footprints_.clear();
	selected_points_.clear();
	if (!canvas)
		return;

	if (footprints) {
		int index = 0;
		FOR_EACH_FACET_CONST(Map, footprints, it) {
			const vec3 c = facetCentroid(it);
			const vec2 screen = canvas->projectionOf(c);
			if (pointInSelection(QPointF(screen.x, screen.y)))
				selected_footprints_.push_back(index);
			++index;
		}
	}

	if (cloud) {
		const std::vector<vec3>& points = cloud->points();
		selected_points_.reserve(points.size() / 8);
		for (std::size_t i = 0; i < points.size(); ++i) {
			const vec2 screen = canvas->projectionOf(points[i]);
			if (pointInSelection(QPointF(screen.x, screen.y)))
				selected_points_.push_back(i);
		}
	}
}

Map* SelectionTool::filteredFootprint(Map* footprints) const {
	if (!footprints)
		return nil;
	if (selected_footprints_.isEmpty())
		return footprints;

	Map* subset = new Map;
	int index = 0;
	FOR_EACH_FACET(Map, footprints, it) {
		if (selected_footprints_.contains(index))
			Geom::merge_into_source(subset, it);
		++index;
	}
	if (subset->size_of_facets() == 0) {
		delete subset;
		return footprints;
	}
	return subset;
}

void SelectionTool::drawOverlay(QPainter& painter, int /*width*/, int /*height*/) const {
	painter.setRenderHint(QPainter::Antialiasing, true);

	QPen pen(QColor(194, 65, 12, 220));
	pen.setWidth(2);
	pen.setStyle(Qt::DashLine);
	painter.setPen(pen);
	painter.setBrush(QColor(194, 65, 12, 45));

	if (mode_ == Rectangle && (drawing_ || origin_ != current_)) {
		painter.drawRect(QRect(origin_, current_).normalized());
	}
	else if (mode_ == Circle && (drawing_ || origin_ != current_)) {
		const double radius = QLineF(origin_, current_).length();
		painter.drawEllipse(QPointF(origin_), radius, radius);
	}
	else if (mode_ == Polygon) {
		if (polygon_.size() >= 2)
			painter.drawPolyline(polygon_);
		for (int i = 0; i < polygon_.size(); ++i)
			painter.drawEllipse(polygon_.at(i), 3.5, 3.5);
		if (drawing_ && !polygon_.isEmpty())
			painter.drawLine(polygon_.back(), QPointF(current_));
		if (polygon_.size() >= 3) {
			painter.setBrush(QColor(194, 65, 12, 35));
			painter.drawPolygon(polygon_);
		}
	}

	if (!selected_footprints_.isEmpty() || !selected_points_.empty()) {
		painter.setPen(QColor(28, 25, 21));
		painter.setBrush(QColor(251, 248, 243, 210));
		const QString label = QString::fromUtf8("Sélection : %1 bât. · %2 pts")
			.arg(selected_footprints_.size())
			.arg(selected_points_.size());
		const QFontMetrics fm(painter.font());
		const int w = fm.horizontalAdvance(label) + 16;
		painter.drawRoundedRect(12, 12, w, 26, 8, 8);
		painter.drawText(20, 30, label);
	}
}
