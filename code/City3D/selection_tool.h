#ifndef SELECTION_TOOL_H
#define SELECTION_TOOL_H

#include "../math/math_types.h"

#include <QPoint>
#include <QPolygonF>
#include <QVector>
#include <vector>

class QMouseEvent;
class QPainter;
class PaintCanvas;
class Map;
class PointSet;

// Screen-space selection used to restrict building / vegetation processing.
class SelectionTool {
public:
	enum Mode {
		None = 0,
		Rectangle,
		Circle,
		Polygon
	};

	SelectionTool();

	void setMode(Mode mode);
	Mode mode() const { return mode_; }
	bool isActive() const { return mode_ != None; }
	bool hasSelection() const { return !selected_footprints_.empty() || !selected_points_.empty(); }

	void clear();
	void clearGeometry();

	bool mousePress(PaintCanvas* canvas, QMouseEvent* e);
	bool mouseMove(PaintCanvas* canvas, QMouseEvent* e);
	bool mouseRelease(PaintCanvas* canvas, QMouseEvent* e);
	bool mouseDoubleClick(PaintCanvas* canvas, QMouseEvent* e);

	void drawOverlay(QPainter& painter, int width, int height) const;

	// Apply current screen shape to footprints (centroids) and vegetation-like points.
	void applyToScene(PaintCanvas* canvas, PointSet* cloud, Map* footprints);

	const QVector<int>& selectedFootprints() const { return selected_footprints_; }
	const std::vector<std::size_t>& selectedPoints() const { return selected_points_; }

	// Build a footprint map that only contains the selected buildings.
	Map* filteredFootprint(Map* footprints) const;

private:
	bool pointInSelection(const QPointF& screen) const;
	void finishShape(PaintCanvas* canvas);

	Mode mode_;
	bool drawing_;
	QPoint origin_;
	QPoint current_;
	QPolygonF polygon_;
	QVector<int> selected_footprints_;
	std::vector<std::size_t> selected_points_;
};

#endif
