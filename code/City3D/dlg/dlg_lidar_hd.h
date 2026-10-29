#ifndef DLG_LIDAR_HD_H
#define DLG_LIDAR_HD_H

#include <QStringList>

class QWidget;
struct LasLoadOptions;

// Returns false when the user cancels. Several LAS/LAZ tiles can be selected together.
bool ask_lidar_load_options(QWidget* parent, const QStringList& files, LasLoadOptions& options);

#endif
