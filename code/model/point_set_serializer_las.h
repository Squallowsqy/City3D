#ifndef _FILEIO_POINT_SERIALIZER_LAS_H_
#define _FILEIO_POINT_SERIALIZER_LAS_H_



#include <functional>
#include <string>
#include <vector>


class PointSet;

struct LasClassTally {
	int code;
	unsigned long long seen;
	unsigned long long kept;
};

enum LasImportPhase {
	LasPhaseReading = 1,
	LasPhaseFileDone = 2,
	LasPhaseDone = 3,
	LasPhaseFailed = 4
};

// One frame of the import, sent while tiles are read and once the cloud is ready.
struct LasImportSnapshot {
	LasImportPhase phase;
	std::string file_name;
	int file_index;
	int file_count;
	long long announced;
	long long file_read;
	long long read_count;
	long long kept_total;
	int version_major;
	int version_minor;
	int point_format;
	bool copc;
	std::string source;
	std::vector<LasClassTally> classes;
	bool has_ground;
	double ground_z;
	bool has_origin;
	double origin_x;
	double origin_y;
	double origin_z;
	std::string message;

	LasImportSnapshot()
		: phase(LasPhaseReading)
		, file_index(0)
		, file_count(0)
		, announced(0)
		, file_read(0)
		, read_count(0)
		, kept_total(0)
		, version_major(0)
		, version_minor(0)
		, point_format(0)
		, copc(false)
		, has_ground(false)
		, ground_z(0.0)
		, has_origin(false)
		, origin_x(0.0)
		, origin_y(0.0)
		, origin_z(0.0) {
	}
};

// Options for LAS/LAZ, including IGN LiDAR HD tiles (LAS 1.4, Lambert-93).
// An empty keep_classes list keeps every class.
struct LasLoadOptions {
	std::vector<int> keep_classes;
	bool drop_withheld;
	bool drop_overlap;
	bool drop_virtual_points;
	bool color_by_classification;
	bool record_ground;
	double voxel_size;
	std::function<void(const LasImportSnapshot&)> on_progress;

	LasLoadOptions()
		: drop_withheld(false)
		, drop_overlap(false)
		, drop_virtual_points(false)
		, color_by_classification(false)
		, record_ground(false)
		, voxel_size(0.0) {
	}

	// Previous City3D behaviour: every point, intensity when RGB is absent.
	static LasLoadOptions legacy();
	// Class 6 only. Class 2 is still scanned so footprints can sit on the ground.
	static LasLoadOptions ign_buildings();
	// Classes 2 and 6.
	static LasLoadOptions ign_buildings_and_ground();
	// Every class except withheld points and IGN virtual points (class 66).
	static LasLoadOptions ign_all_classes();

	static bool file_looks_like_ign_lidar_hd(const std::string& file_name);
};

// Shared with the viewer so class colors match the cloud.
void las_class_rgb(int cls, int& r, int& g, int& b);
const char* las_class_label(int cls);

// for both las and laz formats
// internally it uses the LASlib of martin.isenburg@rapidlasso.com.
// For more information, see http://rapidlasso.com

class PointSetSerializer_las
{
public:
	static void load(PointSet* pointSet, const std::string& file_name);
	// Reads one or several LAS/LAZ tiles (including .copc.laz) into one cloud.
	// Tiles share the origin of the first kept point, so Lambert-93 stays accurate.
	static bool load(PointSet* pointSet, const std::vector<std::string>& file_names, const LasLoadOptions& options);
} ;

#endif
