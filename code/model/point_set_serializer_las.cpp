#include "point_set_serializer_las.h"
#include "../basic/logger.h"
#include "../basic/basic_types.h"
#include "point_set.h"
#include "../3rd_party/3rd_LAStools/LASlib/inc/lasreader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_set>


namespace {

struct VoxelKey {
	int x;
	int y;
	int z;
	bool operator==(const VoxelKey& other) const {
		return x == other.x && y == other.y && z == other.z;
	}
};

struct VoxelKeyHash {
	std::size_t operator()(const VoxelKey& key) const {
		std::size_t h = static_cast<std::size_t>(key.x) * 73856093u;
		h ^= static_cast<std::size_t>(key.y) * 19349663u;
		h ^= static_cast<std::size_t>(key.z) * 83492791u;
		return h;
	}
};

struct LasReadContext {
	bool offset_set;
	double ox, oy, oz;
	std::unordered_set<VoxelKey, VoxelKeyHash> voxels;
	unsigned long long seen[256];
	unsigned long long kept_class[256];
	unsigned long long read_total;
	unsigned long long kept_total;

	LasReadContext() : offset_set(false), ox(0.0), oy(0.0), oz(0.0), read_total(0), kept_total(0) {
		for (int i = 0; i < 256; ++i) {
			seen[i] = 0;
			kept_class[i] = 0;
		}
	}
};

std::string field_text(const char* data, std::size_t n) {
	std::size_t end = 0;
	while (end < n && data[end] != '\0')
		++end;
	while (end > 0 && data[end - 1] == ' ')
		--end;
	return std::string(data, data + end);
}

std::string lower_copy(std::string text) {
	for (std::size_t i = 0; i < text.size(); ++i)
		text[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
	return text;
}

const char* ign_class_label(int cls) {
	switch (cls) {
	case 1: return "non classé";
	case 2: return "sol";
	case 3: return "végétation basse";
	case 4: return "végétation moyenne";
	case 5: return "végétation haute";
	case 6: return "bâtiment";
	case 9: return "eau";
	case 17: return "tablier de pont";
	case 64: return "sursol pérenne";
	case 66: return "points virtuels";
	default: return 0;
	}
}

vec3 ign_class_color(int cls) {
	switch (cls) {
	case 2: return vec3(0.62, 0.42, 0.22);
	case 3: return vec3(0.65, 0.82, 0.35);
	case 4: return vec3(0.25, 0.70, 0.25);
	case 5: return vec3(0.05, 0.40, 0.12);
	case 6: return vec3(0.90, 0.18, 0.16);
	case 9: return vec3(0.20, 0.45, 0.90);
	case 17: return vec3(0.55, 0.30, 0.70);
	case 64: return vec3(0.95, 0.55, 0.10);
	case 66: return vec3(0.75, 0.75, 0.78);
	default: return vec3(0.55, 0.55, 0.55);
	}
}

int point_class(const LASpoint& point) {
	// LAS 1.4 formats 6-10 store the full class in extended_classification.
	// The legacy 5-bit field is 0 for IGN classes 64 and 66.
	if (point.is_extended_point_type())
		return static_cast<int>(point.get_extended_classification());
	return static_cast<int>(point.get_classification());
}

vec3 point_color(const LASpoint& point, int cls, const LasLoadOptions& options) {
	if (options.color_by_classification)
		return ign_class_color(cls);
	if (point.have_rgb) {
		const float r = float(point.get_R()) / 65535.0f;
		const float g = float(point.get_G()) / 65535.0f;
		const float b = float(point.get_B()) / 65535.0f;
		return vec3(r, g, b);
	}
	const float intensity = (point.intensity % 255) / 255.0f;
	return vec3(intensity, intensity, intensity);
}

bool file_name_is_copc(const std::string& file_name) {
	const std::string lower = lower_copy(file_name);
	return lower.size() >= 9 && lower.compare(lower.size() - 9, 9, ".copc.laz") == 0;
}

void build_keep_mask(const LasLoadOptions& options, bool keep_class[256]) {
	const bool filter = !options.keep_classes.empty();
	for (int i = 0; i < 256; ++i)
		keep_class[i] = !filter;

	bool explicit_virtual = false;
	for (std::size_t i = 0; i < options.keep_classes.size(); ++i) {
		const int cls = options.keep_classes[i];
		if (cls >= 0 && cls <= 255)
			keep_class[cls] = true;
		if (cls == 66)
			explicit_virtual = true;
	}
	if (options.drop_virtual_points && !explicit_virtual)
		keep_class[66] = false;
}

void fill_classes(
	const unsigned long long* base_seen,
	const unsigned long long* base_kept,
	const unsigned long long* extra_seen,
	const unsigned long long* extra_kept,
	std::vector<LasClassTally>& out)
{
	out.clear();
	for (int cls = 0; cls < 256; ++cls) {
		const unsigned long long seen = base_seen[cls] + (extra_seen ? extra_seen[cls] : 0);
		const unsigned long long kept = base_kept[cls] + (extra_kept ? extra_kept[cls] : 0);
		if (seen == 0 && kept == 0)
			continue;
		LasClassTally tally;
		tally.code = cls;
		tally.seen = seen;
		tally.kept = kept;
		out.push_back(tally);
	}
}

void publish_import(
	const LasLoadOptions& options,
	const LasReadContext& ctx,
	const PointSet* pset,
	LasImportPhase phase,
	const std::string& file_name,
	int file_index,
	int file_count,
	long long announced,
	long long read_count,
	int version_major,
	int version_minor,
	int point_format,
	bool copc,
	const std::string& source,
	const unsigned long long* extra_seen,
	const unsigned long long* extra_kept,
	unsigned long long extra_read,
	unsigned long long extra_kept_n,
	const std::string& message)
{
	if (!options.on_progress)
		return;

	LasImportSnapshot snapshot;
	snapshot.phase = phase;
	snapshot.file_name = file_name;
	snapshot.file_index = file_index;
	snapshot.file_count = file_count;
	snapshot.announced = announced;
	snapshot.file_read = read_count;
	snapshot.read_count = static_cast<long long>(ctx.read_total + extra_read);
	snapshot.kept_total = static_cast<long long>(ctx.kept_total + extra_kept_n);
	snapshot.version_major = version_major;
	snapshot.version_minor = version_minor;
	snapshot.point_format = point_format;
	snapshot.copc = copc;
	snapshot.source = source;
	fill_classes(ctx.seen, ctx.kept_class, extra_seen, extra_kept, snapshot.classes);
	snapshot.has_origin = ctx.offset_set;
	snapshot.origin_x = ctx.ox;
	snapshot.origin_y = ctx.oy;
	snapshot.origin_z = ctx.oz;
	if (pset && pset->has_ground_z()) {
		snapshot.has_ground = true;
		snapshot.ground_z = pset->ground_z();
	}
	snapshot.message = message;
	options.on_progress(snapshot);
}

bool read_one(PointSet* pset, const std::string& file_name, const LasLoadOptions& options, LasReadContext& ctx, int file_index, int file_count) {
	const bool copc_name = file_name_is_copc(file_name);

	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(file_name.c_str(), true);
	LASreader* lasreader = lasreadopener.open();
	if (!lasreader) {
		Logger::err("-") << "impossible d'ouvrir " << file_name << std::endl;
		std::string message = "Impossible d'ouvrir " + file_name;
		if (copc_name) {
			Logger::err("-") << "ce COPC n'est pas lisible par le LASzip embarqué (2018). "
				<< "Convertissez-le en LAZ classique : pdal translate entree.copc.laz sortie.laz" << std::endl;
			message += ". Ce COPC demande un LAZ classique (pdal translate).";
		}
		publish_import(options, ctx, pset, LasPhaseFailed, file_name, file_index, file_count,
			0, 0, 0, 0, 0, copc_name, std::string(), 0, 0, 0, 0, message);
		return false;
	}

	const LASheader& header = lasreader->header;
	const bool copc = copc_name || header.get_vlr("copc", 1) != 0;
	const I64 announced = lasreader->npoints;
	Logger::out("-") << "LAS " << static_cast<int>(header.version_major) << "."
		<< static_cast<int>(header.version_minor)
		<< ", format " << static_cast<int>(header.point_data_format)
		<< ", " << static_cast<long long>(announced) << " points"
		<< (copc ? ", COPC" : "") << std::endl;

	const std::string system = field_text(header.system_identifier, 32);
	const std::string software = field_text(header.generating_software, 32);
	if (!system.empty() || !software.empty())
		Logger::out("-") << "  source : " << system << " / " << software << std::endl;

	const std::string source = system.empty() ? software : (software.empty() ? system : system + " / " + software);
	publish_import(options, ctx, pset, LasPhaseReading, file_name, file_index, file_count,
		static_cast<long long>(announced), 0,
		static_cast<int>(header.version_major), static_cast<int>(header.version_minor),
		static_cast<int>(header.point_data_format), copc, source,
		0, 0, 0, 0, std::string());

	if (options.keep_classes.empty() && announced > 8000000) {
		Logger::warn("-") << "dalle volumineuse (" << static_cast<long long>(announced)
			<< " points). Pour reconstruire, la classe 6 (bâtiment) suffit." << std::endl;
	}

	bool keep_class[256];
	build_keep_mask(options, keep_class);

	unsigned long long seen[256];
	unsigned long long kept_by_class[256];
	for (int i = 0; i < 256; ++i) {
		seen[i] = 0;
		kept_by_class[i] = 0;
	}

	std::vector<vec3>& points = pset->points();
	std::vector<vec3>& colors = pset->colors();
	const std::size_t before = points.size();
	if (announced > 0) {
		const std::size_t hint = static_cast<std::size_t>(announced);
		const std::size_t extra = (options.keep_classes.empty() && options.voxel_size <= 0.0) ? hint : hint / 8 + 1024;
		points.reserve(before + extra);
		colors.reserve(before + extra);
	}

	unsigned long long read_count = 0;
	unsigned long long kept = 0;
	const double voxel = options.voxel_size;

	while (lasreader->read_point()) {
		LASpoint& point = lasreader->point;
		++read_count;
		if ((read_count % 400000ull) == 0) {
			publish_import(options, ctx, pset, LasPhaseReading, file_name, file_index, file_count,
				static_cast<long long>(announced), static_cast<long long>(read_count),
				static_cast<int>(header.version_major), static_cast<int>(header.version_minor),
				static_cast<int>(header.point_data_format), copc, source,
				seen, kept_by_class, read_count, kept, std::string());
		}
		if ((read_count % 5000000ull) == 0)
			Logger::out("-") << "  " << read_count << " points lus..." << std::endl;

		if (options.drop_withheld && point.get_withheld_flag())
			continue;

		const int cls = point_class(point);
		const int cls_index = (cls >= 0 && cls <= 255) ? cls : 0;
		++seen[cls_index];

		point.compute_coordinates();
		const double x = point.coordinates[0];
		const double y = point.coordinates[1];
		const double z = point.coordinates[2];
		if (!ctx.offset_set) {
			ctx.ox = x;
			ctx.oy = y;
			ctx.oz = z;
			ctx.offset_set = true;
			pset->set_offset(vec3(ctx.ox, ctx.oy, ctx.oz));
			char buffer[160];
			std::snprintf(buffer, sizeof(buffer),
				"nuage translaté par rapport au premier point : (%.3f %.3f %.3f)",
				ctx.ox, ctx.oy, ctx.oz);
			Logger::warn("-") << buffer << std::endl;
		}

		const double lx = x - ctx.ox;
		const double ly = y - ctx.oy;
		const double lz = z - ctx.oz;
		if (options.record_ground && cls_index == 2)
			pset->note_ground_z(lz);

		if (options.drop_overlap && point.get_extended_overlap_flag())
			continue;
		if (!keep_class[cls_index])
			continue;

		if (voxel > 0.0) {
			VoxelKey key;
			key.x = static_cast<int>(std::floor(lx / voxel));
			key.y = static_cast<int>(std::floor(ly / voxel));
			key.z = static_cast<int>(std::floor(lz / voxel));
			if (!ctx.voxels.insert(key).second)
				continue;
		}

		points.push_back(vec3(lx, ly, lz));
		colors.push_back(point_color(point, cls_index, options));
		++kept_by_class[cls_index];
		++kept;
	}

	lasreader->close();
	delete lasreader;

	Logger::out("-") << "  " << read_count << " lus, " << kept << " gardés dans " << file_name << std::endl;
	for (int cls = 0; cls < 256; ++cls) {
		if (seen[cls] == 0)
			continue;
		const char* label = ign_class_label(cls);
		if (label) {
			Logger::out("-") << "  classe " << cls << " (" << label << ") : "
				<< seen[cls] << " lus, " << kept_by_class[cls] << " gardés" << std::endl;
		}
		else {
			Logger::out("-") << "  classe " << cls << " : "
				<< seen[cls] << " lus, " << kept_by_class[cls] << " gardés" << std::endl;
		}
	}

	for (int cls = 0; cls < 256; ++cls) {
		ctx.seen[cls] += seen[cls];
		ctx.kept_class[cls] += kept_by_class[cls];
	}
	ctx.read_total += read_count;
	ctx.kept_total += kept;

	if (kept == 0) {
		Logger::err("-") << "aucun point gardé. Changez les classes, ou désactivez le filtre." << std::endl;
		publish_import(options, ctx, pset, LasPhaseFailed, file_name, file_index, file_count,
			static_cast<long long>(announced), static_cast<long long>(read_count),
			static_cast<int>(header.version_major), static_cast<int>(header.version_minor),
			static_cast<int>(header.point_data_format), copc, source,
			0, 0, 0, 0, "Aucun point gardé. Changez les classes, ou désactivez le filtre.");
		return false;
	}

	publish_import(options, ctx, pset, LasPhaseFileDone, file_name, file_index, file_count,
		static_cast<long long>(announced), static_cast<long long>(read_count),
		static_cast<int>(header.version_major), static_cast<int>(header.version_minor),
		static_cast<int>(header.point_data_format), copc, source,
		0, 0, 0, 0, std::string());
	return true;
}

}


LasLoadOptions LasLoadOptions::legacy() {
	return LasLoadOptions();
}

LasLoadOptions LasLoadOptions::ign_buildings() {
	LasLoadOptions options;
	options.keep_classes.push_back(6);
	options.drop_withheld = true;
	options.drop_virtual_points = true;
	options.color_by_classification = true;
	options.record_ground = true;
	return options;
}

LasLoadOptions LasLoadOptions::ign_buildings_and_ground() {
	LasLoadOptions options = ign_buildings();
	options.keep_classes.push_back(2);
	return options;
}

LasLoadOptions LasLoadOptions::ign_all_classes() {
	LasLoadOptions options;
	options.drop_withheld = true;
	options.drop_virtual_points = true;
	options.color_by_classification = true;
	options.record_ground = true;
	return options;
}

void las_class_rgb(int cls, int& r, int& g, int& b) {
	const vec3 color = ign_class_color(cls);
	r = static_cast<int>(color.x * 255.0 + 0.5);
	g = static_cast<int>(color.y * 255.0 + 0.5);
	b = static_cast<int>(color.z * 255.0 + 0.5);
}

const char* las_class_label(int cls) {
	const char* label = ign_class_label(cls);
	return label ? label : "";
}

bool LasLoadOptions::file_looks_like_ign_lidar_hd(const std::string& file_name) {
	std::string base = lower_copy(file_name);
	const std::string::size_type slash = base.find_last_of("/\\");
	if (slash != std::string::npos)
		base = base.substr(slash + 1);
	if (base.find("lhd_") != std::string::npos)
		return true;
	if (base.find("lamb93") != std::string::npos)
		return true;
	if (base.find("ign69") != std::string::npos)
		return true;
	if (base.find("ign78") != std::string::npos)
		return true;
	if (base.find("lidarhd") != std::string::npos)
		return true;
	if (base.find("lidar_hd") != std::string::npos)
		return true;
	return false;
}

void PointSetSerializer_las::load(PointSet* pointSet, const std::string& file_name) {
	std::vector<std::string> files;
	files.push_back(file_name);
	load(pointSet, files, LasLoadOptions::legacy());
}

bool PointSetSerializer_las::load(PointSet* pointSet, const std::vector<std::string>& file_names, const LasLoadOptions& options) {
	if (!pointSet || file_names.empty())
		return false;

	LasReadContext ctx;
	if (pointSet->num_points() > 0) {
		const vec3& offset = pointSet->offset();
		ctx.offset_set = true;
		ctx.ox = offset.x;
		ctx.oy = offset.y;
		ctx.oz = offset.z;
	}

	bool any = false;
	for (std::size_t i = 0; i < file_names.size(); ++i) {
		if (file_names.size() > 1)
			Logger::out("-") << "dalle " << (i + 1) << "/" << file_names.size() << std::endl;
		if (read_one(pointSet, file_names[i], options, ctx, static_cast<int>(i + 1), static_cast<int>(file_names.size())))
			any = true;
	}

	pointSet->points().shrink_to_fit();
	pointSet->colors().shrink_to_fit();
	pointSet->invalidate_bbox();

	if (pointSet->num_points() == 0)
		return false;

	publish_import(options, ctx, pointSet, LasPhaseDone, file_names[0],
		static_cast<int>(file_names.size()), static_cast<int>(file_names.size()),
		0, static_cast<long long>(ctx.read_total),
		0, 0, 0, false, std::string(),
		0, 0, 0, 0, std::string());

	Logger::out("-") << "total " << pointSet->num_points() << " points" << std::endl;
	if (pointSet->has_ground_z()) {
		char buffer[128];
		std::snprintf(buffer, sizeof(buffer),
			"sol (classe 2) : Z local %.3f m. Les empreintes seront posées à cette altitude.",
			pointSet->ground_z());
		Logger::out("-") << buffer << std::endl;
	}
	return any;
}
