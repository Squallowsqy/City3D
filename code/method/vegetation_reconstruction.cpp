#include "vegetation_reconstruction.h"

#include "dbscan.h"
#include "../basic/basic_types.h"
#include "../basic/file_utils.h"
#include "../basic/logger.h"
#include "../basic/progress.h"
#include "../model/map_geometry.h"
#include "../model/map_io.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#include <cstdlib>
#else
#include <cstdlib>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

bool fileExists(const std::string& path) {
	std::ifstream in(path.c_str());
	return in.good();
}

std::string joinPath(const std::string& a, const std::string& b) {
	if (a.empty())
		return b;
	if (a[a.size() - 1] == '/' || a[a.size() - 1] == '\\')
		return a + b;
	return a + "/" + b;
}

int runCommand(const std::string& cmd) {
	Logger::out("-") << "AdTree: " << cmd << std::endl;
	return std::system(cmd.c_str());
}

}


VegetationReconstruction::VegetationReconstruction() {
	adtree_exe_ = findDefaultAdTree();
}

void VegetationReconstruction::setAdTreeExecutable(const std::string& path) {
	adtree_exe_ = path;
}

std::string VegetationReconstruction::findDefaultAdTree() const {
	std::vector<std::string> candidates;
#ifdef CITY3D_ADTREE_EXE
	candidates.push_back(CITY3D_ADTREE_EXE);
#endif
	const std::string exe = FileUtils::parent_directory(FileUtils::executable());
	candidates.push_back(joinPath(exe, "AdTree"));
	candidates.push_back(joinPath(exe, "AdTree.exe"));
	candidates.push_back(joinPath(exe, "AdTree.app/Contents/MacOS/AdTree"));
	// Typical City3D.app layout next to build/adtree
	candidates.push_back(joinPath(FileUtils::parent_directory(FileUtils::parent_directory(FileUtils::parent_directory(exe))),
		"../../adtree/bin/AdTree.app/Contents/MacOS/AdTree"));
	for (std::size_t i = 0; i < candidates.size(); ++i) {
		if (!candidates[i].empty() && fileExists(candidates[i]))
			return candidates[i];
	}
	return std::string();
}

bool VegetationReconstruction::looksLikeVegetation(const vec3& color) const {
	// IGN palette: vegetation is green-dominant; buildings are red-dominant.
	return color.y > color.x + 0.05 && color.y > color.z;
}

std::vector<std::vector<std::size_t> > VegetationReconstruction::clusterPoints(
	PointSet* cloud,
	const std::vector<std::size_t>& indices) const
{
	std::vector<std::vector<std::size_t> > clusters;
	if (!cloud || indices.empty())
		return clusters;

	std::vector<vec3> samples;
	samples.reserve(indices.size());
	for (std::size_t i = 0; i < indices.size(); ++i)
		samples.push_back(cloud->points()[indices[i]]);

	// DBSCAN in XY (Z ignored for crown separation): eps ~ 2.5 m, min 40 pts.
	typedef std::vector<double> VecD;
	std::vector<VecD> data;
	data.reserve(samples.size());
	for (std::size_t i = 0; i < samples.size(); ++i) {
		VecD v(2);
		v[0] = samples[i].x;
		v[1] = samples[i].y;
		data.push_back(v);
	}

	DBSCAN<VecD, double> db;
	db.Run(&data, 2, 2.5, 40,
		[](const VecD& a, const VecD& b) -> double {
			const double dx = a[0] - b[0];
			const double dy = a[1] - b[1];
			return std::sqrt(dx * dx + dy * dy);
		});

	const std::vector<std::vector<unsigned int> >& raw = db.Clusters;
	clusters.resize(raw.size());
	for (std::size_t c = 0; c < raw.size(); ++c) {
		clusters[c].reserve(raw[c].size());
		for (std::size_t k = 0; k < raw[c].size(); ++k)
			clusters[c].push_back(indices[raw[c][k]]);
	}
	return clusters;
}

bool VegetationReconstruction::runAdTreeOnCluster(
	PointSet* cloud,
	const std::vector<std::size_t>& indices,
	const std::string& work_dir,
	int cluster_id,
	Map* result) const
{
	if (adtree_exe_.empty() || !fileExists(adtree_exe_)) {
		Logger::err("-") << "exécutable AdTree introuvable. Compilez build/adtree." << std::endl;
		return false;
	}
	if (indices.size() < 40)
		return false;

	std::ostringstream name;
	name << "tree_" << cluster_id;
	const std::string xyz = joinPath(work_dir, name.str() + ".xyz");
	const std::string out_dir = joinPath(work_dir, name.str() + "_out");
	FileUtils::create_directory(out_dir);

	std::ofstream out(xyz.c_str());
	if (!out)
		return false;
	for (std::size_t i = 0; i < indices.size(); ++i) {
		const vec3& p = cloud->points()[indices[i]];
		out << p.x << " " << p.y << " " << p.z << "\n";
	}
	out.close();

	std::ostringstream cmd;
	cmd << "\"" << adtree_exe_ << "\" \"" << xyz << "\" \"" << out_dir << "\"";
	const int code = runCommand(cmd.str());
	if (code != 0) {
		Logger::warn("-") << "AdTree a échoué pour le cluster " << cluster_id
			<< " (code " << code << ")" << std::endl;
		return false;
	}

	// AdTree writes <name>_branches.obj and optionally leaves.
	std::vector<std::string> candidates;
	candidates.push_back(joinPath(out_dir, name.str() + "_branches.obj"));
	candidates.push_back(joinPath(out_dir, name.str() + "_leaves.obj"));
	candidates.push_back(joinPath(out_dir, FileUtils::base_name(xyz) + "_branches.obj"));

	// Also scan the output directory for any .obj
	std::vector<std::string> files;
	FileUtils::get_files(out_dir, files);
	for (std::size_t i = 0; i < files.size(); ++i) {
		std::string ext = FileUtils::extension(files[i]);
		String::to_lowercase(ext);
		if (ext == "obj")
			candidates.push_back(files[i]);
	}

	bool any = false;
	for (std::size_t i = 0; i < candidates.size(); ++i) {
		if (!fileExists(candidates[i]))
			continue;
		Map* mesh = MapIO::read(candidates[i], vec3(0, 0, 0));
		if (!mesh || mesh->size_of_facets() == 0) {
			if (mesh)
				delete mesh;
			continue;
		}
		Geom::merge_into_source(result, mesh);
		delete mesh;
		any = true;
	}
	return any;
}

int VegetationReconstruction::reconstruct(
	PointSet* cloud,
	const std::vector<std::size_t>& point_indices,
	Map* result,
	bool vegetation_color_only)
{
	if (!cloud || !result || point_indices.empty())
		return 0;

	std::vector<std::size_t> veg;
	veg.reserve(point_indices.size());
	const bool has_colors = cloud->has_colors();
	for (std::size_t i = 0; i < point_indices.size(); ++i) {
		const std::size_t id = point_indices[i];
		if (id >= cloud->num_points())
			continue;
		if (vegetation_color_only && has_colors && !looksLikeVegetation(cloud->colors()[id]))
			continue;
		veg.push_back(id);
	}
	if (veg.empty()) {
		Logger::warn("-") << "aucun point de végétation dans la sélection" << std::endl;
		return 0;
	}

	Logger::out("-") << "AdTree: " << veg.size() << " points végétation → clustering…" << std::endl;
	const std::vector<std::vector<std::size_t> > clusters = clusterPoints(cloud, veg);
	if (clusters.empty()) {
		Logger::warn("-") << "aucun arbre détecté (clusters trop petits)" << std::endl;
		return 0;
	}

	const std::string work = joinPath(FileUtils::parent_directory(FileUtils::executable()), "adtree_work");
	FileUtils::create_directory(work);

	ProgressLogger progress(clusters.size(), "AdTree végétation");
	int ok = 0;
	for (std::size_t i = 0; i < clusters.size(); ++i) {
		if (progress.is_canceled())
			break;
		Logger::out("-") << "arbre " << (i + 1) << "/" << clusters.size()
			<< " (" << clusters[i].size() << " pts)" << std::endl;
		if (runAdTreeOnCluster(cloud, clusters[i], work, static_cast<int>(i), result))
			++ok;
		progress.next();
	}
	return ok;
}
