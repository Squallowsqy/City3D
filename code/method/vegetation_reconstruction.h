#pragma once

#include "../model/map.h"
#include "../model/point_set.h"

#include <string>
#include <vector>

// Reconstruct trees from selected vegetation points via the AdTree algorithm
// (Du et al., Remote Sensing 2019) — https://github.com/tudelft3d/AdTree
class VegetationReconstruction {
public:
	VegetationReconstruction();

	void setAdTreeExecutable(const std::string& path);
	const std::string& adTreeExecutable() const { return adtree_exe_; }

	// Cluster selected points and reconstruct each tree. Merges meshes into `result`.
	// Returns the number of trees successfully reconstructed.
	int reconstruct(
		PointSet* cloud,
		const std::vector<std::size_t>& point_indices,
		Map* result,
		bool vegetation_color_only = true);

private:
	bool looksLikeVegetation(const vec3& color) const;
	std::vector<std::vector<std::size_t> > clusterPoints(
		PointSet* cloud,
		const std::vector<std::size_t>& indices) const;
	bool runAdTreeOnCluster(
		PointSet* cloud,
		const std::vector<std::size_t>& indices,
		const std::string& work_dir,
		int cluster_id,
		Map* result) const;
	std::string findDefaultAdTree() const;

	std::string adtree_exe_;
};
