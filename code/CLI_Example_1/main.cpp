/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include "../model/point_set.h"
#include "../model/map.h"
#include "../model/map_io.h"
#include "../model/point_set_io.h"
#include "../model/point_set_serializer_las.h"
#include "../method/method_global.h"
#include "../method/reconstruction.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


namespace {

void print_usage() {
    std::cout
        << "Usage:\n"
        << "  CLI_Example_1\n"
        << "      demo on data/002.ply\n"
        << "  CLI_Example_1 <cloud.laz> <footprint.geojson|obj> <out.obj> [options]\n"
        << "\n"
        << "Options:\n"
        << "  --buildings          classe 6 seulement (défaut si le fichier est du LiDAR HD IGN)\n"
        << "  --buildings-ground   classes 6 et 2\n"
        << "  --classes 6,2        classes LAS à garder\n"
        << "  --all                aucun filtre de classe\n"
        << "  --voxel 0.30         un point par voxel, en mètres (0 = tout garder)\n"
        << "\n"
        << "Les empreintes doivent être dans le même système que le nuage.\n"
        << "LiDAR HD IGN : Lambert-93 (EPSG:2154), pas du WGS84.\n";
}

bool parse_classes(const std::string& text, std::vector<int>& classes) {
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (token.empty())
            continue;
        char* end = 0;
        const long value = std::strtol(token.c_str(), &end, 10);
        if (end == token.c_str() || value < 0 || value > 255)
            return false;
        classes.push_back(static_cast<int>(value));
    }
    return !classes.empty();
}

void warn_if_footprint_looks_geographic(const Map* footprint, const PointSet* cloud) {
    if (!footprint || !cloud || footprint->size_of_vertices() == 0)
        return;
    const vec3& offset = cloud->offset();
    if (std::abs(offset.x) < 10000.0 && std::abs(offset.y) < 10000.0)
        return;
    FOR_EACH_VERTEX_CONST(Map, footprint, it) {
        const vec3 world = it->point() + footprint->offset();
        if (std::abs(world.x) <= 180.0 && std::abs(world.y) <= 90.0) {
            std::cerr << "l'empreinte ressemble à des longitude/latitude, alors que le nuage est projeté. "
                      << "Reprojetez-la en Lambert-93 (EPSG:2154)." << std::endl;
        }
        break;
    }
}

}


int main(int argc, char **argv)
{
    ///ToDo: user may need to tune these parameters for their datasets
    Method::min_points = 40;
    Method::pixel_size = 0.15;

    std::string input_cloud_file;
    std::string input_footprint_file;
    std::string output_file;
    bool use_demo = true;
    bool force_all = false;
    bool preset_buildings = false;
    bool preset_buildings_ground = false;
    bool classes_set = false;
    bool voxel_set = false;
    double voxel = 0.0;
    std::vector<int> classes;

    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return EXIT_SUCCESS;
        }
        if (arg == "--all") {
            force_all = true;
            continue;
        }
        if (arg == "--buildings") {
            preset_buildings = true;
            continue;
        }
        if (arg == "--buildings-ground") {
            preset_buildings_ground = true;
            continue;
        }
        if (arg == "--voxel") {
            if (i + 1 >= argc) {
                std::cerr << "--voxel attend une taille en mètres" << std::endl;
                return EXIT_FAILURE;
            }
            voxel = std::atof(argv[++i]);
            voxel_set = true;
            continue;
        }
        if (arg == "--classes") {
            if (i + 1 >= argc || !parse_classes(argv[++i], classes)) {
                std::cerr << "--classes attend une liste, par exemple 6 ou 6,2" << std::endl;
                return EXIT_FAILURE;
            }
            classes_set = true;
            continue;
        }
        if (!arg.empty() && arg[0] == '-') {
            std::cerr << "option inconnue : " << arg << std::endl;
            print_usage();
            return EXIT_FAILURE;
        }
        positional.push_back(arg);
    }

    if (!positional.empty()) {
        if (positional.size() != 3) {
            print_usage();
            return EXIT_FAILURE;
        }
        use_demo = false;
        input_cloud_file = positional[0];
        input_footprint_file = positional[1];
        output_file = positional[2];
    }
    else {
        const std::string directory = std::string(CITY3D_ROOT_DIR) + "/../data/";
        input_cloud_file = directory + "002.ply";
        input_footprint_file = directory + "002_footprint.obj";
        output_file = directory + "002_result.obj";
    }

    std::cout << "loading input point cloud data from file: " << input_cloud_file << std::endl;
    PointSet *pset = 0;
    if (use_demo) {
        pset = PointSetIO::read(input_cloud_file);
    }
    else {
        LasLoadOptions las = LasLoadOptions::legacy();
        const bool ign = LasLoadOptions::file_looks_like_ign_lidar_hd(input_cloud_file);
        if (classes_set) {
            las = LasLoadOptions::ign_buildings();
            las.keep_classes = classes;
        }
        else if (preset_buildings_ground)
            las = LasLoadOptions::ign_buildings_and_ground();
        else if (preset_buildings || (ign && !force_all))
            las = LasLoadOptions::ign_buildings();
        else if (force_all || !ign)
            las = LasLoadOptions::legacy();

        las.record_ground = true;
        if (voxel_set)
            las.voxel_size = voxel;
        if (ign && !force_all)
            las.color_by_classification = true;

        std::vector<std::string> tiles(1, input_cloud_file);
        pset = PointSetIO::read(tiles, las);
    }
    if (!pset) {
        std::cerr << "failed loading point cloud data from file: " << input_cloud_file << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "loading input footprint data from file: " << input_footprint_file << std::endl;
    const vec3& offset = pset->offset();
    const double anchor_z = pset->footprint_z();
    /// Footprints are placed on LiDAR class 2 when that class was present,
    /// otherwise on the lowest loaded point.
    Map *footprint = MapIO::read(input_footprint_file, vec3(offset.x, offset.y, -anchor_z));
    if (!footprint) {
        std::cerr << "failed loading footprint data from file: " << input_footprint_file << std::endl;
        return EXIT_FAILURE;
    }
    if (pset->has_ground_z())
        std::cout << "footprint altitude set from LiDAR ground (class 2)" << std::endl;
    warn_if_footprint_looks_geographic(footprint, pset);

    Reconstruction recon;

    // Step 1: segmentation to obtain point clouds of individual buildings
    std::cout << "segmenting individual buildings..." << std::endl;
    recon.segmentation(pset, footprint);

    // Step 2: extract planes from the point cloud of each building (for all buildings)
    std::cout << "extracting roof planes..." << std::endl;
    if (!recon.extract_roofs(pset, footprint)) {
        std::cerr << "no roofs could be extracted from the point cloud" << std::endl;
        return EXIT_FAILURE;
    }

    // Step 3: reconstruction of all the buildings in the scene
    Map *result = new Map;
#ifdef HAS_GUROBI
    std::cout << "reconstructing the buildings (using the Gurobi solver)..." << std::endl;
    bool status = recon.reconstruct(pset, footprint, result, LinearProgramSolver::GUROBI);
#else
    std::cout << "reconstructing the buildings (using the SCIP solver)..." << std::endl;
    bool status = recon.reconstruct(pset, footprint, result, LinearProgramSolver::SCIP);
#endif

    if (status && result->size_of_facets() > 0)
    {
        if (MapIO::save(output_file, result))
        {
            std::cout << "reconstruction result saved to file: " << output_file << std::endl;
            return EXIT_SUCCESS;
        } else
            std::cerr << "failed to save reconstruction result to file: " << output_file << std::endl;
    } else
        std::cerr << "reconstruction failed" << std::endl;

    delete pset;
    delete footprint;
    delete result;

    return EXIT_FAILURE;
}
