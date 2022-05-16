
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <filesystem>

// include ouster types 
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

// for reading the config.json
#include <json/json.h>

// object to store json dict 
Json::Value root;

int main(int argc, char* argv[]) {

    std::cout << "Hello Dave." << std::endl;
    
    std::ifstream config_doc("rami_config.json", std::ifstream::binary);
    config_doc >> root;

    auto Ouster_config = root["Ouster_config"];

    const std::string mode = Ouster_config["lidar_mode"].asString();

    // filesystem object that will be used to iterate through the files 
    std::filesystem::path input_path{root["output_path"].asString()};

    // set up output path 
    std::filesystem::path output_path;
    output_path = input_path;
    output_path += "csv_files";

    std::cout << "files will be saved in:" << output_path << std::endl;

    // scan width and height - currently hardcoded 
    size_t scan_w = 2048;
    size_t scan_h = 32;

    ouster::LidarScan::Points cloud;
    cloud.resize(scan_w*scan_h, 3);

    input_path += "bin_files";

    for (auto const& dir_entry : std::filesystem::directory_iterator{input_path}) 
    {
        
        if(dir_entry.path().extension() == ".bin")
        { 

            std::ifstream in(dir_entry.path(), std::ios::in | std::ios::binary);

            in.read((char *) cloud.data() , scan_w*scan_h*3*sizeof(double));

            std::filesystem::path fn = dir_entry.path().filename().replace_extension(".csv"); 
            fn = output_path / fn;

            std::cout << fn << std::endl;

            std::ofstream out(fn);
            out << std::fixed << std::setprecision(4);

    
                for (int i = 0; i < cloud.rows(); i++) {
                    auto xyz = cloud.row(i);
                    if (!xyz.isApproxToConstant(0.0))
                        out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
                }

            in.close();
            out.close();
            
        }

    }

    return 0;
}