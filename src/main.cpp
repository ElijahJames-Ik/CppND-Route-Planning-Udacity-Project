#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include <sstream>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;
using std::istringstream;
using std::cin;
using std::cout;
using std::vector;
using std::string;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    float value;
    vector<float> coordinates;
    string program_coordinates;
    while(true){
        cout << "Please enter the x and y coordinates for the starting and ending position.\nCoordinates must be seperated with space.\n";

        std::getline(cin,program_coordinates);
        istringstream input_stream(program_coordinates);
        while(input_stream){
            input_stream >> value;
            coordinates.push_back(value);
        }
       
        if(coordinates[0] >= 0 && coordinates[0] <= 100 && coordinates[1] >= 0 && coordinates[1] <= 100 && coordinates[2] >= 0 
        && coordinates[2] <= 100 && coordinates[3] >= 0 && coordinates[3] <= 100 && coordinates.size() >= 4){
            break;
        }else{
            cout <<"Invalid input! Coordinates must be in the range 0 - 100.\nYou need to provide 4 values seperated by space e.g 10 10 90 90.\n";
            
        }
        
        coordinates.clear();
        
    }
   

    start_x = coordinates[0];
    start_y = coordinates[1];
    end_x = coordinates[2];
    end_y = coordinates[3];
    
    

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
