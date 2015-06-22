#pragma once
#include "stdafx.h"

using boost::property_tree::ptree;

//Gloabal parameters
float delta_limit;
float plane_threshold;
float plane_claster_tolerance;
int plane_minpoints;
int plane_maxpoints;

float zepsangle;
bool enable_planefiltration;
bool perpendicularonly;
bool enable_noizefiltration;

float claster_tolerance;
//float relative_claster_min = 0.001;
//float relative_claster_max = 0.2;
int minpoints;
int maxpoints;

float object_minx;
float object_miny;
float object_minz;
float object_maxx;
float object_maxy;
float object_maxz;
float valid_percent;
int nearest_point_count;

bool working_with_file;
bool saving_state;
float object_density;
float plane_density;
bool enable_voxelgridfiltration;
float cloudzstep;

using namespace std;

void loadSettingsFile(){
	//Loading settings file
	ptree pt;
	read_ini("config.ini", pt);
	for (auto& section : pt)
	{
		std::cout << '[' << section.first << "]\n";
		for (auto& key : section.second){
			std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";
			string fieldname = key.first;
			if (key.first == "delta_limit")
				delta_limit = key.second.get_value<float>();
			if (key.first == "plane_threshold")
				plane_threshold = key.second.get_value<float>();

			if (key.first == "plane_claster_tolerance")
				plane_claster_tolerance = key.second.get_value<float>();
			if (key.first == "plane_minpoints")
				plane_minpoints = key.second.get_value<int>();
			if (key.first == "plane_maxpoints")
				plane_maxpoints = key.second.get_value<int>();
			
			if (key.first == "object_density")
				object_density = key.second.get_value<float>();
			if (key.first == "plane_density")
				plane_density = key.second.get_value<float>();

			if (key.first == "enable_voxelgridfiltration")
				enable_voxelgridfiltration = key.second.get_value<bool>();
			

			if (key.first == "cloudzstep")
				cloudzstep = key.second.get_value<float>();
			if (key.first == "zepsangle")
				zepsangle = key.second.get_value<float>();
			if (key.first == "enable_planefiltration")
				enable_planefiltration = key.second.get_value<bool>();
			if (key.first == "perpendicularonly")
				perpendicularonly = key.second.get_value<bool>();
			if (key.first == "enable_noizefiltration")
				enable_noizefiltration = key.second.get_value<bool>();

			if (key.first == "claster_tolerance")
				claster_tolerance = key.second.get_value<float>();
			if (key.first == "minpoints")
				minpoints = key.second.get_value<float>();
			if (key.first == "maxpoints")
				maxpoints = key.second.get_value<float>();

			if (key.first == "object_minx")
				object_minx = key.second.get_value<float>();
			if (key.first == "object_miny")
				object_miny = key.second.get_value<float>();
			if (key.first == "object_minz")
				object_minz = key.second.get_value<float>();
			if (key.first == "object_maxx")
				object_maxx = key.second.get_value<float>();
			if (key.first == "object_maxy")
				object_maxy = key.second.get_value<float>();
			if (key.first == "object_maxz")
				object_maxz = key.second.get_value<float>();
			if (key.first == "valid_percent")
				valid_percent = key.second.get_value<float>();
			if (key.first == "nearest_point_count")
				nearest_point_count = key.second.get_value<int>();

			if (key.first == "work_with_file")
				working_with_file = key.second.get_value<bool>();
			if (key.first == "saving_state")
				saving_state = key.second.get_value<bool>();
		}
	}
}