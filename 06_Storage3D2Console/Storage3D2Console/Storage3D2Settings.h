#pragma once
#include "stdafx.h"

using boost::property_tree::ptree;

//Gloabal parameters
float delta_limit = 0.09;
float plane_threshold = 0.001;
float zepsangle = 0.1;
bool enable_planefiltration = false;
bool perpendicularonly = false;
bool enable_noizefiltration = false;

float claster_tolerance = 0.08;
//float relative_claster_min = 0.001;
//float relative_claster_max = 0.2;
int minpoints = 200;
int maxpoints = 25000;

float object_minx = 0.2;
float object_miny = 0.2;
float object_minz = 0.2;
float object_maxx = 0.4;
float object_maxy = 0.4;
float object_maxz = 0.4;

bool working_with_file = false;
bool saving_state = false;
float voxel_density = 0.008f;
bool enable_voxelgridfiltration = false;

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
			if (key.first == "voxel_density")
				voxel_density = key.second.get_value<float>();
			if (key.first == "enable_voxelgridfiltration")
				enable_voxelgridfiltration = key.second.get_value<bool>();
			

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

			if (key.first == "work_with_file")
				working_with_file = key.second.get_value<bool>();
			if (key.first == "saving_state")
				saving_state = key.second.get_value<bool>();
		}
	}
}