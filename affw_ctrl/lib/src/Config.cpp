/*
 * Config.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "affw/Config.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>

namespace affw {

typedef std::map<std::string, std::string>::iterator it_type;
typedef std::map<std::string, std::string>::const_iterator it_type_const;

Config::Config() {
}

Config::~Config() {
}

bool Config::read(const char* file) {
	std::string line;
	std::fstream fileStream(file);
	if(!fileStream.is_open())
	{
		return false;
	}
	while (std::getline(fileStream, line))
	{
		std::istringstream is_line(line);
		std::string key;
		if (std::getline(is_line, key, '='))
		{
			std::string value;
			if (key[0] == '#')
				continue;

			if (std::getline(is_line, value))
			{
				this->map[key] = value;
			}
		}
	}
	return true;
}

bool Config::write(const char* file) {
	std::ofstream f;
	f.open (file, std::ios::out);
	if(!f.is_open())
	{
		return false;
	}
	for(it_type it = map.begin(); it != map.end(); it++)
	{
		f << it->first << "=" << it->second << std::endl;
	}
	f.close();
	return true;
}

void Config::print() {
	for(it_type it = map.begin(); it != map.end(); it++)
	{
		std::cout << it->first << "=" << it->second << std::endl;
	}
}

std::ostream& operator<< (std::ostream& stream, const Config& config)
{
	for(it_type_const it = config.map.begin(); it != config.map.end(); it++)
	{
		stream << it->first << "=" << it->second << std::endl;
	}
	return stream;
}

std::string Config::getString(const std::string& key, const std::string& defValue) {
	it_type it = map.find(key);
	if(it == map.end())
	{
		map[key] = defValue;
		return defValue;
	}
	return it->second;
}

void Config::setString(const std::string& key, const std::string& value) {
	map[key] = value;
}

double Config::getDouble(const std::string& key, double defValue) {
	it_type it = map.find(key);
	if(it == map.end())
	{
		setDouble(key, defValue);
		return defValue;
	}
	const std::string value(it->second);
	return std::stod(value);
}

void Config::setDouble(const std::string& key, double value) {
	std::ostringstream strs;
	strs << value;
	map[key] = strs.str();
}

int Config::getInt(const std::string& key, int defValue) {
	it_type it = map.find(key);
	if(it == map.end())
	{
		setInt(key, defValue);
		return defValue;
	}
	const std::string value(it->second);
	return std::stoi(value);
}

void Config::setInt(const std::string& key, int value) {
	std::ostringstream strs;
	strs << value;
	map[key] = strs.str();
}

int Config::getBool(const std::string& key, bool defValue) {
	it_type it = map.find(key);
	if(it == map.end())
	{
		setBool(key, defValue);
		return defValue;
	}
	if(boost::iequals(it->second,"true"))
	{
		return true;
	}
	return false;
}

void Config::setBool(const std::string& key, bool value) {
	if(value)
		map[key] = "true";
	else
		map[key] = "false";
}



std::vector<double> Config::getDoubleVector(const std::string& key, const std::vector<double>& defValue)
{
	it_type it = map.find(key);
	if(it == map.end())
	{
		setDoubleVector(key, defValue);
		return defValue;
	}

	std::istringstream iss(it->second);
	double v;
	std::vector<double> out;
	while(iss >> v)
		out.push_back(v);
	return out;
}

void Config::setDoubleVector(const std::string& key, const std::vector<double>& value)
{
	std::ostringstream ss;
	for(std::vector<double>::const_iterator it = value.begin(); it<value.end();it++)
		ss << *it << " ";
	map[key] = ss.str();
}

} /* namespace affw */
