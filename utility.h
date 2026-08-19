/**
 * This work contains valuable confidential and proprietary information.
 * Disclosure or reproduction without the written authorization of Neya
 * Systems, LLC is prohibited. This unpublished work by Neya Systems, LLC
 * is protected by the laws of the United States and other countries. If
 * publication of the work should occur, the following notice shall apply.
 *
 * Copyright (c) 2025 Neya Systems, LLC
 * All Rights Reserved
 *
 * The software/firmware is provided to you on an As-Is basis
 *
 * Delivered to the U.S. Government with Government Purpose Rights, as defined in
 * DFARS Part 252.227-7013 or 7014 (Feb 2014). This material may be
 * reproduced by or for the U.S Government pursuant to the copyright
 * license under the clause at DFARS 227-7203-5(a), DFARS 227.7103-5(a),
 * DFARS 252.227-7013(b)(1)(June 1995), DFARS 252.227-7014 (June 1995),
 * and FAR 52.227-14(a). Use of this work other than as specifically
 * authorized by the U.S. Government may violate any copyrights that
 * exist in this work.
 *
 * "WARNING - This file contains software and technical data whose export is
 * restricted by the Arms Export Control Act (Title 22, U.S.C., Sec
 * 2751, et seq.) or the Export Administration Act of 1979 (Title 50,
 * U.S.C., App. 2401 et seq.), as amended. Violations of these export
 * laws are subject to severe criminal penalties. Disseminate in
 * accordance with provisions of DoD Directive 5230.25."
 */
#pragma once

#include <string>
#include <map>

#include <MMSTypes/GeoPoint.h>
#include <MMSTypes/GeoList.h>
#include <MMSTypes/libgeojson.h>
#include <MMSTypes/json.hpp>

struct Waypoint
{
	double m_x;
	double m_y;
	double m_rad;
	int zone; // used for UTM

	Waypoint() : m_x(0), m_y(0), m_rad(0.0), zone(0) {}
	Waypoint(double x, double y) : m_x(x), m_y(y), m_rad(0.0), zone(0) {}
	Waypoint(double x, double y, double w) : m_x(x), m_y(y), m_rad(w), zone(0) {}
	Waypoint(double x, double y, double w,int zone) : m_x(x), m_y(y), m_rad(w), zone(zone) {}
};

double cartesianDistanceBetweenPoints(Waypoint pt1,Waypoint pt2);
Waypoint calculatePointOnLine(Waypoint pt1,Waypoint pt2,double distance);

MMS::GeoList::List parseWaypointWKTString( const std::string& s );
MMS::GeoList::List jsonToGeoList( nlohmann::json& geo_json );
MMS::GeoList::List jsonStringToGeoList( const std::string& s );
nlohmann::json geoPointToJson( const MMS::GeoPoint& point );
nlohmann::json geoListToJson( const MMS::GeoList::List& list );

/// @brief  Estimates waypoint traversal time for ATA task bidding
/// @param  position Current robot position
/// @param  waypointsGeoJson GeoJSON waypoint list in lat/lon
/// @param  maxSpeed Maximum robot speed
/// @return Estimate of waypoint traversal time in seconds
/// @note   Includes travel time from current position to 1st waypoint
double computeWaypointTraversalTime( MMS::GeoPoint position,
                                     nlohmann::json waypointsGeoJson,
                                     double maxSpeed );

bool equalsIgnoreCase( const std::string& s1, const std::string& s2 );
std::string toUpper( const std::string& s );
std::string trim(const std::string& s);
std::map<std::string, std::string> parseConfigFile(const std::string& filename);
void writeConfigFile(const std::string& filename, const std::map<std::string, std::string>& config);
unsigned short& jausSystemID( int& jaus_id );
unsigned char& jausNodeID( int& jaus_id );
unsigned char& jausComponentID( int& jaus_id );
int jausID( unsigned short system_id, unsigned char node_id, unsigned char component_id );

static const std::string NORMAL("\033[0m");
static const std::string RED("\033[38;5;9m");
static const std::string GREEN("\033[38;5;10m");
static const std::string BLUE("\033[38;5;27m");
static const std::string YELLOW("\033[38;5;11m");
