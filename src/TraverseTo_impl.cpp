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

/**
 * TraverseTo behavior implementation for the Skydio X10D.
 *
 * Translates the abstract MPMS TraverseTo behavior into the X10D's native
 * RAS-A/MAVLink protocol (see "X10D Control and Telemetry ICD"):
 *
 *   start  -> Mission Protocol upload (MISSION_COUNT / MISSION_ITEM_INT /
 *             MISSION_ACK), MAV_CMD_COMPONENT_ARM_DISARM,
 *             MAV_CMD_MISSION_START; progress monitored via
 *             MISSION_ITEM_REACHED / MISSION_CURRENT
 *   update -> re-upload of the flight plan with the new waypoint list
 *   pause  -> MAV_CMD_DO_PAUSE_CONTINUE (param1 = 0, hold)
 *   resume -> MAV_CMD_DO_PAUSE_CONTINUE (param1 = 1, continue)
 *   stop   -> MAV_CMD_DO_PAUSE_CONTINUE (hold) + MISSION_CLEAR_ALL
 *
 * WaypointListComplete is fired from the mission worker thread (never from
 * an input signal handler) once the final NAV_WAYPOINT item is reached.
 */

#include "TraverseTo_impl.h"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace
{
constexpr float WAYPOINT_ACCEPTANCE_RADIUS_M = 2.0f;
}

TraverseTo_impl::TraverseTo_impl() : TraverseToInterface()
{

}

TraverseTo_impl::~TraverseTo_impl()
{
    stopWorker();
}

void TraverseTo_impl::user_configure()
{
    m_altitude_m = getConfigParam_altitude_m();
    m_speed_mps  = getConfigParam_speed_mps();

    const float maxVelocity = getConfigParam_maxVelocity_mps();
    const float minVelocity = getConfigParam_minVelocity_mps();
    if ( maxVelocity > 0.f && m_speed_mps > maxVelocity )
        m_speed_mps = maxVelocity;
    if ( minVelocity > 0.f && m_speed_mps < minVelocity )
        m_speed_mps = minVelocity;

    setStatus( "PENDING" );
}

void TraverseTo_impl::user_unconfigure()
{
    stopWorker();
}

MMS::ServiceInterface::Ptr TraverseTo_impl::clone() const
{
    return MMS::ServiceInterface::Ptr( new TraverseTo_impl( ) );
}

MMS::GeoList::List TraverseTo_impl::resolveWaypoints(const nlohmann::json & signalJson)
{
    // Prefer waypoints carried on the signal; fall back to the
    // waypoint_list config parameter populated at mission instantiation.
    nlohmann::json waypointJson = signalJson;

    if ( waypointJson.is_object() && waypointJson.contains( "waypoint_list" ) )
        waypointJson = waypointJson["waypoint_list"];

    MMS::GeoList::List waypoints;
    if ( !waypointJson.empty() && !waypointJson.is_number() )
        waypoints = jsonToGeoList( waypointJson );

    if ( waypoints.empty() )
    {
        nlohmann::json configJson = getConfigParam_waypoint_list();
        if ( !configJson.empty() && !configJson.is_number() )
            waypoints = jsonToGeoList( configJson );
    }

    return waypoints;
}

std::vector<skydio::MissionItem> TraverseTo_impl::buildFlightPlan(const MMS::GeoList::List & waypoints) const
{
    std::vector<skydio::MissionItem> flightPlan;
    uint16_t seq = 0;

    // Item 0: takeoff to traversal altitude at the current location.
    skydio::MissionItem takeoff;
    takeoff.seq     = seq++;
    takeoff.command = skydio::MAV_CMD_NAV_TAKEOFF;
    takeoff.current = 1;
    takeoff.param4  = NAN; // yaw: hold current
    takeoff.z       = m_altitude_m;
    flightPlan.push_back( takeoff );

    // Item 1: traversal speed for the remainder of the plan.
    skydio::MissionItem speed;
    speed.seq     = seq++;
    speed.command = skydio::MAV_CMD_DO_CHANGE_SPEED;
    speed.param1  = 1.f;         // speed type: ground speed
    speed.param2  = m_speed_mps; // m/s
    speed.param3  = -1.f;        // throttle: no change
    flightPlan.push_back( speed );

    // NAV_WAYPOINT items, one per traversal point, at relative altitude.
    for ( const auto & waypoint : waypoints )
    {
        skydio::MissionItem item;
        item.seq     = seq++;
        item.command = skydio::MAV_CMD_NAV_WAYPOINT;
        item.param1  = 0.f;                           // hold time (s)
        item.param2  = WAYPOINT_ACCEPTANCE_RADIUS_M;  // acceptance radius (m)
        item.param3  = 0.f;                           // pass through waypoint
        item.param4  = NAN;                           // yaw: vehicle default
        item.x       = static_cast<int32_t>( std::lround( waypoint.getLat() * 1e7 ) );
        item.y       = static_cast<int32_t>( std::lround( waypoint.getLon() * 1e7 ) );
        item.z       = m_altitude_m;
        flightPlan.push_back( item );
    }

    return flightPlan;
}

void TraverseTo_impl::executeMission(std::vector<skydio::MissionItem> flightPlan)
{
    auto & drone = skydio::SkydioMavlinkClient::instance();

    if ( flightPlan.empty() )
    {
        m_missionActive.store( false );
        return;
    }

    const uint16_t lastSeq = flightPlan.back().seq;

    setStatus( "EXECUTING_NOMINAL" );

    if ( !drone.waitForHeartbeat( 10s ) )
    {
        std::cerr << RED << "[TraverseTo_impl] no RAS-A heartbeat from X10D, aborting traverse" << NORMAL << std::endl;
        setStatus( "PENDING" );
        m_missionActive.store( false );
        return;
    }

    drone.resetMissionProgress();

    // Mission Protocol upload transaction.
    if ( !drone.uploadMission( flightPlan, 30s ) )
    {
        std::cerr << RED << "[TraverseTo_impl] flight plan upload to X10D failed" << NORMAL << std::endl;
        setStatus( "PENDING" );
        m_missionActive.store( false );
        return;
    }

    // Arm (no-op if already airborne/armed) and start the mission.
    if ( !drone.getTelemetry().armed && !drone.arm( true ) )
    {
        std::cerr << RED << "[TraverseTo_impl] X10D rejected arming request" << NORMAL << std::endl;
        setStatus( "PENDING" );
        m_missionActive.store( false );
        return;
    }

    if ( !drone.startMission( 0, lastSeq ) )
    {
        std::cerr << RED << "[TraverseTo_impl] X10D rejected MAV_CMD_MISSION_START" << NORMAL << std::endl;
        setStatus( "PENDING" );
        m_missionActive.store( false );
        return;
    }

    std::cout << GREEN << "[TraverseTo_impl] X10D executing " << flightPlan.size()
              << "-item flight plan at " << m_altitude_m << " m AGL, " << m_speed_mps
              << " m/s" << NORMAL << std::endl;

    // Monitor MISSION_ITEM_REACHED / MISSION_CURRENT until the final
    // NAV_WAYPOINT is reached or a stop is requested.
    while ( !m_stopRequested.load() )
    {
        const skydio::Telemetry telemetry = drone.getTelemetry();
        if ( telemetry.reachedSeq >= static_cast<int>( lastSeq ) )
        {
            std::cout << GREEN << "[TraverseTo_impl] final waypoint reached, traverse complete" << NORMAL << std::endl;
            setStatus( "COMPLETE" );
            fireWaypointListComplete();
            break;
        }
        std::this_thread::sleep_for( 200ms );
    }

    m_missionActive.store( false );
}

void TraverseTo_impl::stopWorker()
{
    m_stopRequested.store( true );
    std::lock_guard<std::mutex> lock( m_missionMutex );
    if ( m_missionThread.joinable() )
        m_missionThread.join();
}

void TraverseTo_impl::userSignal_handlestart(const startInputSignalData & signalData)
{
    std::cout << "Handling start Signal" << std::endl;

    nlohmann::json signalJson;
    signalData.getdata( signalJson ); // optional; falls back to config waypoint_list

    MMS::GeoList::List waypoints = resolveWaypoints( signalJson );
    if ( waypoints.empty() )
    {
        std::cerr << RED << "[TraverseTo_impl] start received without a usable waypoint list" << NORMAL << std::endl;
        return;
    }

    stopWorker();
    m_stopRequested.store( false );
    m_missionActive.store( true );

    std::lock_guard<std::mutex> lock( m_missionMutex );
    m_missionThread = std::thread( &TraverseTo_impl::executeMission, this, buildFlightPlan( waypoints ) );
}

void TraverseTo_impl::userSignal_handleupdate(const updateInputSignalData & signalData)
{
    std::cout << "Handling update Signal" << std::endl;

    nlohmann::json signalJson;
    if ( !signalData.getdata( signalJson ) )
        return;

    MMS::GeoList::List waypoints = resolveWaypoints( signalJson );
    if ( waypoints.empty() )
        return;

    // Hold in place, then restart execution against the new waypoint list.
    auto & drone = skydio::SkydioMavlinkClient::instance();
    drone.pauseMission();

    stopWorker();
    m_stopRequested.store( false );
    m_missionActive.store( true );

    std::lock_guard<std::mutex> lock( m_missionMutex );
    m_missionThread = std::thread( &TraverseTo_impl::executeMission, this, buildFlightPlan( waypoints ) );
}

void TraverseTo_impl::userSignal_handlepause()
{
    std::cout << "Handling pause Signal" << std::endl;

    skydio::SkydioMavlinkClient::instance().pauseMission();
}

void TraverseTo_impl::userSignal_handleresume()
{
    std::cout << "Handling resume Signal" << std::endl;

    skydio::SkydioMavlinkClient::instance().resumeMission();
}

void TraverseTo_impl::userSignal_handlestop()
{
    std::cout << "Handling stop Signal" << std::endl;

    stopWorker();

    auto & drone = skydio::SkydioMavlinkClient::instance();
    drone.pauseMission();          // hold in place
    if ( !drone.clearMission( std::chrono::seconds( 15 ) ) ) // remove the uploaded plan
        std::cerr << RED << "[TraverseTo_impl] MISSION_CLEAR_ALL failed after retries; "
                     "vehicle is holding but the uploaded plan may remain onboard" << NORMAL << std::endl;
    setStatus( "PENDING" );
}
