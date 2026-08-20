// This work contains valuable confidential and proprietary information.
// Disclosure or reproduction without the written authorization of Neya
// Systems, LLC is prohibited. This unpublished work by Neya Systems, LLC
// is protected by the laws of the United States and other countries. If
// publication of the work should occur, the following notice shall apply.

// Copyright (c) 2025 Neya Systems, LLC
// All Rights Reserved

// The software/firmware is provided to you on an As-Is basis

// Delivered to the U.S. Government with Government Purpose Rights, as defined in
// DFARS Part 252.227-7013 or 7014 (Feb 2014). This material may be
// reproduced by or for the U.S Government pursuant to the copyright
// license under the clause at DFARS 227-7203-5(a), DFARS 227.7103-5(a),
// DFARS 252.227-7013(b)(1)(June 1995), DFARS 252.227-7014 (June 1995),
// and FAR 52.227-14(a). Use of this work other than as specifically
// authorized by the U.S. Government may violate any copyrights that
// exist in this work.

// "WARNING - This file contains software and technical data whose export is
// restricted by the Arms Export Control Act (Title 22, U.S.C., Sec
// 2751, et seq.) or the Export Administration Act of 1979 (Title 50,
// U.S.C., App. 2401 et seq.), as amended. Violations of these export
// laws are subject to severe criminal penalties. Disseminate in
// accordance with provisions of DoD Directive 5230.25."

#include <csignal>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <iomanip>

#include "skydio_me_nodeAsset_impl.h"
#include "SkydioMavlinkClient.h"
#include <UDPCommInterface/UDPCommInterface.h>
#include "utility.h"

static const std::string ASSET_NAME = "ASSET_NAME";
static const std::string ASSET_TYPE = "ASSET_TYPE";
static const std::string ASSET_CLASS = "ASSET_CLASS";
static const std::string UDP_RX_PORT = "UDP_RX_PORT";
static const std::string UDP_HOSTS_FILE = "UDP_HOSTS_FILE";
static const std::string UDP_REPORT_TRAFFIC = "UDP_REPORT_TRAFFIC";
static const std::string SKYDIO_VEHICLE_IP = "SKYDIO_VEHICLE_IP";
static const std::string SKYDIO_VEHICLE_PORT = "SKYDIO_VEHICLE_PORT";
static const std::string SKYDIO_LOCAL_PORT = "SKYDIO_LOCAL_PORT";
static const std::string SKYDIO_GCS_SYSTEM_ID = "SKYDIO_GCS_SYSTEM_ID";
static const std::string SKYDIO_TARGET_SYSTEM_ID = "SKYDIO_TARGET_SYSTEM_ID";


// Signal handling
std::atomic<bool> g_shutdown( false );
void signalHandler( int signum )
{
    g_shutdown.store( true );
}

bool configDefaults( std::map<std::string, std::string>& config )
{
    std::map<std::string,std::string> defaults;
    bool missing = false;

    // default config values
    defaults[ ASSET_NAME ]         = "DEFAULT_NAME";
    defaults[ ASSET_TYPE ]         = "DEFAULT_TYPE";
    defaults[ ASSET_CLASS ]        = "DEFAULT_CLASS";
    defaults[ UDP_RX_PORT ]        = "5001";
    defaults[ UDP_HOSTS_FILE ]     = "./hosts.json";
    defaults[ UDP_REPORT_TRAFFIC ] = "false";
    defaults[ SKYDIO_VEHICLE_IP ]        = "192.168.10.1";
    defaults[ SKYDIO_VEHICLE_PORT ]      = "14550";
    defaults[ SKYDIO_LOCAL_PORT ]        = "14551";
    defaults[ SKYDIO_GCS_SYSTEM_ID ]     = "255";
    defaults[ SKYDIO_TARGET_SYSTEM_ID ]  = "1";

    // add any missing default values to config
    for( const std::pair<std::string,std::string>& entry : defaults )
    {
        std::string key( toUpper( trim( entry.first ) ) );
        std::string value = (key == UDP_HOSTS_FILE || key == SKYDIO_VEHICLE_IP) ? entry.second : toUpper( trim( entry.second ) );

        if( config.find( key ) == config.end() )
        {
            std::cout << "\"" << key << "\" missing from config file, setting to default \"" << value << "\"" << std::endl;
            config[ key ] = value;
            missing = true;
        }
    }

    return missing;
}

int main( int argc, char* argv[] )
{
   // Read in configuration file.
    std::string config_file( "skydio_me_node.cfg" );
    std::map<std::string,std::string> config;

    // TODO: currently assumes UDP comms.
    unsigned short udp_rx_port;
    bool udp_report_traffic;

    std::cout << std::setprecision(5) << std::fixed;
    std::cout << "Starting skydio_me_node Mission Executor Node" << std::endl;

    // get config file from command line
    if ( argc > 1 )
        config_file = argv[1];

    std::cout << "Loading configs from \"" << config_file << "\"" << std::endl;

    // parse config file
    //  add defaults if necessary
    //  write a new config file if defaults added
    config = parseConfigFile( config_file );
    if ( configDefaults( config ) )
    {
        std::cout << "Writing to \"" << config_file << "\" with missing default values" << std::endl;
        writeConfigFile(config_file, config);
    }

    // print config values
    std::cout << std::endl << "Config values: " << std::endl;
    for ( const std::pair<std::string, std::string> entry : config )
    {
        const std::string& key( entry.first );
        const std::string& value( entry.second );

        std::cout << "Parameter \"" << key << "\" = \"" << value << "\"" << std::endl;
    }

    udp_rx_port        = std::stoul( config[ UDP_RX_PORT ] );
    udp_report_traffic = equalsIgnoreCase( config[ UDP_REPORT_TRAFFIC ], "TRUE" ) ? true : false;

    // set up the Asset Class
    skydio_me_nodeAsset_impl asset( config[ ASSET_NAME ] );
    asset.setAssetType( config[ ASSET_TYPE ] );
    asset.setAssetClass( config[ ASSET_CLASS ] );

    // Load UDP comm interface
    auto udpCommInterfacePtr = new NeyaSystems::UDPCommInterface(
            udp_rx_port, config[ UDP_HOSTS_FILE ], "", udp_report_traffic );
    asset.addCommInterface( MMS::CommInterface::Ptr( udpCommInterfacePtr ) );

    // Connect the native RAS-A/MAVLink link to the Skydio X10D
    auto & drone = skydio::SkydioMavlinkClient::instance();
    drone.configure( config[ SKYDIO_VEHICLE_IP ],
                     static_cast<unsigned short>( std::stoul( config[ SKYDIO_VEHICLE_PORT ] ) ),
                     static_cast<unsigned short>( std::stoul( config[ SKYDIO_LOCAL_PORT ] ) ),
                     static_cast<uint8_t>( std::stoul( config[ SKYDIO_GCS_SYSTEM_ID ] ) ),
                     static_cast<uint8_t>( std::stoul( config[ SKYDIO_TARGET_SYSTEM_ID ] ) ) );
    if ( !drone.connect() )
        std::cerr << "Failed to open MAVLink UDP link to Skydio X10D at "
                  << config[ SKYDIO_VEHICLE_IP ] << ":" << config[ SKYDIO_VEHICLE_PORT ] << std::endl;

    // set signal handler
    signal( SIGINT, signalHandler );

    // main loop
    while ( true )
    {
        // run at 10 Hz
        std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

        if ( g_shutdown.load() ) break; // exit normally after SIGINT

        // publish X10D telemetry (position, altitude, heading, speed) at 10 Hz
        asset.publishTelemetry();
    }

    // update status
    asset.updateAssetParameter( MMS::Parameter( "status", "string", "SHUTDOWN" ) );

    drone.disconnect();

    std::cout << std::endl << "Shutting down skydio_me_node Mission Executor Node ..." << std::endl;

    return 0;
}
