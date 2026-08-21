/**
 * Unit tests for utility.cpp: GeoJSON <-> GeoList conversion, WKT parsing,
 * string helpers, and config-file parsing.
 */
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "utility.h"

TEST( JsonToGeoList, ParsesPoint )
{
    nlohmann::json j = nlohmann::json::parse( R"({"type":"Point","coordinates":[-79.982,40.446]})" );
    auto list = jsonToGeoList( j );
    ASSERT_EQ( list.size(), 1u );
    EXPECT_DOUBLE_EQ( list[0].getLon(), -79.982 );
    EXPECT_DOUBLE_EQ( list[0].getLat(), 40.446 );
}

TEST( JsonToGeoList, ParsesLineString )
{
    nlohmann::json j = nlohmann::json::parse(
        R"({"type":"LineString","coordinates":[[-79.98,40.44],[-79.99,40.45],[-80.00,40.46]]})" );
    auto list = jsonToGeoList( j );
    ASSERT_EQ( list.size(), 3u );
    EXPECT_DOUBLE_EQ( list[1].getLon(), -79.99 );
    EXPECT_DOUBLE_EQ( list[1].getLat(), 40.45 );
}

TEST( JsonToGeoList, ParsesPolygonOuterRing )
{
    nlohmann::json j = nlohmann::json::parse(
        R"({"type":"Polygon","coordinates":[[[0,0],[1,0],[1,1],[0,0]],[[9,9],[9,8],[8,8],[9,9]]]})" );
    auto list = jsonToGeoList( j );
    ASSERT_EQ( list.size(), 4u );
    EXPECT_DOUBLE_EQ( list[2].getLon(), 1.0 );
    EXPECT_DOUBLE_EQ( list[2].getLat(), 1.0 );
}

TEST( JsonToGeoList, UnwrapsFeature )
{
    nlohmann::json j = nlohmann::json::parse(
        R"({"type":"Feature","geometry":{"type":"Point","coordinates":[10.5,20.25]},"properties":{}})" );
    auto list = jsonToGeoList( j );
    ASSERT_EQ( list.size(), 1u );
    EXPECT_DOUBLE_EQ( list[0].getLon(), 10.5 );
    EXPECT_DOUBLE_EQ( list[0].getLat(), 20.25 );
}

TEST( JsonToGeoList, CaseInsensitiveType )
{
    nlohmann::json j = nlohmann::json::parse( R"({"type":"POINT","coordinates":[1,2]})" );
    EXPECT_EQ( jsonToGeoList( j ).size(), 1u );
}

TEST( JsonToGeoList, EmptyJsonReturnsEmpty )
{
    nlohmann::json j;
    EXPECT_TRUE( jsonToGeoList( j ).empty() );
}

TEST( JsonToGeoList, RejectsMalformedInputWithoutThrowing )
{
    // Each of these previously risked a nlohmann type_error / crash.
    for ( const char * text : {
              R"([1,2])",                                            // array, not object
              R"("just a string")",                                  // primitive
              R"({"type":123,"coordinates":[1,2]})",                 // non-string type
              R"({"type":"Point"})",                                 // no coordinates
              R"({"type":"Point","coordinates":"oops"})",            // wrong coordinate type
              R"({"type":"Point","coordinates":[1]})",               // too few numbers
              R"({"type":"Point","coordinates":["a","b"]})",         // non-numeric
              R"({"type":"LineString","coordinates":[[1,2],["x"]]})",// bad inner pair
              R"({"type":"Polygon","coordinates":[]})",              // empty rings
              R"({"type":"Polygon","coordinates":"bad"})",           // wrong ring type
              R"({"type":"Feature"})",                               // no geometry
              R"({"type":"Feature","geometry":42})",                 // non-object geometry
              R"({"type":"MultiPoint","coordinates":[[1,2]]})",      // unsupported type
          } )
    {
        nlohmann::json j = nlohmann::json::parse( text );
        EXPECT_TRUE( jsonToGeoList( j ).empty() ) << "input: " << text;
    }
}

TEST( JsonStringToGeoList, HandlesInvalidJsonText )
{
    EXPECT_TRUE( jsonStringToGeoList( "{not json" ).empty() );
    EXPECT_TRUE( jsonStringToGeoList( "" ).empty() );
    EXPECT_EQ( jsonStringToGeoList( R"({"type":"Point","coordinates":[3,4]})" ).size(), 1u );
}

TEST( GeoListJson, RoundTripsThroughLineString )
{
    MMS::GeoList::List list;
    list.push_back( MMS::GeoPoint( 40.44, -79.98 ) );
    list.push_back( MMS::GeoPoint( 40.45, -79.99 ) );

    nlohmann::json j    = geoListToJson( list );
    auto           back = jsonToGeoList( j );
    ASSERT_EQ( back.size(), 2u );
    EXPECT_DOUBLE_EQ( back[0].getLat(), 40.44 );
    EXPECT_DOUBLE_EQ( back[0].getLon(), -79.98 );
    EXPECT_DOUBLE_EQ( back[1].getLat(), 40.45 );
    EXPECT_DOUBLE_EQ( back[1].getLon(), -79.99 );
}

TEST( GeoPointJson, EncodesFeaturePoint )
{
    nlohmann::json j = geoPointToJson( MMS::GeoPoint( 40.44, -79.98 ) );
    EXPECT_EQ( j["type"], "Feature" );
    EXPECT_EQ( j["geometry"]["type"], "Point" );
    EXPECT_DOUBLE_EQ( j["geometry"]["coordinates"][0], -79.98 );
    EXPECT_DOUBLE_EQ( j["geometry"]["coordinates"][1], 40.44 );
}

TEST( ParseWaypointWKTString, ParsesPointAndLineString )
{
    auto point = parseWaypointWKTString( "POINT (-79.98 40.44)" );
    ASSERT_EQ( point.size(), 1u );
    EXPECT_DOUBLE_EQ( point[0].getLon(), -79.98 );
    EXPECT_DOUBLE_EQ( point[0].getLat(), 40.44 );

    auto line = parseWaypointWKTString( "linestring (1 2, 3 4)" );
    ASSERT_EQ( line.size(), 2u );
    EXPECT_DOUBLE_EQ( line[1].getLon(), 3.0 );
    EXPECT_DOUBLE_EQ( line[1].getLat(), 4.0 );

    EXPECT_TRUE( parseWaypointWKTString( "POLYGON ((0 0))" ).empty() );
}

TEST( StringHelpers, EqualsIgnoreCaseAndToUpperAndTrim )
{
    EXPECT_TRUE( equalsIgnoreCase( "LineString", "LINESTRING" ) );
    EXPECT_FALSE( equalsIgnoreCase( "Point", "Points" ) );
    EXPECT_EQ( toUpper( "AbC-123" ), "ABC-123" );
    EXPECT_EQ( trim( "  hi there \t " ), "hi there" );
    EXPECT_EQ( trim( " \t " ), "" );
}

TEST( CalculatePointOnLine, HandlesAxisAlignedSegments )
{
    // deltaY == 0 (horizontal) and deltaX == 0 (vertical) previously read an
    // uninitialized slope value.
    Waypoint horizontal = calculatePointOnLine( Waypoint( 0, 0 ), Waypoint( 10, 0 ), 4.0 );
    EXPECT_DOUBLE_EQ( horizontal.m_x, 4.0 );
    EXPECT_DOUBLE_EQ( horizontal.m_y, 0.0 );

    Waypoint vertical = calculatePointOnLine( Waypoint( 0, 0 ), Waypoint( 0, 10 ), 4.0 );
    EXPECT_DOUBLE_EQ( vertical.m_x, 0.0 );
    EXPECT_DOUBLE_EQ( vertical.m_y, 4.0 );

    Waypoint diagonal = calculatePointOnLine( Waypoint( 0, 0 ), Waypoint( 3, 4 ), 5.0 );
    EXPECT_NEAR( diagonal.m_x, 3.0, 1e-9 );
    EXPECT_NEAR( diagonal.m_y, 4.0, 1e-9 );
}

TEST( ParseConfigFile, ParsesKeysAndLastLineWithoutNewline )
{
    const std::string path = "/tmp/skydio_test_config.cfg";
    {
        std::ofstream out( path );
        out << "Vehicle_IP = 192.168.10.1\n";
        out << "not a key value line\n";
        out << "LOCAL_PORT = 14551"; // no trailing newline
    }

    auto config = parseConfigFile( path );
    EXPECT_EQ( config["VEHICLE_IP"], "192.168.10.1" );
    EXPECT_EQ( config["LOCAL_PORT"], "14551" );
    EXPECT_EQ( config.size(), 2u );
    std::remove( path.c_str() );
}
