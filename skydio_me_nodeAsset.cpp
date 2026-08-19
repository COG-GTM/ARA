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



#include <skydio_me_nodeAsset.h>

skydio_me_nodeAsset::skydio_me_nodeAsset(std::string name) : MMS::Robot(name)
{
    addAssetParameters();
}

skydio_me_nodeAsset::~skydio_me_nodeAsset()
{

}

void skydio_me_nodeAsset::addAssetParameters()
{
    addAssetParameter( MMS::Parameter( "position", "geojson", std::string("0") ) );
    addAssetParameter( MMS::Parameter( "altitude", "float", std::string("0") ) );
    addAssetParameter( MMS::Parameter( "heading", "float", std::string("0") ) );
    addAssetParameter( MMS::Parameter( "speed", "float", std::string("0") ) );
}

void skydio_me_nodeAsset::setAssetParam_position(nlohmann::json newValue) 
{ 
    m_assetParam_position = newValue;
 
    MMS::Parameter tmpParam;
    if ( !getAssetParameter( "position", tmpParam ) )
    {
        std::cerr << "[skydio_me_nodeAsset::setAssetParam_position] skydio_me_nodeAsset has no parameter named: position";
        return;
    }
    tmpParam.setValue(newValue);
    updateAssetParameter(tmpParam);
}
void skydio_me_nodeAsset::setAssetParam_altitude(float newValue) 
{ 
    m_assetParam_altitude = newValue;
 
    MMS::Parameter tmpParam;
    if ( !getAssetParameter( "altitude", tmpParam ) )
    {
        std::cerr << "[skydio_me_nodeAsset::setAssetParam_altitude] skydio_me_nodeAsset has no parameter named: altitude";
        return;
    }
    tmpParam.setValue(newValue);
    updateAssetParameter(tmpParam);
}
void skydio_me_nodeAsset::setAssetParam_heading(float newValue) 
{ 
    m_assetParam_heading = newValue;
 
    MMS::Parameter tmpParam;
    if ( !getAssetParameter( "heading", tmpParam ) )
    {
        std::cerr << "[skydio_me_nodeAsset::setAssetParam_heading] skydio_me_nodeAsset has no parameter named: heading";
        return;
    }
    tmpParam.setValue(newValue);
    updateAssetParameter(tmpParam);
}
void skydio_me_nodeAsset::setAssetParam_speed(float newValue) 
{ 
    m_assetParam_speed = newValue;
 
    MMS::Parameter tmpParam;
    if ( !getAssetParameter( "speed", tmpParam ) )
    {
        std::cerr << "[skydio_me_nodeAsset::setAssetParam_speed] skydio_me_nodeAsset has no parameter named: speed";
        return;
    }
    tmpParam.setValue(newValue);
    updateAssetParameter(tmpParam);
}
