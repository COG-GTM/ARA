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



#include <skydio_me_nodeAsset_impl.h>

#include <TraverseTo_impl.h>

skydio_me_nodeAsset_impl::skydio_me_nodeAsset_impl(std::string name) : skydio_me_nodeAsset(name)
{
    addBehaviorImplementations();

    nlohmann::json positionJson = geojson::Feature( geojson::Point( 0.0,0.0 ), nlohmann::json() );

    setAssetParam_position(positionJson);
    setAssetParam_heading(0.0);
    setAssetParam_speed(0.0);
    setAssetParam_altitude(0.0);
}

skydio_me_nodeAsset_impl::~skydio_me_nodeAsset_impl()
{

}


void skydio_me_nodeAsset_impl::addBehaviorImplementations()
{
    TraverseTo_impl::Ptr traversetoPtr;
    traversetoPtr.reset(new TraverseTo_impl);
    addServiceInterface(MMS::ServiceInterface::Ptr(traversetoPtr));

}