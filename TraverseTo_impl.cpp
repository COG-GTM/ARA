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

#include "TraverseTo_impl.h"

TraverseTo_impl::TraverseTo_impl() : TraverseToInterface() 
{

}

void TraverseTo_impl::user_configure()
{

}

void TraverseTo_impl::user_unconfigure()
{

}

MMS::ServiceInterface::Ptr TraverseTo_impl::clone() const
{
    return MMS::ServiceInterface::Ptr( new TraverseTo_impl( ) );
}

void TraverseTo_impl::userSignal_handlestart(const startInputSignalData & signalData)
{
    std::cout << "Handling start Signal" << std::endl;

}
void TraverseTo_impl::userSignal_handleupdate(const updateInputSignalData & signalData)
{
    std::cout << "Handling update Signal" << std::endl;

}
void TraverseTo_impl::userSignal_handlepause()
{
    std::cout << "Handling pause Signal" << std::endl;

}

void TraverseTo_impl::userSignal_handleresume()
{
    std::cout << "Handling resume Signal" << std::endl;

}

void TraverseTo_impl::userSignal_handlestop()
{
    std::cout << "Handling stop Signal" << std::endl;

}

