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

//  ************************************
//  ***  THIS CODE IS AUTO-GENERATED ***
//  ***    DO NOT EDIT THIS CODE     ***
//  ************************************
//  Behavior Type:    TraverseTo
//  Behavior Version: 2.0


#pragma once

#include <MMSLib/ServiceInterface.h>
#include <MMSTypes/libgeojson.h>
#include "utility.h"

// DEFAULT TASK STATUS VALUES:
//
//  COMPLETE
//  PENDING
//  EXECUTING_NOMINAL
//

class TraverseToInterface : public MMS::ServiceInterface
{

protected:

    class startInputSignalData 
    {
        public:
        startInputSignalData() :
        m_data_valid(false)
        {};

        ~startInputSignalData() {};

        bool getdata(nlohmann::json & data) const
        {
            if (m_data_valid)
            {
                data = m_data;
            }
            return m_data_valid;
        }
        void setdata(nlohmann::json data)
        {
            m_data_valid = true;
            m_data = data;
        }

        private:
            bool m_data_valid;
            nlohmann::json m_data;
    };


    class updateInputSignalData 
    {
        public:
        updateInputSignalData() :
        m_data_valid(false)
        {};

        ~updateInputSignalData() {};

        bool getdata(nlohmann::json & data) const
        {
            if (m_data_valid)
            {
                data = m_data;
            }
            return m_data_valid;
        }
        void setdata(nlohmann::json data)
        {
            m_data_valid = true;
            m_data = data;
        }

        private:
            bool m_data_valid;
            nlohmann::json m_data;
    };



public:

    typedef std::shared_ptr<TraverseToInterface> Ptr;
    typedef std::shared_ptr<TraverseToInterface const> ConstPtr;

    TraverseToInterface() : MMS::ServiceInterface("TraverseTo", 1.0) {
        addIncomingSignal("start");
        addIncomingSignal("update");
        addIncomingSignal("pause");
        addIncomingSignal("resume");
        addIncomingSignal("stop");
        addOutgoingSignal("WaypointListComplete");

        initializeConfigParameters();
        initializeStatusValues();
    }
    virtual ~TraverseToInterface() {}

    // Developer implements these methods in their inherited class to handle startup and bringdown of behaviors.
    virtual void user_configure() = 0;
    virtual void user_unconfigure() = 0;
    virtual MMS::ServiceInterface::Ptr clone() const = 0;

    // Developer implements these handlers for the input signals to the behavior.
    virtual void userSignal_handlestart(const startInputSignalData & signalData) = 0;
    virtual void userSignal_handleupdate(const updateInputSignalData & signalData) = 0;
    virtual void userSignal_handlepause() = 0;
    virtual void userSignal_handleresume() = 0;
    virtual void userSignal_handlestop() = 0;

    float getConfigParam_maxVelocity_mps() { return m_cparam_maxVelocity_mps; }
    void setConfigParam_maxVelocity_mps(float newValue) { m_cparam_maxVelocity_mps = newValue; }
    float getConfigParam_minVelocity_mps() { return m_cparam_minVelocity_mps; }
    void setConfigParam_minVelocity_mps(float newValue) { m_cparam_minVelocity_mps = newValue; }
    float getConfigParam_altitude_m() { return m_cparam_altitude_m; }
    void setConfigParam_altitude_m(float newValue) { m_cparam_altitude_m = newValue; }
    float getConfigParam_speed_mps() { return m_cparam_speed_mps; }
    void setConfigParam_speed_mps(float newValue) { m_cparam_speed_mps = newValue; }
    nlohmann::json getConfigParam_waypoint_list() { return m_cparam_waypoint_list; }
    void setConfigParam_waypoint_list(nlohmann::json newValue) { m_cparam_waypoint_list = newValue; }

protected:

    //Functions below are implemented in this header, do not overload.
    virtual void handlestart(double version, const MMS::Parameter::ParameterMap & incomingParams) 
    {
        startInputSignalData data_start;
        MMS::Parameter parameter;
        std::string key;

        key = "data";
        if (incomingParams.find(key) != incomingParams.end())
        {
            parameter = incomingParams.at(key);
            nlohmann::json tmpData;
            if ( !parameter.getValue<nlohmann::json>( tmpData  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::handlestart] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            } else
            {
                std::cerr << GREEN << "[TraverseToInterface::handlestart] Parsed Signal data value for \"" << key << "\" : " << tmpData << NORMAL << std::endl;
                data_start.setdata(tmpData);
            }
        }

        userSignal_handlestart(data_start);
    }
    virtual void handleupdate(double version, const MMS::Parameter::ParameterMap & incomingParams) 
    {
        updateInputSignalData data_update;
        MMS::Parameter parameter;
        std::string key;

        key = "data";
        if (incomingParams.find(key) != incomingParams.end())
        {
            parameter = incomingParams.at(key);
            nlohmann::json tmpData;
            if ( !parameter.getValue<nlohmann::json>( tmpData  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::handleupdate] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            } else
            {
                std::cerr << GREEN << "[TraverseToInterface::handleupdate] Parsed Signal data value for \"" << key << "\" : " << tmpData << NORMAL << std::endl;
                data_update.setdata(tmpData);
            }
        }

        userSignal_handleupdate(data_update);
    }
    virtual void handlepause(double version, const MMS::Parameter::ParameterMap & incomingParams) 
    {
        userSignal_handlepause();
    }
    virtual void handleresume(double version, const MMS::Parameter::ParameterMap & incomingParams) 
    {
        userSignal_handleresume();
    }
    virtual void handlestop(double version, const MMS::Parameter::ParameterMap & incomingParams) 
    {
        userSignal_handlestop();
    }

    //Functions below are implemented in this header, do not overload.
    inline void fireWaypointListComplete() const {
        MMS::Parameter::ParameterMap outgoingParams;
        fireOutgoingSignal("WaypointListComplete", outgoingParams);
    }

private:

    //Initializes configuration parameters using defaults from MSD file
    inline void initializeConfigParameters() 
    {
        MMS::Parameter::ParameterMap & configParameters = getConfigParameters();

        configParameters["maxVelocity_mps"] = MMS::Parameter("maxVelocity_mps", "float", std::string("1"));
        configParameters["minVelocity_mps"] = MMS::Parameter("minVelocity_mps", "float", std::string("1"));
        configParameters["altitude_m"] = MMS::Parameter("altitude_m", "float", std::string("20"));
        configParameters["speed_mps"] = MMS::Parameter("speed_mps", "float", std::string("2"));
        configParameters["waypoint_list"] = MMS::Parameter("waypoint_list", "geojson", std::string("0"));
    }

    inline void initializeStatusValues() 
    {
        addStatusType("COMPLETE","TASK_COMPLETE");
        addStatusType("PENDING","TASK_PENDING");
        addStatusType("EXECUTING_NOMINAL","TASK_EXECUTING_NOMINAL");
    }

    std::map<std::string,std::string> m_statusMap;

    // typed Configuration parameters.
    float m_cparam_maxVelocity_mps;
    float m_cparam_minVelocity_mps;
    float m_cparam_altitude_m;
    float m_cparam_speed_mps;
    nlohmann::json m_cparam_waypoint_list;

public:

    /// *********************************************************
    /// MPMS specific backend controls, do not edit or overload.
    /// *********************************************************
    
    inline void initializeCallbacks() {
        registerIncomingCallback("start", std::bind(&TraverseToInterface::handlestart, this, std::placeholders::_1, std::placeholders::_2));
        registerIncomingCallback("update", std::bind(&TraverseToInterface::handleupdate, this, std::placeholders::_1, std::placeholders::_2));
        registerIncomingCallback("pause", std::bind(&TraverseToInterface::handlepause, this, std::placeholders::_1, std::placeholders::_2));
        registerIncomingCallback("resume", std::bind(&TraverseToInterface::handleresume, this, std::placeholders::_1, std::placeholders::_2));
        registerIncomingCallback("stop", std::bind(&TraverseToInterface::handlestop, this, std::placeholders::_1, std::placeholders::_2));
    }

    inline const std::string & getMSDDescriptionString() const {
        const static std::string msdString = "TODO: insert full json here.";
        return msdString;
    }

    inline virtual bool configureService()
    {
        MMS::Parameter::ParameterMap &config_params( getConfigParameters() );
        std::string key;
        MMS::Parameter parameter;

        std::cout << BLUE << "[TraverseToInterface::configureService]" << NORMAL << " configuring \"" << getName() << "\"" << std::endl;

        key = "maxVelocity_mps";
        if (config_params.find("maxVelocity_mps") != config_params.end())
        {
            parameter = config_params[key];
            if ( !parameter.getValue<float>( m_cparam_maxVelocity_mps  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::configureService] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            }
            std::cerr << GREEN << "[TraverseToInterface::configureService] Parsed Config value for \"" << key << "\" : " << m_cparam_maxVelocity_mps << NORMAL << std::endl;
        } else 
        {
            std::cerr << RED << "[TraverseToInterface::configureService] Could not find configuration parameter:  \"" << key << "\"" << NORMAL << std::endl;
        }
        key = "minVelocity_mps";
        if (config_params.find("minVelocity_mps") != config_params.end())
        {
            parameter = config_params[key];
            if ( !parameter.getValue<float>( m_cparam_minVelocity_mps  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::configureService] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            }
            std::cerr << GREEN << "[TraverseToInterface::configureService] Parsed Config value for \"" << key << "\" : " << m_cparam_minVelocity_mps << NORMAL << std::endl;
        } else 
        {
            std::cerr << RED << "[TraverseToInterface::configureService] Could not find configuration parameter:  \"" << key << "\"" << NORMAL << std::endl;
        }
        key = "altitude_m";
        if (config_params.find("altitude_m") != config_params.end())
        {
            parameter = config_params[key];
            if ( !parameter.getValue<float>( m_cparam_altitude_m  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::configureService] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            }
            std::cerr << GREEN << "[TraverseToInterface::configureService] Parsed Config value for \"" << key << "\" : " << m_cparam_altitude_m << NORMAL << std::endl;
        } else 
        {
            std::cerr << RED << "[TraverseToInterface::configureService] Could not find configuration parameter:  \"" << key << "\"" << NORMAL << std::endl;
        }
        key = "speed_mps";
        if (config_params.find("speed_mps") != config_params.end())
        {
            parameter = config_params[key];
            if ( !parameter.getValue<float>( m_cparam_speed_mps  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::configureService] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            }
            std::cerr << GREEN << "[TraverseToInterface::configureService] Parsed Config value for \"" << key << "\" : " << m_cparam_speed_mps << NORMAL << std::endl;
        } else 
        {
            std::cerr << RED << "[TraverseToInterface::configureService] Could not find configuration parameter:  \"" << key << "\"" << NORMAL << std::endl;
        }
        key = "waypoint_list";
        if (config_params.find("waypoint_list") != config_params.end())
        {
            parameter = config_params[key];
            if ( !parameter.getValue<nlohmann::json>( m_cparam_waypoint_list  ) )
            {
                    std::cerr << RED << "[TraverseToInterface::configureService] failed to parse \"" << key << "\" : " << parameter.toString() << NORMAL << std::endl;
            }
            std::cerr << GREEN << "[TraverseToInterface::configureService] Parsed Config value for \"" << key << "\" : " << m_cparam_waypoint_list << NORMAL << std::endl;
        } else 
        {
            std::cerr << RED << "[TraverseToInterface::configureService] Could not find configuration parameter:  \"" << key << "\"" << NORMAL << std::endl;
        }

        user_configure();

        return true;
    }

    virtual void unconfigureService()
    {
        std::cerr << BLUE << "[TraverseToInterface::unconfigureService]" << NORMAL << " unconfiguring \"" << getName() << "\"" << std::endl;

        user_unconfigure();
    }

    inline void setStatus(std::string statusKey) 
    { 
        if (m_statusMap.find(statusKey) != m_statusMap.end())
        {
            setStatusValue("status", m_statusMap[statusKey]);
        } else 
        {
            std::cerr << RED << "[TraverseToInterface::setStatus] Attemped to set Behavior status with uninitialized key type: " << statusKey << NORMAL << std::endl;
        }
    }

    void addStatusType(std::string key, std::string value) { m_statusMap[key] = value; }
};
