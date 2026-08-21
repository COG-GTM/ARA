/**
 * Test stub for MMSLib/ServiceInterface.h (MPMS SDK).
 *
 * Implements just enough of MMS::ServiceInterface and MMS::Parameter for the
 * generated TraverseToInterface and the TraverseTo_impl behavior to compile
 * and run in unit tests. Fired output signals and status values are recorded
 * so tests can assert on behavior-level side effects.
 */
#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <MMSTypes/json.hpp>

namespace MMS
{

class Parameter
{
public:
    using ParameterMap = std::map<std::string, Parameter>;

    Parameter() = default;
    Parameter( const std::string & name, const std::string & type, const std::string & value )
        : m_name( name ), m_type( type ), m_value( value )
    {
    }

    template <typename T>
    bool getValue( T & out ) const;

    std::string toString() const { return m_value; }
    const std::string & getName() const { return m_name; }
    const std::string & getType() const { return m_type; }

private:
    std::string m_name;
    std::string m_type;
    std::string m_value;
};

template <>
inline bool Parameter::getValue<float>( float & out ) const
{
    try
    {
        out = std::stof( m_value );
        return true;
    }
    catch ( ... )
    {
        return false;
    }
}

template <>
inline bool Parameter::getValue<nlohmann::json>( nlohmann::json & out ) const
{
    out = nlohmann::json::parse( m_value, nullptr, false );
    return !out.is_discarded();
}

class ServiceInterface
{
public:
    using Ptr      = std::shared_ptr<ServiceInterface>;
    using ConstPtr = std::shared_ptr<ServiceInterface const>;
    using SignalCallback = std::function<void( double, const Parameter::ParameterMap & )>;

    ServiceInterface( const std::string & name, double version )
        : m_name( name ), m_version( version )
    {
    }
    virtual ~ServiceInterface() = default;

    const std::string & getName() const { return m_name; }

    void addIncomingSignal( const std::string & name ) { m_incomingSignals.push_back( name ); }
    void addOutgoingSignal( const std::string & name ) { m_outgoingSignals.push_back( name ); }

    Parameter::ParameterMap & getConfigParameters() { return m_configParameters; }

    void registerIncomingCallback( const std::string & name, SignalCallback callback )
    {
        m_callbacks[name] = std::move( callback );
    }

    void fireOutgoingSignal( const std::string & name, const Parameter::ParameterMap & params ) const
    {
        std::lock_guard<std::mutex> lock( m_recordMutex );
        m_firedSignals.push_back( name );
        (void)params;
    }

    void setStatusValue( const std::string & key, const std::string & value )
    {
        std::lock_guard<std::mutex> lock( m_recordMutex );
        m_statusValues[key] = value;
        m_statusHistory.push_back( value );
    }

    // ---- test observation hooks (not part of the real SDK API) ----
    std::vector<std::string> testFiredSignals() const
    {
        std::lock_guard<std::mutex> lock( m_recordMutex );
        return m_firedSignals;
    }
    std::string testStatusValue( const std::string & key ) const
    {
        std::lock_guard<std::mutex> lock( m_recordMutex );
        auto it = m_statusValues.find( key );
        return it == m_statusValues.end() ? std::string() : it->second;
    }
    std::vector<std::string> testStatusHistory() const
    {
        std::lock_guard<std::mutex> lock( m_recordMutex );
        return m_statusHistory;
    }
    const std::vector<std::string> & testIncomingSignals() const { return m_incomingSignals; }
    const std::vector<std::string> & testOutgoingSignals() const { return m_outgoingSignals; }

    void testInvokeIncomingSignal( const std::string & name, const Parameter::ParameterMap & params )
    {
        m_callbacks.at( name )( m_version, params );
    }

private:
    std::string m_name;
    double      m_version = 0.0;

    std::vector<std::string>              m_incomingSignals;
    std::vector<std::string>              m_outgoingSignals;
    Parameter::ParameterMap               m_configParameters;
    std::map<std::string, SignalCallback> m_callbacks;

    mutable std::mutex                 m_recordMutex;
    mutable std::vector<std::string>   m_firedSignals;
    std::map<std::string, std::string> m_statusValues;
    std::vector<std::string>           m_statusHistory;
};

} // namespace MMS
