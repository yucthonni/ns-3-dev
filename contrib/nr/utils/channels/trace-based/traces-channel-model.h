/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 SIGNET Lab, Department of Information Engineering,
 * University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * The ns-3 QD app code (https://github.com/signetlabdei/qd-channel) was used as the baseline for
 * this code.
 * Modified by NIST <tanguy.ropitault@nist.gov>
 */

#ifndef TRACES_CHANNEL_MODEL_H
#define TRACES_CHANNEL_MODEL_H

#include "ns3/angles.h"
#include "ns3/boolean.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include <ns3/matrix-based-channel-model.h>
#include <ns3/three-gpp-channel-model.h>

#include <complex.h>
#include <map>

namespace ns3
{

class PhasedArrayModel;
class MatrixBasedChannelModel;
class MobilityModel;

/**
 * \ingroup spectrum
 *
 */
class TracesChannelModel : public MatrixBasedChannelModel
{
  public:
    /**
     * Constructor
     *
     * \param path folder path containing the scenario of interest
     * \param scenario scenario folder name, containg the Input/ and the Output/Ns3/ folders
     */
    TracesChannelModel(std::string path = "", std::string scenario = "");

    /**
     * Destructor
     */
    virtual ~TracesChannelModel() override;

    /**
     * Get the type ID
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Returns a matrix with a realization of the channel between
     * the nodes with mobility objects passed as input parameters.
     *
     * \param aMob mobility model of the a device
     * \param bMob mobility model of the b device
     * \param aAntenna antenna of the a device
     * \param bAntenna antenna of the b device
     * \return the channel matrix
     */
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> GetChannel(
        Ptr<const MobilityModel> aMob,
        Ptr<const MobilityModel> bMob,
        Ptr<const PhasedArrayModel> aAntenna,
        Ptr<const PhasedArrayModel> bAntenna) override;

    /**
     * Looks for the channel params associated to the aMob and bMob pair in
     * m_channelParamsMap. If not found it will return a nullptr.
     *
     * \param aMob mobility model of the a device
     * \param bMob mobility model of the b device
     * \return the channel params
     */
    Ptr<const MatrixBasedChannelModel::ChannelParams> GetParams(
        Ptr<const MobilityModel> aMob,
        Ptr<const MobilityModel> bMob) const override;

    /*
     * Set the folder path containing the scenario of interest
     *
     * \param path folder path containing the scenario of interest
     */
    void SetPath(std::string path);

    /*
     * Get the folder path of the scenario of interest
     *
     * \return folder path containing the scenario of interest
     */
    std::string GetPath() const;

    /*
     * Set the scenario folder name, containg the Input/ and the Output/Ns3/ folders.
     * The scenario has to be set only after the Path has already been set.
     * This triggers the import of the scenario trace files.
     *
     * \param scenario folder name, containg the Input/ and the Output/Ns3/ folders
     */
    void SetScenario(std::string scenario);

    /*
     * Get the scenario folder name, containg the Input/ and the Output/Ns3/ folders.
     *
     * \return scenario folder name, containg the Input/ and the Output/Ns3/ folders
     */
    std::string GetScenario() const;

    /**
     * Just a dummy setter for compatibility reasons.
     * NOTE: the carrier frequency is imported from the trace input
     * files. This setter should not be manually used, and it is here
     * only because attributes are required to have a setter.
     *
     * \param fc the center frequency in Hz
     */
    void SetFrequency(double fc);

    /**
     * Returns the center frequency
     *
     * \return the center frequency in Hz
     */
    double GetFrequency(void) const;

    /**
     * Get the total simulation time
     * \return the simulation time considered in the trace files
     */
    Time GetTracesSimTime() const;

  private:
    using RtIdToNs3IdMap_t = std::map<uint32_t, uint32_t>;
    using Ns3IdToRtIdMap_t = std::map<uint32_t, uint32_t>;

    /**
     * Read paraCfgCurrent.txt file and imports necessary member variables
     */
    void ReadParaCfgFile(void);

    /**
     * Get the channel matrix between a and b using the ray tracer data
     *
     * \param aMob mobility model of the a device
     * \param bMob mobility model of the b device
     * \param aAntenna antenna of the a device
     * \param bAntenna antenna of the b device
     * \return the channel matrix
     */
    Ptr<MatrixBasedChannelModel::ChannelMatrix> GetNewChannel(Ptr<const MobilityModel> aMob,
                                                              Ptr<const MobilityModel> bMob,
                                                              Ptr<const PhasedArrayModel> aAntenna,
                                                              Ptr<const PhasedArrayModel> bAntenna);

    /**
     * Check if the channel matrix has to be updated
     * \param channelMatrix channel matrix
     * \return true if the channel matrix has to be updated, false otherwise
     */
    bool ChannelMatrixNeedsUpdate(Ptr<MatrixBasedChannelModel::ChannelMatrix> channelMatrix) const;

    /**
     * Get traces-channel time-step of current time
     * \return traces-channel time-step of current time
     */
    uint64_t GetTimestep(void) const;

    /**
     * Get traces-channel time-step
     * \param t time to convert in timestep
     * \return traces-channel time-step
     */
    uint64_t GetTimestep(Time t) const;

    /**
     * Read all the configuration files
     */
    void ReadAllInputFiles();

    /**
     * Read all NodesPosition for the given scenario
     */
    RtIdToNs3IdMap_t ReadNodesPosition(void);

    /**
     * Extract device ID from filename (e.g., "device5.csv" -> 5)
     * \param filename the filename to extract device ID from
     * \return the device ID as an integer
     */
    int ExtractDeviceIdFromFilename(const std::string& filename);

    /**
     * Read all trace files for the given scenario
     */
    void ReadTraceFiles(RtIdToNs3IdMap_t rtIdToNs3IdMap);

    /**
     * Get list of files matching a pattern
     * \param pattern the pattern to match
     * \return vector of matching file names
     */
    std::vector<std::string> GetTraceFilesList(const std::string& pattern);

    /**
     * Parse CSV string into vector of doubles
     * \param str the CSV string to parse
     * \param toRad convert to radians if true
     * \return vector of parsed values
     */
    std::vector<double> ParseCsv(const std::string& str, bool toRad = false);

    /**
     * Trim folder name
     * \param folder the folder name to trim
     */
    static void TrimFolderName(std::string& folder);

    struct TraceInfo
    {
        uint64_t numMpcs;
        std::vector<double> delay_s;
        std::vector<double> pathGain_dbpow;
        std::vector<double> phase_rad;
        std::vector<double> elAod_rad;
        std::vector<double> azAod_rad;
        std::vector<double> elAoa_rad;
        std::vector<double> azAoa_rad;
    };

    std::string m_path;                       //!< Path to the scenario folder
    std::string m_scenario;                   //!< Scenario folder name
    double m_frequency;                       //!< Carrier frequency
    Time m_totalTimeDuration;                 //!< Total time duration
    uint32_t m_totTimesteps;                  //!< Total number of time steps
    Time m_updatePeriod;                      //!< Update period for channel matrix
    std::vector<Vector3D> m_nodePositionList; //!< List of node positions
    RtIdToNs3IdMap_t m_rtIdToNs3IdMap;        //!< Map from trace device ID to ns-3 node ID
    Ns3IdToRtIdMap_t m_ns3IdToRtIdMap;        //!< Map from ns-3 node ID to trace device ID
    std::map<uint64_t, std::vector<TraceInfo>> m_traceInfoMap; //!< Map of trace information
    std::map<uint64_t, Ptr<MatrixBasedChannelModel::ChannelMatrix>>
        m_channelMatrixMap; //!< Map of channel matrices
    std::map<uint64_t, Ptr<MatrixBasedChannelModel::ChannelParams>>
        m_channelParamsMap; //!< Map of channel parameters
    std::map<uint64_t, Time>
        m_channelMatrixLastUpdateMap; //!< Map of last update times for channel matrices
    std::map<uint64_t, Time>
        m_channelParamsLastUpdateMap; //!< Map of last update times for channel parameters
};

} // namespace ns3

#endif /* TRACES_CHANNEL_MODEL_H */