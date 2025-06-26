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

#include "ns3/traces-channel-model.h"

#include "ns3/csv-reader.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/string.h"
#include <ns3/node-list.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glob.h>
#include <random>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TracesChannelModel");

NS_OBJECT_ENSURE_REGISTERED(TracesChannelModel);

/**
 * Constructor for TracesChannelModel
 * @param path Path to the scenarios folder
 * @param scenario Name of the scenario to use
 */
TracesChannelModel::TracesChannelModel(std::string path, std::string scenario)
{
    NS_LOG_FUNCTION(this);

    SetPath(path);
    SetScenario(scenario);
}

/**
 * Destructor for TracesChannelModel
 */
TracesChannelModel::~TracesChannelModel()
{
    NS_LOG_FUNCTION(this);
}

TypeId
TracesChannelModel::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::TracesChannelModel")
            .SetParent<MatrixBasedChannelModel>()
            .SetGroupName("Spectrum")
            .AddConstructor<TracesChannelModel>()
            .AddAttribute("Path",
                          "Folder path containing the channel traces input files",
                          StringValue("TestFolder"),
                          MakeStringAccessor(&TracesChannelModel::m_path),
                          MakeStringChecker())
            .AddAttribute("Scenario",
                          "Folder name for the trace based scenario (contains Input/ and Output/ "
                          "directories)",
                          StringValue("Custom"),
                          MakeStringAccessor(&TracesChannelModel::m_scenario),
                          MakeStringChecker())
            .AddAttribute("Frequency",
                          "The operating Frequency in Hz. This attribute is here "
                          "only for compatibility with ns3::ThreeGppSpectrumPropagationLossModel.",
                          DoubleValue(__DBL_MIN__),
                          MakeDoubleAccessor(&TracesChannelModel::SetFrequency,
                                             &TracesChannelModel::GetFrequency),
                          MakeDoubleChecker<double>());

    return tid;
}

/**
 * Get list of trace files matching a pattern using glob
 * @param pattern Glob pattern to match files
 * @return Vector of matching file paths
 */
std::vector<std::string>
TracesChannelModel::GetTraceFilesList(const std::string& pattern)
{
    NS_LOG_FUNCTION(this << pattern);

    glob_t glob_result;
    glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    std::vector<std::string> files;
    for (uint32_t i = 0; i < glob_result.gl_pathc; ++i)
    {
        files.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return files;
}

/**
 * Parse CSV string into vector of doubles
 * @param str CSV string to parse
 * @param toRad Convert values to radians if true
 * @return Vector of parsed double values
 */
std::vector<double>
TracesChannelModel::ParseCsv(const std::string& str, bool toRad)
{
    NS_LOG_FUNCTION(this << str);

    std::stringstream ss(str);
    CsvReader csv(ss, ',');
    csv.FetchNextRow();

    std::vector<double> vect{};
    vect.reserve(csv.ColumnCount());

    double value;
    bool ok;
    for (size_t i = 0; i < csv.ColumnCount(); i++)
    {
        ok = csv.GetValue(i, value);
        NS_ABORT_MSG_IF(!ok, "Something went wrong while parsing the line: " << str);

        if (toRad)
        {
            value = DegreesToRadians(value);
        }

        vect.push_back(value);
    }

    return vect;
}

/**
 * Extract device ID from filename (e.g., "device5.csv" -> 5)
 * @param filename Filename containing device ID
 * @return Extracted device ID as integer
 */
int
TracesChannelModel::ExtractDeviceIdFromFilename(const std::string& filename)
{
    // Find the position of "device" in the filename
    size_t devicePos = filename.find("device");
    if (devicePos == std::string::npos)
    {
        NS_FATAL_ERROR("Filename does not contain 'device': " << filename);
    }

    // Find the position of ".csv" after "device"
    size_t csvPos = filename.find(".csv", devicePos);
    if (csvPos == std::string::npos)
    {
        NS_FATAL_ERROR("Filename does not contain '.csv': " << filename);
    }

    // Extract the number between "device" and ".csv"
    std::string deviceIdStr = filename.substr(devicePos + 6, csvPos - devicePos - 6);

    try
    {
        return std::stoi(deviceIdStr);
    }
    catch (const std::exception& e)
    {
        NS_FATAL_ERROR("Could not parse device ID from filename: " << filename
                                                                   << " Error: " << e.what());
    }
}

/**
 * Read node positions from device CSV files and create mapping
 * @return Mapping from trace device IDs to ns-3 node IDs
 */
TracesChannelModel::RtIdToNs3IdMap_t
TracesChannelModel::ReadNodesPosition()
{
    NS_LOG_FUNCTION(this);

    // Get list of deviceX.csv files
    std::string devicePattern = m_path + m_scenario + "Output/Ns3/NodesPosition/device*.csv";
    auto deviceFileList = GetTraceFilesList(devicePattern);

    NS_LOG_DEBUG("Found " << deviceFileList.size() << " device files");

    TracesChannelModel::RtIdToNs3IdMap_t rtIdToNs3IdMap;

    for (const auto& deviceFile : deviceFileList)
    {
        // Extract device ID from filename
        int deviceId = ExtractDeviceIdFromFilename(deviceFile);

        // Read the first position from the device file
        CsvReader csv(deviceFile, ',');
        if (!csv.FetchNextRow())
        {
            NS_FATAL_ERROR("Could not read from device file: " << deviceFile);
        }

        // Ignore blank lines
        if (csv.IsBlankRow())
        {
            NS_FATAL_ERROR("Device file is empty: " << deviceFile);
        }

        // Expecting cartesian coordinates
        double x, y, z;
        bool ok = csv.GetValue(0, x);
        ok |= csv.GetValue(1, y);
        ok |= csv.GetValue(2, z);

        NS_ABORT_MSG_IF(!ok, "Something went wrong while parsing the file: " << deviceFile);
        Vector3D nodePosition{x, y, z};

        NS_LOG_DEBUG("Device " << deviceId << " position from file: " << nodePosition);
        m_nodePositionList.push_back(nodePosition);

        // Simple sequential mapping: trace device ID from filename maps to ns-3 node ID
        // This assumes the device IDs in filenames correspond to ns-3 node IDs
        uint32_t matchedNodeId = deviceId;

        rtIdToNs3IdMap.insert(std::make_pair(deviceId, matchedNodeId));
        m_ns3IdToRtIdMap.insert(std::make_pair(matchedNodeId, deviceId));

        std::cout << "Trace Device " << deviceId << " -> ns-3 Node " << matchedNodeId
                  << " (position: " << nodePosition << ")" << std::endl;

        NS_LOG_INFO("traceId=" << deviceId << " maps to NodeId=" << matchedNodeId
                               << " with position=" << nodePosition);

    } // for each device file

    // Print summary of device assignments
    std::cout << "\n=== Traces Channel Model Device Mapping ===" << std::endl;
    std::cout << "Total devices found: " << deviceFileList.size() << std::endl;

    // Count gNBs and UEs based on the assumption that first devices are gNBs
    // This will be determined by the user's scenario setup
    std::cout << "Note: First devices in trace scenario are assumed to be gNBs" << std::endl;
    std::cout << "      Remaining devices are assumed to be UEs" << std::endl;
    std::cout << "==========================================\n" << std::endl;

    for (auto elem : m_nodePositionList)
    {
        NS_LOG_INFO(elem);
    }

    return rtIdToNs3IdMap;
}

/**
 * Read parameter configuration file to get simulation parameters
 */
void
TracesChannelModel::ReadParaCfgFile()
{
    NS_LOG_FUNCTION(this);

    // Read configuration file with tab-separated values
    std::string paraCfgCurrentFileName{m_path + m_scenario + "Input/paraCfgCurrent.txt"};
    CsvReader csv(paraCfgCurrentFileName, '\t');
    csv.FetchNextRow(); // ignore first line (header)

    std::string varName, varValue;
    while (csv.FetchNextRow())
    {
        // Ignore blank lines
        if (csv.IsBlankRow())
        {
            continue;
        }

        // Parse variable name and value from each line
        bool ok = csv.GetValue(0, varName);
        ok |= csv.GetValue(1, varValue);
        NS_ABORT_MSG_IF(!ok,
                        "Something went wrong while parsing the file: " << paraCfgCurrentFileName);

        // Parse different configuration parameters
        if (varName.compare("numberOfTimeDivisions") == 0)
        {
            m_totTimesteps = atoi(varValue.c_str());
            NS_LOG_DEBUG("numberOfTimeDivisions (int) = " << m_totTimesteps);
        }
        else if (varName.compare("totalTimeDuration") == 0)
        {
            m_totalTimeDuration = Seconds(atof(varValue.c_str()));
            NS_LOG_DEBUG("m_totalTimeDuration = " << m_totalTimeDuration.GetSeconds() << " s");
        }
        else if (varName.compare("carrierFrequency") == 0)
        {
            m_frequency = atof(varValue.c_str());
            NS_LOG_DEBUG("carrierFrequency (float) = " << m_frequency);
        }

    } // while FetchNextRow
}

/**
 * Read trace files containing channel data for all node pairs
 * @param rtIdToNs3IdMap Mapping from trace device IDs to ns-3 node IDs
 */
void
TracesChannelModel::ReadTraceFiles(TracesChannelModel::RtIdToNs3IdMap_t rtIdToNs3IdMap)
{
    NS_LOG_FUNCTION(this);

    // Get list of all trace files in the channel directory
    NS_LOG_INFO("m_path + m_scenario = " << m_path + m_scenario);
    auto traceFileList = GetTraceFilesList(m_path + m_scenario + "Output/Ns3/Channel/*");
    NS_LOG_DEBUG("traceFileList.size ()=" << traceFileList.size());

    // Get current working directory for debugging
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string pathString = currentPath.string();

    // Process each trace file
    for (auto fileName : traceFileList)
    {
        // Extract TX and RX node IDs from filename (format: TxX_RxY.txt)
        int txIndex = fileName.find("Tx");
        int rxIndex = fileName.find("Rx");
        int txtIndex = fileName.find(".txt");

        int len{rxIndex - txIndex - 2};
        int id_tx{::atoi(fileName.substr(txIndex + 2, len).c_str())};
        len = txtIndex - rxIndex - 2;
        int id_rx{::atoi(fileName.substr(rxIndex + 2, len).c_str())};

        // Map trace IDs to ns-3 node IDs
        NS_ABORT_MSG_IF(rtIdToNs3IdMap.find(id_tx) == rtIdToNs3IdMap.end(), "ID not found for TX!");
        uint32_t nodeIdTx = rtIdToNs3IdMap.find(id_tx)->second;
        NS_ABORT_MSG_IF(rtIdToNs3IdMap.find(id_rx) == rtIdToNs3IdMap.end(), "ID not found for RX!");
        uint32_t nodeIdRx = rtIdToNs3IdMap.find(id_rx)->second;

        NS_LOG_DEBUG("id_tx: " << id_tx << ", id_rx: " << id_rx);

        // Create unique key for this node pair
        uint64_t key = GetKey(nodeIdTx, nodeIdRx);

        // Open and read trace file
        std::ifstream traceFile{fileName.c_str()};
        std::string line{};
        std::vector<TraceInfo> traceInfoVector;

        // Optional: Skip initial lines if needed (currently disabled)
        bool chop = false;
        if (chop)
        {
            int chopLine = 85;
            int choppedLine = 0;
            while (choppedLine < chopLine)
            {
                int nbLineForMPCs = 7;
                int nbLineRead = 0;
                int numMPCs = 0;
                std::getline(traceFile, line);
                numMPCs = std::stoul(line, 0, 10);

                if (numMPCs > 0)
                {
                    while (nbLineRead < nbLineForMPCs)
                    {
                        std::getline(traceFile, line);
                        nbLineRead++;
                    }
                    choppedLine++;
                }
            }
        }

        // Parse each timestep in the trace file
        while (std::getline(traceFile, line))
        {
            TraceInfo traceInfo{};
            // First line contains number of multipath components
            traceInfo.numMpcs = std::stoul(line, 0, 10);
            NS_LOG_DEBUG("numMpcs " << traceInfo.numMpcs);

            if (traceInfo.numMpcs > 0)
            {
                // Parse path delays
                std::getline(traceFile, line);
                auto pathDelays = ParseCsv(line);
                NS_ABORT_MSG_IF(pathDelays.size() != traceInfo.numMpcs,
                                "mismatch between number of path delays ("
                                    << pathDelays.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.delay_s = pathDelays;

                // Parse path gains
                std::getline(traceFile, line);
                auto pathGains = ParseCsv(line);
                NS_ABORT_MSG_IF(pathGains.size() != traceInfo.numMpcs,
                                "mismatch between number of path gains ("
                                    << pathGains.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.pathGain_dbpow = pathGains;

                // Parse path phases
                std::getline(traceFile, line);
                auto pathPhases = ParseCsv(line);
                NS_ABORT_MSG_IF(pathPhases.size() != traceInfo.numMpcs,
                                "mismatch between number of path phases ("
                                    << pathPhases.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.phase_rad = pathPhases;

                // Parse elevation angles of departure (AoD)
                std::getline(traceFile, line);
                auto pathElevAod = ParseCsv(line, true);
                NS_ABORT_MSG_IF(pathElevAod.size() != traceInfo.numMpcs,
                                "mismatch between number of path elev AoDs ("
                                    << pathElevAod.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.elAod_rad = pathElevAod;

                // Parse azimuth angles of departure (AoD)
                std::getline(traceFile, line);
                auto pathAzAod = ParseCsv(line, true);
                NS_ABORT_MSG_IF(pathAzAod.size() != traceInfo.numMpcs,
                                "mismatch between number of path az AoDs ("
                                    << pathAzAod.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.azAod_rad = pathAzAod;

                // Parse elevation angles of arrival (AoA)
                std::getline(traceFile, line);
                auto pathElevAoa = ParseCsv(line, true);
                NS_ABORT_MSG_IF(pathElevAoa.size() != traceInfo.numMpcs,
                                "mismatch between number of path elev AoAs ("
                                    << pathElevAoa.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.elAoa_rad = pathElevAoa;

                // Parse azimuth angles of arrival (AoA)
                std::getline(traceFile, line);
                auto pathAzAoa = ParseCsv(line, true);
                NS_ABORT_MSG_IF(pathAzAoa.size() != traceInfo.numMpcs,
                                "mismatch between number of path az AoAs ("
                                    << pathAzAoa.size() << ") and number of MPCs ("
                                    << traceInfo.numMpcs << "), timestep="
                                    << traceInfoVector.size() + 1 << ", fileName=" << fileName);
                traceInfo.azAoa_rad = pathAzAoa;
            }
            traceInfoVector.push_back(traceInfo);
        }

        // Store trace data for this node pair
        NS_LOG_DEBUG("traceInfoVector.size ()=" << traceInfoVector.size());
        m_traceInfoMap.insert(std::make_pair(key, traceInfoVector));
    }
}

/**
 * Read all input files for the current scenario
 */
void
TracesChannelModel::ReadAllInputFiles()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("ReadAllInputFiles for scenario " << m_scenario << " path " << m_path);

    m_ns3IdToRtIdMap.clear();
    m_traceInfoMap.clear();

    ReadParaCfgFile();
    TracesChannelModel::RtIdToNs3IdMap_t rtIdToNs3IdMap = ReadNodesPosition();
    ReadTraceFiles(rtIdToNs3IdMap);

    // Setup simulation timings assuming constant periodicity
    m_updatePeriod = Seconds((double)m_totalTimeDuration.GetSeconds() /
                             (double)m_totTimesteps); // Dictate how often the channel is updated

    NS_LOG_DEBUG("m_totalTimeDuration=" << m_totalTimeDuration.GetSeconds()
                                        << " s"
                                           ", m_updatePeriod="
                                        << m_updatePeriod.GetNanoSeconds() / 1e6
                                        << " ms"
                                           ", m_totTimesteps="
                                        << m_totTimesteps);
}

/**
 * Get the total simulation time from trace data
 * @return Total simulation duration
 */
Time
TracesChannelModel::GetTracesSimTime() const
{
    NS_LOG_FUNCTION(this);
    return m_totalTimeDuration;
}

/**
 * Set the operating frequency (for compatibility with ThreeGppSpectrumPropagationLossModel)
 * @param fc Frequency in Hz
 */
void
TracesChannelModel::SetFrequency(double fc)
{
    NS_LOG_FUNCTION(this);
}

/**
 * Get the operating frequency
 * @return Frequency in Hz
 */
double
TracesChannelModel::GetFrequency() const
{
    NS_LOG_FUNCTION(this);
    return m_frequency;
}

/**
 * Trim folder name to ensure proper formatting (no multiple slashes)
 * @param folder Folder path to trim
 */
void
TracesChannelModel::TrimFolderName(std::string& folder)
{
    // Remove leading multiple slashes to avoid path issues
    while (folder.front() == '/' && folder.substr(1, folder.size()).front() == '/')
    {
        folder = folder.substr(1, folder.size());
    }

    // Remove trailing slashes and add single trailing slash
    while (folder.back() == '/')
    {
        folder = folder.substr(0, folder.size() - 1);
    }

    folder += '/';
}

/**
 * Set the scenario name and read associated input files
 * @param scenario Name of the scenario
 */
void
TracesChannelModel::SetScenario(std::string scenario)
{
    NS_LOG_FUNCTION(this << scenario);
    // scenario = "Indoor1";
    NS_ABORT_MSG_IF(m_path == "", "m_path empty, use SetPath first");

    TrimFolderName(scenario);

    if (scenario != m_scenario // avoid re-reading input files
        && scenario != "")
    {
        m_scenario = scenario;
        // read the information for this scenario
        ReadAllInputFiles();
    }
}

/**
 * Get the current scenario name
 * @return Current scenario name
 */
std::string
TracesChannelModel::GetScenario() const
{
    NS_LOG_FUNCTION(this);
    return m_scenario;
}

/**
 * Set the path to the scenarios folder
 * @param path Path to scenarios folder
 */
void
TracesChannelModel::SetPath(std::string path)
{
    NS_LOG_FUNCTION(this << path);
    TrimFolderName(path);
    m_path = path;
}

/**
 * Get the current path to scenarios folder
 * @return Current path
 */
std::string
TracesChannelModel::GetPath() const
{
    NS_LOG_FUNCTION(this);
    return m_path;
}

/**
 * Check if channel matrix needs to be updated based on coherence time
 * @param channelMatrix Current channel matrix
 * @return True if update is needed, false otherwise
 */
bool
TracesChannelModel::ChannelMatrixNeedsUpdate(
    Ptr<MatrixBasedChannelModel::ChannelMatrix> channelMatrix) const
{
    NS_LOG_FUNCTION(this << channelMatrix);
    uint64_t nowTimestep = GetTimestep();
    uint64_t lastChanUpdateTimestep = GetTimestep(channelMatrix->m_generatedTime);
    NS_ASSERT_MSG(nowTimestep >= lastChanUpdateTimestep,
                  "Current timestep=" << nowTimestep << ", last channel update timestep="
                                      << lastChanUpdateTimestep);

    bool update = false;
    // if the coherence time is over the channel has to be updated
    if (lastChanUpdateTimestep < nowTimestep)
    {
        NS_LOG_LOGIC("Generation time " << channelMatrix->m_generatedTime.GetNanoSeconds()
                                        << " now " << Simulator::Now().GetNanoSeconds()
                                        << " update needed");
        update = true;
    }
    else
    {
        NS_LOG_LOGIC("Generation time " << channelMatrix->m_generatedTime.GetNanoSeconds()
                                        << " now " << Simulator::Now().GetNanoSeconds()
                                        << " update not needed");
    }
    return update;
}

/**
 * Get channel matrix for a pair of mobility models and antennas
 * @param aMob First mobility model
 * @param bMob Second mobility model
 * @param aAntenna First antenna array
 * @param bAntenna Second antenna array
 * @return Channel matrix
 */
Ptr<const MatrixBasedChannelModel::ChannelMatrix>
TracesChannelModel::GetChannel(Ptr<const MobilityModel> aMob,
                               Ptr<const MobilityModel> bMob,
                               Ptr<const PhasedArrayModel> aAntenna,
                               Ptr<const PhasedArrayModel> bAntenna)
{
    NS_LOG_FUNCTION(this << aMob << bMob << aAntenna << bAntenna);

    // Create keys for channel lookup and caching
    // Channel params key uses node IDs (trace data is based on node pairs)
    uint32_t aNodeId = aMob->GetObject<Node>()->GetId();
    uint32_t bNodeId = bMob->GetObject<Node>()->GetId();
    uint64_t channelParamsKey = GetKey(aNodeId, bNodeId);

    // Channel matrix key uses antenna IDs (matrices depend on antenna configurations)
    uint32_t aAntennaId = aAntenna->GetId();
    uint32_t bAntennaId = bAntenna->GetId();
    uint64_t channelMatrixKey = GetKey(aAntennaId, bAntennaId);

    NS_LOG_DEBUG("channelParamsKey "
                 << channelParamsKey << ", channelMatrixKey " << channelMatrixKey
                 << ", node aId=" << aNodeId << " bId=" << bNodeId << ", antenna aId=" << aAntennaId
                 << " bId=" << bAntennaId << ", RT sim. aId=" << m_ns3IdToRtIdMap[aNodeId]
                 << " bId=" << m_ns3IdToRtIdMap[bNodeId]);

    // Check if channel matrix exists in cache and needs update
    bool update = false;
    bool notFound = false;
    Ptr<MatrixBasedChannelModel::ChannelMatrix> channelMatrix;
    if (m_channelMatrixMap.find(channelMatrixKey) != m_channelMatrixMap.end())
    {
        // Channel matrix found in cache
        NS_LOG_LOGIC("channel matrix present in the map");
        channelMatrix = m_channelMatrixMap[channelMatrixKey];

        // Check if coherence time has expired and update is needed
        update = ChannelMatrixNeedsUpdate(channelMatrix);
    }
    else
    {
        NS_LOG_LOGIC("channel matrix not found");
        notFound = true;
    }

    // Generate new channel matrix if not found or needs update
    if (notFound || update)
    {
        NS_LOG_LOGIC("channelMatrix notFound=" << notFound << " || update=" << update);
        channelMatrix = GetNewChannel(aMob, bMob, aAntenna, bAntenna);
        channelMatrix->m_antennaPair = std::make_pair(aAntennaId, bAntennaId);

        // Update generation timestamp for coherence time tracking
        channelMatrix->m_generatedTime = Simulator::Now();

        // Store in cache using antenna IDs as key
        m_channelMatrixMap[channelMatrixKey] = channelMatrix;
    }

    return channelMatrix;
}

/**
 * Generate a new channel matrix from trace data
 * @param aMob First mobility model
 * @param bMob Second mobility model
 * @param aAntenna First antenna array
 * @param bAntenna Second antenna array
 * @return Newly generated channel matrix
 */
Ptr<MatrixBasedChannelModel::ChannelMatrix>
TracesChannelModel::GetNewChannel(Ptr<const MobilityModel> aMob,
                                  Ptr<const MobilityModel> bMob,
                                  Ptr<const PhasedArrayModel> aAntenna,
                                  Ptr<const PhasedArrayModel> bAntenna)
{
    NS_LOG_FUNCTION(this << aMob << bMob << aAntenna << bAntenna);

    // Create new channel matrix structure
    Ptr<MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
        Create<MatrixBasedChannelModel::ChannelMatrix>();

    // Get current simulation timestep
    uint32_t timestep = GetTimestep();

    // Extract node IDs for trace data lookup
    uint32_t aNodeId = aMob->GetObject<Node>()->GetId();
    uint32_t bNodeId = bMob->GetObject<Node>()->GetId();
    uint64_t traceChannelId = GetKey(aNodeId, bNodeId);

    // Extract antenna IDs for matrix identification
    uint32_t aAntennaId = aAntenna->GetId();
    uint32_t bAntennaId = bAntenna->GetId();

    // Get trace data for current timestep
    TraceInfo traceInfo = m_traceInfoMap.at(traceChannelId)[timestep];

    // Calculate antenna array dimensions
    uint64_t bSize = bAntenna->GetNumColumns() * bAntenna->GetNumRows(); // TR++ To check with Paolo
    uint64_t aSize = aAntenna->GetNumColumns() * aAntenna->GetNumRows(); // TR++ To check with Paolo

    NS_LOG_DEBUG("timestep=" << timestep << ", node aId=" << aNodeId << ", node bId=" << bNodeId
                             << ", antenna aId=" << aAntennaId << ", antenna bId=" << bAntennaId
                             << ", m_ns3IdToRtIdMap[aNodeId]=" << m_ns3IdToRtIdMap.at(aNodeId)
                             << ", m_ns3IdToRtIdMap[bNodeId]=" << m_ns3IdToRtIdMap.at(bNodeId)
                             << ", traceChannelId=" << traceChannelId << ", bSize=" << bSize
                             << ", aSize=" << aSize);

    // Initialize 3D channel matrix:
    // H[receiver_elements][transmitter_elements][multipath_components]
    MatrixBasedChannelModel::Complex3DVector H(bSize, aSize, traceInfo.numMpcs);

    // Process each multipath component
    for (uint64_t mpcIndex = 0; mpcIndex < traceInfo.numMpcs; ++mpcIndex)
    {
        // Calculate initial phase from delay and frequency
        double initialPhase =
            -2 * M_PI * traceInfo.delay_s[mpcIndex] * m_frequency + traceInfo.phase_rad[mpcIndex];

        // Get path gain from trace data
        double pathGain = traceInfo.pathGain_dbpow[mpcIndex]; // TR++ Fix Paolo

        // Create angle objects for antenna pattern calculations
        Angles bAngle = Angles(traceInfo.azAoa_rad[mpcIndex], traceInfo.elAoa_rad[mpcIndex]);
        Angles aAngle = Angles(traceInfo.azAod_rad[mpcIndex], traceInfo.elAod_rad[mpcIndex]);
        NS_LOG_DEBUG("aAngle: " << aAngle << ", bAngle: " << bAngle);

        // Calculate antenna element gains (ignore polarization)
        double bFieldPattH, bFieldPattV, aFieldPattH, aFieldPattV;
        std::tie(bFieldPattH, bFieldPattV) = bAntenna->GetElementFieldPattern(bAngle);
        double bElementGain = std::sqrt(bFieldPattH * bFieldPattH + bFieldPattV * bFieldPattV);

        std::tie(aFieldPattH, aFieldPattV) = aAntenna->GetElementFieldPattern(aAngle);
        double aElementGain = std::sqrt(aFieldPattH * aFieldPattH + aFieldPattV * aFieldPattV);

        // Calculate total path gain including antenna effects
        double pgTimesGains = pathGain * bElementGain * aElementGain;
        std::complex<double> complexRay = pgTimesGains * std::polar(1.0, initialPhase);

        NS_LOG_DEBUG("traceInfo.delay_s[mpcIndex]="
                     << traceInfo.delay_s[mpcIndex] << ", traceInfo.phase_rad[mpcIndex]="
                     << traceInfo.phase_rad[mpcIndex] << ", traceInfo.pathGain_dbpow[mpcIndex]="
                     << traceInfo.pathGain_dbpow[mpcIndex] << ", bAngle=" << bAngle << ", aAngle="
                     << aAngle << ", initialPhase=" << initialPhase << ", pathGain=" << pathGain
                     << ", bElementGain=" << bElementGain << ", aElementGain=" << aElementGain
                     << ", pgTimesGains=" << pgTimesGains << ", complexRay=" << complexRay);

        // Calculate channel coefficients for each antenna element pair
        for (uint64_t bIndex = 0; bIndex < bSize; ++bIndex)
        {
            // Get receiver element location and calculate phase shift
            Vector uLoc = bAntenna->GetElementLocation(bIndex);
            double bPhaseElementPhase =
                2 * M_PI *
                (sin(traceInfo.elAoa_rad[mpcIndex]) * cos(traceInfo.azAoa_rad[mpcIndex]) * uLoc.x +
                 sin(traceInfo.elAoa_rad[mpcIndex]) * sin(traceInfo.azAoa_rad[mpcIndex]) * uLoc.y +
                 cos(traceInfo.elAoa_rad[mpcIndex]) * uLoc.z);
            std::complex<double> bWeight = std::polar(1.0, bPhaseElementPhase);

            for (uint64_t aIndex = 0; aIndex < aSize; ++aIndex)
            {
                // Get transmitter element location and calculate phase shift
                Vector sLoc = aAntenna->GetElementLocation(aIndex);
                // Minus sign: complex conjugate for TX steering vector
                double aPhaseElementPhase = 2 * M_PI *
                                            (sin(traceInfo.elAod_rad[mpcIndex]) *
                                                 cos(traceInfo.azAod_rad[mpcIndex]) * sLoc.x +
                                             sin(traceInfo.elAod_rad[mpcIndex]) *
                                                 sin(traceInfo.azAod_rad[mpcIndex]) * sLoc.y +
                                             cos(traceInfo.elAod_rad[mpcIndex]) * sLoc.z);
                std::complex<double> aWeight = std::polar(1.0, aPhaseElementPhase);

                // Combine all contributions for this element pair and MPC
                std::complex<double> ray = complexRay * bWeight * aWeight;
                H(bIndex, aIndex, mpcIndex) += ray;
            }
        }
    }

    // Create and populate channel parameters structure
    Ptr<MatrixBasedChannelModel::ChannelParams> channelParams =
        Create<MatrixBasedChannelModel::ChannelParams>();
    channelMatrix->m_channel = H;
    channelParams->m_delay = traceInfo.delay_s;
    channelParams->m_angle.clear();
    channelParams->m_angle.push_back(traceInfo.azAoa_rad);
    channelParams->m_angle.push_back(traceInfo.elAoa_rad);
    channelParams->m_angle.push_back(traceInfo.azAod_rad);
    channelParams->m_angle.push_back(traceInfo.elAod_rad);
    channelParams->m_generatedTime = Simulator::Now();
    channelParams->m_nodeIds = std::make_pair(aNodeId, bNodeId);

    // Initialize Doppler terms (set to 0 for trace-based model)
    // These terms account for additional Doppler contribution due to moving objects
    // in vehicular scenarios, but are not used in trace-based simulations
    DoubleVector dopplerTermAlpha;
    DoubleVector dopplerTermD;
    for (uint8_t cIndex = 0; cIndex < H.GetNumPages(); cIndex++)
    {
        // Set the alpha and D as described in 3GPP TR 37.885 v15.3.0, Sec. 6.2.3,
        // both to 0, to avoid introducing additional scatter terms
        dopplerTermAlpha.push_back(0.0);
        dopplerTermD.push_back(0.0);
    }
    channelParams->m_alpha = dopplerTermAlpha;
    channelParams->m_D = dopplerTermD;

    // Store channel parameters using node IDs (since trace data is based on node pairs)
    m_channelParamsMap[traceChannelId] = channelParams;
    return channelMatrix;
}

/**
 * Get channel parameters for a pair of mobility models
 * @param aMob First mobility model
 * @param bMob Second mobility model
 * @return Channel parameters or nullptr if not found
 */
Ptr<const MatrixBasedChannelModel::ChannelParams>
TracesChannelModel::GetParams(Ptr<const MobilityModel> aMob, Ptr<const MobilityModel> bMob) const
{
    NS_LOG_FUNCTION(this);

    // Compute the channel key. The key is reciprocal, i.e., key (a, b) = key (b, a)
    uint64_t channelParamsKey =
        GetKey(aMob->GetObject<Node>()->GetId(), bMob->GetObject<Node>()->GetId());

    if (m_channelParamsMap.find(channelParamsKey) != m_channelParamsMap.end())
    {
        return m_channelParamsMap.find(channelParamsKey)->second;
    }
    else
    {
        NS_LOG_WARN("Channel params map not found. Returning a nullptr.");
        return nullptr;
    }
}

/**
 * Get current timestep based on simulation time
 * @return Current timestep
 */
uint64_t
TracesChannelModel::GetTimestep(void) const
{
    return GetTimestep(Simulator::Now());
}

/**
 * Get timestep for a given time
 * @param t Time to get timestep for
 * @return Timestep corresponding to the given time
 */
uint64_t
TracesChannelModel::GetTimestep(Time t) const
{
    NS_LOG_FUNCTION(this << t);

    // Ensure update period is properly set
    NS_ASSERT_MSG(m_updatePeriod.GetNanoSeconds() > 0.0,
                  "TracesChannelModel update period not set correctly");

    // Calculate timestep based on time and update period
    uint64_t timestep = t.GetNanoSeconds() / m_updatePeriod.GetNanoSeconds();
    NS_LOG_DEBUG("t = " << t.GetNanoSeconds() << " ns"
                        << ", updatePeriod = " << m_updatePeriod.GetNanoSeconds() << " ns"
                        << ", timestep = " << timestep);

    // Ensure simulation doesn't exceed trace data duration
    NS_ASSERT_MSG(timestep <= m_totTimesteps,
                  "Simulator is running for longer that expected: timestep > m_totTimesteps");

    return timestep;
}

} // namespace ns3