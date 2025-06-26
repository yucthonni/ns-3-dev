/**
 * @file traces-channel-example.cc
 * @brief NR simulation with Traces channel model for realistic channel modeling
 *
 * This example demonstrates a complete NR simulation setup with:
 * - Multiple gNBs and UEs configuration
 * - Traces channel model for realistic channel modeling
 * - SINR measurements and logging
 * - UDP traffic generation and flow monitoring
 * - Beamforming with CellScanBeamforming
 *
 * Key features:
 * - Uses Traces channel model for realistic channel modeling
 * - Implements beamforming with CellScanBeamforming
 * - Logs SINR values, positions, and flow statistics to CSV files
 * - Configurable simulation parameters (packet size, lambda, antenna config, etc.)
 * - Configurable UE-gNB attachment modes (closest distance or ID-based assignment)
 *
 * Attachment modes:
 * - "closest": UEs attach to the nearest gNB based on distance
 * - "id-based": UEs attach to gNBs based on their index (UE 0,1 → gNB 0, UE 2,3 → gNB 1, etc.)
 *
 * Output files:
 * - sinr_trace.csv: Contains timestamp, cell ID, RNTI, SINR (dB), and BWP ID
 * - position_trace.csv: Contains UE position and velocity data
 * - flow_stats.csv: Contains flow statistics (throughput, delay, jitter, PDR)
 * - beamformingVector.csv: Contains beamforming vector data
 * - simulation_results.txt: Contains end-of-simulation summary
 *
 * @author NIST
 * @date 2024
 */

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @brief NR simulation example using Traces channel model
 *
 * This example describes how to setup a simulation using the Traces channel model.
 * The simulation consists of a configurable number of gNBs and UEs with realistic
 * channel modeling and comprehensive performance monitoring.
 *
 * The example will print on-screen the end-to-end results of flows,
 * as well as writing detailed statistics to files.
 *
 * \code{.unparsed}
$ ./ns3 run "traces-channel-example --PrintHelp"
    \endcode
 */

/**
 * Useful references:
 * [1] 3GPP TS 38.300 - NR and NG-RAN Overall Description
 * [2] 3GPP TR 38.901 - Study on channel model for frequencies from 0.5 to 100 GHz
 * [3] ns-3 documentation - Logging module
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traces-channel-model.h"

#include <cstring>
#include <errno.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TracesChannelExample");

// Global variables for file management
std::string g_sinrTraceFile;
std::string g_positionTraceFile;
std::string tracesFolder = "contrib/nr/utils/channels/trace-based/Scenarios/";
std::string tracesScenario = "Etoile";

// Global variables for flow monitoring
std::map<uint32_t, uint64_t> g_lastRxBytes;
std::map<uint32_t, Time> g_lastTime;
std::map<uint32_t, uint32_t> g_lastRxPackets;
std::map<uint32_t, uint32_t> g_lastTxPackets;
std::map<uint32_t, Time> g_lastDelaySum;
std::map<uint32_t, Time> g_lastJitterSum;

// Global variables for statistics
std::ofstream g_statsFile;
Ptr<FlowMonitor> g_monitor;

/**
 * Callback function to log SINR measurements
 */
void
ReportSinrTrace(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    std::ofstream sinrFile;
    sinrFile.open(g_sinrTraceFile, std::ios_base::app);
    double sinrDb = 10 * std::log10(sinr);
    sinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << sinrDb
             << "," << static_cast<int>(bwpId) << std::endl;
    sinrFile.close();
}

/**
 * Callback function to track UE position changes
 * @param context The node ID and device type (e.g., "NodeList/5/$ns3::MobilityModel/CourseChange")
 * @param mobility The mobility model that triggered the callback
 */
void
CourseChange(std::string context, Ptr<const MobilityModel> mobility)
{
    // Extract node ID from context string
    // Context format: "/NodeList/X/$ns3::MobilityModel/CourseChange"
    size_t nodeListPos = context.find("/NodeList/");
    size_t mobilityPos = context.find("/$ns3::MobilityModel/CourseChange");

    if (nodeListPos != std::string::npos && mobilityPos != std::string::npos)
    {
        std::string nodeIdStr = context.substr(nodeListPos + 10, mobilityPos - nodeListPos - 10);
        uint32_t nodeId = std::stoul(nodeIdStr);

        Vector pos = mobility->GetPosition();
        Vector vel = mobility->GetVelocity();

        std::ofstream posFile;
        posFile.open(g_positionTraceFile, std::ios_base::app);
        posFile << Simulator::Now().GetSeconds() << "," << nodeId << "," << pos.x << "," << pos.y
                << "," << pos.z << "," << vel.x << "," << vel.y << "," << vel.z << std::endl;
        posFile.close();

        std::cout << "UE Node " << nodeId
                  << " position changed at t=" << Simulator::Now().GetSeconds() << "s -> (" << pos.x
                  << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    else
    {
        // Fallback if context parsing fails
        Vector pos = mobility->GetPosition();
        Vector vel = mobility->GetVelocity();

        std::ofstream posFile;
        posFile.open(g_positionTraceFile, std::ios_base::app);
        posFile << Simulator::Now().GetSeconds() << ",-1," << pos.x << "," << pos.y << "," << pos.z
                << "," << vel.x << "," << vel.y << "," << vel.z << std::endl;
        posFile.close();

        std::cout << "Unknown device position changed at t=" << Simulator::Now().GetSeconds()
                  << "s -> (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
}

/**
 * Callback function to report flow statistics
 */
void
ReportStats()
{
    FlowMonitor::FlowStatsContainer stats = g_monitor->GetFlowStats();
    Time now = Simulator::Now();

    // Open file if not already open
    if (!g_statsFile.is_open())
    {
        std::string flowStatsFile =
            g_sinrTraceFile.substr(0, g_sinrTraceFile.find_last_of('/')) + "/flow_stats.csv";
        std::string dir = flowStatsFile.substr(0, flowStatsFile.find_last_of('/'));

        g_statsFile.open(flowStatsFile);

        if (!g_statsFile.is_open())
        {
            std::cout << "Failed to open file: " << flowStatsFile << std::endl;
            std::cout << "Error: " << strerror(errno) << std::endl;
        }
        else
        {
            g_statsFile
                << "Time,FlowId,Throughput(Mbps),AvgDelay(ms),AvgJitter(ms),PDR,InstDelay(ms),"
                   "InstJitter(ms),InstPDR"
                << std::endl;
        }
    }

    for (auto& stat : stats)
    {
        uint32_t flowId = stat.first;
        uint64_t currentRxBytes = stat.second.rxBytes;
        uint32_t rxPackets = stat.second.rxPackets;
        uint32_t txPackets = stat.second.txPackets;

        // Calculate metrics if we have previous data and received packets
        if (g_lastRxBytes.find(flowId) != g_lastRxBytes.end())
        {
            double timeDiff = (now - g_lastTime[flowId]).GetSeconds();

            // Calculate throughput (Mbps)
            double throughput =
                (currentRxBytes - g_lastRxBytes[flowId]) * 8.0 / timeDiff / 1000.0 / 1000.0;

            // Calculate average delay (ms)
            double avgDelay =
                rxPackets > 0 ? (1000 * stat.second.delaySum.GetSeconds() / rxPackets) : 0;

            // Calculate average jitter (ms)
            double avgJitter =
                rxPackets > 0 ? (1000 * stat.second.jitterSum.GetSeconds() / rxPackets) : 0;

            // Calculate packet delivery ratio
            double pdr = txPackets > 0 ? static_cast<double>(rxPackets) / txPackets : 0;

            // Calculate instantaneous metrics for current window
            uint32_t currentRxPackets = stat.second.rxPackets - g_lastRxPackets[flowId];
            uint32_t currentTxPackets = stat.second.txPackets - g_lastTxPackets[flowId];
            double instDelay =
                currentRxPackets > 0
                    ? (1000 *
                       (stat.second.delaySum.GetSeconds() - g_lastDelaySum[flowId].GetSeconds()) /
                       currentRxPackets)
                    : 0;
            double instJitter =
                currentRxPackets > 0
                    ? (1000 *
                       (stat.second.jitterSum.GetSeconds() - g_lastJitterSum[flowId].GetSeconds()) /
                       currentRxPackets)
                    : 0;
            double instPdr =
                currentTxPackets > 0 ? static_cast<double>(currentRxPackets) / currentTxPackets : 0;

            // Print statistics to console
            std::cout << "Time: " << now.GetSeconds() << "s, Flow: " << flowId
                      << ", Throughput: " << throughput << " Mbps"
                      << ", Avg Delay: " << avgDelay << " ms"
                      << ", Avg Jitter: " << avgJitter << " ms"
                      << ", PDR: " << pdr << ", Inst Delay: " << instDelay << " ms"
                      << ", Inst Jitter: " << instJitter << " ms"
                      << ", Inst PDR: " << instPdr << std::endl;

            // Write to CSV file
            g_statsFile << now.GetSeconds() << "," << flowId << "," << throughput << "," << avgDelay
                        << "," << avgJitter << "," << pdr << "," << instDelay << "," << instJitter
                        << "," << instPdr << std::endl;
        }

        // Store current values for next calculation
        g_lastRxBytes[flowId] = currentRxBytes;
        g_lastTime[flowId] = now;
        g_lastRxPackets[flowId] = rxPackets;
        g_lastTxPackets[flowId] = txPackets;
        g_lastDelaySum[flowId] = stat.second.delaySum;
        g_lastJitterSum[flowId] = stat.second.jitterSum;
    }

    // Schedule next measurement
    Simulator::Schedule(MilliSeconds(100), MakeCallback(&ReportStats));
}

/**
 * Cleanup function to close files at simulation end
 * Note: Files opened as local variables in callback functions (sinrFile, posFile)
 * are automatically closed when those functions return, so they don't need
 * explicit cleanup here.
 */
void
CleanupAtEnd()
{
    // Close global file handles
    if (g_statsFile.is_open())
    {
        g_statsFile.close();
        std::cout << "Flow statistics file closed." << std::endl;
    }

    std::cout << "Simulation cleanup completed." << std::endl;
}

/**
 * Read position data from CSV file
 */
std::vector<Vector>
ReadPositionsFromFile(const std::string& filename)
{
    std::vector<Vector> positions;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open())
    {
        NS_FATAL_ERROR("Could not open file: " << filename);
    }

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> coords;

        // Parse comma-separated values
        while (std::getline(ss, value, ','))
        {
            coords.push_back(std::stod(value));
        }

        if (coords.size() >= 3)
        {
            positions.push_back(Vector(coords[0], coords[1], coords[2]));
        }
    }

    file.close();
    return positions;
}

/**
 * Create directory structure if it doesn't exist
 */
void
CreateDirectoryStructure(const std::string& path)
{
    std::cout << "Creating directory structure: " << path << std::endl;
    std::string currentPath;
    std::stringstream ss(path);
    std::string segment;

    while (std::getline(ss, segment, '/'))
    {
        if (!segment.empty())
        {
            currentPath += segment + "/";
            struct stat st = {0};
            if (stat(currentPath.c_str(), &st) == -1)
            {
                std::cout << "Creating directory: " << currentPath << std::endl;
                int result = mkdir(currentPath.c_str(), 0755);
                if (result != 0)
                {
                    std::cout << "Failed to create directory: " << currentPath
                              << " Error: " << strerror(errno) << std::endl;
                }
                else
                {
                    std::cout << "Successfully created directory: " << currentPath << std::endl;
                }
            }
            else
            {
                std::cout << "Directory already exists: " << currentPath << std::endl;
            }
        }
    }
}

/**
 * Read total time duration from trace scenario configuration file
 * @param tracesFolder Path to traces folder
 * @param tracesScenario Scenario name
 * @return Total time duration in seconds, or 0 if file cannot be read
 */
double
ReadTotalTimeDurationFromTraces(const std::string& tracesFolder, const std::string& tracesScenario)
{
    std::string paraCfgFile = tracesFolder + tracesScenario + "/Input/paraCfgCurrent.txt";
    std::ifstream file(paraCfgFile);

    if (!file.is_open())
    {
        std::cout << "Warning: Could not open trace configuration file: " << paraCfgFile
                  << std::endl;
        return 0.0;
    }

    std::string line;
    // Skip header line
    std::getline(file, line);

    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }

        // Parse tab-separated values
        size_t tabPos = line.find('\t');
        if (tabPos != std::string::npos)
        {
            std::string varName = line.substr(0, tabPos);
            std::string varValue = line.substr(tabPos + 1);

            // Remove any trailing whitespace
            varValue.erase(varValue.find_last_not_of(" \t\r\n") + 1);

            if (varName == "totalTimeDuration")
            {
                try
                {
                    double duration = std::stod(varValue);
                    file.close();
                    return duration;
                }
                catch (const std::exception& e)
                {
                    std::cout << "Warning: Could not parse totalTimeDuration value: " << varValue
                              << " Error: " << e.what() << std::endl;
                    file.close();
                    return 0.0;
                }
            }
        }
    }

    file.close();
    std::cout << "Warning: totalTimeDuration not found in trace configuration file" << std::endl;
    return 0.0;
}

int
main(int argc, char* argv[])
{
    // Simulation parameters
    uint16_t gNbNum = 1;
    uint16_t ueNumPergNb = 1;
    bool logging = false;
    std::string channelModel = "Traces";
    std::string attachmentMode = "closest"; // Options: "closest" or "id-based"

    // Traffic parameters
    uint32_t udpPacketSize = 1500;
    uint32_t lambda = 1;

    // Simulation timing
    Time simTime = MilliSeconds(10000);
    Time udpAppStartTime = MilliSeconds(10);
    bool simTimeAutoSet = true; // Flag to track if simulation time was auto-set from traces

    // NR parameters (Reference: 3GPP TR 38.901 V17.0.0 Table 7.8-1)
    uint16_t numerologyBwp1 = 3;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 100e6;
    double totalTxPower = 35;

    // Antenna configuration
    uint16_t gnbNumRows = 2;
    uint16_t gnbNumColumns = 16;
    uint16_t ueNumRows = 2;
    uint16_t ueNumColumns = 16;

    // Beamforming configuration
    bool useAngularScanning = true;
    double txZenithStep = 10.0;
    double rxZenithStep = 10.0;
    double txAzimuthStep = 1.0;
    double rxAzimuthStep = 90.0;
    uint8_t oversamplingFactor = 1;

    // Angular range configuration (only used when useAngularScanning = true)
    double txZenithStart = 117.5;
    double txZenithEnd = 118.5;
    double rxZenithStart = 42.5;
    double rxZenithEnd = 43.5;
    double txAzimuthStart = 0.0;
    double txAzimuthEnd = 360.0;
    double rxAzimuthStart = 0.0;
    double rxAzimuthEnd = 360.0;

    // RLC buffer size
    uint32_t rlcBufferSize = 10000;

    // Command line argument parsing
    CommandLine cmd(__FILE__);

    cmd.AddValue("gNbNum", "The number of gNBs in multiple-UE topology", gNbNum);
    cmd.AddValue("ueNumPergNb", "The number of UEs per gNB in multiple-UE topology", ueNumPergNb);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("channelModel", "Channel model to use (Traces or 3GPP)", channelModel);
    cmd.AddValue("packetSize", "Packet size in bytes to be used by traffic", udpPacketSize);
    cmd.AddValue("lambda", "Number of UDP packets in one second", lambda);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerologyBwp1", "The numerology to be used in bandwidth part 1", numerologyBwp1);
    cmd.AddValue("centralFrequencyBand1",
                 "The system frequency to be used in band 1",
                 centralFrequencyBand1);
    cmd.AddValue("bandwidthBand1", "The system bandwidth to be used in band 1", bandwidthBand1);
    cmd.AddValue("totalTxPower",
                 "Total TX power that will be proportionally assigned to bands, CCs and bandwidth "
                 "parts depending on each BWP bandwidth",
                 totalTxPower);
    cmd.AddValue("gnbNumRows", "Number of antenna rows for gNodeB", gnbNumRows);
    cmd.AddValue("gnbNumColumns", "Number of antenna columns for gNodeB", gnbNumColumns);
    cmd.AddValue("ueNumRows", "Number of antenna rows for UE", ueNumRows);
    cmd.AddValue("ueNumColumns", "Number of antenna columns for UE", ueNumColumns);
    cmd.AddValue("tracesScenario", "Traces channel model scenario name", tracesScenario);
    cmd.AddValue("rlcBufferSize", "RLC buffer size in bytes (0 = unlimited)", rlcBufferSize);
    cmd.AddValue("attachmentMode", "Attachment mode (closest or id-based)", attachmentMode);

    // Beamforming command-line arguments
    cmd.AddValue("useAngularScanning",
                 "Use angular scanning instead of sector-based scanning",
                 useAngularScanning);
    cmd.AddValue("txZenithStep",
                 "TX zenith angle step in degrees for angular scanning",
                 txZenithStep);
    cmd.AddValue("rxZenithStep",
                 "RX zenith angle step in degrees for angular scanning",
                 rxZenithStep);
    cmd.AddValue("txAzimuthStep",
                 "TX azimuth angle step in degrees for angular scanning",
                 txAzimuthStep);
    cmd.AddValue("rxAzimuthStep",
                 "RX azimuth angle step in degrees for angular scanning",
                 rxAzimuthStep);
    cmd.AddValue("oversamplingFactor",
                 "Oversampling factor for sector-based scanning",
                 oversamplingFactor);

    // Angular range command-line arguments
    cmd.AddValue("txZenithStart",
                 "TX zenith angle start in degrees for angular scanning",
                 txZenithStart);
    cmd.AddValue("txZenithEnd", "TX zenith angle end in degrees for angular scanning", txZenithEnd);
    cmd.AddValue("rxZenithStart",
                 "RX zenith angle start in degrees for angular scanning",
                 rxZenithStart);
    cmd.AddValue("rxZenithEnd", "RX zenith angle end in degrees for angular scanning", rxZenithEnd);
    cmd.AddValue("txAzimuthStart",
                 "TX azimuth angle start in degrees for angular scanning",
                 txAzimuthStart);
    cmd.AddValue("txAzimuthEnd",
                 "TX azimuth angle end in degrees for angular scanning",
                 txAzimuthEnd);
    cmd.AddValue("rxAzimuthStart",
                 "RX azimuth angle start in degrees for angular scanning",
                 rxAzimuthStart);
    cmd.AddValue("rxAzimuthEnd",
                 "RX azimuth angle end in degrees for angular scanning",
                 rxAzimuthEnd);

    cmd.Parse(argc, argv);

    // Display beamforming configuration
    std::cout << "\n=== Beamforming Configuration ===" << std::endl;
    if (useAngularScanning)
    {
        std::cout << "Mode: Angular-based scanning" << std::endl;
        std::cout << "TX Zenith: " << txZenithStart << "° to " << txZenithEnd
                  << "° (step: " << txZenithStep << "°)" << std::endl;
        std::cout << "RX Zenith: " << rxZenithStart << "° to " << rxZenithEnd
                  << "° (step: " << rxZenithStep << "°)" << std::endl;
        std::cout << "TX Azimuth: " << txAzimuthStart << "° to " << txAzimuthEnd
                  << "° (step: " << txAzimuthStep << "°)" << std::endl;
        std::cout << "RX Azimuth: " << rxAzimuthStart << "° to " << rxAzimuthEnd
                  << "° (step: " << rxAzimuthStep << "°)" << std::endl;
    }
    else
    {
        std::cout << "Mode: Sector-based scanning" << std::endl;
        std::cout << "Oversampling Factor: " << static_cast<int>(oversamplingFactor) << std::endl;
    }
    std::cout << "================================\n" << std::endl;

    // Auto-set simulation time from trace files if using Traces channel model and no explicit time
    // provided
    if (channelModel == "Traces" && simTimeAutoSet) // Default value indicates not explicitly set
    {
        double traceDuration = ReadTotalTimeDurationFromTraces(tracesFolder, tracesScenario);
        if (traceDuration > 0.0)
        {
            simTime = Seconds(traceDuration);
            std::cout << "\n=== Simulation Time Auto-Set ===" << std::endl;
            std::cout << "Simulation time automatically retrieved from trace files and set to: "
                      << traceDuration << " seconds" << std::endl;
            std::cout << "================================\n" << std::endl;
        }
        else
        {
            std::cout << "\n=== Simulation Time Warning ===" << std::endl;
            std::cout << "Could not read total time duration from trace files." << std::endl;
            std::cout << "Using default simulation time: " << simTime.GetSeconds() << " seconds"
                      << std::endl;
            std::cout << "================================\n" << std::endl;
        }
    }
    else
    {
        std::cout << "\n=== Simulation Time ===" << std::endl;
        std::cout << "Simulation time set to: " << simTime.GetSeconds() << " seconds" << std::endl;
        std::cout << "========================\n" << std::endl;
    }

    // Create directory structure
    std::string resultsDir = "Results";
    std::string scenarioDir = resultsDir + "/" + tracesScenario;
    std::string channelModelDir;

    if (channelModel == "Traces")
    {
        channelModelDir = scenarioDir + "/externalMPCs";
    }
    else if (channelModel == "3GPP")
    {
        channelModelDir = scenarioDir + "/3GPP";
    }
    else
    {
        NS_FATAL_ERROR("Unknown channel model: " << channelModel);
    }

    CreateDirectoryStructure(channelModelDir);

    // Create antenna configuration subfolder
    std::string antennaConfigDir = channelModelDir + "/GnodeB_" + std::to_string(gnbNumRows) + "x" +
                                   std::to_string(gnbNumColumns) + "_UE_" +
                                   std::to_string(ueNumRows) + "x" + std::to_string(ueNumColumns);

    CreateDirectoryStructure(antennaConfigDir);

    // Set up file names
    g_sinrTraceFile = antennaConfigDir + "/sinr_trace.csv";
    g_positionTraceFile = antennaConfigDir + "/position_trace.csv";

    std::cout << "Directory structure created." << std::endl;
    std::cout << "channelModelDir: " << channelModelDir << std::endl;
    std::cout << "antennaConfigDir: " << antennaConfigDir << std::endl;
    std::cout << "g_sinrTraceFile: " << g_sinrTraceFile << std::endl;

    // Clean up existing files
    std::string beamformingFile = antennaConfigDir + "/beamformingVector.csv";
    std::ifstream fileCheck(beamformingFile);
    if (fileCheck.good())
    {
        std::remove(beamformingFile.c_str());
    }
    fileCheck.close();

    std::string statsFile = antennaConfigDir + "/flow_stats.csv";
    std::ifstream statsFileCheck(statsFile);
    if (statsFileCheck.good())
    {
        std::remove(statsFile.c_str());
    }
    statsFileCheck.close();

    std::ifstream sinrFileCheck(g_sinrTraceFile);
    if (sinrFileCheck.good())
    {
        std::remove(g_sinrTraceFile.c_str());
    }
    sinrFileCheck.close();

    std::ifstream posFileCheck(g_positionTraceFile);
    if (posFileCheck.good())
    {
        std::remove(g_positionTraceFile.c_str());
    }
    posFileCheck.close();

    // Create files with headers
    std::ofstream sinrFileHeader;
    sinrFileHeader.open(g_sinrTraceFile);
    sinrFileHeader << "Time(s),CellId,RNTI,SINR(dB),BWP_ID" << std::endl;
    sinrFileHeader.close();

    std::ofstream posFileHeader;
    posFileHeader.open(g_positionTraceFile);
    posFileHeader << "Time(s),NodeId,X,Y,Z,VelocityX,VelocityY,VelocityZ" << std::endl;
    posFileHeader.close();

    // Validate frequency range
    NS_ABORT_IF(centralFrequencyBand1 < 0.5e9 && centralFrequencyBand1 > 100e9);

    // Configure logging
    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);
        LogComponentEnable("NrHelper", LOG_LEVEL_ALL);
        LogComponentEnable("TracesChannelModel", LOG_LEVEL_ALL);
    }

    // Set random seed
    RngSeedManager::SetSeed(5);

    // Configure global attributes
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(rlcBufferSize));
    Config::SetDefault("ns3::NrMacSchedulingStats::DlOutputFilename",
                       StringValue(antennaConfigDir + "/NrDlMacStats.txt"));

    // Create nodes
    int64_t randomStream = 1;
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(gNbNum);
    ueNodes.Create(gNbNum * ueNumPergNb);

    // Print node assignments
    std::cout << "\n=== Node Assignment Summary ===" << std::endl;
    std::cout << "gNBs created: " << gNbNum << std::endl;
    for (uint16_t i = 0; i < gNbNum; i++)
    {
        std::cout << "  gNB " << i << " -> ns-3 Node " << gnbNodes.Get(i)->GetId()
                  << " (will use device" << i << ".csv)" << std::endl;
    }

    std::cout << "UEs created: " << (gNbNum * ueNumPergNb) << std::endl;
    for (uint16_t i = 0; i < gNbNum * ueNumPergNb; i++)
    {
        std::cout << "  UE " << i << " -> ns-3 Node " << ueNodes.Get(i)->GetId()
                  << " (will use device" << (gNbNum + i) << ".csv)" << std::endl;
    }
    std::cout << "================================\n" << std::endl;

    // Setup mobility model
    if (channelModel == "Traces" || channelModel == "3GPP")
    {
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::WaypointMobilityModel");

        mobility.Install(gnbNodes);
        mobility.Install(ueNodes);

        // Read gNB positions from files
        for (uint16_t i = 0; i < gNbNum; i++)
        {
            std::string gnbPositionFile = tracesFolder + tracesScenario +
                                          "/Output/Ns3/NodesPosition/device" + std::to_string(i) +
                                          ".csv";
            std::vector<Vector> gnbPositions = ReadPositionsFromFile(gnbPositionFile);
            if (!gnbPositions.empty())
            {
                gnbNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(gnbPositions[0]);
            }
            else
            {
                NS_FATAL_ERROR("Could not read gNB " << i
                                                     << " position from file: " << gnbPositionFile);
            }
        }

        // Read UE positions from files and setup waypoints
        for (uint16_t i = 0; i < gNbNum * ueNumPergNb; i++)
        {
            std::string uePositionFile = tracesFolder + tracesScenario +
                                         "/Output/Ns3/NodesPosition/device" +
                                         std::to_string(gNbNum + i) + ".csv";
            std::vector<Vector> uePositions = ReadPositionsFromFile(uePositionFile);
            if (uePositions.empty())
            {
                NS_FATAL_ERROR("Could not read UE " << i
                                                    << " position from file: " << uePositionFile);
            }

            Ptr<WaypointMobilityModel> ueMobility =
                DynamicCast<WaypointMobilityModel>(ueNodes.Get(i)->GetObject<MobilityModel>());
            Time positionUpdateInterval = MilliSeconds(100);

            for (size_t j = 0; j < uePositions.size(); ++j)
            {
                ueMobility->AddWaypoint(Waypoint(positionUpdateInterval * j, uePositions[j]));
            }

            // Connect mobility callback for this UE
            uint32_t nodeId = ueNodes.Get(i)->GetId();
            Config::Connect("/NodeList/" + std::to_string(nodeId) +
                                "/$ns3::MobilityModel/CourseChange",
                            MakeCallback(&CourseChange));

            std::cout << "Connected mobility callback for UE " << i << " (Node " << nodeId
                      << ") with " << uePositions.size() << " waypoints" << std::endl;
        }
    }

    // Setup NR module helpers
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Set output directory for beamforming files
    IdealBeamformingHelper::SetOutputDirectory(antennaConfigDir);

    // Configure beamforming
    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(CQI_PDSCH_SISO));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(CellScanBeamforming::GetTypeId()));
    idealBeamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(100)));

    // Configure dual-mode beamforming parameters
    // Choose between sector-based scanning (false) or angular-based scanning (true)
    Config::SetDefault("ns3::CellScanBeamforming::UseAngularScanning",
                       BooleanValue(useAngularScanning));

    // Angular scanning parameters (only used when UseAngularScanning = true)
    Config::SetDefault("ns3::CellScanBeamforming::TxZenithStep",
                       DoubleValue(txZenithStep)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::RxZenithStep",
                       DoubleValue(rxZenithStep)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::TxAzimuthStep",
                       DoubleValue(txAzimuthStep)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::RxAzimuthStep",
                       DoubleValue(rxAzimuthStep)); // degrees

    // Angular range parameters (only used when UseAngularScanning = true)
    Config::SetDefault("ns3::CellScanBeamforming::TxZenithStart",
                       DoubleValue(txZenithStart)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::TxZenithEnd",
                       DoubleValue(txZenithEnd)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::RxZenithStart",
                       DoubleValue(rxZenithStart)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::RxZenithEnd",
                       DoubleValue(rxZenithEnd)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::TxAzimuthStart",
                       DoubleValue(txAzimuthStart)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::TxAzimuthEnd",
                       DoubleValue(txAzimuthEnd)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::RxAzimuthStart",
                       DoubleValue(rxAzimuthStart)); // degrees
    Config::SetDefault("ns3::CellScanBeamforming::RxAzimuthEnd",
                       DoubleValue(rxAzimuthEnd)); // degrees

    // Sector-based scanning parameter (only used when UseAngularScanning = false)
    Config::SetDefault("ns3::CellScanBeamforming::OversamplingFactor",
                       UintegerValue(oversamplingFactor));

    // Create spectrum configuration
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand);

    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    allBwps = CcBwpCreator::GetAllBwps({band1});

    // Calculate power distribution
    double x = pow(10, totalTxPower / 10);
    double totalBandwidth = bandwidthBand1;

    // Create and configure channel model
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    if (channelModel == "Traces")
    {
        Ptr<TracesChannelModel> tracesChannelModel =
            CreateObject<TracesChannelModel>(tracesFolder, tracesScenario);
        Config::SetDefault("ns3::TracesSpectrumPropagationLossModel::ChannelModel",
                           PointerValue(tracesChannelModel));

        channelHelper->ConfigureSpectrumFactory(TracesSpectrumPropagationLossModel::GetTypeId());
        channelHelper->AssignChannelsToBands({band1}, NrChannelHelper::INIT_FADING);
    }
    else if (channelModel == "3GPP")
    {
        channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
        channelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                         TimeValue(MilliSeconds(100)));
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
        channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
        channelHelper->AssignChannelsToBands({band1});
    }
    else
    {
        NS_FATAL_ERROR("Invalid channel model selected. Choose either 'Traces' or '3GPP'");
    }

    // Enable packet checking and printing
    Packet::EnableChecking();
    Packet::EnablePrinting();

    // Configure core network latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Configure UE antennas
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(ueNumRows));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(ueNumColumns));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Configure gNB antennas
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(gnbNumRows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(gnbNumColumns));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Configure bearer routing
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));

    // Install devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Configure per-node attributes
    for (uint16_t i = 0; i < gNbNum; i++)
    {
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)
            ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)
            ->SetAttribute("TxPower",
                           DoubleValue(10 * log10((bandwidthBand1 / totalBandwidth) * x)));
    }

    // Setup internet connectivity
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Connect remote host to PGW
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);

    // Assign IP addresses to UEs
    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // Set default gateways for UEs
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach UEs to gNBs based on selected mode
    if (attachmentMode == "closest")
    {
        nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
        std::cout << "UEs attached to closest gNBs based on distance" << std::endl;
    }
    else if (attachmentMode == "id-based")
    {
        // ID-based attachment: UE i goes to gNB (i / ueNumPergNb)
        for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
        {
            uint32_t targetGnbIndex = i / ueNumPergNb;
            if (targetGnbIndex >= gnbNetDev.GetN())
            {
                NS_FATAL_ERROR("UE " << i << " cannot be assigned to gNB " << targetGnbIndex
                                     << " (only " << gnbNetDev.GetN() << " gNBs available)");
            }

            Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
            Ptr<NetDevice> gnbDevice = gnbNetDev.Get(targetGnbIndex);

            if (ueDevice && gnbDevice)
            {
                nrHelper->AttachToGnb(ueDevice, gnbDevice);
                std::cout << "UE " << i << " (Node " << ueNodes.Get(i)->GetId()
                          << ") attached to gNB " << targetGnbIndex << " (Node "
                          << gnbNodes.Get(targetGnbIndex)->GetId() << ")" << std::endl;
            }
            else
            {
                NS_FATAL_ERROR("Failed to get UE or gNB device for attachment");
            }
        }
        std::cout << "UEs attached to gNBs based on ID assignment" << std::endl;
    }
    else
    {
        NS_FATAL_ERROR("Unknown attachment mode: " << attachmentMode
                                                   << ". Use 'closest' or 'id-based'");
    }

    // Print attachment summary
    std::cout << "\n=== UE-gNB Attachment Summary ===" << std::endl;
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDevice = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        if (ueDevice && ueDevice->GetTargetGnb())
        {
            Ptr<const NrGnbNetDevice> attachedGnb = ueDevice->GetTargetGnb();
            if (attachedGnb)
            {
                // Find the gNB index
                uint32_t gnbIndex = 0;
                for (uint32_t j = 0; j < gnbNetDev.GetN(); ++j)
                {
                    if (gnbNetDev.Get(j) == attachedGnb)
                    {
                        gnbIndex = j;
                        break;
                    }
                }
                std::cout << "UE " << i << " (Node " << ueNodes.Get(i)->GetId() << ") → gNB "
                          << gnbIndex << " (Node " << gnbNodes.Get(gnbIndex)->GetId() << ")"
                          << std::endl;
            }
        }
    }
    std::cout << "================================\n" << std::endl;

    // Setup UDP traffic
    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    UdpServerHelper dlPacketSink(dlPort);
    serverApps.Add(dlPacketSink.Install(ueNodes));

    UdpClientHelper dlClient;
    dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda)));

    // Configure bearer and traffic filter
    NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
    Ptr<NrEpcTft> tft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    tft->Add(dlpf);

    // Install applications
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
        Address ueAddress = ueIpIface.GetAddress(i);

        dlClient.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClient.Install(remoteHost));

        nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearer, tft);
    }

    // Start applications
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // Enable traces
    nrHelper->EnableTraces();

    // Connect to SINR traces
    for (auto nd = ueNetDev.Begin(); nd != ueNetDev.End(); ++nd)
    {
        Ptr<NrUeNetDevice> ueNetDevice = DynamicCast<NrUeNetDevice>(*nd);
        Ptr<NrUePhy> uePhy = ueNetDevice->GetPhy(0);
        uePhy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&ReportSinrTrace));
    }

    // Setup flow monitoring
    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueNodes);

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    g_monitor = monitor;

    // Start statistics monitoring
    Simulator::Schedule(MilliSeconds(100), MakeCallback(&ReportStats));

    // Run simulation
    Simulator::Stop(simTime);
    Simulator::Run();

    // Process results
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    // Write results to file
    std::ofstream outFile;
    std::string filename = antennaConfigDir + "/simulation_results.txt";
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    double flowDuration = (simTime - udpAppStartTime).GetSeconds();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0
                << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            averageFlowThroughput += i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000
                    << " Mbps\n";
            outFile << "  Mean delay:  "
                    << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            outFile << "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    double meanFlowThroughput = averageFlowThroughput / stats.size();
    double meanFlowDelay = averageFlowDelay / stats.size();

    outFile << "\n\n  Mean flow throughput: " << meanFlowThroughput << "\n";
    outFile << "  Mean flow delay: " << meanFlowDelay << "\n";

    outFile.close();

    // Display results
    std::ifstream f(filename.c_str());
    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    // Cleanup and destroy simulation
    Simulator::Schedule(Seconds(simTime.GetSeconds()), &CleanupAtEnd);
    Simulator::Destroy();

    return EXIT_SUCCESS;
}
