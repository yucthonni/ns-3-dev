/**
 * @file oneBWPSNR.cc
 * @brief A simplified NR simulation focusing on SINR measurements with a single UE and gNodeB
 *
 * This example demonstrates a basic NR simulation setup with:
 * - Single UE and single gNodeB configuration
 * - QD channel model for realistic channel modeling
 * - SINR measurements and logging
 * - UDP traffic generation
 *
 * Key features:
 * - Uses QD channel model for realistic channel modeling
 * - Implements beamforming with CellScanBeamformingAzimuthZenith
 * - Logs SINR values to a CSV file
 * - Configurable simulation parameters (packet size, lambda, etc.)
 *
 * Output files:
 * - sinr_trace.csv: Contains timestamp, cell ID, RNTI, SINR (dB), and BWP ID
 * - default: Contains flow statistics (throughput, delay, etc.)
 *
 * @author [Your Name]
 * @date [Current Date]
 */

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @file cttc-nr-demo.cc
 * @brief A cozy, simple, NR demo (in a tutorial style)
 *
 * Notice: this entire program uses technical terms defined by the 3GPP TS 38.300 [1].
 *
 * This example describes how to setup a simulation using the 3GPP channel model from TR 38.901 [2].
 * This example consists of a simple grid topology, in which you
 * can choose the number of gNbs and UEs. Have a look at the possible parameters
 * to know what you can configure through the command line.
 *
 * With the default configuration, the example will create two flows that will
 * go through two different subband numerologies (or bandwidth parts). For that,
 * specifically, two bands are created, each with a single CC, and each CC containing
 * one bandwidth part.
 *
 * The example will print on-screen the end-to-end result of one (or two) flows,
 * as well as writing them on a file.
 *
 * \code{.unparsed}
$ ./ns3 run "cttc-nr-demo --PrintHelp"
    \endcode
 *
 */

// NOLINTBEGIN
// clang-format off

/**
 * Useful references that will be used for this tutorial:
 * [1] <a href="https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=3191">3GPP TS 38.300</a>
 * [2] <a href="https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=3173">3GPP channel model from TR 38.901</a>
 * [3] <a href="https://www.nsnam.org/docs/release/3.38/tutorial/html/tweaking.html#using-the-logging-module">ns-3 documentation</a>
 */

// clang-format on
// NOLINTEND

/*
 * Include part. Often, you will have to include the headers for an entire module;
 * do that by including the name of the module you need with the suffix "-module.h".
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

#include <cstring>
#include <errno.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

/*
 * Use, always, the namespace ns3. All the NR classes are inside such namespace.
 */
using namespace ns3;

/*
 * With this line, we will be able to see the logs of the file by enabling the
 * component "CttcNrDemo".
 * Further information on how logging works can be found in the ns-3 documentation [3].
 */
NS_LOG_COMPONENT_DEFINE("CttcNrDemo");

// Global variable for the SINR trace file name
std::string g_sinrTraceFile;
std::string g_positionTraceFile;
std::string qdFolder =
    "contrib/nr/utils/channels/external-mpcs/Scenarios/"; // Path to QD channel model scenarios
std::string qdScenario = "Etoiles90TracesBackup";         // QD channel model scenario name

// Add these global variables after the existing global variables
std::map<uint32_t, uint64_t> g_lastRxBytes;   // Track last received bytes for each flow
std::map<uint32_t, Time> g_lastTime;          // Track last measurement time for each flow
std::map<uint32_t, uint32_t> g_lastRxPackets; // Track last received packets for each flow
std::map<uint32_t, uint32_t> g_lastTxPackets; // Track last transmitted packets for each flow
std::map<uint32_t, Time> g_lastDelaySum;      // Track last delay sum for each flow
std::map<uint32_t, Time> g_lastJitterSum;     // Track last jitter sum for each flow

// Global variable for the output file
std::ofstream g_statsFile;

// Global variable for the flow monitor
Ptr<FlowMonitor> g_monitor;

void
ReportSinrTrace(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    std::ofstream sinrFile;
    sinrFile.open(g_sinrTraceFile, std::ios_base::app);
    double sinrDb = 10 * std::log10(sinr); // Convert linear SINR to dB
    sinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << sinrDb
             << "," << static_cast<int>(bwpId) << std::endl;

    //   std::cout <<std::endl;
    sinrFile.close();
}

// Callback function to track UE position changes
void
CourseChange(std::string context, Ptr<const MobilityModel> mobility)
{
    Vector pos = mobility->GetPosition();
    Vector vel = mobility->GetVelocity();
    std::ofstream posFile;
    posFile.open(g_positionTraceFile, std::ios_base::app);
    posFile << Simulator::Now().GetSeconds() << "," << pos.x << "," << pos.y << "," << pos.z << ","
            << vel.x << "," << vel.y << "," << vel.z << std::endl;
    posFile.close();

    std::cout << "UE position changed at t=" << Simulator::Now().GetSeconds() << "s -> (" << pos.x
              << ", " << pos.y << ", " << pos.z << ")" << std::endl;
}

// Add this callback function before main()
void
ReportStats()
{
    FlowMonitor::FlowStatsContainer stats = g_monitor->GetFlowStats();
    Time now = Simulator::Now();

    // Open file if it's not already open
    if (!g_statsFile.is_open())
    {
        // Get the directory from g_sinrTraceFile and use it for flow_stats.csv
        std::string flowStatsFile =
            g_sinrTraceFile.substr(0, g_sinrTraceFile.find_last_of('/')) + "/flow_stats.csv";

        // Check if directory exists
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

        // Only calculate metrics if we have previous data and received packets
        if (g_lastRxBytes.find(flowId) != g_lastRxBytes.end())
        {
            double timeDiff = (now - g_lastTime[flowId]).GetSeconds();

            // Throughput calculation (in Mbps)
            double throughput =
                (currentRxBytes - g_lastRxBytes[flowId]) * 8.0 / timeDiff / 1000.0 / 1000.0;

            // Average delay (in ms)
            double avgDelay =
                rxPackets > 0 ? (1000 * stat.second.delaySum.GetSeconds() / rxPackets) : 0;

            // Average jitter (in ms)
            double avgJitter =
                rxPackets > 0 ? (1000 * stat.second.jitterSum.GetSeconds() / rxPackets) : 0;

            // Packet Delivery Ratio (PDR)
            double pdr = txPackets > 0 ? static_cast<double>(rxPackets) / txPackets : 0;

            // Instantaneous metrics for current window
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

// Don't forget to close the file at the end of the simulation
void
CleanupAtEnd()
{
    if (g_statsFile.is_open())
    {
        g_statsFile.close();
    }
}

// Add this function before main()
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

        // Split the line by comma
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

// Function to create directories if they don't exist
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

int
main(int argc, char* argv[])
{
    /*
     * Variables that represent the parameters we will accept as input by the
     * command line. Each of them is initialized with a default value, and
     * possibly overridden below when command-line arguments are parsed.
     */
    // Scenario parameters (that we will use inside this script):
    uint16_t gNbNum = 1;
    uint16_t ueNumPergNb = 1;
    bool logging = false;
    std::string channelModel = "Traces"; // Options: "QD" or "3GPP"

    // Traffic parameters (that we will use inside this script):
    uint32_t udpPacketSize = 1500;
    uint32_t lambda = 1;
    // uint32_t lambda = 10000;

    // Simulation parameters. Please don't use double to indicate seconds; use
    // ns-3 Time values which use integers to avoid portability issues.
    Time simTime = MilliSeconds(10000);
    Time udpAppStartTime = MilliSeconds(10);

    // NR parameters (Reference: 3GPP TR 38.901 V17.0.0 (Release 17)
    // Table 7.8-1 for the power and BW).
    uint16_t numerologyBwp1 = 3;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 100e6;
    double totalTxPower = 35;

    // Antenna configuration parameters
    uint16_t gnbNumRows = 16;
    uint16_t gnbNumColumns = 16;
    uint16_t ueNumRows = 4;
    uint16_t ueNumColumns = 4;

    // RLC buffer size parameter (in bytes)
    uint32_t rlcBufferSize = 10000; // Default: very large (like original)

    /*
     * From here, we instruct the ns3::CommandLine class of all the input parameters
     * that we may accept as input, as well as their description, and the storage
     * variable.
     */
    CommandLine cmd(__FILE__);

    cmd.AddValue("gNbNum", "The number of gNbs in multiple-ue topology", gNbNum);
    cmd.AddValue("ueNumPergNb", "The number of UE per gNb in multiple-ue topology", ueNumPergNb);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("channelModel", "Channel model to use (QD or 3GPP)", channelModel);
    cmd.AddValue("packetSize", "packet size in bytes to be used by traffic", udpPacketSize);
    cmd.AddValue("lambda", "Number of UDP packets in one second", lambda);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerologyBwp1", "The numerology to be used in bandwidth part 1", numerologyBwp1);
    cmd.AddValue("centralFrequencyBand1",
                 "The system frequency to be used in band 1",
                 centralFrequencyBand1);
    cmd.AddValue("bandwidthBand1", "The system bandwidth to be used in band 1", bandwidthBand1);
    cmd.AddValue("totalTxPower",
                 "total tx power that will be proportionally assigned to"
                 " bands, CCs and bandwidth parts depending on each BWP bandwidth ",
                 totalTxPower);
    cmd.AddValue("gnbNumRows", "Number of antenna rows for gNodeB", gnbNumRows);
    cmd.AddValue("gnbNumColumns", "Number of antenna columns for gNodeB", gnbNumColumns);
    cmd.AddValue("ueNumRows", "Number of antenna rows for UE", ueNumRows);
    cmd.AddValue("ueNumColumns", "Number of antenna columns for UE", ueNumColumns);
    cmd.AddValue("qdScenario", "QD channel model scenario name", qdScenario);
    cmd.AddValue("rlcBufferSize", "RLC buffer size in bytes (0 = unlimited)", rlcBufferSize);

    // Parse the command line
    cmd.Parse(argc, argv);

    // Create hierarchical directory structure: Results/qdScenario/channelModel
    std::string resultsDir = "Results";
    std::string scenarioDir = resultsDir + "/" + qdScenario;
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

    // Create the directory structure if it doesn't exist
    CreateDirectoryStructure(channelModelDir);

    // Create antenna configuration subfolder
    std::string antennaConfigDir = channelModelDir + "/GnodeB_" + std::to_string(gnbNumRows) + "x" +
                                   std::to_string(gnbNumColumns) + "_UE_" +
                                   std::to_string(ueNumRows) + "x" + std::to_string(ueNumColumns);

    // Create the antenna configuration directory if it doesn't exist
    CreateDirectoryStructure(antennaConfigDir);

    // Set up file names with the new directory structure (now includes antenna config)
    g_sinrTraceFile = antennaConfigDir + "/sinr_trace" + ".csv";
    g_positionTraceFile = antennaConfigDir + "/position_trace" + ".csv";

    std::cout << "Directory structure created." << std::endl;
    std::cout << "channelModelDir: " << channelModelDir << std::endl;
    std::cout << "antennaConfigDir: " << antennaConfigDir << std::endl;
    std::cout << "g_sinrTraceFile: " << g_sinrTraceFile << std::endl;

    // Delete beamformingVector.csv if it exists
    std::string beamformingFile = antennaConfigDir + "/beamformingVector.csv";
    std::ifstream fileCheck(beamformingFile);
    if (fileCheck.good())
    {
        std::remove(beamformingFile.c_str());
    }
    fileCheck.close();

    // Delete flow_stats.csv file if it exists
    std::string statsFile = antennaConfigDir + "/flow_stats.csv";
    std::ifstream statsFileCheck(statsFile);
    if (statsFileCheck.good())
    {
        std::remove(statsFile.c_str());
    }
    statsFileCheck.close();

    // Delete SINR trace file if it exists
    std::ifstream sinrFileCheck(g_sinrTraceFile);
    if (sinrFileCheck.good())
    {
        std::remove(g_sinrTraceFile.c_str());
    }
    sinrFileCheck.close();

    // Create the file and write the header
    std::ofstream sinrFileHeader;
    sinrFileHeader.open(g_sinrTraceFile);
    sinrFileHeader << "Time(s),CellId,RNTI,SINR(dB),BWP_ID" << std::endl;
    sinrFileHeader.close();

    // Delete position trace file if it exists
    std::ifstream posFileCheck(g_positionTraceFile);
    if (posFileCheck.good())
    {
        std::remove(g_positionTraceFile.c_str());
    }
    posFileCheck.close();

    // Create the position file and write the header
    std::ofstream posFileHeader;
    posFileHeader.open(g_positionTraceFile);
    posFileHeader << "Time(s),X,Y,Z,VelocityX,VelocityY,VelocityZ" << std::endl;
    posFileHeader.close();

    /*
     * Check if the frequency is in the allowed range.
     * If you need to add other checks, here is the best position to put them.
     */
    NS_ABORT_IF(centralFrequencyBand1 < 0.5e9 && centralFrequencyBand1 > 100e9);

    /*
     * If the logging variable is set to true, enable the log of some components
     * through the code. The same effect can be obtained through the use
     * of the NS_LOG environment variable:
     *
     * export NS_LOG="UdpClient=level_info|prefix_time|prefix_func|prefix_node:UdpServer=..."
     *
     * Usually, the environment variable way is preferred, as it is more customizable,
     * and more expressive.
     */
    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);
        LogComponentEnable("NrHelper", LOG_LEVEL_ALL);
        LogComponentEnable("TracesChannelModel", LOG_LEVEL_ALL);
    }

    RngSeedManager::SetSeed(5);
    /*
     * In general, attributes for the NR module are typically configured in NrHelper.  However, some
     * attributes need to be configured globally through the Config::SetDefault() method. Below is
     * an example: if you want to make the RLC buffer very large, you can pass a very large integer
     * here.
     */
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(rlcBufferSize));

    // Set MAC scheduling stats output file to the antenna config directory
    Config::SetDefault("ns3::NrMacSchedulingStats::DlOutputFilename",
                       StringValue(antennaConfigDir + "/NrDlMacStats.txt"));

    /*
     * Create the scenario. In our examples, we heavily use helpers that setup
     * the gnbs and ue following a pre-defined pattern. Please have a look at the
     * GridScenarioHelper documentation to see how the nodes will be distributed.
     */
    int64_t randomStream = 1;

    // Create the nodes for gNB and UE
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    // Create the mobility model
    // TR++ TODO: Manage that in a better way - Probably read the file in the QD channel model and
    // manage multiples gNodeB and UEs case
    std::string gnbPositionFile =
        qdFolder + qdScenario + "/Output/Ns3/NodesPosition" + "/device0.csv";
    std::string uePositionFile =
        qdFolder + qdScenario + "/Output/Ns3/NodesPosition" + "/device1.csv";
    if (channelModel == "Traces" || channelModel == "3GPP")
    {
        // For both QD and 3GPP models, read positions from file
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::WaypointMobilityModel");

        // Install mobility on gNB and UE
        mobility.Install(gnbNodes);
        mobility.Install(ueNodes);

        // Read gNB position from file
        std::vector<Vector> gnbPositions = ReadPositionsFromFile(gnbPositionFile);
        if (!gnbPositions.empty())
        {
            gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(gnbPositions[0]);
        }
        else
        {
            NS_FATAL_ERROR("Could not read gNB position from file: " << gnbPositionFile);
        }

        // Read UE positions from file
        std::vector<Vector> uePositions = ReadPositionsFromFile(uePositionFile);
        if (uePositions.empty())
        {
            NS_FATAL_ERROR("Could not read UE positions from file: " << uePositionFile);
        }

        // Set up waypoints for UE movement
        Ptr<WaypointMobilityModel> ueMobility =
            DynamicCast<WaypointMobilityModel>(ueNodes.Get(0)->GetObject<MobilityModel>());
        Time positionUpdateInterval = MilliSeconds(100);

        for (size_t i = 0; i < uePositions.size(); ++i)
        {
            ueMobility->AddWaypoint(Waypoint(positionUpdateInterval * i, uePositions[i]));
        }

        // Connect the CourseChange callback to the UE mobility model
        Config::Connect("/NodeList/" + std::to_string(ueNodes.Get(0)->GetId()) +
                            "/$ns3::MobilityModel/CourseChange",
                        MakeCallback(&CourseChange));
    }

    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - nrEpcHelper, which will setup the core network
     * - RealisticBeamformingHelper, which takes care of the beamforming part
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     * - NrChannelHelper, which takes care of the spectrum channel
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Set the output directory for beamforming files
    IdealBeamformingHelper::SetOutputDirectory(antennaConfigDir);

    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(CQI_PDSCH_SISO));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(CellScanBeamforming::GetTypeId()));

    idealBeamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(100)));

    /*
     * Spectrum division. We create one operational band containing
     * one component carrier, and the CC containing a single bandwidth part
     * centered at the frequency specified by the input parameters.
     * The spectrum part length is specified by the input parameters.
     * The operational band will use the StreetCanyon channel modeling.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1; // in this example, we have a single CC

    // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
    // a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand);

    // Create the band and install the channel into it
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);

    // Get all BWPs
    allBwps = CcBwpCreator::GetAllBwps({band1});

    /*
     * Start to account for the bandwidth used by the example, as well as
     * the total power that has to be divided among the BWPs.
     */
    double x = pow(10, totalTxPower / 10);
    double totalBandwidth = bandwidthBand1;

    // Create and configure the channel model based on the selected option
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    if (channelModel == "Traces")
    {
        // Configure QD channel model
        Ptr<TracesChannelModel> tracesChannelModel =
            CreateObject<TracesChannelModel>(qdFolder, qdScenario);
        Config::SetDefault("ns3::TracesSpectrumPropagationLossModel::ChannelModel",
                           PointerValue(tracesChannelModel));
        //  Config::SetDefault("ns3::CellScanBeamforming::BeamSearchAngleStep",
        //  DoubleValue(30));

        channelHelper->ConfigureSpectrumFactory(TracesSpectrumPropagationLossModel::GetTypeId());

        channelHelper->AssignChannelsToBands({band1}, NrChannelHelper::INIT_FADING);
    }
    else if (channelModel == "3GPP")
    {
        // Configure 3GPP channel model
        channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
        // Show how to change the channel condition update period
        channelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                         TimeValue(MilliSeconds(100)));

        // Show how to change the channel matrix update period
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));

        channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
        channelHelper->AssignChannelsToBands({band1});
    }
    else
    {
        NS_FATAL_ERROR("Invalid channel model selected. Choose either 'Traces' or '3GPP'");
    }

    /*
     * allBwps contains all the spectrum configuration needed for the nrHelper.
     *
     * Now, we can setup the attributes. We can have three kind of attributes:
     * (i) parameters that are valid for all the bandwidth parts and applies to
     * all nodes, (ii) parameters that are valid for all the bandwidth parts
     * and applies to some node only, and (iii) parameters that are different for
     * every bandwidth parts. The approach is:
     *
     * - for (i): Configure the attribute through the helper, and then install;
     * - for (ii): Configure the attribute through the helper, and then install
     * for the first set of nodes. Then, change the attribute through the helper,
     * and install again;
     * - for (iii): Install, and then configure the attributes by retrieving
     * the pointer needed, and calling "SetAttribute" on top of such pointer.
     *
     */

    Packet::EnableChecking();
    Packet::EnablePrinting();

    /*
     *  Case (i): Attributes valid for all the nodes
     */
    // Beamforming method
    //  realisticBeamformingHelper->SetAttribute("BeamformingMethod",
    //                                       TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Antennas for all the UEs

    nrHelper->SetUeAntennaAttribute(
        "NumRows",
        UintegerValue(ueNumRows)); // this is just one antenna but then within
                                   // the antenna there are multiple beams.
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(ueNumColumns));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(gnbNumRows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(gnbNumColumns));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // gNb routing between Bearer and bandwidh part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));

    // Ue routing between Bearer and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));

    /*
     * We miss many other parameters. By default, not configuring them is equivalent
     * to use the default values. Please, have a look at the documentation to see
     * what are the default values for all the attributes you are not seeing here.
     */

    /*
     * Case (ii): Attributes valid for a subset of the nodes
     */

    // NOT PRESENT IN THIS SIMPLE EXAMPLE

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
    /*
     * Case (iii): Go node for node and change the attributes we have to setup
     * per-node.
     */

    // Get the first netdevice (gnbNetDev.Get (0)) and the first bandwidth part (0)
    // and set the attribute.
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(10 * log10((bandwidthBand1 / totalBandwidth) * x)));

    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
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

    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    /*
     * Traffic part. Install UDP traffic
     */
    uint16_t dlPort = 1234;

    ApplicationContainer serverApps;

    // The sink will always listen to the specified ports
    UdpServerHelper dlPacketSink(dlPort);

    // The server, that is the application which is listening, is installed in the UE
    serverApps.Add(dlPacketSink.Install(ueNodes));

    /*
     * Configure attributes for the different generators, using user-provided
     * parameters for generating a CBR traffic
     */
    UdpClientHelper dlClient;
    dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda)));

    // The bearer that will carry traffic
    NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);

    // The filter for the traffic
    Ptr<NrEpcTft> tft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpf;
    dlpf.localPortStart = dlPort;
    dlpf.localPortEnd = dlPort;
    tft->Add(dlpf);

    /*
     * Let's install the applications!
     */
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
        Address ueAddress = ueIpIface.GetAddress(i);

        // The client, who is transmitting, is installed in the remote host,
        // with destination address set to the address of the UE
        dlClient.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClient.Install(remoteHost));

        // Activate a dedicated bearer for the traffic type
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearer, tft);
    }

    // start UDP server and client apps
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    // Connect to SINR traces for each UE
    for (auto nd = ueNetDev.Begin(); nd != ueNetDev.End(); ++nd)
    {
        Ptr<NrUeNetDevice> ueNetDevice = DynamicCast<NrUeNetDevice>(*nd);
        Ptr<NrUePhy> uePhy = ueNetDevice->GetPhy(0); // Using index 0 for single bandwidth part
        uePhy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&ReportSinrTrace));
    }

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueNodes);

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    // Set the global monitor variable
    g_monitor = monitor;

    // Start stats monitoring
    Simulator::Schedule(MilliSeconds(100), MakeCallback(&ReportStats));

    Simulator::Stop(simTime);

    Simulator::Run();

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

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
            // Measure the duration of the flow from receiver's perspective
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

    std::ifstream f(filename.c_str());

    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Schedule(Seconds(simTime.GetSeconds()), &CleanupAtEnd);
    Simulator::Destroy();

    return EXIT_SUCCESS;
}
