// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// Modified by NIST <tanguy.ropitault@nist.gov>
// SPDX-License-Identifier: GPL-2.0-only

#include "ideal-beamforming-helper.h"

#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/object-factory.h"
#include "ns3/vector.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("IdealBeamformingHelper");
NS_OBJECT_ENSURE_REGISTERED(IdealBeamformingHelper);

// Initialize static member variable
std::string IdealBeamformingHelper::m_outputDirectory = "./";

// Wrapper function for LogBeamforming with pair identification
void
LogBeamformingWithContext(std::string context,
                          uint32_t gnbId,
                          uint32_t ueId,
                          double power,
                          BeamformingVector gnbBeamformingVector,
                          BeamformingVector ueBeamformingVector)
{
    static std::set<std::tuple<uint32_t, uint32_t, uint64_t>> loggedEntries;

    // Create a unique key for this beamforming event using microsecond precision
    auto entryKey = std::make_tuple(gnbId, ueId, Simulator::Now().GetMicroSeconds());

    // Check if this exact entry has already been logged
    if (loggedEntries.find(entryKey) != loggedEntries.end())
    {
        // This entry has already been logged, skip it
        NS_LOG_DEBUG("Duplicate beamforming entry detected for gNB "
                     << gnbId << " UE " << ueId << " at time " << Simulator::Now().GetMicroSeconds()
                     << "μs, skipping");
        return;
    }
    std::cout << "Beamforming performed: gNB ID = " << gnbId << ", UE ID = " << ueId << std::endl;

    // Add to logged entries set
    loggedEntries.insert(entryKey);
    NS_LOG_DEBUG("New beamforming entry logged for gNB " << gnbId << " UE " << ueId << " at time "
                                                         << Simulator::Now().GetMicroSeconds()
                                                         << "μs");

    // Use the output directory from the helper
    std::string filename = IdealBeamformingHelper::GetOutputDirectory() + "beamformingVector.csv";

    double timestamp = Simulator::Now().GetMilliSeconds();

    std::ofstream file;
    bool fileExists = std::ifstream(filename).good();

    // Open file in append mode
    file.open(filename, std::ios_base::app);

    // Write header if the file is new
    if (!fileExists)
    {
        file << "Timestamp,gNB_ID,UE_ID,Device,ElementIndex,Real,Imag,Sector,Elevation,Power\n";
    }

    // Write gNB beamforming vector
    PhasedArrayModel::ComplexVector gnbComplexVector = gnbBeamformingVector.first;
    BeamId gnbBeamId = gnbBeamformingVector.second;

    for (size_t i = 0; i < gnbComplexVector.GetSize(); ++i)
    {
        file << timestamp << "," << gnbId << "," << ueId << ",TX," << i + 1 << ","
             << gnbComplexVector[i].real() << "," << gnbComplexVector[i].imag() << ","
             << gnbBeamId.GetSector() << "," << gnbBeamId.GetElevation() << "," << power << "\n";
    }

    // Write UE beamforming vector
    PhasedArrayModel::ComplexVector ueComplexVector = ueBeamformingVector.first;
    BeamId ueBeamId = ueBeamformingVector.second;

    for (size_t i = 0; i < ueComplexVector.GetSize(); ++i)
    {
        file << timestamp << "," << gnbId << "," << ueId << ",RX," << i + 1 << ","
             << ueComplexVector[i].real() << "," << ueComplexVector[i].imag() << ","
             << ueBeamId.GetSector() << "," << ueBeamId.GetElevation() << "," << power << "\n";
    }

    file.close();
}

IdealBeamformingHelper::IdealBeamformingHelper()
{
    NS_LOG_FUNCTION(this);
}

IdealBeamformingHelper::~IdealBeamformingHelper()
{
    NS_LOG_FUNCTION(this);
    m_beamformingAlgorithm = nullptr;
}

void
IdealBeamformingHelper::DoInitialize()
{
    m_beamformingTimer = Simulator::Schedule(m_beamformingPeriodicity,
                                             &IdealBeamformingHelper::ExpireBeamformingTimer,
                                             this);
    BeamformingHelperBase::DoInitialize();
}

TypeId
IdealBeamformingHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::IdealBeamformingHelper")
            .SetParent<BeamformingHelperBase>()
            .AddConstructor<IdealBeamformingHelper>()
            .AddAttribute("BeamformingMethod",
                          "Type of the ideal beamforming method in the case that it is enabled, by "
                          "default is \"cell scan\" method.",
                          TypeIdValue(CellScanBeamforming::GetTypeId()),
                          MakeTypeIdAccessor(&IdealBeamformingHelper::SetBeamformingMethod),
                          MakeTypeIdChecker())
            .AddAttribute("BeamformingPeriodicity",
                          "Interval between consecutive beamforming method executions. If set to 0 "
                          "it will not be updated.",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&IdealBeamformingHelper::SetPeriodicity,
                                           &IdealBeamformingHelper::GetPeriodicity),
                          MakeTimeChecker());
    return tid;
}

void
IdealBeamformingHelper::AddBeamformingTask(const Ptr<NrGnbNetDevice>& gnbDev,
                                           const Ptr<NrUeNetDevice>& ueDev)
{
    NS_LOG_FUNCTION(this);

    if (!m_beamformingAlgorithm)
    {
        m_beamformingAlgorithm = m_algorithmFactory.Create<IdealBeamformingAlgorithm>();
    }

    // Create a unique context string for this gNB-UE pair
    std::string pairContext = "gNB" + std::to_string(gnbDev->GetNode()->GetId()) + "-UE" +
                              std::to_string(ueDev->GetNode()->GetId());

    // Check if this pair has already been registered
    static std::set<std::string> registeredPairs;
    if (registeredPairs.find(pairContext) != registeredPairs.end())
    {
        NS_LOG_INFO("Beamforming task already registered for " << pairContext << ", skipping");
        return;
    }
    registeredPairs.insert(pairContext);

    for (std::size_t ccId = 0; ccId < gnbDev->GetCcMapSize(); ccId++)
    {
        Ptr<NrSpectrumPhy> gnbSpectrumPhy = gnbDev->GetPhy(ccId)->GetSpectrumPhy();
        Ptr<NrSpectrumPhy> ueSpectrumPhy = ueDev->GetPhy(ccId)->GetSpectrumPhy();

        m_beamformingAlgorithm->TraceConnect("BeamformingPerformed",
                                             pairContext,
                                             MakeCallback(&LogBeamformingWithContext));
        m_spectrumPhyPair.emplace_back(gnbSpectrumPhy, ueSpectrumPhy);
        RunTask(gnbSpectrumPhy, ueSpectrumPhy);
    }
}

void
IdealBeamformingHelper::Run() const
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Running the beamforming method. There are :" << m_spectrumPhyPair.size()
                                                              << " tasks.");

    for (const auto& task : m_spectrumPhyPair)
    {
        RunTask(task.first, task.second);
    }
}

BeamformingVectorPair
IdealBeamformingHelper::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                              const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_LOG_FUNCTION(this);
    return m_beamformingAlgorithm->GetBeamformingVectors(gnbSpectrumPhy, ueSpectrumPhy);
}

void
IdealBeamformingHelper::SetBeamformingMethod(const TypeId& beamformingMethod)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(beamformingMethod.IsChildOf(IdealBeamformingAlgorithm::GetTypeId()));
    m_algorithmFactory.SetTypeId(beamformingMethod);
}

void
IdealBeamformingHelper::ExpireBeamformingTimer()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Beamforming timer expired; programming a beamforming");

    Run();                       // Run beamforming tasks
    m_beamformingTimer.Cancel(); // Cancel any previous beamforming event
    m_beamformingTimer = Simulator::Schedule(m_beamformingPeriodicity,
                                             &IdealBeamformingHelper::ExpireBeamformingTimer,
                                             this);
}

void
IdealBeamformingHelper::SetPeriodicity(const Time& v)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(v == MilliSeconds(0), "Periodicity must be greater than 0 ms.");
    m_beamformingPeriodicity = v;
}

Time
IdealBeamformingHelper::GetPeriodicity() const
{
    NS_LOG_FUNCTION(this);
    return m_beamformingPeriodicity;
}

// Implement the setter method
void
IdealBeamformingHelper::SetOutputDirectory(const std::string& dir)
{
    m_outputDirectory = dir;
    // Ensure directory ends with '/'
    if (!m_outputDirectory.empty() && m_outputDirectory.back() != '/')
    {
        m_outputDirectory += "/";
    }
}

// Implement the getter method
std::string
IdealBeamformingHelper::GetOutputDirectory()
{
    return m_outputDirectory;
}

} // namespace ns3
