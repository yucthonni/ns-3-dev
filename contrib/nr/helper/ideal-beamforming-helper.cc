// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ideal-beamforming-helper.h"

#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/log.h"
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

// TR++
void
LogBeamforming(uint32_t gnbId,
               uint32_t ueId,
               double power,
               BeamformingVector gnbBeamformingVector,
               BeamformingVector ueBeamformingVector)
{
    std::cout << "Beamforming performed: gNB ID = " << gnbId << ", UE ID = " << ueId << std::endl;
    static std::set<std::tuple<double, std::string, std::string>> loggedEntries;
    static const std::string filename = "beamformingVector.csv";

    double timestamp = Simulator::Now().GetMilliSeconds();
    auto entryKey = std::make_tuple(timestamp, "TX", "RX");

    // Check if the entry already exists
    if (loggedEntries.find(entryKey) != loggedEntries.end())
    {
        return; // Skip duplicate entry
    }

    loggedEntries.insert(entryKey);

    std::ofstream file;
    bool fileExists = std::ifstream(filename).good();

    // Open file in append mode
    file.open(filename, std::ios_base::app);

    // Write header if the file is new
    if (!fileExists)
    {
        file << "Timestamp,Device,ElementIndex,Real,Imag,Sector,Elevation,Power\n";
    }

    // Write gNB beamforming vector
    PhasedArrayModel::ComplexVector gnbComplexVector = gnbBeamformingVector.first;
    BeamId gnbBeamId = gnbBeamformingVector.second;

    for (size_t i = 0; i < gnbComplexVector.GetSize(); ++i)
    {
        file << timestamp << ",TX," << i + 1 << "," << gnbComplexVector[i].real() << ","
             << gnbComplexVector[i].imag() << "," << gnbBeamId.GetSector() << ","
             << gnbBeamId.GetElevation() << "," << power << "\n";
    }

    // Write UE beamforming vector
    PhasedArrayModel::ComplexVector ueComplexVector = ueBeamformingVector.first;
    BeamId ueBeamId = ueBeamformingVector.second;

    for (size_t i = 0; i < ueComplexVector.GetSize(); ++i)
    {
        file << timestamp << ",RX," << i + 1 << "," << ueComplexVector[i].real() << ","
             << ueComplexVector[i].imag() << "," << ueBeamId.GetSector() << ","
             << ueBeamId.GetElevation() << "," << power << "\n";
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

    for (std::size_t ccId = 0; ccId < gnbDev->GetCcMapSize(); ccId++)
    {
        Ptr<NrSpectrumPhy> gnbSpectrumPhy = gnbDev->GetPhy(ccId)->GetSpectrumPhy();
        Ptr<NrSpectrumPhy> ueSpectrumPhy = ueDev->GetPhy(ccId)->GetSpectrumPhy();
        m_beamformingAlgorithm->TraceConnectWithoutContext("BeamformingPerformed",
                                                           MakeCallback(&LogBeamforming)); // TR++
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

} // namespace ns3
