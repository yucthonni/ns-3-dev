// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// Modified by NIST <tanguy.ropitault@nist.gov>
// SPDX-License-Identifier: GPL-2.0-only

#include "ideal-beamforming-algorithm.h"

#include "beamforming-vector.h"
#include "nr-spectrum-phy.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/node.h"
#include "ns3/nr-spectrum-value-helper.h"
#include "ns3/uinteger.h"
#include "ns3/uniform-planar-array.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("IdealBeamformingAlgorithm");
NS_OBJECT_ENSURE_REGISTERED(CellScanBeamforming);
NS_OBJECT_ENSURE_REGISTERED(CellScanQuasiOmniBeamforming);
NS_OBJECT_ENSURE_REGISTERED(DirectPathBeamforming);
NS_OBJECT_ENSURE_REGISTERED(QuasiOmniDirectPathBeamforming);
NS_OBJECT_ENSURE_REGISTERED(DirectPathQuasiOmniBeamforming);
NS_OBJECT_ENSURE_REGISTERED(OptimalCovMatrixBeamforming);
NS_OBJECT_ENSURE_REGISTERED(KroneckerBeamforming);
NS_OBJECT_ENSURE_REGISTERED(KroneckerQuasiOmniBeamforming);

TypeId
IdealBeamformingAlgorithm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::IdealBeamformingAlgorithm")
            .SetParent<Object>()
            .AddTraceSource(
                "BeamformingPerformed",
                "Traced callback for when beamforming is performed",
                MakeTraceSourceAccessor(&IdealBeamformingAlgorithm::m_beamformingPerformed),
                "ns3::IdealBeamformingAlgorithm::BeamformingPerformedCallback");
    return tid;
}

TypeId
CellScanBeamforming::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::CellScanBeamforming")
            .SetParent<IdealBeamformingAlgorithm>()
            .AddConstructor<CellScanBeamforming>()
            .AddAttribute("OversamplingFactor",
                          "Samples per antenna row/column",
                          UintegerValue(1),
                          MakeUintegerAccessor(&CellScanBeamforming::m_oversamplingFactor),
                          MakeUintegerChecker<uint8_t>(1, 4))
            .AddAttribute("UseAngularScanning",
                          "Use angular scanning instead of sector-based scanning",
                          BooleanValue(true),
                          MakeBooleanAccessor(&CellScanBeamforming::GetUseAngularScanning,
                                              &CellScanBeamforming::SetUseAngularScanning),
                          MakeBooleanChecker())
            .AddAttribute("TxZenithStep",
                          "TX zenith angle step in degrees for angular scanning",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetTxZenithStep,
                                             &CellScanBeamforming::SetTxZenithStep),
                          MakeDoubleChecker<double>(0.1, 180.0))
            .AddAttribute("RxZenithStep",
                          "RX zenith angle step in degrees for angular scanning",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetRxZenithStep,
                                             &CellScanBeamforming::SetRxZenithStep),
                          MakeDoubleChecker<double>(0.1, 180.0))
            .AddAttribute("TxAzimuthStep",
                          "TX azimuth angle step in degrees for angular scanning",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetTxAzimuthStep,
                                             &CellScanBeamforming::SetTxAzimuthStep),
                          MakeDoubleChecker<double>(0.1, 360.0))
            .AddAttribute("RxAzimuthStep",
                          "RX azimuth angle step in degrees for angular scanning",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetRxAzimuthStep,
                                             &CellScanBeamforming::SetRxAzimuthStep),
                          MakeDoubleChecker<double>(0.1, 360.0))
            .AddAttribute("TxZenithStart",
                          "TX zenith angle start in degrees for angular scanning",
                          DoubleValue(0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetTxZenithStart,
                                             &CellScanBeamforming::SetTxZenithStart),
                          MakeDoubleChecker<double>(0.0, 180.0))
            .AddAttribute("TxZenithEnd",
                          "TX zenith angle end in degrees for angular scanning",
                          DoubleValue(180),
                          MakeDoubleAccessor(&CellScanBeamforming::GetTxZenithEnd,
                                             &CellScanBeamforming::SetTxZenithEnd),
                          MakeDoubleChecker<double>(0.0, 180.0))
            .AddAttribute("RxZenithStart",
                          "RX zenith angle start in degrees for angular scanning",
                          DoubleValue(0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetRxZenithStart,
                                             &CellScanBeamforming::SetRxZenithStart),
                          MakeDoubleChecker<double>(0.0, 180.0))
            .AddAttribute("RxZenithEnd",
                          "RX zenith angle end in degrees for angular scanning",
                          DoubleValue(180),
                          MakeDoubleAccessor(&CellScanBeamforming::GetRxZenithEnd,
                                             &CellScanBeamforming::SetRxZenithEnd),
                          MakeDoubleChecker<double>(0.0, 180.0))
            .AddAttribute("TxAzimuthStart",
                          "TX azimuth angle start in degrees for angular scanning",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetTxAzimuthStart,
                                             &CellScanBeamforming::SetTxAzimuthStart),
                          MakeDoubleChecker<double>(0.0, 360.0))
            .AddAttribute("TxAzimuthEnd",
                          "TX azimuth angle end in degrees for angular scanning",
                          DoubleValue(360.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetTxAzimuthEnd,
                                             &CellScanBeamforming::SetTxAzimuthEnd),
                          MakeDoubleChecker<double>(0.0, 360.0))
            .AddAttribute("RxAzimuthStart",
                          "RX azimuth angle start in degrees for angular scanning",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetRxAzimuthStart,
                                             &CellScanBeamforming::SetRxAzimuthStart),
                          MakeDoubleChecker<double>(0.0, 360.0))
            .AddAttribute("RxAzimuthEnd",
                          "RX azimuth angle end in degrees for angular scanning",
                          DoubleValue(360.0),
                          MakeDoubleAccessor(&CellScanBeamforming::GetRxAzimuthEnd,
                                             &CellScanBeamforming::SetRxAzimuthEnd),
                          MakeDoubleChecker<double>(0.0, 360.0));

    return tid;
}

/**
 * @brief Function that generates the beamforming vectors for a pair of
 * communicating devices using either sector-based or angular-based cell scan method
 * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
 * @param [in] ueSpectrumPhy the spectrum phy of the UE device
 * @return the beamforming vector pair of the gNB and the UE
 */
BeamformingVectorPair
CellScanBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                           const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_ABORT_MSG_IF(gnbSpectrumPhy == nullptr || ueSpectrumPhy == nullptr,
                    "Something went wrong, gnb or UE PHY layer not set.");
    double distance = gnbSpectrumPhy->GetMobility()->GetDistanceFrom(ueSpectrumPhy->GetMobility());
    NS_ABORT_MSG_IF(distance == 0,
                    "Beamforming method cannot be performed between two devices that are placed in "
                    "the same position.");

    Ptr<SpectrumChannel> gnbSpectrumChannel =
        gnbSpectrumPhy
            ->GetSpectrumChannel(); // SpectrumChannel should be const.. but need to change ns-3-dev
    Ptr<SpectrumChannel> ueSpectrumChannel = ueSpectrumPhy->GetSpectrumChannel();

    Ptr<const PhasedArraySpectrumPropagationLossModel> gnbThreeGppSpectrumPropModel =
        gnbSpectrumChannel->GetPhasedArraySpectrumPropagationLossModel();
    Ptr<const PhasedArraySpectrumPropagationLossModel> ueThreeGppSpectrumPropModel =
        ueSpectrumChannel->GetPhasedArraySpectrumPropagationLossModel();
    NS_ASSERT_MSG(gnbThreeGppSpectrumPropModel == ueThreeGppSpectrumPropModel,
                  "Devices should be connected on the same spectrum channel");

    std::vector<int> activeRbs;
    for (size_t rbId = 0; rbId < gnbSpectrumPhy->GetRxSpectrumModel()->GetNumBands(); rbId++)
    {
        activeRbs.push_back(rbId);
    }

    Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        0.0,
        activeRbs,
        gnbSpectrumPhy->GetRxSpectrumModel(),
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW);
    Ptr<SpectrumSignalParameters> fakeParams = Create<SpectrumSignalParameters>();
    fakeParams->psd = fakePsd->Copy();

    double max = 0;
    double maxTxTheta = 0;
    double maxRxTheta = 0;
    uint16_t maxTxSector = 0;
    uint16_t maxRxSector = 0;
    PhasedArrayModel::ComplexVector maxTxW;
    PhasedArrayModel::ComplexVector maxRxW;

    Ptr<UniformPlanarArray> gnbUpa = DynamicCast<UniformPlanarArray>(gnbSpectrumPhy->GetAntenna());
    Ptr<UniformPlanarArray> ueUpa = DynamicCast<UniformPlanarArray>(ueSpectrumPhy->GetAntenna());
    NS_ASSERT_MSG(gnbUpa, "gNB antenna should be UniformPlanarArray");
    NS_ASSERT_MSG(ueUpa, "UE antenna should be UniformPlanarArray");

    uint16_t txNumCols = gnbUpa->GetNumColumns();
    uint16_t txNumRows = gnbUpa->GetNumRows();
    uint16_t rxNumCols = ueUpa->GetNumColumns();
    uint16_t rxNumRows = ueUpa->GetNumRows();

    NS_ASSERT(gnbUpa->GetNumElems() && ueUpa->GetNumElems());

    // Dual-mode beamforming implementation:
    // - Angular scanning: Direct control of azimuth and zenith angles with configurable step sizes
    // - Sector scanning: Traditional 5G-LENA approach based on antenna array dimensions and
    // oversampling
    if (m_useAngularScanning)
    {
        // Angular-based scanning mode
        NS_LOG_INFO("Using angular-based beamforming scanning");

        // Use the configured angular step values
        double txZenithStep = m_txZenithStep;
        double rxZenithStep = m_rxZenithStep;
        double txAzimuthStep = m_txAzimuthStep;
        double rxAzimuthStep = m_rxAzimuthStep;

        // Scan through zenith angles using configurable range
        for (double txZenith = m_txZenithStart; txZenith < m_txZenithEnd; txZenith += txZenithStep)
        {
            // Use the current zenith angle directly for angular scanning
            double txTheta = txZenith;

            // Scan through azimuth angles using configurable range
            for (double txAzimuth = m_txAzimuthStart; txAzimuth < m_txAzimuthEnd;
                 txAzimuth += txAzimuthStep)
            {
                NS_ASSERT(txAzimuth < UINT16_MAX);

                // Use the new SetAngles method for angular-based beamforming
                gnbSpectrumPhy->GetBeamManager()->SetAngles(txAzimuth, txTheta);
                PhasedArrayModel::ComplexVector txW =
                    gnbSpectrumPhy->GetBeamManager()->GetCurrentBeamformingVector();

                if (maxTxW.GetSize() == 0)
                {
                    maxTxW = txW; // initialize maxTxW
                }

                // Scan through RX zenith angles using configurable range
                for (double rxZenith = m_rxZenithStart; rxZenith < m_rxZenithEnd;
                     rxZenith += rxZenithStep)
                {
                    // Use the current zenith angle directly for angular scanning
                    double rxTheta = rxZenith;

                    // Scan through RX azimuth angles using configurable range
                    for (double rxAzimuth = m_rxAzimuthStart; rxAzimuth < m_rxAzimuthEnd;
                         rxAzimuth += rxAzimuthStep)
                    {
                        NS_ASSERT(rxAzimuth < UINT16_MAX);

                        // Use the new SetAngles method for angular-based beamforming
                        ueSpectrumPhy->GetBeamManager()->SetAngles(rxAzimuth, rxTheta);
                        PhasedArrayModel::ComplexVector rxW =
                            ueSpectrumPhy->GetBeamManager()->GetCurrentBeamformingVector();

                        if (maxRxW.GetSize() == 0)
                        {
                            maxRxW = rxW; // initialize maxRxW
                        }

                        NS_ABORT_MSG_IF(
                            txW.GetSize() == 0 || rxW.GetSize() == 0,
                            "Beamforming vectors must be initialized in order to calculate "
                            "the long term matrix.");

                        Ptr<SpectrumSignalParameters> rxParams =
                            gnbThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity(
                                fakeParams,
                                gnbSpectrumPhy->GetMobility(),
                                ueSpectrumPhy->GetMobility(),
                                gnbSpectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>(),
                                ueSpectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>());

                        size_t nbands = rxParams->psd->GetSpectrumModel()->GetNumBands();
                        double power = Sum(*(rxParams->psd)) / nbands;

                        NS_LOG_LOGIC(" Rx power: " << power << " txTheta " << txTheta << " rxTheta "
                                                   << rxTheta << " txAzimuth " << txAzimuth
                                                   << " rxAzimuth " << rxAzimuth);

                        double delta = power - max;
                        // Handle numerical precision to avoid floating-point comparison issues
                        if (delta > max * 1e-6)
                        {
                            max = power;
                            maxTxSector = static_cast<uint16_t>(txAzimuth);
                            maxRxSector = static_cast<uint16_t>(rxAzimuth);
                            maxTxTheta = txTheta;
                            maxRxTheta = rxTheta;
                            maxTxW = txW;
                            maxRxW = rxW;
                        }
                    }
                }
            }
        }
    }
    else
    {
        // Original sector-based scanning mode
        NS_LOG_INFO("Using sector-based beamforming scanning");

        // In the original 5G LENA implementation, beamforming resolution is tied to the number
        // of elements (rows and columns) in the Phased Array Antenna (PAA). This sector-based
        // approach uses the antenna array dimensions to determine scanning resolution.
        double txZenithStep = 180 / ((txNumRows > 1 ? m_oversamplingFactor : 1) * txNumRows);
        double txSectorStep = 1.0 / (txNumCols > 1 ? m_oversamplingFactor : 1);
        double rxZenithStep = 180 / ((rxNumRows > 1 ? m_oversamplingFactor : 1) * rxNumRows);
        double rxSectorStep = 1.0 / (rxNumCols > 1 ? m_oversamplingFactor : 1);

        std::cout << txSectorStep << std::endl;
        std::cout << rxSectorStep << std::endl;

        // Scan through zenith angles
        for (double txZenith = 0; txZenith < 180; txZenith += txZenithStep)
        {
            // Calculate beam elevation to center it into the middle of the wedge, and not at the
            // start
            double txTheta = txZenith + txZenithStep * 0.5;

            // Scan through sectors
            for (double txSector = 0; txSector < txNumCols;
                 txSector += txSectorStep) // TR++ Modif good // Normally 10
            {
                NS_ASSERT(txSector < UINT16_MAX);

                // Use the original SetSector method for sector-based beamforming
                gnbSpectrumPhy->GetBeamManager()->SetSector(txSector, txTheta);
                PhasedArrayModel::ComplexVector txW =
                    gnbSpectrumPhy->GetBeamManager()->GetCurrentBeamformingVector();

                if (maxTxW.GetSize() == 0)
                {
                    maxTxW = txW; // initialize maxTxW
                }

                // Scan through RX zenith angles
                for (double rxZenith = 0; rxZenith < 180; rxZenith += rxZenithStep)
                {
                    // Calculate beam elevation to center it into the middle of the wedge, and not
                    // at the start
                    double rxTheta = rxZenith + rxZenithStep * 0.5;

                    // Scan through RX sectors
                    for (double rxSector = 0; rxSector < rxNumCols; rxSector += rxSectorStep)
                    {
                        NS_ASSERT(rxSector < UINT16_MAX);

                        // Use the original SetSector method for sector-based beamforming
                        ueSpectrumPhy->GetBeamManager()->SetSector(rxSector, rxTheta);
                        PhasedArrayModel::ComplexVector rxW =
                            ueSpectrumPhy->GetBeamManager()->GetCurrentBeamformingVector();

                        if (maxRxW.GetSize() == 0)
                        {
                            maxRxW = rxW; // initialize maxRxW
                        }

                        NS_ABORT_MSG_IF(
                            txW.GetSize() == 0 || rxW.GetSize() == 0,
                            "Beamforming vectors must be initialized in order to calculate "
                            "the long term matrix.");

                        Ptr<SpectrumSignalParameters> rxParams =
                            gnbThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity(
                                fakeParams,
                                gnbSpectrumPhy->GetMobility(),
                                ueSpectrumPhy->GetMobility(),
                                gnbSpectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>(),
                                ueSpectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>());

                        size_t nbands = rxParams->psd->GetSpectrumModel()->GetNumBands();
                        double power = Sum(*(rxParams->psd)) / nbands;

                        NS_LOG_LOGIC(" Rx power: " << power << " txTheta " << txTheta << " rxTheta "
                                                   << rxTheta << " tx sector "
                                                   << (M_PI * static_cast<double>(txSector) /
                                                           static_cast<double>(txNumCols) -
                                                       0.5 * M_PI) /
                                                          M_PI * 180
                                                   << " rx sector "
                                                   << (M_PI * static_cast<double>(rxSector) /
                                                           static_cast<double>(rxNumCols) -
                                                       0.5 * M_PI) /
                                                          M_PI * 180);

                        double delta = power - max;
                        // Handle numerical precision to avoid floating-point comparison issues
                        if (delta > max * 1e-6)
                        {
                            max = power;
                            maxTxSector = static_cast<uint16_t>(txSector);
                            maxRxSector = static_cast<uint16_t>(rxSector);
                            maxTxTheta = txTheta;
                            maxRxTheta = rxTheta;
                            maxTxW = txW;
                            maxRxW = rxW;
                        }
                    }
                }
            }
        }
    }

    BeamformingVector gnbBfv = BeamformingVector(std::make_pair(
        maxTxW,
        BeamId(maxTxSector * (txNumCols > 1 ? m_oversamplingFactor : 1), maxTxTheta)));
    BeamformingVector ueBfv = BeamformingVector(std::make_pair(
        maxRxW,
        BeamId(maxRxSector * (rxNumCols > 1 ? m_oversamplingFactor : 1), maxRxTheta)));

    NS_LOG_DEBUG(
        "Beamforming vectors with max power "
        << max
        << " for gNB with node id: " << gnbSpectrumPhy->GetMobility()->GetObject<Node>()->GetId()
        << " (" << gnbSpectrumPhy->GetMobility()->GetPosition()
        << ") and UE with node id: " << ueSpectrumPhy->GetMobility()->GetObject<Node>()->GetId()
        << " (" << ueSpectrumPhy->GetMobility()->GetPosition() << ") are txTheta " << maxTxTheta
        << " tx sector "
        << (M_PI * static_cast<double>(maxTxSector) / static_cast<double>(txNumCols) - 0.5 * M_PI) /
               M_PI * 180
        << " rxTheta " << maxRxTheta << " rx sector "
        << (M_PI * static_cast<double>(maxRxSector) / static_cast<double>(rxNumCols) - 0.5 * M_PI) /
               M_PI * 180);

    // Trace callback to log beamforming results for analysis
    // This triggers the LogBeamforming function with gNB ID, UE ID, power, and beamforming vectors
    // Please note that the beamforming is also performed between gNBs so this should be adapted to
    // handle that. However, this is not needed for the current use case. Basically just change the
    // signature to use Tx and Rx instead of gnbId and ueId.
    m_beamformingPerformed(gnbSpectrumPhy->GetDevice()->GetNode()->GetId(),
                           ueSpectrumPhy->GetDevice()->GetNode()->GetId(),
                           max,
                           gnbBfv,
                           ueBfv);

    NS_ASSERT(maxTxW.GetSize() && maxRxW.GetSize());

    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

TypeId
CellScanQuasiOmniBeamforming::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::CellScanQuasiOmniBeamforming")
            .SetParent<IdealBeamformingAlgorithm>()
            .AddConstructor<CellScanQuasiOmniBeamforming>()
            .AddAttribute("BeamSearchAngleStep",
                          "Angle step when searching for the best beam",
                          DoubleValue(30),
                          MakeDoubleAccessor(&CellScanQuasiOmniBeamforming::SetBeamSearchAngleStep,
                                             &CellScanQuasiOmniBeamforming::GetBeamSearchAngleStep),
                          MakeDoubleChecker<double>());

    return tid;
}

void
CellScanQuasiOmniBeamforming::SetBeamSearchAngleStep(double beamSearchAngleStep)
{
    m_beamSearchAngleStep = beamSearchAngleStep;
}

double
CellScanQuasiOmniBeamforming::GetBeamSearchAngleStep() const
{
    return m_beamSearchAngleStep;
}

BeamformingVectorPair
CellScanQuasiOmniBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                                    const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_ABORT_MSG_IF(gnbSpectrumPhy == nullptr || ueSpectrumPhy == nullptr,
                    "Something went wrong, gnb or UE PHY layer not set.");
    double distance = gnbSpectrumPhy->GetMobility()->GetDistanceFrom(ueSpectrumPhy->GetMobility());
    NS_ABORT_MSG_IF(distance == 0,
                    "Beamforming method cannot be performed between two devices that are placed in "
                    "the same position.");

    Ptr<const PhasedArraySpectrumPropagationLossModel> txThreeGppSpectrumPropModel =
        gnbSpectrumPhy->GetSpectrumChannel()->GetPhasedArraySpectrumPropagationLossModel();
    Ptr<const PhasedArraySpectrumPropagationLossModel> rxThreeGppSpectrumPropModel =
        ueSpectrumPhy->GetSpectrumChannel()->GetPhasedArraySpectrumPropagationLossModel();
    NS_ASSERT_MSG(txThreeGppSpectrumPropModel == rxThreeGppSpectrumPropModel,
                  "Devices should be connected to the same spectrum channel");

    std::vector<int> activeRbs;
    for (size_t rbId = 0; rbId < gnbSpectrumPhy->GetRxSpectrumModel()->GetNumBands(); rbId++)
    {
        activeRbs.push_back(rbId);
    }

    Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        0.0,
        activeRbs,
        gnbSpectrumPhy->GetRxSpectrumModel(),
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW);
    Ptr<SpectrumSignalParameters> fakeParams = Create<SpectrumSignalParameters>();
    fakeParams->psd = fakePsd->Copy();

    double max = 0;
    double maxTxTheta = 0;
    uint16_t maxTxSector = 0;
    PhasedArrayModel::ComplexVector maxTxW;

    UintegerValue uintValue;
    gnbSpectrumPhy->GetAntenna()->GetAttribute("NumColumns", uintValue);
    uint16_t txNumCols = static_cast<uint16_t>(uintValue.Get());

    ueSpectrumPhy->GetBeamManager()
        ->ChangeToQuasiOmniBeamformingVector(); // we have to set it immediately to q-omni so that
                                                // we can perform calculations when calling spectrum
                                                // model above

    PhasedArrayModel::ComplexVector rxW =
        ueSpectrumPhy->GetBeamManager()->GetCurrentBeamformingVector();
    BeamformingVector ueBfv = std::make_pair(rxW, OMNI_BEAM_ID);

    for (double txTheta = 60; txTheta < 121; txTheta = txTheta + m_beamSearchAngleStep)
    {
        for (uint16_t txSector = 0; txSector < txNumCols; txSector++)
        {
            NS_ASSERT(txSector < UINT16_MAX);

            gnbSpectrumPhy->GetBeamManager()->SetSector(txSector, txTheta);
            PhasedArrayModel::ComplexVector txW =
                gnbSpectrumPhy->GetBeamManager()->GetCurrentBeamformingVector();

            NS_ABORT_MSG_IF(txW.GetSize() == 0 || rxW.GetSize() == 0,
                            "Beamforming vectors must be initialized in order to calculate the "
                            "long term matrix.");
            Ptr<SpectrumSignalParameters> rxParams =
                txThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity(
                    fakeParams,
                    gnbSpectrumPhy->GetMobility(),
                    ueSpectrumPhy->GetMobility(),
                    gnbSpectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>(),
                    ueSpectrumPhy->GetAntenna()->GetObject<PhasedArrayModel>());

            double power = Sum(*(rxParams->psd));

            NS_LOG_LOGIC(" Rx power: "
                         << power << "txTheta " << txTheta << " tx sector "
                         << (M_PI * static_cast<double>(txSector) / static_cast<double>(txNumCols) -
                             0.5 * M_PI) /
                                M_PI * 180);

            if (max < power)
            {
                max = power;
                maxTxSector = txSector;
                maxTxTheta = txTheta;
                maxTxW = txW;
            }
        }
    }

    BeamformingVector gnbBfv =
        BeamformingVector(std::make_pair(maxTxW, BeamId(maxTxSector, maxTxTheta)));

    NS_LOG_DEBUG(
        "Beamforming vectors for gNB with node id: "
        << gnbSpectrumPhy->GetMobility()->GetObject<Node>()->GetId()
        << " and UE with node id: " << ueSpectrumPhy->GetMobility()->GetObject<Node>()->GetId()
        << " are txTheta " << maxTxTheta << " tx sector "
        << (M_PI * static_cast<double>(maxTxSector) / static_cast<double>(txNumCols) - 0.5 * M_PI) /
               M_PI * 180);

    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

TypeId
DirectPathBeamforming::GetTypeId()
{
    static TypeId tid = TypeId("ns3::DirectPathBeamforming")
                            .SetParent<IdealBeamformingAlgorithm>()
                            .AddConstructor<DirectPathBeamforming>();
    return tid;
}

BeamformingVectorPair
DirectPathBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                             const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_LOG_FUNCTION(this);

    Ptr<const UniformPlanarArray> gnbAntenna =
        gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();
    Ptr<const UniformPlanarArray> ueAntenna =
        ueSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();

    PhasedArrayModel::ComplexVector gNbAntennaWeights =
        CreateDirectPathBfv(gnbSpectrumPhy->GetMobility(),
                            ueSpectrumPhy->GetMobility(),
                            gnbAntenna);
    // store the antenna weights
    BeamformingVector gnbBfv =
        BeamformingVector(std::make_pair(gNbAntennaWeights, BeamId::GetEmptyBeamId()));

    PhasedArrayModel::ComplexVector ueAntennaWeights =
        CreateDirectPathBfv(ueSpectrumPhy->GetMobility(), gnbSpectrumPhy->GetMobility(), ueAntenna);
    // store the antenna weights
    BeamformingVector ueBfv =
        BeamformingVector(std::make_pair(ueAntennaWeights, BeamId::GetEmptyBeamId()));

    // TR++
    std::cout << "Call the callback" << std::endl;

    m_beamformingPerformed(gnbSpectrumPhy->GetDevice()->GetNode()->GetId(),
                           ueSpectrumPhy->GetDevice()->GetNode()->GetId(),
                           0.0,
                           gnbBfv,
                           ueBfv);

    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

TypeId
QuasiOmniDirectPathBeamforming::GetTypeId()
{
    static TypeId tid = TypeId("ns3::QuasiOmniDirectPathBeamforming")
                            .SetParent<DirectPathBeamforming>()
                            .AddConstructor<QuasiOmniDirectPathBeamforming>();
    return tid;
}

BeamformingVectorPair
QuasiOmniDirectPathBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                                      const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_LOG_FUNCTION(this);
    Ptr<const UniformPlanarArray> gnbAntenna =
        gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();
    Ptr<const UniformPlanarArray> ueAntenna =
        ueSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();

    // configure gNb beamforming vector to be quasi omni
    UintegerValue numCols;
    UintegerValue numColumns;
    gnbAntenna->GetAttribute("NumColumns", numCols);
    gnbAntenna->GetAttribute("NumColumns", numColumns);
    BeamformingVector gnbBfv = {CreateQuasiOmniBfv(gnbAntenna), OMNI_BEAM_ID};

    // configure UE beamforming vector to be directed towards gNB
    PhasedArrayModel::ComplexVector ueAntennaWeights =
        CreateDirectPathBfv(ueSpectrumPhy->GetMobility(), gnbSpectrumPhy->GetMobility(), ueAntenna);
    // store the antenna weights
    BeamformingVector ueBfv = BeamformingVector({ueAntennaWeights, BeamId::GetEmptyBeamId()});
    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

TypeId
DirectPathQuasiOmniBeamforming::GetTypeId()
{
    static TypeId tid = TypeId("ns3::DirectPathQuasiOmniBeamforming")
                            .SetParent<DirectPathBeamforming>()
                            .AddConstructor<DirectPathQuasiOmniBeamforming>();
    return tid;
}

BeamformingVectorPair
DirectPathQuasiOmniBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                                      const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_LOG_FUNCTION(this);
    Ptr<const UniformPlanarArray> gnbAntenna =
        gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();
    Ptr<const UniformPlanarArray> ueAntenna =
        ueSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();

    // configure ue beamforming vector to be quasi omni
    UintegerValue numCols;
    UintegerValue numColumns;
    ueAntenna->GetAttribute("NumColumns", numCols);
    ueAntenna->GetAttribute("NumColumns", numColumns);
    BeamformingVector ueBfv = {CreateQuasiOmniBfv(ueAntenna), OMNI_BEAM_ID};

    // configure gNB beamforming vector to be directed towards UE
    PhasedArrayModel::ComplexVector gnbAntennaWeights =
        CreateDirectPathBfv(gnbSpectrumPhy->GetMobility(),
                            ueSpectrumPhy->GetMobility(),
                            gnbAntenna);
    // store the antenna weights
    BeamformingVector gnbBfv = {gnbAntennaWeights, BeamId::GetEmptyBeamId()};

    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

TypeId
OptimalCovMatrixBeamforming::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OptimalCovMatrixBeamforming")
                            .SetParent<IdealBeamformingAlgorithm>()
                            .AddConstructor<OptimalCovMatrixBeamforming>();

    return tid;
}

BeamformingVectorPair
OptimalCovMatrixBeamforming::GetBeamformingVectors(
    [[maybe_unused]] const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
    [[maybe_unused]] const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_LOG_FUNCTION(this);
    return BeamformingVectorPair();
}

TypeId
KroneckerBeamforming::GetTypeId()
{
    static TypeId tid = TypeId("ns3::KroneckerBeamforming")
                            .SetParent<IdealBeamformingAlgorithm>()
                            .AddConstructor<KroneckerBeamforming>();
    return tid;
}

void
KroneckerBeamforming::SetColRxBeamAngles(std::vector<double> colAngles)
{
    m_colRxBeamAngles = colAngles;
}

void
KroneckerBeamforming::SetColTxBeamAngles(std::vector<double> colAngles)
{
    m_colTxBeamAngles = colAngles;
}

void
KroneckerBeamforming::SetRowRxBeamAngles(std::vector<double> rowAngles)
{
    m_rowRxBeamAngles = rowAngles;
}

void
KroneckerBeamforming::SetRowTxBeamAngles(std::vector<double> rowAngles)
{
    m_rowTxBeamAngles = rowAngles;
}

std::vector<double>
KroneckerBeamforming::GetColRxBeamAngles() const
{
    return m_colRxBeamAngles;
}

std::vector<double>
KroneckerBeamforming::GetColTxBeamAngles() const
{
    return m_colTxBeamAngles;
}

std::vector<double>
KroneckerBeamforming::GetRowRxBeamAngles() const
{
    return m_rowRxBeamAngles;
}

std::vector<double>
KroneckerBeamforming::GetRowTxBeamAngles() const
{
    return m_rowTxBeamAngles;
}

BeamformingVectorPair
KroneckerBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                            const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_ABORT_MSG_IF(gnbSpectrumPhy == nullptr || ueSpectrumPhy == nullptr,
                    "Something went wrong, gnb or UE PHY layer not set.");

    Ptr<SpectrumChannel> gnbSpectrumChannel = gnbSpectrumPhy->GetSpectrumChannel();
    Ptr<SpectrumChannel> ueSpectrumChannel = ueSpectrumPhy->GetSpectrumChannel();

    Ptr<const PhasedArraySpectrumPropagationLossModel> gnbThreeGppSpectrumPropModel =
        gnbSpectrumChannel->GetPhasedArraySpectrumPropagationLossModel();
    Ptr<const PhasedArraySpectrumPropagationLossModel> ueThreeGppSpectrumPropModel =
        ueSpectrumChannel->GetPhasedArraySpectrumPropagationLossModel();
    NS_ASSERT_MSG(gnbThreeGppSpectrumPropModel == ueThreeGppSpectrumPropModel,
                  "Devices should be connected on the same spectrum channel");

    std::vector<int> activeRbs;
    for (size_t rbId = 0; rbId < gnbSpectrumPhy->GetRxSpectrumModel()->GetNumBands(); rbId++)
    {
        activeRbs.push_back(rbId);
    }
    Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        0.0,
        activeRbs,
        gnbSpectrumPhy->GetRxSpectrumModel(),
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW);
    Ptr<SpectrumSignalParameters> fakeParams = Create<SpectrumSignalParameters>();

    double maxPower = 0;
    uint8_t activePanelIndex = 0;
    BeamformingVector gnbBfv;
    BeamformingVector ueBfv;
    // configure gNB and ue beamforming vectors to be Kronecer
    for (uint8_t b = 0; b < ueSpectrumPhy->GetNumPanels(); b++)
    {
        for (size_t k = 0; k < m_colTxBeamAngles.size(); k++)
        {
            for (size_t m = 0; m < m_rowTxBeamAngles.size(); m++)
            {
                for (size_t i = 0; i < m_colRxBeamAngles.size(); i++)
                {
                    for (size_t j = 0; j < m_rowRxBeamAngles.size(); j++)
                    {
                        auto bfUe = CreateKroneckerBfv(
                            ueSpectrumPhy->GetPanelByIndex(b)->GetObject<UniformPlanarArray>(),
                            m_rowTxBeamAngles[m],
                            m_colTxBeamAngles[k]);

                        ueSpectrumPhy->GetPanelByIndex(b)
                            ->GetObject<UniformPlanarArray>()
                            ->SetBeamformingVector(bfUe);

                        auto bf = CreateKroneckerBfv(
                            gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>(),
                            m_rowRxBeamAngles[j],
                            m_colRxBeamAngles[i]);
                        gnbSpectrumPhy->GetAntenna()
                            ->GetObject<UniformPlanarArray>()
                            ->SetBeamformingVector(bf);
                        fakeParams->psd = Copy<SpectrumValue>(fakePsd);
                        auto rxParams = gnbThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity(
                            fakeParams,
                            gnbSpectrumPhy->GetMobility(),
                            ueSpectrumPhy->GetMobility(),
                            gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>(),
                            ueSpectrumPhy->GetPanelByIndex(b)->GetObject<UniformPlanarArray>());

                        double power = Sum(*(rxParams->psd));
                        if (power > maxPower)
                        {
                            maxPower = power;
                            gnbBfv = {bf, BeamId(i, j)};
                            ueBfv = {bfUe, BeamId(k, m)}; // for the best Panel
                            activePanelIndex =
                                b; // active panel has to be update to K as better beam has found
                        }
                    }
                }
            }
        }
    }

    ueSpectrumPhy->SetActivePanel(activePanelIndex);
    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

TypeId
KroneckerQuasiOmniBeamforming::GetTypeId()
{
    static TypeId tid = TypeId("ns3::KroneckerQuasiOmniBeamforming")
                            .SetParent<IdealBeamformingAlgorithm>()
                            .AddConstructor<KroneckerQuasiOmniBeamforming>();
    return tid;
}

void
KroneckerQuasiOmniBeamforming::SetColBeamAngles(std::vector<double> colAngles)
{
    m_colBeamAngles = colAngles;
}

void
KroneckerQuasiOmniBeamforming::SetRowBeamAngles(std::vector<double> rowAngles)
{
    m_rowBeamAngles = rowAngles;
}

std::vector<double>
KroneckerQuasiOmniBeamforming::GetColBeamAngles() const
{
    return m_colBeamAngles;
}

std::vector<double>
KroneckerQuasiOmniBeamforming::GetRowBeamAngles() const
{
    return m_rowBeamAngles;
}

BeamformingVectorPair
KroneckerQuasiOmniBeamforming::GetBeamformingVectors(const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
                                                     const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const
{
    NS_ABORT_MSG_IF(gnbSpectrumPhy == nullptr || ueSpectrumPhy == nullptr,
                    "Something went wrong, gnb or UE PHY layer not set.");

    Ptr<SpectrumChannel> gnbSpectrumChannel = gnbSpectrumPhy->GetSpectrumChannel();
    Ptr<SpectrumChannel> ueSpectrumChannel = ueSpectrumPhy->GetSpectrumChannel();

    Ptr<const PhasedArraySpectrumPropagationLossModel> gnbThreeGppSpectrumPropModel =
        gnbSpectrumChannel->GetPhasedArraySpectrumPropagationLossModel();
    Ptr<const PhasedArraySpectrumPropagationLossModel> ueThreeGppSpectrumPropModel =
        ueSpectrumChannel->GetPhasedArraySpectrumPropagationLossModel();
    NS_ASSERT_MSG(gnbThreeGppSpectrumPropModel == ueThreeGppSpectrumPropModel,
                  "Devices should be connected on the same spectrum channel");

    std::vector<int> activeRbs;
    for (size_t rbId = 0; rbId < gnbSpectrumPhy->GetRxSpectrumModel()->GetNumBands(); rbId++)
    {
        activeRbs.push_back(rbId);
    }
    Ptr<const SpectrumValue> fakePsd = NrSpectrumValueHelper::CreateTxPowerSpectralDensity(
        0.0,
        activeRbs,
        gnbSpectrumPhy->GetRxSpectrumModel(),
        NrSpectrumValueHelper::UNIFORM_POWER_ALLOCATION_BW);

    // configure ue beamforming vector to be quasi
    Ptr<const UniformPlanarArray> ueAntenna =
        ueSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>();
    PhasedArrayModel::ComplexVector uebfV = CreateQuasiOmniBfv(ueAntenna);
    BeamformingVector ueBfv = {uebfV, OMNI_BEAM_ID};
    ueSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>()->SetBeamformingVector(uebfV);

    // configure gNB beamforming vector to be Kronecker
    Ptr<SpectrumSignalParameters> fakeParams = Create<SpectrumSignalParameters>();
    double maxPower = 0;
    BeamformingVector gnbBfv;

    for (size_t i = 0; i < m_colBeamAngles.size(); i++)
    {
        for (size_t j = 0; j < m_rowBeamAngles.size(); j++)
        {
            auto bf =
                CreateKroneckerBfv(gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>(),
                                   m_rowBeamAngles[j],
                                   m_colBeamAngles[i]);
            gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>()->SetBeamformingVector(bf);
            fakeParams->psd = Copy<SpectrumValue>(fakePsd);
            auto rxParams = gnbThreeGppSpectrumPropModel->CalcRxPowerSpectralDensity(
                fakeParams,
                gnbSpectrumPhy->GetMobility(),
                ueSpectrumPhy->GetMobility(),
                gnbSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>(),
                ueSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>());

            double power = Sum(*(rxParams->psd));
            if (power > maxPower)
            {
                maxPower = power;
                gnbBfv = {bf, BeamId(i, j)};
            }
        }
    }
    return BeamformingVectorPair(std::make_pair(gnbBfv, ueBfv));
}

// Getter and setter methods for dual-mode beamforming attributes
bool
CellScanBeamforming::GetUseAngularScanning() const
{
    return m_useAngularScanning;
}

void
CellScanBeamforming::SetUseAngularScanning(bool useAngular)
{
    m_useAngularScanning = useAngular;
}

double
CellScanBeamforming::GetTxZenithStep() const
{
    return m_txZenithStep;
}

void
CellScanBeamforming::SetTxZenithStep(double step)
{
    m_txZenithStep = step;
}

double
CellScanBeamforming::GetRxZenithStep() const
{
    return m_rxZenithStep;
}

void
CellScanBeamforming::SetRxZenithStep(double step)
{
    m_rxZenithStep = step;
}

double
CellScanBeamforming::GetTxAzimuthStep() const
{
    return m_txAzimuthStep;
}

void
CellScanBeamforming::SetTxAzimuthStep(double step)
{
    m_txAzimuthStep = step;
}

double
CellScanBeamforming::GetRxAzimuthStep() const
{
    return m_rxAzimuthStep;
}

void
CellScanBeamforming::SetRxAzimuthStep(double step)
{
    m_rxAzimuthStep = step;
}

double
CellScanBeamforming::GetTxZenithStart() const
{
    return m_txZenithStart;
}

void
CellScanBeamforming::SetTxZenithStart(double start)
{
    m_txZenithStart = start;
}

double
CellScanBeamforming::GetTxZenithEnd() const
{
    return m_txZenithEnd;
}

void
CellScanBeamforming::SetTxZenithEnd(double end)
{
    m_txZenithEnd = end;
}

double
CellScanBeamforming::GetRxZenithStart() const
{
    return m_rxZenithStart;
}

void
CellScanBeamforming::SetRxZenithStart(double start)
{
    m_rxZenithStart = start;
}

double
CellScanBeamforming::GetRxZenithEnd() const
{
    return m_rxZenithEnd;
}

void
CellScanBeamforming::SetRxZenithEnd(double end)
{
    m_rxZenithEnd = end;
}

double
CellScanBeamforming::GetTxAzimuthStart() const
{
    return m_txAzimuthStart;
}

void
CellScanBeamforming::SetTxAzimuthStart(double start)
{
    m_txAzimuthStart = start;
}

double
CellScanBeamforming::GetTxAzimuthEnd() const
{
    return m_txAzimuthEnd;
}

void
CellScanBeamforming::SetTxAzimuthEnd(double end)
{
    m_txAzimuthEnd = end;
}

double
CellScanBeamforming::GetRxAzimuthStart() const
{
    return m_rxAzimuthStart;
}

void
CellScanBeamforming::SetRxAzimuthStart(double start)
{
    m_rxAzimuthStart = start;
}

double
CellScanBeamforming::GetRxAzimuthEnd() const
{
    return m_rxAzimuthEnd;
}

void
CellScanBeamforming::SetRxAzimuthEnd(double end)
{
    m_rxAzimuthEnd = end;
}
} // namespace ns3
