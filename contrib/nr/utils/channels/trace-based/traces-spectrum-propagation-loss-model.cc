/*
 * Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering,
 * New York University
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
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

 * The NYU-Padova mmWave threeGppSpectrumPropagationLossModel code was used as the baseline for
 * this code
 (https://github.com/nyuwireless-unipd/ns3-mmwave/blob/new-handover/src/spectrum/model/three-gpp-spectrum-propagation-loss-model.cc)
 * Modified by NIST <tanguy.ropitault@nist.gov>
 */

#include "traces-spectrum-propagation-loss-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/string.h"
#include "ns3/three-gpp-channel-model.h"

#include <map>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TracesSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(TracesSpectrumPropagationLossModel);

/**
 * Constructor for TracesSpectrumPropagationLossModel
 */
TracesSpectrumPropagationLossModel::TracesSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

/**
 * Destructor for TracesSpectrumPropagationLossModel
 */
TracesSpectrumPropagationLossModel::~TracesSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

/**
 * Clean up resources and dispose of the channel model
 */
void
TracesSpectrumPropagationLossModel::DoDispose()
{
    // Clear long-term component cache
    m_longTermMap.clear();

    // Dispose of the channel model
    m_channelModel->Dispose();
    m_channelModel = nullptr;
}

TypeId
TracesSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TracesSpectrumPropagationLossModel")
            .SetParent<PhasedArraySpectrumPropagationLossModel>()
            .SetGroupName("Spectrum")
            .AddConstructor<TracesSpectrumPropagationLossModel>()
            .AddAttribute(
                "ChannelModel",
                "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                StringValue("ns3::ThreeGppChannelModel"),
                MakePointerAccessor(&TracesSpectrumPropagationLossModel::SetChannelModel,
                                    &TracesSpectrumPropagationLossModel::GetChannelModel),
                MakePointerChecker<MatrixBasedChannelModel>());

    return tid;
}

/**
 * Set the channel model to use for spectrum propagation calculations
 * @param channel Pointer to the channel model implementing MatrixBasedChannelModel interface
 */
void
TracesSpectrumPropagationLossModel::SetChannelModel(Ptr<MatrixBasedChannelModel> channel)
{
    m_channelModel = channel;
}

/**
 * Get the current channel model
 * @return Pointer to the current channel model
 */
Ptr<MatrixBasedChannelModel>
TracesSpectrumPropagationLossModel::GetChannelModel() const
{
    return m_channelModel;
}

/**
 * Get the operating frequency from the channel model
 * @return Operating frequency in Hz
 */
double
TracesSpectrumPropagationLossModel::GetFrequency() const
{
    DoubleValue freq;
    m_channelModel->GetAttribute("Frequency", freq);
    return freq.Get();
}

/**
 * Set an attribute of the channel model
 * @param name Attribute name
 * @param value Attribute value
 */
void
TracesSpectrumPropagationLossModel::SetChannelModelAttribute(const std::string& name,
                                                             const AttributeValue& value)
{
    m_channelModel->SetAttribute(name, value);
}

/**
 * Get an attribute of the channel model
 * @param name Attribute name
 * @param value Attribute value (output)
 */
void
TracesSpectrumPropagationLossModel::GetChannelModelAttribute(const std::string& name,
                                                             AttributeValue& value) const
{
    m_channelModel->GetAttribute(name, value);
}

/**
 * Calculate the long-term component of the channel for given beamforming vectors
 * @param params Channel matrix containing the channel coefficients
 * @param sW Transmitter beamforming vector
 * @param uW Receiver beamforming vector
 * @return Long-term component vector for each cluster
 */
PhasedArrayModel::ComplexVector
TracesSpectrumPropagationLossModel::CalcLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
    const PhasedArrayModel::ComplexVector& sW,
    const PhasedArrayModel::ComplexVector& uW) const
{
    NS_LOG_FUNCTION(this);

    // Get antenna array dimensions
    size_t uAntennaNum = uW.GetSize();
    size_t sAntennaNum = sW.GetSize();

    // Verify dimensions match the channel matrix
    NS_ASSERT(uAntennaNum == params->m_channel.GetNumRows());
    NS_ASSERT(sAntennaNum == params->m_channel.GetNumCols());
    NS_LOG_DEBUG("CalcLongTerm with " << uAntennaNum << " u antenna elements and " << sAntennaNum
                                      << " s antenna elements.");

    // Calculate long-term component: uW^T * H * sW
    // This reduces computation load by pre-calculating the beamforming contribution
    // Only small-scale fading needs to be updated if large-scale parameters and antenna weights
    // remain unchanged
    return params->m_channel.MultiplyByLeftAndRightMatrix(uW.Transpose(), sW);
}

/**
 * Calculate beamforming gain including Doppler effects and propagation delays
 * @param txPsd Transmitted power spectral density
 * @param longTerm Long-term component vector for each cluster
 * @param channelMatrix Channel matrix containing the channel coefficients
 * @param channelParams Channel parameters including delays and angles
 * @param sSpeed Transmitter velocity vector
 * @param uSpeed Receiver velocity vector
 * @return Received power spectral density with beamforming gain applied
 */
Ptr<SpectrumValue>
TracesSpectrumPropagationLossModel::CalcBeamformingGain(
    Ptr<SpectrumValue> txPsd,
    PhasedArrayModel::ComplexVector longTerm,
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
    const ns3::Vector& sSpeed,
    const ns3::Vector& uSpeed) const
{
    NS_LOG_FUNCTION(this);

    // Create a copy of the transmitted PSD to modify
    Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue>(txPsd);

    // Get number of multipath clusters
    uint16_t numCluster = channelMatrix->m_channel.GetNumPages();

    // Calculate Doppler factor based on current time and frequency
    double slotTime = Simulator::Now().GetSeconds();
    double factor = 2 * M_PI * slotTime * GetFrequency() / 3e8;
    PhasedArrayModel::ComplexVector doppler(numCluster);

    // Verify all data structures have correct dimensions
    // Using .at() instead of [] for bounds checking
    NS_ASSERT(numCluster <= channelParams->m_alpha.size());
    NS_ASSERT(numCluster <= channelParams->m_D.size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX].size());
    NS_ASSERT(numCluster <= longTerm.GetSize());

    // Check if channel parameters were generated in the same direction as the channel matrix
    bool isSameDirection = (channelParams->m_nodeIds == channelMatrix->m_nodeIds);

    // Extract angle vectors based on direction
    MatrixBasedChannelModel::DoubleVector zoa;
    MatrixBasedChannelModel::DoubleVector zod;
    MatrixBasedChannelModel::DoubleVector aoa;
    MatrixBasedChannelModel::DoubleVector aod;

    if (isSameDirection)
    {
        // Use angles as they are (same direction)
        zoa = channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX];
        zod = channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX];
        aoa = channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX];
        aod = channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX];
    }
    else
    {
        // Flip angles for opposite direction
        zod = channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX];
        zoa = channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX];
        aod = channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX];
        aoa = channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX];
    }

    // Calculate Doppler terms for each cluster
    for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
        // Get alpha and D parameters for additional Doppler contribution
        // These account for moving objects in vehicular scenarios
        double alpha = channelParams->m_alpha[cIndex];
        double D = channelParams->m_D[cIndex];

        // Calculate Doppler shift including velocity components and scatter terms
        double tempDoppler =
            factor * ((sin(zoa[cIndex] * M_PI / 180) * cos(aoa[cIndex] * M_PI / 180) * uSpeed.x +
                       sin(zoa[cIndex] * M_PI / 180) * sin(aoa[cIndex] * M_PI / 180) * uSpeed.y +
                       cos(zoa[cIndex] * M_PI / 180) * uSpeed.z) +
                      (sin(zod[cIndex] * M_PI / 180) * cos(aod[cIndex] * M_PI / 180) * sSpeed.x +
                       sin(zod[cIndex] * M_PI / 180) * sin(aod[cIndex] * M_PI / 180) * sSpeed.y +
                       cos(zod[cIndex] * M_PI / 180) * sSpeed.z) +
                      2 * alpha * D);
        doppler[cIndex] = std::complex<double>(cos(tempDoppler), sin(tempDoppler));
    }
    NS_ASSERT(numCluster <= doppler.GetSize());

    // Apply Doppler terms and propagation delays to calculate beamforming gain
    auto vit = tempPsd->ValuesBegin();      // PSD value iterator
    auto sbit = tempPsd->ConstBandsBegin(); // Frequency band iterator
    while (vit != tempPsd->ValuesEnd())
    {
        if ((*vit) != 0.00)
        {
            std::complex<double> subsbandGain(0.0, 0.0);
            double fsb = (*sbit).fc; // Center frequency of the sub-band

            // Sum contributions from all clusters
            for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
            {
                // Calculate phase shift due to propagation delay
                double delay = -2 * M_PI * fsb * (channelParams->m_delay[cIndex]);

                // Combine long-term, Doppler, and delay components
                subsbandGain = subsbandGain + longTerm[cIndex] * doppler[cIndex] *
                                                  std::complex<double>(cos(delay), sin(delay));
            }
            // Apply beamforming gain to the PSD
            *vit = (*vit) * (norm(subsbandGain));
        }
        vit++;
        sbit++;
    }

    // Calculate total received power for verification
    double totalPowerW = 0.0;
    auto vit2 = tempPsd->ValuesBegin();
    while (vit2 != tempPsd->ValuesEnd())
    {
        totalPowerW += (*vit2);
        vit2++;
    }
    return tempPsd;
}

/**
 * Get or compute the long-term component for a pair of phased array models
 * @param channelMatrix Channel matrix containing the channel coefficients
 * @param aPhasedArrayModel First phased array model
 * @param bPhasedArrayModel Second phased array model
 * @return Long-term component vector for each cluster
 */
PhasedArrayModel::ComplexVector
TracesSpectrumPropagationLossModel::GetLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    PhasedArrayModel::ComplexVector
        longTerm; // Vector containing the long term component for each cluster

    // Determine which antenna is transmitter (s) and which is receiver (u)
    // Check if the channel matrix was generated considering a as the s-node and b as the u-node
    PhasedArrayModel::ComplexVector sW;
    PhasedArrayModel::ComplexVector uW;
    if (!channelMatrix->IsReverse(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId()))
    {
        // a is transmitter, b is receiver
        sW = aPhasedArrayModel->GetBeamformingVector();
        uW = bPhasedArrayModel->GetBeamformingVector();
    }
    else
    {
        // b is transmitter, a is receiver
        sW = bPhasedArrayModel->GetBeamformingVector();
        uW = aPhasedArrayModel->GetBeamformingVector();
    }

    // Flags for cache management
    bool update = false;   // Indicates whether the long term has to be updated
    bool notFound = false; // Indicates if the long term has not been computed yet

    // Compute unique key for this transmitter-receiver pair
    uint64_t longTermId =
        MatrixBasedChannelModel::GetKey(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId());

    // Check if long-term component exists in cache
    if (m_longTermMap.find(longTermId) != m_longTermMap.end())
    {
        NS_LOG_DEBUG("found the long term component in the map");
        longTerm = m_longTermMap[longTermId]->m_longTerm;

        // Check if update is needed due to:
        // - Channel matrix update (new generation time)
        // - Transmitter beamforming vector change
        // - Receiver beamforming vector change
        update = (m_longTermMap[longTermId]->m_channel->m_generatedTime !=
                      channelMatrix->m_generatedTime ||
                  m_longTermMap[longTermId]->m_sW != sW || m_longTermMap[longTermId]->m_uW != uW);
    }
    else
    {
        NS_LOG_DEBUG("long term component NOT found");
        notFound = true;
    }

    // Compute new long-term component if needed
    if (update || notFound)
    {
        NS_LOG_DEBUG("compute the long term");
        // Compute the long-term component
        longTerm = CalcLongTerm(channelMatrix, sW, uW);

        // Store the long-term component in cache
        Ptr<LongTerm> longTermItem = Create<LongTerm>();
        longTermItem->m_longTerm = longTerm;
        longTermItem->m_channel = channelMatrix;
        longTermItem->m_sW = sW;
        longTermItem->m_uW = uW;

        m_longTermMap[longTermId] = longTermItem;
    }

    return longTerm;
}

/**
 * Calculate received power spectral density including channel effects and beamforming
 * @param params Transmitted signal parameters
 * @param a Transmitter mobility model
 * @param b Receiver mobility model
 * @param aPhasedArrayModel Transmitter phased array model
 * @param bPhasedArrayModel Receiver phased array model
 * @return Received signal parameters with updated PSD
 */
Ptr<SpectrumSignalParameters>
TracesSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);

    // Create a copy of the signal parameters for the receiver
    Ptr<SpectrumSignalParameters> rxParams = params->Copy();

    // Get positions of transmitter and receiver
    Vector aPosition = a->GetPosition();
    Vector bPosition = b->GetPosition();

    NS_LOG_DEBUG("Position of a: (" << aPosition.x << ", " << aPosition.y << ", " << aPosition.z
                                    << ")");
    NS_LOG_DEBUG("Position of b: (" << bPosition.x << ", " << bPosition.y << ", " << bPosition.z
                                    << ")");

    NS_LOG_DEBUG("aPhasedArrayModel ID: " << aPhasedArrayModel->GetId()
                                          << ", bPhasedArrayModel ID: "
                                          << bPhasedArrayModel->GetId());

    // Get channel information and calculate beamforming gain
    Ptr<SpectrumValue> newPsd = Copy<SpectrumValue>(rxParams->psd);
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
        m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
        m_channelModel->GetParams(a, b);
    PhasedArrayModel::ComplexVector longTerm =
        GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);

    // Apply beamforming gain including Doppler effects and propagation delays
    newPsd = CalcBeamformingGain(newPsd,
                                 longTerm,
                                 channelMatrix,
                                 channelParams,
                                 a->GetVelocity(),
                                 b->GetVelocity());

    // Calculate total received power by integrating over all frequency bands
    double rxPowerTotal = 0.0;
    Values::const_iterator vit = newPsd->ConstValuesBegin();
    Bands::const_iterator sbit = newPsd->ConstBandsBegin();

    while (vit != newPsd->ConstValuesEnd())
    {
        // Power = PSD * bandwidth for each frequency band
        rxPowerTotal += (*vit) * (sbit->fh - sbit->fl);
        ++vit;
        ++sbit;
    }

    // Update the received signal parameters with the new PSD
    rxParams->psd = newPsd;
    return rxParams;
}

/**
 * Assign random streams to the model (no-op for trace-based model)
 * @param stream Starting stream number
 * @return Next available stream number
 */
int64_t
TracesSpectrumPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return stream;
}

} // namespace ns3