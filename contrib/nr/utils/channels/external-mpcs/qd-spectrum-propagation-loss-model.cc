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
 *
 */

#include "qd-spectrum-propagation-loss-model.h"

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

NS_LOG_COMPONENT_DEFINE("QdSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(QdSpectrumPropagationLossModel);

QdSpectrumPropagationLossModel::QdSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

QdSpectrumPropagationLossModel::~QdSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

void
QdSpectrumPropagationLossModel::DoDispose()
{
    m_longTermMap.clear();
    m_channelModel->Dispose();
    m_channelModel = nullptr;
}

TypeId
QdSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::QdSpectrumPropagationLossModel")
            .SetParent<PhasedArraySpectrumPropagationLossModel>()
            .SetGroupName("Spectrum")
            .AddConstructor<QdSpectrumPropagationLossModel>()
            .AddAttribute(
                "ChannelModel",
                "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                StringValue("ns3::ThreeGppChannelModel"),
                MakePointerAccessor(&QdSpectrumPropagationLossModel::SetChannelModel,
                                    &QdSpectrumPropagationLossModel::GetChannelModel),
                MakePointerChecker<MatrixBasedChannelModel>());

    return tid;
}

void
QdSpectrumPropagationLossModel::SetChannelModel(Ptr<MatrixBasedChannelModel> channel)
{
    m_channelModel = channel;
}

Ptr<MatrixBasedChannelModel>
QdSpectrumPropagationLossModel::GetChannelModel() const
{
    return m_channelModel;
}

double
QdSpectrumPropagationLossModel::GetFrequency() const
{
    DoubleValue freq;
    m_channelModel->GetAttribute("Frequency", freq);
    return freq.Get();
}

void
QdSpectrumPropagationLossModel::SetChannelModelAttribute(const std::string& name,
                                                         const AttributeValue& value)
{
    m_channelModel->SetAttribute(name, value);
}

void
QdSpectrumPropagationLossModel::GetChannelModelAttribute(const std::string& name,
                                                         AttributeValue& value) const
{
    m_channelModel->GetAttribute(name, value);
}

PhasedArrayModel::ComplexVector
QdSpectrumPropagationLossModel::CalcLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
    const PhasedArrayModel::ComplexVector& sW,
    const PhasedArrayModel::ComplexVector& uW) const
{
    NS_LOG_FUNCTION(this);

    size_t uAntennaNum = uW.GetSize();
    size_t sAntennaNum = sW.GetSize();
    // std::cout << "uAntennaNum: " << uAntennaNum << std::endl;
    // std::cout << "params->m_channel.GetNumRows(): " << params->m_channel.GetNumRows() <<
    // std::endl; std::cout << "sAntennaNum: " << sAntennaNum << std::endl; std::cout <<
    // "params->m_channel.GetNumCols(): " << params->m_channel.GetNumCols() << std::endl;

    //  std::cout << "\n=== Channel Matrix ===\n";
    //  for (size_t cluster = 0; cluster < params->m_channel.GetNumPages(); ++cluster)
    //  {
    //      std::cout << "\nCluster " << cluster << ":\n";
    //      for (size_t rx = 0; rx < params->m_channel.GetNumRows(); ++rx)
    //      {
    //          std::cout << "Rx " << rx << ": ";
    //          for (size_t tx = 0; tx < params->m_channel.GetNumCols(); ++tx)
    //          {
    //              std::cout
    //                       << params->m_channel(rx, tx, cluster) << "\t";
    //          }
    //          std::cout << "\n";
    //      }
    //  }
    //  std::cout << "==================\n\n";
    NS_ASSERT(uAntennaNum == params->m_channel.GetNumRows());
    NS_ASSERT(sAntennaNum == params->m_channel.GetNumCols());
    NS_LOG_DEBUG("CalcLongTerm with " << uAntennaNum << " u antenna elements and " << sAntennaNum
                                      << " s antenna elements.");
    // store the long term part to reduce computation load
    // only the small scale fading needs to be updated if the large scale parameters and antenna
    // weights remain unchanged. here we calculate long term uW * Husn * sW, the result is an array
    // of values per cluster
    return params->m_channel.MultiplyByLeftAndRightMatrix(uW.Transpose(), sW);
}

Ptr<SpectrumValue>
QdSpectrumPropagationLossModel::CalcBeamformingGain(
    Ptr<SpectrumValue> txPsd,
    PhasedArrayModel::ComplexVector longTerm,
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
    const ns3::Vector& sSpeed,
    const ns3::Vector& uSpeed) const
{
    NS_LOG_FUNCTION(this);

    Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue>(txPsd);

    uint16_t numCluster = channelMatrix->m_channel.GetNumPages();

    // compute the doppler term
    // NOTE the update of Doppler is simplified by only taking the center angle of
    // each cluster in to consideration.
    double slotTime = Simulator::Now().GetSeconds();
    double factor = 2 * M_PI * slotTime * GetFrequency() / 3e8;
    PhasedArrayModel::ComplexVector doppler(numCluster);

    // The following asserts might seem paranoic, but it is important to
    // make sure that all the structures that are passed to this function
    // are of the correct dimensions before using the operator [].
    // If you dont understand the comment read about the difference of .at()
    // and [] operators, ...
    NS_ASSERT(numCluster <= channelParams->m_alpha.size());
    NS_ASSERT(numCluster <= channelParams->m_D.size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX].size());
    NS_ASSERT(numCluster <= longTerm.GetSize());

    // check if channelParams structure is generated in direction s-to-u or u-to-s
    bool isSameDirection = (channelParams->m_nodeIds == channelMatrix->m_nodeIds);

    MatrixBasedChannelModel::DoubleVector zoa;
    MatrixBasedChannelModel::DoubleVector zod;
    MatrixBasedChannelModel::DoubleVector aoa;
    MatrixBasedChannelModel::DoubleVector aod;

    // if channel params is generated in the same direction in which we
    // generate the channel matrix, angles and zenith od departure and arrival are ok,
    // just set them to corresponding variable that will be used for the generation
    // of channel matrix, otherwise we need to flip angles and zeniths of departure and arrival
    if (isSameDirection)
    {
        zoa = channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX];
        zod = channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX];
        aoa = channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX];
        aod = channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX];
    }
    else
    {
        zod = channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX];
        zoa = channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX];
        aod = channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX];
        aoa = channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX];
    }

    for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
        // Compute alpha and D as described in 3GPP TR 37.885 v15.3.0, Sec. 6.2.3
        // These terms account for an additional Doppler contribution due to the
        // presence of moving objects in the surrounding environment, such as in
        // vehicular scenarios.
        // This contribution is applied only to the delayed (reflected) paths and
        // must be properly configured by setting the value of
        // m_vScatt, which is defined as "maximum speed of the vehicle in the
        // layout".
        // By default, m_vScatt is set to 0, so there is no additional Doppler
        // contribution.

        double alpha = channelParams->m_alpha[cIndex];
        double D = channelParams->m_D[cIndex];
        // std::cout << "alpha: " << alpha << std::endl;
        // std::cout << "D: " << D << std::endl;
        // cluster angle angle[direction][n], where direction = 0(aoa), 1(zoa).
        double tempDoppler =
            factor * ((sin(zoa[cIndex] * M_PI / 180) * cos(aoa[cIndex] * M_PI / 180) * uSpeed.x +
                       sin(zoa[cIndex] * M_PI / 180) * sin(aoa[cIndex] * M_PI / 180) * uSpeed.y +
                       cos(zoa[cIndex] * M_PI / 180) * uSpeed.z) +
                      (sin(zod[cIndex] * M_PI / 180) * cos(aod[cIndex] * M_PI / 180) * sSpeed.x +
                       sin(zod[cIndex] * M_PI / 180) * sin(aod[cIndex] * M_PI / 180) * sSpeed.y +
                       cos(zod[cIndex] * M_PI / 180) * sSpeed.z) +
                      2 * alpha * D);
        doppler[cIndex] = std::complex<double>(cos(tempDoppler), sin(tempDoppler));
        //  std::cout << "Cluster " << cIndex
        //       << "  doppler phase (rad)= " << tempDoppler
        //       << "  |doppler phasor|= "
        //       << std::abs(doppler[cIndex]) << std::endl;
    }
    // std::cout << "BEGINNING OF A TRACE " << std::endl;
    // std::cout << "First doppler value: " << doppler[0] << std::endl;
    // std::cout << "Second doppler value: " << doppler[1] << std::endl;

    //  std::cout << "Velocity of u: " << uSpeed << std::endl;
    NS_ASSERT(numCluster <= doppler.GetSize());

    // apply the doppler term and the propagation delay to the long term component
    // to obtain the beamforming gain
    auto vit = tempPsd->ValuesBegin();      // psd iterator
    auto sbit = tempPsd->ConstBandsBegin(); // band iterator
    while (vit != tempPsd->ValuesEnd())
    {
        if ((*vit) != 0.00)
        {
            std::complex<double> subsbandGain(0.0, 0.0);
            double fsb = (*sbit).fc; // center frequency of the sub-band
                                     //  for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
            for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
            {
                // Calculates the phase shift due to delay
                // it hsould be phase
                double delay = -2 * M_PI * fsb * (channelParams->m_delay[cIndex]);

                subsbandGain = subsbandGain + longTerm[cIndex] * doppler[cIndex] *
                                                  std::complex<double>(cos(delay), sin(delay));
            }
            *vit = (*vit) * (norm(subsbandGain));
        }
        vit++;
        sbit++;
    }

    //  std::cout << "Delays" << std::endl;
    //  for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
    //  {
    //      double delay = -2 * M_PI * (*sbit).fc * (channelParams->m_delay[cIndex]);
    //      std::cout << "(" << cos(delay) << "," << sin(delay) << ") ";
    //       std::cout << "Long Tem and delay product: " << longTerm[cIndex] *
    //       std::complex<double>(cos(delay), sin(delay)) << std::endl;
    //  }

    // Calculate total received power in W and dBm
    double totalPowerW = 0.0;
    auto vit2 = tempPsd->ValuesBegin();
    while (vit2 != tempPsd->ValuesEnd())
    {
        totalPowerW += (*vit2);
        vit2++;
    }

    //  double totalPowerDbm = 10 * std::log10(totalPowerW * 1000); // Convert W to dBm

    //  std::cout << "Time: " << Simulator::Now().GetSeconds()
    //            << " CalcBeamfgorming Gain Total Received Power "
    //            << totalPowerW << " W (" << totalPowerDbm << " dBm)" << std::endl;
    return tempPsd;
}

PhasedArrayModel::ComplexVector
QdSpectrumPropagationLossModel::GetLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    PhasedArrayModel::ComplexVector
        longTerm; // vector containing the long term component for each cluster

    // check if the channel matrix was generated considering a as the s-node and
    // b as the u-node or vice-versa
    PhasedArrayModel::ComplexVector sW;
    PhasedArrayModel::ComplexVector uW;
    //  std::cout << "IS REVERSE " << aPhasedArrayModel->GetId() << bPhasedArrayModel->GetId() <<
    //  std::endl;
    if (!channelMatrix->IsReverse(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId()))
    {
        sW = aPhasedArrayModel->GetBeamformingVector();
        uW = bPhasedArrayModel->GetBeamformingVector();
    }
    else
    {
        sW = bPhasedArrayModel->GetBeamformingVector();
        uW = aPhasedArrayModel->GetBeamformingVector();
    }

    bool update = false;   // indicates whether the long term has to be updated
    bool notFound = false; // indicates if the long term has not been computed yet

    // compute the long term key, the key is unique for each tx-rx pair
    uint64_t longTermId =
        MatrixBasedChannelModel::GetKey(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId());

    // look for the long term in the map and check if it is valid
    if (m_longTermMap.find(longTermId) != m_longTermMap.end())
    {
        NS_LOG_DEBUG("found the long term component in the map");
        longTerm = m_longTermMap[longTermId]->m_longTerm;

        // check if the channel matrix has been updated
        // or the s beam has been changed
        // or the u beam has been changed
        update = (m_longTermMap[longTermId]->m_channel->m_generatedTime !=
                      channelMatrix->m_generatedTime ||
                  m_longTermMap[longTermId]->m_sW != sW || m_longTermMap[longTermId]->m_uW != uW);
    }
    else
    {
        NS_LOG_DEBUG("long term component NOT found");
        notFound = true;
    }

    if (update || notFound)
    {
        NS_LOG_DEBUG("compute the long term");
        // compute the long term component
        longTerm = CalcLongTerm(channelMatrix, sW, uW);

        // store the long term
        Ptr<LongTerm> longTermItem = Create<LongTerm>();
        longTermItem->m_longTerm = longTerm;
        longTermItem->m_channel = channelMatrix;
        longTermItem->m_sW = sW;
        longTermItem->m_uW = uW;

        m_longTermMap[longTermId] = longTermItem;
    }

    return longTerm;
}

//  Ptr<SpectrumSignalParameters>
// QdSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
//     Ptr<const SpectrumSignalParameters> params,
//     Ptr<const MobilityModel> a,
//     Ptr<const MobilityModel> b,
//     Ptr<const PhasedArrayModel> aPhasedArrayModel,
//     Ptr<const PhasedArrayModel> bPhasedArrayModel) const
// {
//     NS_LOG_FUNCTION(this);

//     // Use the correct type from the PHY header.
//     Ptr<const SpectrumSignalParametersPhy> phyParams =
//         DynamicCast<const SpectrumSignalParametersPhy>(params);
//     NS_ASSERT(phyParams != nullptr);

//     uint32_t aId = a->GetObject<Node>()->GetId();
//     uint32_t bId = b->GetObject<Node>()->GetId();
//     NS_ASSERT(aId != bId);
//     NS_ASSERT_MSG(a->GetDistanceFrom(b) > 0.0,
//                   "The position of a and b devices cannot be the same");

//     // Create a new set of PHY signal parameters based on the original
//     Ptr<SpectrumSignalParametersPhy> newParams = Create<SpectrumSignalParametersPhy>(*phyParams);

//     Ptr<SpectrumValue> newPsd = Copy<SpectrumValue>(newParams->psd);
//     Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
//         m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
//     Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
//         m_channelModel->GetParams(a, b);
//     PhasedArrayModel::ComplexVector longTerm =
//         GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);

//     newPsd = CalcBeamformingGain(newPsd,
//                                  longTerm,
//                                  channelMatrix,
//                                  channelParams,
//                                  a->GetVelocity(),
//                                  b->GetVelocity());

//     // Attach the modified PSD to newParams and return.
//     newParams->psd = newPsd;
//     return newParams;
// }

Ptr<SpectrumSignalParameters>
QdSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);

    // std::cout<<"DoCalcRxPowerSpectralDensity"<<std::endl;

    Ptr<SpectrumSignalParameters> rxParams = params->Copy();

    Vector aPosition = a->GetPosition();
    Vector bPosition = b->GetPosition();

    NS_LOG_DEBUG("Position of a: (" << aPosition.x << ", " << aPosition.y << ", " << aPosition.z
                                    << ")");
    NS_LOG_DEBUG("Position of b: (" << bPosition.x << ", " << bPosition.y << ", " << bPosition.z
                                    << ")");

    NS_LOG_DEBUG("aPhasedArrayModel ID: " << aPhasedArrayModel->GetId()
                                          << ", bPhasedArrayModel ID: "
                                          << bPhasedArrayModel->GetId());

    Ptr<SpectrumValue> newPsd = Copy<SpectrumValue>(rxParams->psd);
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
        m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
        m_channelModel->GetParams(a, b);
    PhasedArrayModel::ComplexVector longTerm =
        GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);
    // std::cout << "longTerm: " << longTerm << std::endl;

    newPsd = CalcBeamformingGain(newPsd,
                                 longTerm,
                                 channelMatrix,
                                 channelParams,
                                 a->GetVelocity(),
                                 b->GetVelocity());

    // Calculate total received power by integrating over all frequencies
    double rxPowerTotal = 0.0;
    Values::const_iterator vit = newPsd->ConstValuesBegin();
    Bands::const_iterator sbit = newPsd->ConstBandsBegin();

    while (vit != newPsd->ConstValuesEnd())
    {
        // Power = PSD * bandwidth
        rxPowerTotal += (*vit) * (sbit->fh - sbit->fl);
        ++vit;
        ++sbit;
    }

    // double rxPowerDBm = 10 * std::log10(rxPowerTotal * 1000); // Convert W to dBm
    // // NS_LOG_DEBUG("Total Received Power from node " << a->GetObject<Node>()->GetId()
    // //              << " to node " << b->GetObject<Node>()->GetId() << ": "
    // //              << rxPowerTotal << " W (" << rxPowerDBm << " dBm)");
    // std::cout << "Time: " << Simulator::Now().GetSeconds() << " Total Received Power from node "
    //           << a->GetObject<Node>()->GetId() << " to node " << b->GetObject<Node>()->GetId()
    //           << ": " << rxPowerTotal << " W (" << rxPowerDBm << " dBm)" << std::endl;
    rxParams->psd = newPsd;
    return rxParams;
}

//   Ptr<SpectrumSignalParameters>  QdSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
//      Ptr<const SpectrumSignalParameters> params,
//      Ptr<const MobilityModel> a,
//      Ptr<const MobilityModel> b,
//      Ptr<const PhasedArrayModel> aPhasedArrayModel,
//      Ptr<const PhasedArrayModel> bPhasedArrayModel) const
//  {
//      NS_LOG_FUNCTION(this);

//      Ptr<const SpectrumSignalParametersPhy> phyParams = DynamicCast<const
//      SpectrumSignalParametersPhy>(params);

//     NS_ASSERT(phyParams != nullptr);

//      uint32_t aId = a->GetObject<Node>()->GetId(); // id of the node a
//      uint32_t bId = b->GetObject<Node>()->GetId(); // id of the node b

//      NS_ASSERT(aId != bId);
//      NS_ASSERT_MSG(a->GetDistanceFrom(b) > 0.0,
//                    "The position of a and b devices cannot be the same");

//      Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue>(params->psd);

//      // retrieve the antenna of device a
//      NS_ASSERT_MSG(aPhasedArrayModel, "Antenna not found for node " << aId);
//      NS_LOG_DEBUG("a node " << a->GetObject<Node>() << " antenna " << aPhasedArrayModel);

//      // retrieve the antenna of the device b
//      NS_ASSERT_MSG(bPhasedArrayModel, "Antenna not found for device " << bId);
//      NS_LOG_DEBUG("b node " << bId << " antenna " << bPhasedArrayModel);

//     //  Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
//     //      m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
//     //  Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
//     //      m_channelModel->GetParams(a, b);

//     //  // retrieve the long term component
//     //  PhasedArrayModel::ComplexVector longTerm =
//     //      GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);

//     //  // apply the beamforming gain
//     //  rxPsd = CalcBeamformingGain(rxPsd,
//     //                              longTerm,
//     //                              channelMatrix,
//     //                              channelParams,
//     //                              a->GetVelocity(),
//     //                              b->GetVelocity());

//     //  return rxPsd;

//     Ptr<SpectrumSignalParametersPhy> newParams = Create<SpectrumSignalParametersPhy>(*phyParams);

//     // === Your channel logic ===
//     Ptr<SpectrumValue> newPsd = Copy<SpectrumValue>(newParams->psd);
//     Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
//         m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
//     Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
//         m_channelModel->GetParams(a, b);
//     PhasedArrayModel::ComplexVector longTerm =
//         GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);

//     newPsd = CalcBeamformingGain(newPsd,
//                                  longTerm,
//                                  channelMatrix,
//                                  channelParams,
//                                  a->GetVelocity(),
//                                  b->GetVelocity());

//     // === Attach new PSD to the returned signal ===
//     newParams->psd = newPsd;

//     return newParams;
//  }

int64_t
QdSpectrumPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return stream;
}

} // namespace ns3
