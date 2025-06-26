// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// Modified by NIST <tanguy.ropitault@nist.gov>
// SPDX-License-Identifier: GPL-2.0-only

#include "beamforming-helper-base.h"

#include "ns3/beamforming-vector.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"

#ifndef SRC_NR_HELPER_IDEAL_BEAMFORMING_HELPER_H_
#define SRC_NR_HELPER_IDEAL_BEAMFORMING_HELPER_H_

namespace ns3
{

class NrGnbNetDevice;
class NrUeNetDevice;
class IdealBeamformingAlgorithm;

/**
 * @brief Log the beamforming vectors to a file with a context
 * @param context The context of the beamforming
 * @param gnbId The ID of the gNB
 * @param ueId The ID of the UE
 * @param power The power of the beamforming vector
 * @param gnbBeamformingVector The beamforming vector of the gNB
 */
void LogBeamformingWithContext(std::string context,
                               uint32_t gnbId,
                               uint32_t ueId,
                               double power,
                               BeamformingVector gnbBeamformingVector,
                               BeamformingVector ueBeamformingVector);

/**
 * @ingroup helper
 * @brief The IdealBeamformingHelper class
 */
class IdealBeamformingHelper : public BeamformingHelperBase
{
  public:
    /**
     * @brief IdealBeamformingHelper
     */
    IdealBeamformingHelper();
    /**
     * @brief ~IdealBeamformingHelper
     */
    ~IdealBeamformingHelper() override;

    /**
     * @brief Get the Type ID
     * @return the TypeId of the instance
     */
    static TypeId GetTypeId();

    /**
     * @brief SetBeamformingMethod
     * @param beamformingMethod
     */
    void SetBeamformingMethod(const TypeId& beamformingMethod) override;

    /**
     * @brief SetIdealBeamformingPeriodicity
     * @param v
     */
    void SetPeriodicity(const Time& v);
    /**
     * @brief GetIdealBeamformingPeriodicity
     * @return
     */
    Time GetPeriodicity() const;

    Time Get() const;

    /**
     * @brief Run beamforming task
     */
    virtual void Run() const;

    /**
     * @brief Specify among which devices the beamforming algorithm should be
     * performed
     * @param gNbDev gNB device
     * @param ueDev UE device
     */
    void AddBeamformingTask(const Ptr<NrGnbNetDevice>& gNbDev,
                            const Ptr<NrUeNetDevice>& ueDev) override;

    static std::string GetOutputDirectory();
    static void SetOutputDirectory(const std::string& dir);

  protected:
    // inherited from Object
    void DoInitialize() override;

    /**
     * @brief The beamforming timer has expired; at the next slot, perform beamforming.
     *
     */
    virtual void ExpireBeamformingTimer();

    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;

    Time m_beamformingPeriodicity; //!< The beamforming periodicity or how frequently beamforming
                                   //!< tasks will be executed
    EventId m_beamformingTimer;    //!< Beamforming timer that is used to schedule periodical
                                   //!< beamforming vector updates
    Ptr<IdealBeamformingAlgorithm>
        m_beamformingAlgorithm; //!< The beamforming algorithm that will be used

    std::list<SpectrumPhyPair> m_spectrumPhyPair; //!< The list of beamforming tasks to be executed

    static std::string m_outputDirectory;
};

}; // namespace ns3

#endif /* SRC_NR_HELPER_IDEAL_BEAMFORMING_HELPER_H_ */
