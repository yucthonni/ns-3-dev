// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Modified by NIST <tanguy.ropitault@nist.gov>
// SPDX-License-Identifier: GPL-2.0-only

#ifndef SRC_NR_MODEL_IDEAL_BEAMFORMING_ALGORITHM_H_
#define SRC_NR_MODEL_IDEAL_BEAMFORMING_ALGORITHM_H_

#include "beam-id.h"
#include "beamforming-vector.h"

#include "ns3/object.h"
#include "ns3/traced-callback.h"

namespace ns3
{

class SpectrumModel;
class SpectrumValue;
class NrGnbNetDevice;
class NrUeNetDevice;
class NrSpectrumPhy;

/**
 * @ingroup gnb-phy
 * @brief Generate "Ideal" beamforming vectors
 *
 * IdealBeamformingAlgorithm purpose is to generate beams for the pair
 * of communicating devices.
 *
 * Algorithms that inherit this class assume a perfect knowledge of the channel,
 * because of which this group of algorithms is called "ideal".
 */
class IdealBeamformingAlgorithm : public Object
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief Function that generates the beamforming vectors for a pair of communicating devices
     * @param [in] gnbSpectrumPhy gNb spectrum phy instance
     * @param [in] ueSpectrumPhy UE spectrum phy instance
     */
    virtual BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const = 0;

  protected:
    /**
     * Traced callback for when beamforming is performed
     * Parameters: gNB ID, UE ID, metric value, gNB beamforming vector, UE beamforming vector
     */
    mutable TracedCallback<uint32_t, uint32_t, double, BeamformingVector, BeamformingVector>
        m_beamformingPerformed;
};

/**
 * @ingroup gnb-phy
 * @brief The CellScanBeamforming class provides dual-mode beamforming:
 *        - Sector-based scanning (original 5G LENA approach)
 *        - Angular-based scanning (direct angle control)
 */
class CellScanBeamforming : public IdealBeamformingAlgorithm
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief constructor
     */
    CellScanBeamforming() = default;

    /**
     * @brief destructor
     */
    ~CellScanBeamforming() override = default;

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices using either sector-based or angular-based cell scan method
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE device
     * @return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;

    /**
     * @brief Get the current scanning mode
     * @return true if using angular scanning, false if using sector-based scanning
     */
    bool GetUseAngularScanning() const;

    /**
     * @brief Set the scanning mode
     * @param [in] useAngular true for angular scanning, false for sector-based scanning
     */
    void SetUseAngularScanning(bool useAngular);

    /**
     * @brief Get the TX zenith angle step for angular scanning
     * @return the zenith angle step in degrees
     */
    double GetTxZenithStep() const;

    /**
     * @brief Set the TX zenith angle step for angular scanning
     * @param [in] step the zenith angle step in degrees
     */
    void SetTxZenithStep(double step);

    /**
     * @brief Get the RX zenith angle step for angular scanning
     * @return the zenith angle step in degrees
     */
    double GetRxZenithStep() const;

    /**
     * @brief Set the RX zenith angle step for angular scanning
     * @param [in] step the zenith angle step in degrees
     */
    void SetRxZenithStep(double step);

    /**
     * @brief Get the TX azimuth angle step for angular scanning
     * @return the azimuth angle step in degrees
     */
    double GetTxAzimuthStep() const;

    /**
     * @brief Set the TX azimuth angle step for angular scanning
     * @param [in] step the azimuth angle step in degrees
     */
    void SetTxAzimuthStep(double step);

    /**
     * @brief Get the RX azimuth angle step for angular scanning
     * @return the azimuth angle step in degrees
     */
    double GetRxAzimuthStep() const;

    /**
     * @brief Set the RX azimuth angle step for angular scanning
     * @param [in] step the azimuth angle step in degrees
     */
    void SetRxAzimuthStep(double step);

    /**
     * @brief Get the TX zenith angle start for angular scanning
     * @return the zenith angle start in degrees
     */
    double GetTxZenithStart() const;

    /**
     * @brief Set the TX zenith angle start for angular scanning
     * @param [in] start the zenith angle start in degrees
     */
    void SetTxZenithStart(double start);

    /**
     * @brief Get the TX zenith angle end for angular scanning
     * @return the zenith angle end in degrees
     */
    double GetTxZenithEnd() const;

    /**
     * @brief Set the TX zenith angle end for angular scanning
     * @param [in] end the zenith angle end in degrees
     */
    void SetTxZenithEnd(double end);

    /**
     * @brief Get the RX zenith angle start for angular scanning
     * @return the zenith angle start in degrees
     */
    double GetRxZenithStart() const;

    /**
     * @brief Set the RX zenith angle start for angular scanning
     * @param [in] start the zenith angle start in degrees
     */
    void SetRxZenithStart(double start);

    /**
     * @brief Get the RX zenith angle end for angular scanning
     * @return the zenith angle end in degrees
     */
    double GetRxZenithEnd() const;

    /**
     * @brief Set the RX zenith angle end for angular scanning
     * @param [in] end the zenith angle end in degrees
     */
    void SetRxZenithEnd(double end);

    /**
     * @brief Get the TX azimuth angle start for angular scanning
     * @return the azimuth angle start in degrees
     */
    double GetTxAzimuthStart() const;

    /**
     * @brief Set the TX azimuth angle start for angular scanning
     * @param [in] start the azimuth angle start in degrees
     */
    void SetTxAzimuthStart(double start);

    /**
     * @brief Get the TX azimuth angle end for angular scanning
     * @return the azimuth angle end in degrees
     */
    double GetTxAzimuthEnd() const;

    /**
     * @brief Set the TX azimuth angle end for angular scanning
     * @param [in] end the azimuth angle end in degrees
     */
    void SetTxAzimuthEnd(double end);

    /**
     * @brief Get the RX azimuth angle start for angular scanning
     * @return the azimuth angle start in degrees
     */
    double GetRxAzimuthStart() const;

    /**
     * @brief Set the RX azimuth angle start for angular scanning
     * @param [in] start the azimuth angle start in degrees
     */
    void SetRxAzimuthStart(double start);

    /**
     * @brief Get the RX azimuth angle end for angular scanning
     * @return the azimuth angle end in degrees
     */
    double GetRxAzimuthEnd() const;

    /**
     * @brief Set the RX azimuth angle end for angular scanning
     * @param [in] end the azimuth angle end in degrees
     */
    void SetRxAzimuthEnd(double end);

  private:
    uint8_t m_oversamplingFactor;     //!< Number of samples per row and per column
    bool m_useAngularScanning{false}; //!< Use angular scanning instead of sector-based scanning
    double m_txZenithStep{10.0};      //!< TX zenith angle step in degrees for angular scanning
    double m_rxZenithStep{10.0};      //!< RX zenith angle step in degrees for angular scanning
    double m_txAzimuthStep{1.0};      //!< TX azimuth angle step in degrees for angular scanning
    double m_rxAzimuthStep{90.0};     //!< RX azimuth angle step in degrees for angular scanning
    double m_txZenithStart{117.5};    //!< TX zenith angle start in degrees for angular scanning
    double m_txZenithEnd{118.5};      //!< TX zenith angle end in degrees for angular scanning
    double m_rxZenithStart{42.5};     //!< RX zenith angle start in degrees for angular scanning
    double m_rxZenithEnd{43.5};       //!< RX zenith angle end in degrees for angular scanning
    double m_txAzimuthStart{0.0};     //!< TX azimuth angle start in degrees for angular scanning
    double m_txAzimuthEnd{360.0};     //!< TX azimuth angle end in degrees for angular scanning
    double m_rxAzimuthStart{0.0};     //!< RX azimuth angle start in degrees for angular scanning
    double m_rxAzimuthEnd{360.0};     //!< RX azimuth angle end in degrees for angular scanning
};

/**
 * @ingroup gnb-phy
 * @brief The CellScanQuasiOmniBeamforming class
 */
class CellScanQuasiOmniBeamforming : public IdealBeamformingAlgorithm
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @return Gets value of BeamSearchAngleStep attribute
     */
    double GetBeamSearchAngleStep() const;

    /**
     * @brief Sets the value of BeamSearchAngleStep attribute
     */
    void SetBeamSearchAngleStep(double beamSearchAngleStep);

    /**
     * @brief constructor
     */
    CellScanQuasiOmniBeamforming() = default;

    /**
     * @brief destructor
     */
    ~CellScanQuasiOmniBeamforming() override = default;

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using cell scan method at gNB and a fixed quasi-omni beamforming
     * vector at UE \param [in] gnbSpectrumPhy the spectrum phy of the gNB \param [in] ueSpectrumPhy
     * the spectrum phy of the UE \return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;

  private:
    double m_beamSearchAngleStep{30};
};

/**
 * @ingroup gnb-phy
 * @brief The DirectPathBeamforming class
 */
class DirectPathBeamforming : public IdealBeamformingAlgorithm
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using the direct path direction
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE
     * @return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;
};

/**
 * @ingroup gnb-phy
 * @brief The QuasiOmniDirectPathBeamforming class
 */
class QuasiOmniDirectPathBeamforming : public DirectPathBeamforming
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using the quasi omni beamforming vector for gNB
     * and direct path beamforming vector for UEs
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE
     * @return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;
};

/**
 * @ingroup gnb-phy
 * @brief The QuasiOmniDirectPathBeamforming class
 */
class DirectPathQuasiOmniBeamforming : public DirectPathBeamforming
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using the direct-path beamforming vector for gNB
     * and quasi-omni beamforming vector for UEs
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE
     * @return the beamforming vector pair of the gNB and the UE
     *
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;
};

/**
 * @ingroup gnb-phy
 * @brief The OptimalCovMatrixBeamforming class not implemented yet.
 * TODO The idea was to port one of the initial beamforming methods that
 * were implemented in NYU/University of Padova mmwave module.
 * Method is based on a long term covariation matrix.
 */
class OptimalCovMatrixBeamforming : public IdealBeamformingAlgorithm
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using the direct-path beamforming vector for gNB
     * and quasi-omni beamforming vector for UEs
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE
     * @return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;
};

/**
 * @ingroup gnb-phy
 * @brief The KroneckerBeamforming class
 */
class KroneckerBeamforming : public IdealBeamformingAlgorithm
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @return Gets value of BeamColAngles of Rx attribute
     */
    std::vector<double> GetColRxBeamAngles() const;

    /**
     * @return Gets value of BeamColAngles of Tx attribute
     */
    std::vector<double> GetColTxBeamAngles() const;

    /**
     * @return Gets value of BeamRowAngles of Rx attribute
     */
    std::vector<double> GetRowRxBeamAngles() const;

    /**
     * @return Gets value of BeamRowAngles of Tx attribute
     */
    std::vector<double> GetRowTxBeamAngles() const;

    /**
     * @brief Sets the value of BeamColAngles of Rx attribute
     */
    void SetColRxBeamAngles(std::vector<double> colAngles);

    /**
     * @brief Sets the value of BeamColAngles of Tx attribute
     */
    void SetColTxBeamAngles(std::vector<double> colAngles);

    /**
     * @brief Sets the value of BeamRowAngles of Rx attribute
     */
    void SetRowRxBeamAngles(std::vector<double> rowAngles);

    /**
     * @brief Sets the value of BeamRowAngles of Tx attribute
     */
    void SetRowTxBeamAngles(std::vector<double> rowAngles);

    /**
     * @brief constructor
     */
    KroneckerBeamforming() = default;

    /**
     * @brief destructor
     */
    ~KroneckerBeamforming() override = default;

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using kronecker method for both gNB and UE device
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE device
     * @return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;

  private:
    std::vector<double> m_colRxBeamAngles{0, 90};
    std::vector<double> m_colTxBeamAngles{0, 90};
    std::vector<double> m_rowRxBeamAngles{0, 90};
    std::vector<double> m_rowTxBeamAngles{0, 90};
};

/**
 * @ingroup gnb-phy
 * @brief The KronQuasiBeamforming class
 */
class KroneckerQuasiOmniBeamforming : public IdealBeamformingAlgorithm
{
  public:
    /**
     * @brief Get the type id
     * @return the type id of the class
     */
    static TypeId GetTypeId();

    /**
     * @return Gets value of BeamColAngles attribute
     */
    std::vector<double> GetColBeamAngles() const;

    /**
     * @return Gets value of BeamRowAngles attribute
     */
    std::vector<double> GetRowBeamAngles() const;

    /**
     * @brief Sets the value of BeamColAngles attribute
     */
    void SetColBeamAngles(std::vector<double> colAngles);

    /**
     * @brief Sets the value of BeamRowAngles attribute
     */
    void SetRowBeamAngles(std::vector<double> rowAngles);

    /**
     * @brief constructor
     */
    KroneckerQuasiOmniBeamforming() = default;

    /**
     * @brief destructor
     */
    ~KroneckerQuasiOmniBeamforming() override = default;

    /**
     * @brief Function that generates the beamforming vectors for a pair of
     * communicating devices by using kronecker method for gNB and Quasi for the UE
     * @param [in] gnbSpectrumPhy the spectrum phy of the gNB
     * @param [in] ueSpectrumPhy the spectrum phy of the UE device
     * @return the beamforming vector pair of the gNB and the UE
     */
    BeamformingVectorPair GetBeamformingVectors(
        const Ptr<NrSpectrumPhy>& gnbSpectrumPhy,
        const Ptr<NrSpectrumPhy>& ueSpectrumPhy) const override;

  private:
    std::vector<double> m_colBeamAngles{0, 90};
    std::vector<double> m_rowBeamAngles{0, 90};
};
} // namespace ns3
#endif
