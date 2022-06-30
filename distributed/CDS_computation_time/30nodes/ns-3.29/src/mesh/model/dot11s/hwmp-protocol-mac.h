/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
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
 * Author: Kirill Andreev <andreev@iitp.ru>
 */

#ifndef HWMP_STATE_H
#define HWMP_STATE_H

#include "ns3/mesh-wifi-interface-mac-plugin.h"
#include "ns3/hwmp-protocol.h"

namespace ns3 {

class MeshWifiInterfaceMac;
class WifiActionHeader;

namespace dot11s {

class HwmpProtocol;
class IePreq;
class IePrep;
class IePerr;
class IeLpp;
class IeClaim;

/**
 * \ingroup dot11s
 *
 * \brief Interface MAC plugin for HWMP -- 802.11s routing protocol
 */
class HwmpProtocolMac : public MeshWifiInterfaceMacPlugin
{
public:
  /**
   * Constructor
   *
   * \param ifIndex interface index
   * \param protocol pointer to HWMP protocol instance
   */
  HwmpProtocolMac (uint32_t ifIndex, Ptr<HwmpProtocol> protocol);
  ~HwmpProtocolMac ();
  ///\name Inherited from MAC plugin
  //\{
  void SetParent (Ptr<MeshWifiInterfaceMac> parent);
  bool Receive (Ptr<Packet> packet, const WifiMacHeader & header);
  bool UpdateOutcomingFrame (Ptr<Packet> packet, WifiMacHeader & header, Mac48Address from, Mac48Address to);
  /// Update beacon is empty, because HWMP does not know anything about beacons
  void UpdateBeacon (MeshWifiBeacon & beacon) const {};
  int64_t AssignStreams (int64_t stream);
  //\}

private:
  /// allow HwmpProtocol class friend access
  friend class HwmpProtocol;
  /**
   * \returns a path selection action header
   */
  static WifiActionHeader GetWifiActionHeader ();
  ///\name Interaction with HWMP:
  //\{
  /**
   * Send PREQ function
   * \param preq the PREQ
   */
  void SendPreq (IePreq preq);
  /**
   * Send PREQ function
   * \param preq vector of PREQ information elements
   */
  void SendPreq (std::vector<IePreq> preq);
  /**
   * Send PREP function
   * \param prep the PREP information element
   * \param receiver the MAC address of the receiver
   */
  void SendPrep (IePrep prep, Mac48Address receiver);
  /**
   * Send LPP function
   * \param lpp the LPP information element
   */
  void SendLpp(IeLpp lpp);
  /**
   * Send Claim Packet function
   * \param claimPkt the Claim Packet information element
   */
  void SendClaimPkt(IeClaim claimPkt);
  /**
   * Forward a path error
   * \param destinations vector of failed destinations
   * \param receivers vector of receivers
   */
  void ForwardPerr (std::vector<HwmpProtocol::FailedDestination> destinations, std::vector<Mac48Address> receivers);
  /**
   * initiate my own path error
   * \param destinations vector of failed destinations
   * \param receivers vector of receivers
   */
  void InitiatePerr (std::vector<HwmpProtocol::FailedDestination> destinations, std::vector<Mac48Address> receivers);
  /**
   * Request a destination. If cannot send PREQ immediately, add a
   * destination to existing PREQ generated by me and stored in PREQ queue
   * \param dest is the destination to be resolved
   * \param originator_seqno is a sequence number that shall be preq originator sequenece number
   * \param dst_seqno is a sequence number taken from routing table
   */
  void RequestDestination (Mac48Address dest, uint32_t originator_seqno, uint32_t dst_seqno);
  //\}

  /// Sends one PREQ when PreqMinInterval after last PREQ expires (if any PREQ exists in rhe queue)
  void SendMyPreq ();
  /// Send PERR function
  void SendMyPerr ();
  /**
   * \param peerAddress peer address
   * \return metric to HWMP protocol, needed only by metrics to add peer as routing entry
   */
  uint32_t GetLinkMetric (Mac48Address peerAddress) const;
  /**
   * Get the channel ID
   * \returns the channel ID
   */
  uint16_t GetChannelId () const;
  /**
   * Report statistics
   * \param os The output stream on which to report
   */
  void Report (std::ostream & os) const;
  /// Reset statistics
  void ResetStats ();
private:
  Ptr<MeshWifiInterfaceMac> m_parent; ///< parent
  uint32_t m_ifIndex; ///< IF index
  Ptr<HwmpProtocol> m_protocol; ///< protocol

  ///\name my PREQ and PREQ timer:
  //\{
  EventId m_preqTimer;
  std::vector<IePreq>  m_myPreq;
  //\}
  ///\name PERR timer and stored path error
  //\{
  EventId m_perrTimer;
  /// MyPerr structure
  struct MyPerr {
    std::vector<HwmpProtocol::FailedDestination> destinations; ///< destinations
    std::vector<Mac48Address> receivers; ///< receivers
  };
  MyPerr m_myPerr; ///< PERR
  ///\name Statistics:
  //\{
  /// Statistics structure
  struct Statistics
  {
    uint16_t txPreq; ///< transmit PREQ
    uint16_t rxPreq; ///< receive PREQ
    uint16_t txPrep; ///< transmit PREP
    uint16_t rxPrep; ///< receive PREP
    uint16_t txPerr; ///< transmit PERR
    uint16_t rxPerr; ///< receive PERR
    uint16_t txLpp; ///< transmit LPP
    uint16_t rxLpp; ///< receive LPP
    uint16_t txMgt; ///< transmit management
    uint32_t txMgtBytes; ///< transmit management bytes
    uint16_t rxMgt; ///< receive management
    uint32_t rxMgtBytes; ///< receive management bytes
    uint16_t txData; ///< transmit data
    uint32_t txDataBytes; ///< transmit data bytes
    uint16_t rxData; ///< receive data
    uint32_t rxDataBytes; ///< receive data bytes
    /**
     * Print function
     * \param os the output stream
     */
    void Print (std::ostream & os) const;
    Statistics ();
  };
  Statistics m_stats; ///< statistics
  //\}
private:
  /**
   * Receive data frame
   *
   * \param packet
   * \param header
   * \returns true if a packet was received
   */
  bool ReceiveData (Ptr<Packet> packet, const WifiMacHeader & header);
  /**
   * Receive action management frame
   *
   * \param packet
   * \param header
   * \returns true if a packet was received
   */
  bool ReceiveAction (Ptr<Packet> packet, const WifiMacHeader & header);
};
} // namespace dot11s
} // namespace ns3
#endif
