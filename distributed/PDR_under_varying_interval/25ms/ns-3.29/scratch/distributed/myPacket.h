/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright
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
 *
 */

#ifndef MYPACKET_H
#define MYPACKET_H

#include "ns3/header.h"
#include "ns3/nstime.h"
#include "ns3/hash.h"

namespace ns3 {
/**
 * \ingroup dhtapplication
 * \class SeqTsHeader
 * \brief Packet header for UDP client/server application.
 *
 * The header is made of a 32bits sequence number followed by
 * a 64bits time stamp.
 */
class myPacket : public Header
{
public:

  myPacket ();

  myPacket (uint32_t sourceNodeId,uint32_t keyId,uint32_t vanetSourceHostIp);

  uint64_t GetPacketId (void) const;

  uint16_t GetWhichAttempt (void) const;

  void SetWhichAttempt (uint16_t attemptCount);

  uint64_t GetSendTime (void) const;

  uint32_t GetSourceNodeId (void) const;

  uint32_t GetKeyId (void) const;

  int32_t GetFoundNodeId (void) const;

  void SetFoundNodeId (int32_t foundNodeId);

  uint32_t GetVanetSourceHostIp (void) const;



  static TypeId GetTypeId (void);

  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);

private:

  uint64_t m_packetId;
  uint16_t m_whichAttempt;
  uint64_t m_sendTime;
  uint32_t m_sourceNodeId;
  uint32_t m_keyId;
  int32_t m_foundNodeId;
  uint32_t m_vanetSourceHostIp;
  Hasher   m_hash;

  uint64_t GetHash (const std::string phrase);

};

}// namespace ns3

#endif /* DHT_PACKET_PAYLOAD_H */
