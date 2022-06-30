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
 * Author:
 */

#include "myPacket.h"

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/header.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("myPacket");

NS_OBJECT_ENSURE_REGISTERED (myPacket);


myPacket::myPacket ()
  : m_sendTime (Simulator::Now ().GetMilliSeconds())
{
  NS_LOG_FUNCTION (this);
}

myPacket::myPacket (uint32_t sourceNodeId,uint32_t keyId,uint32_t vanetSourceHostIp)
{
  NS_LOG_FUNCTION (this);
  m_whichAttempt = 1;
  m_sourceNodeId = sourceNodeId;
  m_keyId = keyId;
  m_vanetSourceHostIp = vanetSourceHostIp;
  m_foundNodeId = -1;
  m_sendTime = Simulator::Now ().GetMilliSeconds();

  std::stringstream ss;
  m_hash = Hasher ( Create<Hash::Function::Murmur3> () );
  //"whichAttempt:" << whichAttempt <<
  ss<< "PacketId: "<< m_sourceNodeId <<" "<< m_keyId<< " " << m_vanetSourceHostIp ;
  m_packetId = GetHash(ss.str());
  NS_LOG_DEBUG( ss.str() <<" : "<<m_packetId);
}

uint64_t myPacket::GetHash (const std::string phrase)
    {
      m_hash.clear ();
      uint64_t h = m_hash.GetHash64 (phrase);

      return h;
    }


uint64_t myPacket::GetPacketId (void) const
  {
    NS_LOG_FUNCTION (this);
    return m_packetId;
  }

uint32_t myPacket::GetVanetSourceHostIp (void) const
  {
    NS_LOG_FUNCTION (this);
    return m_vanetSourceHostIp;
  }

uint16_t myPacket::GetWhichAttempt (void) const
  {
    NS_LOG_FUNCTION(this);
    return m_whichAttempt;
  }

void myPacket::SetWhichAttempt (uint16_t attemptCount)
  {
    NS_LOG_FUNCTION(this);
    m_whichAttempt = attemptCount;
  }

uint32_t myPacket::GetSourceNodeId (void) const
  {
     NS_LOG_FUNCTION(this);
     return m_sourceNodeId;
  }

uint32_t myPacket::GetKeyId (void) const
  {
    NS_LOG_FUNCTION(this);
    return m_keyId;
  }

void myPacket::SetFoundNodeId (int32_t foundNodeId)
  {
    NS_LOG_FUNCTION(this);
    m_foundNodeId = foundNodeId;
  }

int32_t myPacket::GetFoundNodeId (void) const
  {
    NS_LOG_FUNCTION(this);
    return m_foundNodeId;
  }

uint64_t
myPacket::GetSendTime (void) const
{
 // NS_LOG_FUNCTION (this);
 // return TimeStep (m_sendTime);
  return m_sendTime;
}

TypeId
myPacket::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::myPacket")
    .SetParent<Header> ()
    .SetGroupName("Applications")
    .AddConstructor<myPacket> ()
  ;
  return tid;
}
TypeId
myPacket::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
void
myPacket::Print (std::ostream &os) const
{
  NS_LOG_FUNCTION (this << &os);
  os << "(PacketId=" << m_packetId << " time=" << TimeStep (m_sendTime).GetSeconds () << " sourceNode=" << m_sourceNodeId << " keyID=" <<m_keyId <<" )";
}
uint32_t
myPacket::GetSerializedSize (void) const
{
  NS_LOG_FUNCTION (this);
  return 8+2+8+4+4+4+4;
}

void
myPacket::Serialize (Buffer::Iterator start) const
{
  NS_LOG_FUNCTION (this << &start);
  Buffer::Iterator i = start;
  i.WriteHtonU64 (m_packetId);
  i.WriteHtonU16 (m_whichAttempt);
  i.WriteHtonU64 (m_sendTime);
  i.WriteHtonU32 (m_sourceNodeId);
  i.WriteHtonU32 (m_keyId);
  i.WriteHtonU32 (m_foundNodeId);
  i.WriteHtonU32 (m_vanetSourceHostIp);


}
uint32_t
myPacket::Deserialize (Buffer::Iterator start)
{
  NS_LOG_FUNCTION (this << &start);
  Buffer::Iterator i = start;
  m_packetId = i.ReadNtohU64 ();
  m_whichAttempt = i.ReadNtohU16 ();
  m_sendTime = i.ReadNtohU64();
  m_sourceNodeId = i.ReadNtohU32();
  m_keyId = i.ReadNtohU32();
  m_foundNodeId = i.ReadNtohU32();
  m_vanetSourceHostIp = i.ReadNtohU32();

  return GetSerializedSize ();
}

} // namespace ns3
