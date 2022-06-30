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
* Based on original mesh dot11s model by:
* Kirill Andreev <andreev@iitp.ru>
*
* Author: Oscar Bautista <obaut004@fiu.edu>, <ogbautista@gmail.com> (2019)
* As part of the implementation of (   ) in dot11s. March, 2019
*/

#include "ie-claim.h"
#include "ns3/address-utils.h"
#include "ns3/assert.h"
#include "ns3/packet.h"

namespace ns3 {
namespace dot11s {
/*******************************
* IeClaim
*******************************/
IeClaim::~IeClaim() {}

IeClaim::IeClaim(): m_flags (0) {}

WifiInformationElementId
IeClaim::ElementId() const
{
	return IE_CLAIM;
}

void
IeClaim::SetMarker (bool marker)
{
		m_marker = marker;
}

void
IeClaim::SetRole (uint8_t role)
{
		m_role = role;
}

bool
IeClaim::GetMarker () const
{
	return m_marker;
}

uint8_t
IeClaim::GetRole () const
{
	return m_role;
}

void
IeClaim::SetLocation (Vector location)
{
 m_nodeLocation = location;
}

Vector
IeClaim::GetLocation() const
{
 return m_nodeLocation;
}

void
IeClaim::SetIsClaimer ()
{
	  m_flags |= 1;
}

void
IeClaim::SetDoIMove ()
{
	  m_flags |= 1 << 1;
}

bool
IeClaim::IsClaimer () const
{
	return (m_flags & (1));
}

bool
IeClaim::IsDoIMove () const
{
	return (m_flags & (1 << 1));
}

void
IeClaim::SetOriginAddress (Mac48Address addr)
{
	m_originAddr = addr;
}

Mac48Address
IeClaim::GetOriginAddress() const
{
	return m_originAddr;
}

void
IeClaim::SetOriginatorSeqNumber (uint32_t originator_seq_number)
{
  m_originatorSeqNumber = originator_seq_number;
}

uint32_t
IeClaim::GetOriginatorSeqNumber () const
{
  return m_originatorSeqNumber;
}

void
IeClaim::SerializeInformationField(Buffer::Iterator i) const
{
	i.WriteU8(m_flags);
  WriteTo(i, m_originAddr);
	i.WriteHtolsbU32 (m_originatorSeqNumber);
	i.WriteHtolsbU16 ((uint16_t) (m_nodeLocation.x*10));
	i.WriteHtolsbU16 ((uint16_t) (m_nodeLocation.y*10));
	i.WriteHtolsbU16 ((uint16_t) (m_nodeLocation.z*10));
	i.WriteU8(m_marker);
	i.WriteU8(m_role);
}

uint8_t
IeClaim::DeserializeInformationField(Buffer::Iterator start, uint8_t length)
{
	Buffer::Iterator i = start;
	m_flags = i.ReadU8 ();
	ReadFrom(i, m_originAddr);
	m_originatorSeqNumber = i.ReadLsbtohU32 ();
	m_nodeLocation.x = (float) i.ReadLsbtohU16()/10;
	m_nodeLocation.y = (float) i.ReadLsbtohU16()/10;
	m_nodeLocation.z = (float) i.ReadLsbtohU16()/10;
	m_marker = i.ReadU8 ();
	m_role = i.ReadU8 ();
	uint8_t dist = i.GetDistanceFrom(start);
	NS_ASSERT(dist == GetInformationFieldSize());
	return dist;
}

uint8_t
IeClaim::GetInformationFieldSize() const
{
	uint8_t retval = 1 //flags
		+ 6   // Source address (originator)
		+ 4   // Originator Sequence Number
		+ 2   // X position (converted to Uint16)
		+ 2   // Y position (converted to Uint16)
		+ 2   // Z position (converted to Uint16)
		+ 2;  // marker and role
	return retval;
}

void
IeClaim::Print(std::ostream &os) const
{
	os << "Claim Packet=(Originator MAC address: " << m_originAddr;
	os << "Is Claimer: " << (bool) (m_flags | 0x01);
	os << "DoI Move: " << (bool) (m_flags | 0x02);
	os << "Location: " << m_nodeLocation << ")";
}

bool
operator== (const IeClaim & a, const IeClaim & b)
{
	if (a.m_originAddr != b.m_originAddr || a.m_flags != b.m_flags || a.m_nodeLocation.x != b.m_nodeLocation.x
		|| a.m_nodeLocation.y != b.m_nodeLocation.y || a.m_nodeLocation.z != b.m_nodeLocation.z
		|| a.m_marker != b.m_marker || a.m_role != b.m_role)
	{
		return false;
	}
	return true;
}

std::ostream &
operator << (std::ostream &os, const IeClaim &a)
{
	a.Print(os);
	return os;
}

} // namespace dot11s
} // namespace ns3
