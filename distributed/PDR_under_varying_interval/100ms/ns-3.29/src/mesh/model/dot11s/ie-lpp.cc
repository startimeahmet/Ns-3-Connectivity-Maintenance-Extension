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
* As part of the implementation of ETX Metric in dot11s. March, 2019
*/

#include "ie-lpp.h"
#include "ns3/address-utils.h"
#include "ns3/assert.h"
#include "ns3/packet.h"

namespace ns3 {
namespace dot11s {
/*******************************
* IeLpp
*******************************/
IeLpp::~IeLpp() {}

IeLpp::IeLpp() : m_nodeID (0), m_marker (false), m_role (0)
//m_positionx (0), m_positiony (0), m_positionz (0), m_neighborpositionx (), m_neighborpositiony (), m_neighborpositionz ()
{
}

WifiInformationElementId
IeLpp::ElementId() const
{
	return IE_LPP;
}

void
IeLpp::SerializeInformationField(Buffer::Iterator i) const
{
	i.WriteU8(m_lppId);
	WriteTo(i, m_originAddr);
	i.WriteHtolsbU32(m_originSeqno);
	i.WriteU8(GetNumberNeighbors());
	std::map<Mac48Address, uint8_t>::const_iterator j;
	for (j = m_neighborsLppCnt.begin(); j != m_neighborsLppCnt.end(); ++j)
	{
		WriteTo(i, (*j).first);
		i.WriteU8((*j).second);
	}
	i.WriteU8 (m_nodeID);
  i.WriteU8 (m_marker);
  i.WriteU8 (m_role);
  // i.WriteHtolsbU32 (m_positionx);
  // i.WriteHtolsbU64 (m_positiony);
  // i.WriteHtolsbU64 (m_positionz);
	// map<uint8_t, double>::const_iterator k;
	// i.WriteU8 ((uint8_t) m_neighborpositionx.size());
	// for (pair<uint8_t, double> k : m_neighborpositionx)
	// {
	// 	i.WriteU8(k.first);
	// 	i.WriteHtonU64(k.second);
	// }
	// for (pair<uint8_t, double> k : m_neighborpositiony)
	// {
	// 	i.WriteU8(k.first);
	// 	i.WriteHtonU64(k.second);
	// }
	// for (pair<uint8_t, double> k : m_neighborpositionz)
	// {
	// 	i.WriteU8(k.first);
	// 	i.WriteHtonU64(k.second);
	// }
	// for (auto const &k : m_neighborpositionx)
	// {
	// 	i.WriteU8(k.first);
	// 	i.WriteHtonU64(k.second);
	// }
	// for (auto const &k : m_neighborpositiony)
	// {
	// 	i.WriteU8(k.first);
	// 	i.WriteHtonU64(k.second);
	// }
	// for (auto const &k : m_neighborpositionz)
	// {
	// 	i.WriteU8(k.first);
	// 	i.WriteHtonU64(k.second);
	// }
	// for (k = m_neighborpositionx.begin(); k != m_neighborpositionx.end(); k++)
	// {
	// 	i.WriteU8((*k).first);
	// 	i.WriteHtolsbU64((*k).second);
	// }
	// for (k = m_neighborpositiony.begin(); k != m_neighborpositiony.end(); k++)
	// {
	// 	i.WriteU8((*k).first);
	// 	i.WriteHtolsbU64((*k).second);
	// }
	// for (k = m_neighborpositionz.begin(); k != m_neighborpositionz.end(); k++)
	// {
	// 	i.WriteU8((*k).first);
	// 	i.WriteHtolsbU64((*k).second);
	// }
}

uint8_t
IeLpp::DeserializeInformationField(Buffer::Iterator start, uint8_t length)
{
	Buffer::Iterator i = start;
	m_lppId = i.ReadU8();
	ReadFrom(i, m_originAddr);
	m_originSeqno = i.ReadLsbtohU32();
	uint8_t numberNeighbors = i.ReadU8();
	m_neighborsLppCnt.clear();
	//m_neighborpositionx.clear();
	//m_neighborpositiony.clear();
	//m_neighborpositionz.clear();
	Mac48Address neighborAddr;
	uint8_t lppCnt;
	for (uint8_t k = 0; k < numberNeighbors; ++k)
	{
		ReadFrom(i, neighborAddr);
		lppCnt = i.ReadU8();
		m_neighborsLppCnt.insert(std::make_pair(neighborAddr, lppCnt));
	}
	m_nodeID = i.ReadU8 ();
	m_marker = i.ReadU8 ();
	m_role = i.ReadU8 ();
	// m_positionx = i.ReadLsbtohU32  ();
	// m_positiony = i.ReadLsbtohU64  ();
	// m_positionz = i.ReadLsbtohU64  ();
	// uint8_t numberNeighborsPosition = i.ReadU8();
	// for (uint8_t k = 0; k < numberNeighborsPosition; k++)
	// {
	// 	uint8_t dump1 = i.ReadU8();
	// 	double dump2 = i.ReadLsbtohU64 ();
	// 	m_neighborpositionx.insert(make_pair(dump1, dump2));
	// }
	// for (uint8_t k = 0; k < numberNeighborsPosition; k++)
	// {
	// 	uint8_t dump1 = i.ReadU8();
	// 	double dump2 = i.ReadLsbtohU64 ();
	// 	m_neighborpositiony.insert(make_pair(dump1, dump2));
	// }
	// for (uint8_t k = 0; k < numberNeighborsPosition; k++)
	// {
	// 	uint8_t dump1 = i.ReadU8();
	// 	double dump2 = i.ReadLsbtohU64 ();
	// 	m_neighborpositionz.insert(make_pair(dump1, dump2));
	// }

	uint8_t dist = i.GetDistanceFrom(start);
	NS_ASSERT(dist == GetInformationFieldSize());
	return dist;
}

uint8_t
IeLpp::GetInformationFieldSize() const
{
	uint8_t retval = 1 //LPP ID
		+ 6   //Source address (originator)
		+ 4   //Originator seqno
		+ 1   //Number of Neighbors
		+ 1   //nodeID
    + 1   //marker
    + 1;  //role
    // + 4   //positionx
    // + 8   //positiony
    // + 8   //positionz
		// + 1;  //numberofneighbors
	//return ( retval + 7 * GetNumberNeighbors() + 23 * m_neighborpositionx.size() );
	return ( retval + 7 * GetNumberNeighbors() );
}

void
IeLpp::Print(std::ostream &os) const
{
	os << "LPP=(Lpp ID: " << m_lppId << ", Originator MAC address: " << m_originAddr;
	os << "Originator Sequence number: " << m_originSeqno;
	os << "Number of neighbors: " << (*this).GetNumberNeighbors();
	os << "Neighbors (Mac address, received LPP count): ";
	std::map<Mac48Address, uint8_t>::const_iterator j;
	for (j = m_neighborsLppCnt.begin(); j != m_neighborsLppCnt.end(); ++j)
	{
		os << (*j).first << ", " << (*j).second;
	}
	os << ")";
}

bool
IeLpp::AddToNeighborsList(Mac48Address neighbor, uint8_t lppCnt)
{
	if (m_neighborsLppCnt.find(neighbor) != m_neighborsLppCnt.end())
	{
		return true;
	}

	NS_ASSERT(GetNumberNeighbors() < 255); // can't support more than 2^8 - 1 neighbors
	m_neighborsLppCnt.insert(std::make_pair(neighbor, lppCnt));
	return true;
}

bool
IeLpp::RemoveFromNeighborsList(std::pair<Mac48Address, uint8_t> & un)
{
	if (m_neighborsLppCnt.empty())
	{
		return false;
	}

	std::map<Mac48Address, uint8_t>::iterator i = m_neighborsLppCnt.begin();
	un = *i;
	m_neighborsLppCnt.erase(i);
	return true;
}

void
IeLpp::ClearNeighborsList()
{
	m_neighborsLppCnt.clear();
}

bool
operator== (const IeLpp & a, const IeLpp & b)
{
	if (a.m_lppId != b.m_lppId || a.m_originAddr != b.m_originAddr || a.m_originSeqno != b.m_originSeqno
		|| a.GetNumberNeighbors() != b.GetNumberNeighbors())
	{
		return false;
	}
	std::map<Mac48Address, uint8_t>::const_iterator j = a.m_neighborsLppCnt.begin();
	std::map<Mac48Address, uint8_t>::const_iterator k = b.m_neighborsLppCnt.begin();
	for (uint8_t i = 0; i < a.GetNumberNeighbors(); ++i)
	{
		if ((j->first != k->first) || (j->second != k->second))
		{
			return false;
		}
		j++;
		k++;
	}
	return true;
}

std::ostream &
operator << (std::ostream &os, const IeLpp &a)
{
	a.Print(os);
	return os;
}

} // namespace dot11s
} // namespace ns3
