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

#ifndef WIFI_LPP_INFORMATION_ELEMENT_H
#define WIFI_LPP_INFORMATION_ELEMENT_H

#include <map>

#include "ns3/mac48-address.h"
#include "ns3/mesh-information-element-vector.h"

using namespace std;
namespace ns3 {
namespace dot11s {

class IeLpp : public WifiInformationElement
{
public:
	///Constructor
	IeLpp();
	///Destructor
	~IeLpp();

	void SetLppId(uint8_t count)
	{
		m_lppId = count;
	}

	uint8_t GetLppId() const
	{
		return m_lppId;
	}

	void SetOriginAddress(Mac48Address addr)
	{
		m_originAddr = addr;
	}

	Mac48Address GetOriginAddress() const
	{
		return m_originAddr;
	}

	void SetOriginSeqno(uint32_t seqno)
	{
		m_originSeqno = seqno;
	}

	uint32_t GetOriginSeqno() const
	{
		return m_originSeqno;
	}

	uint8_t GetNumberNeighbors() const
	{
		return (uint8_t)m_neighborsLppCnt.size();
	}

	std::map<Mac48Address, uint8_t> GetNeighborList ()
	{
		return m_neighborsLppCnt;
	}
	//for CDS
	void SetNodeID (uint8_t nodeID)
	{
	    m_nodeID = nodeID;
	}
	void SetMarker (bool marker)
	{
	    m_marker = marker;
	}
	void SetRole (uint8_t role)
	{
	    m_role = role;
	}
	// void SetPositionx (uint32_t positionx)
	// {
	//     m_positionx = positionx;
	// }
	// void SetPositiony (double positiony)
	// {
	//     m_positiony = positiony;
	// }
	// void SetPositionz (double positionz)
	// {
	//     m_positionz = positionz;
	// }
	// void SetNeighborPositionx (map<uint8_t, uint32_t> neighborpositionx)
	// {
	// 	m_neighborpositionx.clear();
	// 	m_neighborpositionx.insert(neighborpositionx.begin(), neighborpositionx.end());
	// 	//m_neighborpositionx = neighborpositionx;
	// }
	// void SetNeighborPositiony (map<uint8_t, double> neighborpositiony)
	// {
	// 	m_neighborpositiony.clear();
	// 	m_neighborpositiony.insert(neighborpositiony.begin(), neighborpositiony.end());
	// 	//m_neighborpositiony = neighborpositiony;
	// }
	// void SetNeighborPositionz (map<uint8_t, double> neighborpositionz)
	// {
	// 	m_neighborpositionz.clear();
	// 	m_neighborpositionz.insert(neighborpositionz.begin(), neighborpositionz.end());
	// 	//m_neighborpositionz = neighborpositionz;
	// }
	uint8_t GetNodeID () const
	{
	    return m_nodeID;
	}
	bool GetMarker () const
	{
	    return m_marker;
	}
	uint8_t GetRole () const
	{
	    return m_role;
	}
	// uint32_t GetPositionx () const
	// {
	//     return m_positionx;
	// }
	// double GetPositiony () const
	// {
	//     return m_positiony;
	// }
	// double GetPositionz () const
	// {
	//     return m_positionz;
	// }
	// map<uint8_t, double> GetNeighborPositionx()
	// {
	// 	return m_neighborpositionx;
	// }
	// map<uint8_t, double> GetNeighborPositiony() const
	// {
	// 	return m_neighborpositiony;
	// }
	// map<uint8_t, double> GetNeighborPositionz() const
	// {
	// 	return m_neighborpositionz;
	// }

	// Control neighbors list
	bool AddToNeighborsList(Mac48Address neighbor, uint8_t lppCnt);
	bool RemoveFromNeighborsList(std::pair<Mac48Address, uint8_t> & un);
	void ClearNeighborsList();

	// Inherited from WifiInformationElement
	virtual WifiInformationElementId ElementId() const;
	virtual void SerializeInformationField(Buffer::Iterator i) const;
	virtual uint8_t DeserializeInformationField(Buffer::Iterator start, uint8_t length);
	virtual uint8_t GetInformationFieldSize() const;
	virtual void Print(std::ostream& os) const;

private:
	uint8_t       m_lppId;         //< LPP ID which is set to the Lpp Time slot (runs 0 to 11 cyclically )
	Mac48Address  m_originAddr;    //< Originator MAC Address
	uint32_t      m_originSeqno;   //< Originator Sequence number
	//added for CDS
	uint8_t m_nodeID; ///< ID of the mesh node
  bool m_marker; ///< either true or false
  uint8_t m_role; ///< investigator etc.
  // uint32_t m_positionx; ///< x position of the node
  // double m_positiony; ///< y position of the node
  // double m_positionz; ///< z position of the node
	//
	// map<uint8_t, double> m_neighborpositionx;   ///< x position of the neighors of the node
	// map<uint8_t, double> m_neighborpositiony;   ///< y position of the neighors of the node
	// map<uint8_t, double> m_neighborpositionz;   ///< z position of the neighors of the node

	// List of neighbors: MAC addresses and number of LLP received from each in the last checked 10 time slot period
	std::map<Mac48Address, uint8_t> m_neighborsLppCnt;

	/**
	* equality operator
	*
	* \param a lhs
	* \param b rhs
	* \returns true if equal
	*/
	friend bool operator== (const IeLpp & a, const IeLpp & b);
};
bool operator== (const IeLpp & a, const IeLpp & b);
std::ostream &operator << (std::ostream &os, const IeLpp &lpp);

} // namespace dot11s
} // namespace ns3
#endif
