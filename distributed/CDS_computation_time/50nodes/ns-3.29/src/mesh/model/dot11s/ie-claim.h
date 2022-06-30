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

#ifndef WIFI_CLAIM_INFORMATION_ELEMENT_H
#define WIFI_CLAIM_INFORMATION_ELEMENT_H

#include <map>

#include "ns3/mac48-address.h"
#include "ns3/mesh-information-element-vector.h"
#include "ns3/vector.h"

namespace ns3 {
namespace dot11s {

class IeClaim : public WifiInformationElement
{
public:
	///Constructor
	IeClaim();
	///Destructor
	~IeClaim();
	void SetMarker (bool marker);
	void SetRole (uint8_t role);
	bool GetMarker () const;
	uint8_t GetRole () const;
	/**
	 * Stores the Node's Location
	 * \param location, a Vector3D
	 */
	void SetLocation(Vector location);
	/**
	 * Gets the location
	 * \returns the location
	 */
	Vector GetLocation() const;
	/**
   * \Set isClaimer flag
   */
  void SetIsClaimer ();
	/**
   * \Set doIMove flag
   */
  void SetDoIMove ();
  /*
	 * \returns true if node Claims to be the closest node
	 */
  bool IsClaimer () const;
	/*
	 * \returns true if node reports DoI Move
	 */
	bool IsDoIMove () const;
	/**
	 * Set originator address function
	 * \param addr the originator address
	 */
	void SetOriginAddress(Mac48Address addr);
	/**
	 * Get originator address function
	 * \returns the MAC address of the originator
	 */
	Mac48Address GetOriginAddress() const;
	/**
	 * Set originator sequence number
	 * \param originator_seq_number
	 */
	void SetOriginatorSeqNumber (uint32_t originator_seq_number);
	/**
	 * Get originator sequence numnber value
	 * \returns the originator sequence number
	 */
	uint32_t GetOriginatorSeqNumber () const;

	// Inherited from WifiInformationElement
	virtual WifiInformationElementId ElementId() const;
	virtual void SerializeInformationField(Buffer::Iterator i) const;
	virtual uint8_t DeserializeInformationField(Buffer::Iterator start, uint8_t length);
	virtual uint8_t GetInformationFieldSize() const;
	virtual void Print(std::ostream& os) const;

private:
	uint8_t       m_flags;         //< Claim packet flags (0: isClaimer, 1: doIMove)
	Mac48Address  m_originAddr;    //< Originator MAC Address
	uint32_t      m_originatorSeqNumber; //< originator sequence number
  Vector        m_nodeLocation;  //< Location of the node in tenths of meters, all components are assumed to be positive, max component value is 6.55 Km
	bool 					m_marker;
	uint8_t				m_role;
 /**
	* equality operator
	*
	* \param a lhs
	* \param b rhs
	* \returns true if equal
	*/
	friend bool operator== (const IeClaim & a, const IeClaim & b);
};
bool operator== (const IeClaim & a, const IeClaim & b);
std::ostream &operator << (std::ostream &os, const IeClaim &a);

} // namespace dot11s
} // namespace ns3
#endif
