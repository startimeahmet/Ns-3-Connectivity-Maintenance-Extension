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
 * Authors: Kirill Andreev <andreev@iitp.ru>
 */

#include "hwmp-protocol.h"
#include "hwmp-protocol-mac.h"
#include "hwmp-tag.h"
#include "hwmp-rtable.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/mesh-point-device.h"
#include "ns3/wifi-net-device.h"
#include "ns3/mesh-wifi-interface-mac.h"
#include "ns3/random-variable-stream.h"
#include "airtime-metric.h"
#include "ie-dot11s-preq.h"
#include "ie-dot11s-prep.h"
#include "ns3/trace-source-accessor.h"
#include "ie-dot11s-perr.h"
#include "ie-lpp.h"
#include "ie-claim.h"
#include "ns3/vector.h"
#include "ns3/mobility-module.h"
#include "scratch/distributed/10nodes/nodeIDs.h"
#include "scratch/distributed/10nodes/nodeRoles.h"
#include "scratch/distributed/10nodes/finaltopology.h"

#define NIL -1  //for SCC calculations
#define range 100 //the wifi range of the nodes
using namespace std;
namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HwmpProtocol");

namespace dot11s {

NS_OBJECT_ENSURE_REGISTERED (HwmpProtocol);

TypeId
HwmpProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::HwmpProtocol")
    .SetParent<MeshL2RoutingProtocol> ()
    .SetGroupName ("Mesh")
    .AddConstructor<HwmpProtocol> ()
    .AddAttribute ( "RandomStart",
                    "Random delay at first proactive PREQ",
                    TimeValue (Seconds (0.1)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_randomStart),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "LppRandomStart",
                    "Random delay at first LPP",
                    TimeValue(Seconds(50)),
                    MakeTimeAccessor(
                      &HwmpProtocol::m_lppRandomStart),
                    MakeTimeChecker()
                    )
    .AddAttribute ( "MaxQueueSize",
                    "Maximum number of packets we can store when resolving route",
                    UintegerValue (255),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_maxQueueSize),
                    MakeUintegerChecker<uint16_t> (1)
                    )
    .AddAttribute ( "Dot11MeshHWMPmaxPREQretries",
                    "Maximum number of retries before we suppose the destination to be unreachable",
                    UintegerValue (3),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPmaxPREQretries),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "Dot11MeshHWMPnetDiameterTraversalTime",
                    "Time we suppose the packet to go from one edge of the network to another",
                    TimeValue (MicroSeconds (1024*100)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPnetDiameterTraversalTime),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPpreqMinInterval",
                    "Minimal interval between to successive PREQs",
                    TimeValue (MicroSeconds (1024*100)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPpreqMinInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPperrMinInterval",
                    "Minimal interval between to successive PREQs",
                    TimeValue (MicroSeconds (1024*100)),
                    MakeTimeAccessor (&HwmpProtocol::m_dot11MeshHWMPperrMinInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPlppMinInterval",
                    "Minimal interval between LPPs",
                    TimeValue(MicroSeconds(1024 * 100)),
                    MakeTimeAccessor(&HwmpProtocol::m_dot11MeshHWMPlppMinInterval),
                    MakeTimeChecker()
                    )
    .AddAttribute ( "Dot11MeshHWMPactiveRootTimeout",
                    "Lifetime of poractive routing information",
                    TimeValue (MicroSeconds (1024*5000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPactiveRootTimeout),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPactivePathTimeout",
                    "Lifetime of reactive routing information",
                    TimeValue (MicroSeconds (1024*5000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPactivePathTimeout),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPpathToRootInterval",
                    "Interval between two successive proactive PREQs",
                    TimeValue (MicroSeconds (1024*2000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPpathToRootInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "Dot11MeshHWMPrannInterval",
                    "Lifetime of poractive routing information",
                    TimeValue (MicroSeconds (1024*5000)),
                    MakeTimeAccessor (
                      &HwmpProtocol::m_dot11MeshHWMPrannInterval),
                    MakeTimeChecker ()
                    )
    .AddAttribute ( "MaxTtl",
                    "Initial value of Time To Live field",
                    UintegerValue (32),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_maxTtl),
                    MakeUintegerChecker<uint8_t> (2)
                    )
    .AddAttribute ( "UnicastPerrThreshold",
                    "Maximum number of PERR receivers, when we send a PERR as a chain of unicasts",
                    UintegerValue (32),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_unicastPerrThreshold),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "UnicastPreqThreshold",
                    "Maximum number of PREQ receivers, when we send a PREQ as a chain of unicasts",
                    UintegerValue (1),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_unicastPreqThreshold),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "UnicastDataThreshold",
                    "Maximum number ofbroadcast receivers, when we send a broadcast as a chain of unicasts",
                    UintegerValue (1),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_unicastDataThreshold),
                    MakeUintegerChecker<uint8_t> (1)
                    )
    .AddAttribute ( "DoFlag",
                    "Destination only HWMP flag",
                    BooleanValue (false),
                    MakeBooleanAccessor (
                      &HwmpProtocol::m_doFlag),
                    MakeBooleanChecker ()
                    )
    .AddAttribute ( "RfFlag",
                    "Reply and forward flag",
                    BooleanValue (true),
                    MakeBooleanAccessor (
                      &HwmpProtocol::m_rfFlag),
                    MakeBooleanChecker ()
                    )
    .AddAttribute ( "EtxMetric",
                    "Enable use of ETX Metric overriding AirTime Metric",
                    BooleanValue (false),
                    MakeBooleanAccessor (
                      &HwmpProtocol::m_etxMetric),
                    MakeBooleanChecker ()
                    )
    .AddAttribute ( "LinkProbePacket",
                    "Enable the Transmission of LPP needed to calculate the ETX Metric, "
                    "enabled automatically when EtxMetric is selected",
                    BooleanValue (false),
                    MakeBooleanAccessor (
                      &HwmpProtocol::m_enableLpp),
                    MakeBooleanChecker ()
                    )
    .AddAttribute ( "TopologyID",
                    "Sets which topology will be used",
                    UintegerValue (0),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_topoId),
                    MakeUintegerChecker<uint8_t> ()
                    )
    .AddAttribute ( "movingNode",
                    "Sets which node will move",
                    UintegerValue (0),
                    MakeUintegerAccessor (
                      &HwmpProtocol::m_movingNode),
                    MakeUintegerChecker<uint64_t> ()
                    )
    .AddTraceSource ( "RouteDiscoveryTime",
                      "The time of route discovery procedure",
                      MakeTraceSourceAccessor (
                        &HwmpProtocol::m_routeDiscoveryTimeCallback),
                      "ns3::Time::TracedCallback"
                      )
    .AddTraceSource ("RouteChange",
                     "Routing table changed",
                     MakeTraceSourceAccessor (&HwmpProtocol::m_routeChangeTraceSource),
                     "ns3::HwmpProtocol::RouteChangeTracedCallback"
                     )
  ;
  return tid;
}

HwmpProtocol::HwmpProtocol () :
  m_dataSeqno (1),
  m_hwmpSeqno (1),
  m_preqId (0),
  m_rtable (CreateObject<HwmpRtable> ()),
  m_randomStart (Seconds (0.1)),
  m_lppRandomStart (Seconds (50)),
  m_maxQueueSize (255),
  m_dot11MeshHWMPmaxPREQretries (3),
  m_dot11MeshHWMPnetDiameterTraversalTime (MicroSeconds (1024*100)),
  m_dot11MeshHWMPpreqMinInterval (MicroSeconds (1024*100)),
  m_dot11MeshHWMPperrMinInterval (MicroSeconds (1024*100)),
  m_dot11MeshHWMPlppMinInterval(MicroSeconds(1024*100)),
  m_dot11MeshHWMPactiveRootTimeout (MicroSeconds (1024*5000)),
  m_dot11MeshHWMPactivePathTimeout (MicroSeconds (1024*5000)),
  m_dot11MeshHWMPpathToRootInterval (MicroSeconds (1024*2000)),
  m_dot11MeshHWMPrannInterval (MicroSeconds (1024*5000)),
  m_isRoot (false),
  m_maxTtl (32),
  m_unicastPerrThreshold (32),
  m_unicastPreqThreshold (1),
  m_unicastDataThreshold (1),
  m_doFlag (false),
  m_rfFlag (false),
  m_etxMetric (false),
  m_enableLpp (true),
  // m_topoId (32),
  m_isClaimer (false),
  m_doIMove (false),
  m_claimPktRetry (1)
{
  NS_LOG_FUNCTION (this);
  m_coefficient = CreateObject<UniformRandomVariable> ();
}

HwmpProtocol::~HwmpProtocol ()
{
  NS_LOG_FUNCTION (this);
}

void
HwmpProtocol::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  if (m_etxMetric)
    m_enableLpp = true;
  if(m_enableLpp)
  {
    // m_coefficient->SetAttribute ("Max", DoubleValue(m_lppRandomStart.GetSeconds()));
    // Time lppRandomStart = Seconds(m_coefficient->GetValue());
    // m_lppTimer = Simulator::Schedule(lppRandomStart, &HwmpProtocol::SendLpp, this);

    m_coefficient->SetAttribute ("Max", DoubleValue(m_lppRandomStart.GetSeconds()));
    m_coefficient->SetAttribute ("Min", DoubleValue(m_lppRandomStart.GetSeconds()-0.1));
    Time lppRandomStart = Seconds(m_coefficient->GetValue());
    m_lppTimer = Simulator::Schedule(lppRandomStart, &HwmpProtocol::SendLpp, this);
  }
  m_coefficient->SetAttribute ("Max", DoubleValue (m_randomStart.GetSeconds ()));
  if (m_isRoot)
    {
      Time randomStart = Seconds (m_coefficient->GetValue ());
      m_proactivePreqTimer = Simulator::Schedule (randomStart, &HwmpProtocol::SendProactivePreq, this);
    }

  uint8_t current = convertMac48AddresstoInt(GetAddress());
  myNodeID = nodeIDs[0][current-1];
  myRole = nodeRoles[0][current-1];
}

void
HwmpProtocol::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  for (std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.begin (); i != m_preqTimeouts.end (); i++)
    {
      i->second.preqTimeout.Cancel ();
    }
  m_proactivePreqTimer.Cancel ();
  if (m_enableLpp) m_lppTimer.Cancel();
  m_preqTimeouts.clear ();
  m_lastDataSeqno.clear ();
  m_hwmpSeqnoMetricDatabase.clear ();
  m_hwmpClaimSeqnoDatabase.clear ();
  m_claimersList.clear ();
  m_interfaces.clear ();
  m_rqueue.clear ();
  m_rtable = 0;
  m_mp = 0;
}

bool
HwmpProtocol::RequestRoute (
  uint32_t sourceIface,
  const Mac48Address source,
  const Mac48Address destination,
  Ptr<const Packet> constPacket,
  uint16_t protocolType, //ethrnet 'Protocol' field
  MeshL2RoutingProtocol::RouteReplyCallback routeReply
  )
{
  NS_LOG_FUNCTION (this << sourceIface << source << destination << constPacket << protocolType);
  Ptr <Packet> packet = constPacket->Copy ();
  HwmpTag tag;
  if (sourceIface == GetMeshPoint ()->GetIfIndex ())
    {
      // packet from level 3
      if (packet->PeekPacketTag (tag))
        {
          NS_FATAL_ERROR ("HWMP tag has come with a packet from upper layer. This must not occur...");
        }
      //Filling TAG:
      if (destination == Mac48Address::GetBroadcast ())
        {
          tag.SetSeqno (m_dataSeqno++);
        }
      tag.SetTtl (m_maxTtl);
    }
  else
    {
      if (!packet->RemovePacketTag (tag))
        {
          NS_FATAL_ERROR ("HWMP tag is supposed to be here at this point.");
        }
      tag.DecrementTtl ();
      if (tag.GetTtl () == 0)
        {
          m_stats.droppedTtl++;
          return false;
        }
    }
  if (destination == Mac48Address::GetBroadcast ())
    {
      m_stats.txBroadcast++;
      m_stats.txBytes += packet->GetSize ();
      //channel IDs where we have already sent broadcast:
      std::vector<uint16_t> channels;
      for (HwmpProtocolMacMap::const_iterator plugin = m_interfaces.begin (); plugin != m_interfaces.end (); plugin++)
        {
          bool shouldSend = true;
          for (std::vector<uint16_t>::const_iterator chan = channels.begin (); chan != channels.end (); chan++)
            {
              if ((*chan) == plugin->second->GetChannelId ())
                {
                  shouldSend = false;
                }
            }
          if (!shouldSend)
            {
              continue;
            }
          channels.push_back (plugin->second->GetChannelId ());
          std::vector<Mac48Address> receivers = GetBroadcastReceivers (plugin->first);
          for (std::vector<Mac48Address>::const_iterator i = receivers.begin (); i != receivers.end (); i++)
            {
              Ptr<Packet> packetCopy = packet->Copy ();
              //
              // 64-bit Intel valgrind complains about tag.SetAddress (*i).  It
              // likes this just fine.
              //
              Mac48Address address = *i;
              tag.SetAddress (address);
              packetCopy->AddPacketTag (tag);
              routeReply (true, packetCopy, source, destination, protocolType, plugin->first);
            }
        }
    }
  else
    {
      return ForwardUnicast (sourceIface, source, destination, packet, protocolType, routeReply, tag.GetTtl ());
    }
  return true;
}
bool
HwmpProtocol::RemoveRoutingStuff (uint32_t fromIface, const Mac48Address source,
                                  const Mac48Address destination, Ptr<Packet>  packet, uint16_t&  protocolType)
{
  HwmpTag tag;
  if (!packet->RemovePacketTag (tag))
    {
      NS_FATAL_ERROR ("HWMP tag must exist when packet received from the network");
    }
  return true;
}
bool
HwmpProtocol::ForwardUnicast (uint32_t  sourceIface, const Mac48Address source, const Mac48Address destination,
                              Ptr<Packet>  packet, uint16_t  protocolType, RouteReplyCallback  routeReply, uint32_t ttl)
{
  NS_LOG_FUNCTION (this << sourceIface << source << destination << packet << protocolType << ttl);
  NS_ASSERT (destination != Mac48Address::GetBroadcast ());
  HwmpRtable::LookupResult result = m_rtable->LookupReactive (destination);
  NS_LOG_DEBUG ("Requested src = "<<source<<", dst = "<<destination<<", I am "<<GetAddress ()<<", RA = "<<result.retransmitter);
  if (result.retransmitter == Mac48Address::GetBroadcast ())
    {
      result = m_rtable->LookupProactive ();
    }
  HwmpTag tag;
  tag.SetAddress (result.retransmitter);
  tag.SetTtl (ttl);
  //seqno and metric is not used;
  packet->AddPacketTag (tag);
  if (result.retransmitter != Mac48Address::GetBroadcast ())
    {
      //reply immediately:
      routeReply (true, packet, source, destination, protocolType, result.ifIndex);
      m_stats.txUnicast++;
      m_stats.txBytes += packet->GetSize ();
      return true;
    }
  if (sourceIface != GetMeshPoint ()->GetIfIndex ())
    {
      //Start path error procedure:
      NS_LOG_DEBUG ("Must Send PERR");
      result = m_rtable->LookupReactiveExpired (destination);
      NS_LOG_DEBUG ("Path error " << result.retransmitter);
      //1.  Lookup expired reactive path. If exists - start path error
      //    procedure towards a next hop of this path
      //2.  If there was no reactive path, we lookup expired proactive
      //    path. If exist - start path error procedure towards path to
      //    root
      if (result.retransmitter == Mac48Address::GetBroadcast ())
        {
          NS_LOG_DEBUG ("Path error, lookup expired proactive path");
          result = m_rtable->LookupProactiveExpired ();
        }
      if (result.retransmitter != Mac48Address::GetBroadcast ())
        {
          NS_LOG_DEBUG ("Path error, initiate reactive path error");
          std::vector<FailedDestination> destinations = m_rtable->GetUnreachableDestinations (result.retransmitter);
          InitiatePathError (MakePathError (destinations));
        }
      m_stats.totalDropped++;
      return false;
    }
  //Request a destination:
  result = m_rtable->LookupReactiveExpired (destination);
  if (ShouldSendPreq (destination))
    {
      uint32_t originator_seqno = GetNextHwmpSeqno ();
      uint32_t dst_seqno = 0;
      if (result.retransmitter != Mac48Address::GetBroadcast ())
        {
          dst_seqno = result.seqnum;
        }
      m_stats.initiatedPreq++;
      for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
        {
          i->second->RequestDestination (destination, originator_seqno, dst_seqno);
        }
    }
  QueuedPacket pkt;
  pkt.pkt = packet;
  pkt.dst = destination;
  pkt.src = source;
  pkt.protocol = protocolType;
  pkt.reply = routeReply;
  pkt.inInterface = sourceIface;
  if (QueuePacket (pkt))
    {
      m_stats.totalQueued++;
      return true;
    }
  else
    {
      m_stats.totalDropped++;
      NS_LOG_DEBUG ("Dropping packet from " << source << " to " << destination << " due to queue overflow");
      return false;
    }
}
void
HwmpProtocol::ReceivePreq (IePreq preq, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric)
{
  if(m_etxMetric/* && Simulator::Now().GetSeconds() >= 50*/)
  {
    metric = m_nbEtx.GetEtxForNeighbor(from);	// This line substitutes the airtime link metric by the etx metric
  }

  NS_LOG_FUNCTION (this << from << interface << fromMp << metric);
  preq.IncrementMetric (metric);
  //acceptance cretirea:
  std::map<Mac48Address, std::pair<uint32_t, uint32_t> >::const_iterator i = m_hwmpSeqnoMetricDatabase.find (
      preq.GetOriginatorAddress ());
  bool freshInfo (true);
  if (i != m_hwmpSeqnoMetricDatabase.end ())
    {
      if ((int32_t)(i->second.first - preq.GetOriginatorSeqNumber ())  > 0)
        {
          return;
        }
      if (i->second.first == preq.GetOriginatorSeqNumber ())
        {
          freshInfo = false;
          if (i->second.second <= preq.GetMetric ())
            {
              return;
            }
        }
    }
  m_hwmpSeqnoMetricDatabase[preq.GetOriginatorAddress ()] =
    std::make_pair (preq.GetOriginatorSeqNumber (), preq.GetMetric ());
  NS_LOG_DEBUG ("I am " << GetAddress () << ", Accepted preq from address" << from << ", preq:" << preq);
  std::vector<Ptr<DestinationAddressUnit> > destinations = preq.GetDestinationList ();
  //Add reactive path to originator:
  if (
    (freshInfo) ||
    (
      (m_rtable->LookupReactive (preq.GetOriginatorAddress ()).retransmitter == Mac48Address::GetBroadcast ()) ||
      (m_rtable->LookupReactive (preq.GetOriginatorAddress ()).metric > preq.GetMetric ())
    )
    )
    {
      m_rtable->AddReactivePath (
        preq.GetOriginatorAddress (),
        from,
        interface,
        preq.GetMetric (),
        MicroSeconds (preq.GetLifetime () * 1024),
        preq.GetOriginatorSeqNumber ()
        );
      // Notify trace source of routing change
      struct RouteChange rChange;
      rChange.type = "Add Reactive";
      rChange.destination = preq.GetOriginatorAddress ();
      rChange.retransmitter = from;
      rChange.interface = interface;
      rChange.metric = preq.GetMetric ();
      rChange.lifetime = MicroSeconds (preq.GetLifetime () * 1024);
      rChange.seqnum = preq.GetOriginatorSeqNumber ();
      m_routeChangeTraceSource (rChange);
      ReactivePathResolved (preq.GetOriginatorAddress ());
    }
  if (
    (m_rtable->LookupReactive (fromMp).retransmitter == Mac48Address::GetBroadcast ()) ||
    (m_rtable->LookupReactive (fromMp).metric > metric)
    )
    {
      m_rtable->AddReactivePath (
        fromMp,
        from,
        interface,
        metric,
        MicroSeconds (preq.GetLifetime () * 1024),
        preq.GetOriginatorSeqNumber ()
        );
      // Notify trace source of routing change
      struct RouteChange rChange;
      rChange.type = "Add Reactive";
      rChange.destination = fromMp;
      rChange.retransmitter = from;
      rChange.interface = interface;
      rChange.metric = metric;
      rChange.lifetime = MicroSeconds (preq.GetLifetime () * 1024);
      rChange.seqnum = preq.GetOriginatorSeqNumber ();
      m_routeChangeTraceSource (rChange);
      ReactivePathResolved (fromMp);
    }
  for (std::vector<Ptr<DestinationAddressUnit> >::const_iterator i = destinations.begin (); i != destinations.end (); i++)
    {
      if ((*i)->GetDestinationAddress () == Mac48Address::GetBroadcast ())
        {
          //only proactive PREQ contains destination
          //address as broadcast! Proactive preq MUST
          //have destination count equal to 1 and
          //per destination flags DO and RF
          NS_ASSERT (preq.GetDestCount () == 1);
          NS_ASSERT (((*i)->IsDo ()) && ((*i)->IsRf ()));
          //Add proactive path only if it is the better then existed
          //before
          if (
            ((m_rtable->LookupProactive ()).retransmitter == Mac48Address::GetBroadcast ()) ||
            ((m_rtable->LookupProactive ()).metric > preq.GetMetric ())
            )
            {
              m_rtable->AddProactivePath (
                preq.GetMetric (),
                preq.GetOriginatorAddress (),
                from,
                interface,
                MicroSeconds (preq.GetLifetime () * 1024),
                preq.GetOriginatorSeqNumber ()
                );
              // Notify trace source of routing change
              struct RouteChange rChange;
              rChange.type = "Add Proactive";
              rChange.destination = preq.GetOriginatorAddress ();
              rChange.retransmitter = from;
              rChange.interface = interface;
              rChange.metric = preq.GetMetric ();
              rChange.lifetime = MicroSeconds (preq.GetLifetime () * 1024);
              rChange.seqnum = preq.GetOriginatorSeqNumber ();
              m_routeChangeTraceSource (rChange);
              ProactivePathResolved ();
            }
          if (!preq.IsNeedNotPrep ())
            {
              SendPrep (
                GetAddress (),
                preq.GetOriginatorAddress (),
                from,
                (uint32_t)0,
                preq.GetOriginatorSeqNumber (),
                GetNextHwmpSeqno (),
                preq.GetLifetime (),
                interface
                );
            }
          break;
        }
      if ((*i)->GetDestinationAddress () == GetAddress ())
        {
          SendPrep (
            GetAddress (),
            preq.GetOriginatorAddress (),
            from,
            (uint32_t)0,
            preq.GetOriginatorSeqNumber (),
            GetNextHwmpSeqno (),
            preq.GetLifetime (),
            interface
            );
          NS_ASSERT (m_rtable->LookupReactive (preq.GetOriginatorAddress ()).retransmitter != Mac48Address::GetBroadcast ());
          preq.DelDestinationAddressElement ((*i)->GetDestinationAddress ());
          continue;
        }
      //check if can answer:
      HwmpRtable::LookupResult result = m_rtable->LookupReactive ((*i)->GetDestinationAddress ());
      if ((!((*i)->IsDo ())) && (result.retransmitter != Mac48Address::GetBroadcast ()))
        {
          //have a valid information and can answer
          uint32_t lifetime = result.lifetime.GetMicroSeconds () / 1024;
          if ((lifetime > 0) && ((int32_t)(result.seqnum - (*i)->GetDestSeqNumber ()) >= 0))
            {
              SendPrep (
                (*i)->GetDestinationAddress (),
                preq.GetOriginatorAddress (),
                from,
                result.metric,
                preq.GetOriginatorSeqNumber (),
                result.seqnum,
                lifetime,
                interface
                );
              m_rtable->AddPrecursor ((*i)->GetDestinationAddress (), interface, from,
                                      MicroSeconds (preq.GetLifetime () * 1024));
              if ((*i)->IsRf ())
                {
                  (*i)->SetFlags (true, false, (*i)->IsUsn ()); //DO = 1, RF = 0
                }
              else
                {
                  preq.DelDestinationAddressElement ((*i)->GetDestinationAddress ());
                  continue;
                }
            }
        }
    }
  //check if must retransmit:
  if (preq.GetDestCount () == 0)
    {
      return;
    }
  //Forward PREQ to all interfaces:
  NS_LOG_DEBUG ("I am " << GetAddress () << "retransmitting PREQ:" << preq);
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->SendPreq (preq);
    }
}
// vector<pair<double,int>>
// HwmpProtocol::sortTheNodesbyProximity()
// {
//   vector<pair<double,int>> sortedNodes;
//   int n = nodepositions[m_topoId].size();
//
//   for (int i=1; i<n+1; i++) {
//     double distance = calculateDistance(i);
//     sortedNodes.push_back (make_pair (distance, i));
//   }
//
//   sort (sortedNodes.begin(), sortedNodes.end());
//
//   // for (int i=0; i<n; i++) {
//   //   cout << sortedNodes[i].first << " " << +sortedNodes[i].second << "\n";
//   // }
//   return sortedNodes;
// }
vector<pair<int, double>>
HwmpProtocol::sortAllNodesByProximity()
{
  map<int, double> nodeDistances;
  // create a empty vector of pairs
  vector<myPair> sorted;
  for (int i=1; i<11; i++)
  {
    double distance = pow((nodepositions[m_topoId][i-1][0]-newLocation[0]),2)+
                      pow((nodepositions[m_topoId][i-1][1]-newLocation[1]),2)+
                      pow((nodepositions[m_topoId][i-1][2]-newLocation[2]),2);
    distance = sqrt(distance);
    nodeDistances.insert(pair<int, double>(i, distance));
  }

  // copy key-value pairs from the map to the vector
  copy(nodeDistances.begin(),
       nodeDistances.end(),
       back_inserter<vector<myPair>>(sorted));

  // sort the vector by increasing order of its pair's second value
  // if second value are equal, order by the pair's first value
  sort(sorted.begin(), sorted.end(),
      [](const myPair& l, const myPair& r) {
        if (l.second != r.second)
          return l.second < r.second;

        return l.first < r.first;
      });

  return sorted;
}
double
HwmpProtocol::calculateDistance(uint8_t i)
{
  double distance = pow( (newLocation[0]-nodepositions[m_topoId][i-1][0]), 2 ) + pow( (newLocation[1]-nodepositions[m_topoId][i-1][1]), 2 ) + pow( (newLocation[2]-nodepositions[m_topoId][i-1][2]), 2 );
  distance = sqrt(distance);
  return distance;
}
void
HwmpProtocol::ReceivePrep (IePrep prep, Mac48Address from, uint32_t interface, Mac48Address fromMp, uint32_t metric)
{
  if (m_etxMetric/* && Simulator::Now().GetSeconds() >= 50*/)
  {
    metric = m_nbEtx.GetEtxForNeighbor(from);	// This line substitutes the airtime link metric by the etx metric
  }

  NS_LOG_FUNCTION (this << from << interface << fromMp << metric);
  prep.IncrementMetric (metric);
  //acceptance cretirea:
  std::map<Mac48Address, std::pair<uint32_t, uint32_t> >::const_iterator i = m_hwmpSeqnoMetricDatabase.find (
      prep.GetOriginatorAddress ());
  bool freshInfo (true);
  uint32_t sequence = prep.GetDestinationSeqNumber ();
  if (i != m_hwmpSeqnoMetricDatabase.end ())
    {
      if ((int32_t)(i->second.first - sequence) > 0)
        {
          return;
        }
      if (i->second.first == sequence)
        {
          freshInfo = false;
        }
    }
  m_hwmpSeqnoMetricDatabase[prep.GetOriginatorAddress ()] = std::make_pair (sequence, prep.GetMetric ());
  //update routing info
  //Now add a path to destination and add precursor to source
  NS_LOG_DEBUG ("I am " << GetAddress () << ", received prep from " << prep.GetOriginatorAddress () << ", receiver was:" << from);
  HwmpRtable::LookupResult result = m_rtable->LookupReactive (prep.GetDestinationAddress ());
  //Add a reactive path only if seqno is fresher or it improves the
  //metric
  if (
    (freshInfo) ||
    (
      ((m_rtable->LookupReactive (prep.GetOriginatorAddress ())).retransmitter == Mac48Address::GetBroadcast ()) ||
      ((m_rtable->LookupReactive (prep.GetOriginatorAddress ())).metric > prep.GetMetric ())
    )
    )
    {
      m_rtable->AddReactivePath (
        prep.GetOriginatorAddress (),
        from,
        interface,
        prep.GetMetric (),
        MicroSeconds (prep.GetLifetime () * 1024),
        sequence);
      // Notify trace source of routing change
      struct RouteChange rChange;
      rChange.type = "Add Reactive";
      rChange.destination = prep.GetOriginatorAddress ();
      rChange.retransmitter = from;
      rChange.interface = interface;
      rChange.metric = prep.GetMetric ();
      rChange.lifetime = MicroSeconds (prep.GetLifetime () * 1024);
      rChange.seqnum = sequence;
      m_routeChangeTraceSource (rChange);
      m_rtable->AddPrecursor (prep.GetDestinationAddress (), interface, from,
                              MicroSeconds (prep.GetLifetime () * 1024));
      if (result.retransmitter != Mac48Address::GetBroadcast ())
        {
          m_rtable->AddPrecursor (prep.GetOriginatorAddress (), interface, result.retransmitter,
                                  result.lifetime);
        }
      ReactivePathResolved (prep.GetOriginatorAddress ());
    }
  if (
    ((m_rtable->LookupReactive (fromMp)).retransmitter == Mac48Address::GetBroadcast ()) ||
    ((m_rtable->LookupReactive (fromMp)).metric > metric)
    )
    {
      m_rtable->AddReactivePath (
        fromMp,
        from,
        interface,
        metric,
        MicroSeconds (prep.GetLifetime () * 1024),
        sequence);
      // Notify trace source of routing change
      struct RouteChange rChange;
      rChange.type = "Add Reactive";
      rChange.destination = fromMp;
      rChange.retransmitter = from;
      rChange.interface = interface;
      rChange.metric = metric;
      rChange.lifetime = MicroSeconds (prep.GetLifetime () * 1024);
      rChange.seqnum = sequence;
      m_routeChangeTraceSource (rChange);
      ReactivePathResolved (fromMp);
    }
  if (prep.GetDestinationAddress () == GetAddress ())
    {
      NS_LOG_DEBUG ("I am "<<GetAddress ()<<", resolved "<<prep.GetOriginatorAddress ());
      return;
    }
  if (result.retransmitter == Mac48Address::GetBroadcast ())
    {
      return;
    }
  //Forward PREP
  HwmpProtocolMacMap::const_iterator prep_sender = m_interfaces.find (result.ifIndex);
  NS_ASSERT (prep_sender != m_interfaces.end ());
  prep_sender->second->SendPrep (prep, result.retransmitter);
}
uint8_t
HwmpProtocol::convertMac48AddresstoInt (Mac48Address MAC)
{
  stringstream buffer;
  buffer << MAC << endl;
  string s = buffer.str();
  s.pop_back();
  s = s.substr(s.length()-2);

  uint8_t result = stoi(s, nullptr, 16);
  return result;
}
bool
HwmpProtocol::checkIfEdgeExists(uint8_t node1, uint8_t node2)
{
	bool flag = false;
	vector<uint8_t> edge = {node1, node2};
	for (auto i : edges)
	{
		vector<uint8_t> temp = {i.src, i.dest};
		if (edge == temp)
			flag=true;
	}
	return flag;
}
void
HwmpProtocol::AddEdges(uint8_t node1, uint8_t node2)
{
  if ( !checkIfEdgeExists(node1, node2) )
  {
    edges.push_back({node1, node2});
  }
}
vector<uint8_t>
HwmpProtocol::get_vertices(vector<Edge> const &edges)
{
    vector<uint8_t> nodes;
    for (auto edge : edges)
    {
      nodes.push_back(edge.src);
      nodes.push_back(edge.dest);
    }
    //remove duplicates
    sort(nodes.begin(), nodes.end());
    nodes.erase( unique( nodes.begin(), nodes.end() ), nodes.end() );

    return nodes;
}
vector<uint8_t>
HwmpProtocol::getAdjacent(uint8_t u, vector<Edge> const &edges)
{
  vector<uint8_t> adjacent;

  for (auto edge : edges)
  {
      if (u == edge.src)
          adjacent.push_back(edge.dest);
  }

  return adjacent;
}
vector<uint8_t>
HwmpProtocol::get_k_neighbors(vector<uint8_t> V_i, uint8_t u)
{
  vector<uint8_t> k_neighbors;

  for (auto i : V_i)
  {
    for (auto edge : edges)
    {
      if (i==edge.src && edge.dest!=u)
        k_neighbors.push_back(edge.dest);
      else if (i==edge.dest && edge.src!=u)
        k_neighbors.push_back(edge.src);
    }
  }
  //remove duplicates
  sort(k_neighbors.begin(), k_neighbors.end());
  k_neighbors.erase( unique( k_neighbors.begin(), k_neighbors.end() ), k_neighbors.end() );

  return k_neighbors;
}
vector<Edge>
HwmpProtocol::normalize_edges(vector<Edge> const &edges)
{
  vector<Edge> normalized_edges = edges;
  vector<uint8_t> vertices = get_vertices(edges);
  sort(vertices.begin(), vertices.end());
  uint8_t n = vertices.size();

  for (uint8_t j=0; j<normalized_edges.size(); j++)
  {
    for (uint8_t i=0; i<n; i++)
    {
      if (normalized_edges[j].src == vertices[i])
        normalized_edges[j].src = i;
      else if (normalized_edges[j].dest == vertices[i])
        normalized_edges[j].dest = i;
    }
  }
  return normalized_edges;
}
void
HwmpProtocol::recover_SCC(vector<Edge> const &edges)
{
  vector<uint8_t> vertices = get_vertices(edges);
  sort(vertices.begin(), vertices.end());

  for (uint8_t i=0; i<SCC.size(); i++)
    for (uint8_t j=0; j<SCC[i].size(); j++)
      SCC[i][j] = vertices[SCC[i][j]];
}
// A recursive function that finds and prints strongly connected
// components using DFS traversal
// u --> The vertex to be visited next
// disc[] --> Stores discovery times of visited vertices
// low[] -- >> earliest visited vertex (the vertex with minimum
//             discovery time) that can be reached from subtree
//             rooted with current vertex
// *st -- >> To store all the connected ancestors (could be part
//           of SCC)
// stackMember[] --> bit/index array for faster check whether
//                  a node is in stack
void
HwmpProtocol::SCCUtil(int u, int disc[], int low[], stack<int> *st, bool stackMember[], vector<Edge> const &edges)
{
    // A static variable is used for simplicity, we can avoid use
    // of static variable by passing a pointer.
    static int time = 0;

    // Initialize discovery time and low value
    disc[u] = low[u] = ++time;
    st->push(u);
    stackMember[u] = true;

    // Go through all vertices adjacent to this
    vector<uint8_t> neighbors_u = getAdjacent(u, edges);
    for (auto v : neighbors_u)    // v is current adjacent of 'u'
    {
        // If v is not visited yet, then recur for it
        if (disc[v] == -1)
        {
            SCCUtil(v, disc, low, st, stackMember, edges);

            // Check if the subtree rooted with 'v' has a
            // connection to one of the ancestors of 'u'
            // Case 1 (per above discussion on Disc and Low value)
            low[u] = min(low[u], low[v]);
        }

        // Update low value of 'u' only of 'v' is still in stack
        // (i.e. it's a back edge, not cross edge).
        // Case 2 (per above discussion on Disc and Low value)
        else if (stackMember[v] == true)
            low[u] = min(low[u], disc[v]);
    }

    // head node found, pop the stack and print an SCC
    int w = 0;  // To store stack extracted vertices
    if (low[u] == disc[u])
    {
        SCC.emplace_back();
        while (st->top() != u)
        {
            w = (int) st->top();
            //cout << w << " ";
            SCC[SCC_count].push_back(w);
            stackMember[w] = false;
            st->pop();
        }
        w = (int) st->top();
        //cout << w << "\n";
        SCC[SCC_count].push_back(w);
        SCC_count++;
        stackMember[w] = false;
        st->pop();
    }
}
// The function to do DFS traversal. It uses SCCUtil()
void
HwmpProtocol::compute_SCC(vector<Edge> const &edges)
{
  vector<Edge> normalized_edges = normalize_edges(edges);
  int V = get_vertices(normalized_edges).size(); //number of vertices
  int *disc = new int[V];
  int *low = new int[V];
  bool *stackMember = new bool[V];
  stack<int> *st = new stack<int>();

  // Initialize disc and low, and stackMember arrays
  for (int i = 0; i < V; i++)
  {
      disc[i] = NIL;
      low[i] = NIL;
      stackMember[i] = false;
  }

  // Call the recursive helper function to find strongly
  // connected components in DFS tree with vertex 'i'
  for (int i = 0; i < V; i++)
      if (disc[i] == NIL)
          SCCUtil(i, disc, low, st, stackMember, normalized_edges);

  recover_SCC(edges);
}
vector<Edge>
HwmpProtocol::build_subgraph(uint8_t u)
{
    vector<uint8_t> V_plus;

    for ( auto w : neighbors )
    {
      if ( markers[w]==true )      //means the neighbor is marked
        if ( myNodeID < IDs[w] )   //if the neighbor's ID is greater than our current node's ID
          V_plus.push_back(w);
    }

    sort(V_plus.begin(), V_plus.end());

    vector<Edge> V_plus_subgraph = edges;    //get the graph of nodes seen by our current node

    for ( auto edge : edges )
    {
        vector<uint8_t> edge_vec = {edge.src, edge.dest};
        sort(edge_vec.begin(), edge_vec.end());
        vector<uint8_t> difference;

        set_difference(edge_vec.begin(), edge_vec.end(), V_plus.begin(), V_plus.end(), back_inserter(difference));

        if (!difference.empty())
            V_plus_subgraph.erase(remove(V_plus_subgraph.begin(), V_plus_subgraph.end(), edge), V_plus_subgraph.end());
    }

    return V_plus_subgraph;
}
bool
HwmpProtocol::MarkingProcess (uint8_t u)
{
  bool flag=false;

  for (auto v : neighbors)
  {
      for (auto w : neighbors)
      {
          if (neighbors.size() > 1     &&
              w != v                   &&
              checkIfEdgeExists(w, u)  &&
              checkIfEdgeExists(u, v)  &&
              !checkIfEdgeExists(w, v)  )
          {
              flag = true;
          }
      }
  }
  return flag;
}
bool
HwmpProtocol::restricted_k_dominant_pruning(uint8_t u)
{
    /*
    Restricted k-dominant pruning algorithm:
    1. Broadcasts its id and marker (id(u),T) to all its neighbors
    2. Builds a subgraph G[V+'], where V+' = {w|w ∈ (V' ∩ N(u)) ∧ (id(u) < id(w))}
    3. Computes the set of strongly connected components {V1', V2', ... , Vl'} of G[V+']
    4. Changes its marker m(u) to F if there exists Vi',1≤i≤l, such that Nd(u)-Vi'⊆Nd(Vi') and Na(u)-Vi'⊆Na(Vi')
    */
    bool marker = true;

    vector<Edge> subgraph = build_subgraph(u);

    compute_SCC(subgraph);

    bool condition = false;

    for ( auto V_i : SCC )
    {
        vector<uint8_t> N_u = neighbors;
        vector<uint8_t> N_vi = get_k_neighbors(V_i, u);
        sort(N_u.begin(), N_u.end());
        sort(N_vi.begin(), N_vi.end());
        sort(V_i.begin(), V_i.end());
        vector<uint8_t> difference;
        set_difference(N_u.begin(), N_u.end(), V_i.begin(), V_i.end(), back_inserter(difference));
        sort(difference.begin(), difference.end());
        condition = includes(N_vi.begin(), N_vi.end(), difference.begin(), difference.end());
        if (condition)
          marker = false;
    }

    return marker;
}
uint8_t
HwmpProtocol::DecideWhoisTheClosest()
{
  vector<uint8_t> two_hop_neighbors = get_vertices(edges);

  double min_distance=numeric_limits<double>::max();
  uint8_t closest_node;

  for (auto i : two_hop_neighbors)
  {
    double distance = calculateDistance(i);
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_node = i;
    }
  }

  //cout << "closest node is: " << +closest_node << " and I am " << +convertMac48AddresstoInt(GetAddress()) << endl;

  return closest_node;
}
bool
HwmpProtocol::checkEligibility()
{
  int counter = 0;
  for (auto i : neighbors)
  {
    if (calculateDistance(i)>range)
      continue;
    else
      counter++;
  }
  if (counter == 0)
    return false;
  else
    return true;
}
void
HwmpProtocol::SendLpp()
{
  IeLpp lpp;
  m_nbEtx.GotoNextTimeStampAndClearOldest ();
  lpp.SetLppId(m_nbEtx.GetLppTimeStamp());
  //Origin Address to be filled by HwmpProtocolMac::SendLpp function
  lpp.SetOriginSeqno(GetNextHwmpSeqno());
  m_nbEtx.FillLppCntData(lpp);

  //Vector position = m_mp->GetNode()->GetObject<MobilityModel> () ->GetPosition();
  //cout << "Node:" << m_mp->GetNode()->GetID() << endl;
  //cout << "Position, X: " << position.x << ", Y: " << position.y << ", Z: " << position.z << endl;
  //Vector velocity = m_mp->GetNode()->GetObject<MobilityModel> () ->GetVelocity();
  //cout << "Velocity, X: " << velocity.x << ", Y: " << velocity.y << ", Z: " << velocity.z << endl;
  //printf("%.7lf\n",position.x);

  lpp.SetMarker(myMarker);
  //lpp.SetPositionx(position.x);
  //lpp.SetPositiony(position.y);
  //lpp.SetPositionz(position.z);
  //lpp.SetNeighborPositionx(positionx);
  //lpp.SetNeighborPositiony(positiony);
  //lpp.SetNeighborPositionz(positionz);

  lpp.SetNodeID(myNodeID);
  lpp.SetRole(myRole);

  //cout << GetAddress() << " " << +myNodeID << endl;

  // if (GetAddress() == "00:00:00:00:00:01")
  // {
  //   myNodeID = 3;
  //   myRole = 1;  //gateway
  //   lpp.SetNodeID(myNodeID);
  //   lpp.SetRole(myRole);
  // }
  // else if (GetAddress() == "00:00:00:00:00:02")
  // {
  //   myNodeID = 1;
  //   myRole = 3;
  //   lpp.SetNodeID(myNodeID);
  //   lpp.SetRole(myRole);
  // }

  for (HwmpProtocolMacMap::const_iterator lpp_sender = m_interfaces.begin(); lpp_sender != m_interfaces.end(); lpp_sender++)
    {
      lpp_sender->second->SendLpp(lpp);
    }
  m_stats.initiatedLpp++;
  m_lppTimer = Simulator::Schedule(m_dot11MeshHWMPlppMinInterval, &HwmpProtocol::SendLpp, this);
}
void
HwmpProtocol::ReceiveLpp(IeLpp lpp, Mac48Address from, uint32_t interface, Mac48Address fromMp)
{
  SCC_count=0;
  SCC.clear();

  uint8_t current_me = convertMac48AddresstoInt(GetAddress());
  uint8_t current_neighbor = convertMac48AddresstoInt(lpp.GetOriginAddress());

  neighbors.push_back(current_neighbor);
  //remove duplicates in neighbors vector
  sort(neighbors.begin(), neighbors.end());
  neighbors.erase( unique( neighbors.begin(), neighbors.end() ), neighbors.end() );

  AddEdges ( current_me, current_neighbor );
  AddEdges ( current_neighbor, current_me );
  for (auto elem : lpp.GetNeighborList())
  {
    uint8_t element = convertMac48AddresstoInt(elem.first);
    AddEdges ( current_neighbor, element );
    AddEdges ( element, current_neighbor );
  }

  markers.insert(pair<uint8_t, bool>(current_neighbor, lpp.GetMarker()));
  IDs.insert(pair<uint8_t, uint8_t>(current_neighbor, lpp.GetNodeID()));
  roles.insert(pair<uint8_t, uint8_t>(current_neighbor, lpp.GetRole()));
  //positionx.insert(pair<uint8_t, uint32_t>(current_neighbor, lpp.GetPositionx()));
  //positiony.insert(pair<uint8_t, double>(current_neighbor, lpp.GetPositiony()));
  //positionz.insert(pair<uint8_t, double>(current_neighbor, lpp.GetPositionz()));

  // map<uint8_t, double>::iterator i;

  // for (i = positionx.begin(); i != positionx.end(); i++)
  // {
  //   cout << +(*i).first << " " << (*i).second << endl;
  // }
  // cout << endl;

  // neighborpositionx = lpp.GetNeighborPositionx();
  // neighborpositiony = lpp.GetNeighborPositiony();
  // neighborpositionz = lpp.GetNeighborPositionz();

  // for (i = lpp.GetNeighborPositionx().begin(); i != lpp.GetNeighborPositionx().end(); i++)
  // {
  //   // cout << +(*i).first << endl;
  //   // cout << (*i).second << endl;
  //   neighborpositionx.insert(pair<uint8_t, double>((*i).first, (*i).second));
  // }
  // for (i = lpp.GetNeighborPositiony().begin(); i != lpp.GetNeighborPositiony().end(); i++)
  // {
  //   neighborpositiony.insert(pair<uint8_t, double>((*i).first, (*i).second));
  // }
  // for (i = lpp.GetNeighborPositionz().begin(); i != lpp.GetNeighborPositionz().end(); i++)
  // {
  //   neighborpositionz.insert(pair<uint8_t, double>((*i).first, (*i).second));
  // }

  // for (i = neighborpositionx.begin(); i != neighborpositionx.end(); i++)
  // {
  //   //cout << GetAddress () << " " << positionx.size() << " " << get_vertices(edges).size() << endl;
  //   cout << +(*i).first << " " << (*i).second << endl;
  // }

  // for (auto i : edges)
  // {
  //   cout << GetAddress() << " " << i.src << i.dest << endl;
  // }
  // cout << endl;

  // if (GetAddress()=="00:00:00:00:00:14")
  // {
  //   for ( auto i : neighbors)
  //     cout << +i << " " << endl;
  // }

  myMarker = MarkingProcess(current_me);
  //cout << "I am  " << +current_me << " and my marker after marking process is " << myMarker << endl;
  if (myRole == 1 || myRole == 2)
  {
    myMarker = true;
  }

  if (myMarker == true && myRole == 3)
  {
    myMarker = restricted_k_dominant_pruning(current_me);
    //cout << "I am " << +current_me << " and my marker after pruning is " << myMarker << endl;
  }

  uint8_t closest = DecideWhoisTheClosest();
  //cout << "I am " << +current_me << " and closest is " <<  +closest << endl;

  // if (myClaimCounter > 100)
  // {
  //   if (control==0 && current_me==2)
  //   {
  //     m_isClaimer=true;
  //     cout << "I sent a claim packet" << endl;
  //     SendClaimPkt();
  //     control++;
  //   }
  // }

  // EventId testTimer;
  // Time testTimeValue = MilliSeconds(100);
  // if (GetAddress() == "00:00:00:00:00:03")
  // {
  //   // cout << "test" << endl;
  //   testTimer = Simulator::Schedule(testTimeValue, &HwmpProtocol::SendClaimPkt, this);
  // }

  // if (!myMarker)
  // {
  //   cout << "I am " << +current_me << " and my marker is " << myMarker << ", my role is " << +myRole << " , my ID is " << +myNodeID << endl;
  // }

  if ( current_me == closest )
  {
    next_claimingNodes.push_back(current_me);
    //remove duplicates
    sort(next_claimingNodes.begin(), next_claimingNodes.end());
    next_claimingNodes.erase( unique( next_claimingNodes.begin(), next_claimingNodes.end() ), next_claimingNodes.end() );
    if (flag==true) //do this only once
    {
      prev_claimingNodes = next_claimingNodes;
      flag = false;
    }

    if (next_claimingNodes == prev_claimingNodes)
      counter++;
    else
      prev_claimingNodes = next_claimingNodes;

    if (counter>=10 && !myMarker)
      m_isClaimer=true;

    if (counter >= 15 && alreadySentClaim==false)  //if list of claiming nodes don't change for 15 consecutive times, then claim being closest
    {
      // if (checkEligibility() && myMarker != true)
      // {
        // m_isClaimer = true;
        if (!myMarker)
        {
          SendClaimPkt();
          alreadySentClaim=true;
          cout << "I am " << +current_me << " and I claimed to be the closest " << endl;
          cout << "Found the claimers at " << Simulator::Now().GetSeconds() << endl;
          vector<myPair> sortedAllNodes = sortAllNodesByProximity();

          // print the vector
          cout << "----------------" << endl;
          for (auto const &myPair: sortedAllNodes) {
            cout << '{' << myPair.first << "," << myPair.second << '}' << '\n';
          }

          if (!isMoved && current_me == m_movingNode)
          {
            isMoved = true;
            cout << "now moving node " << +current_me << " to the new location" << endl;
            Ptr<ConstantVelocityMobilityModel> mob = m_mp->GetNode()->GetObject<ConstantVelocityMobilityModel> ();
            // Vector m_position = m_mp->GetNode()->GetObject<MobilityModel> () ->GetPosition();
            // Ptr<ConstantVelocityMobilityModel> mob = nodes.Get(9)->GetObject<ConstantVelocityMobilityModel>();
            Vector m_position = mob->GetPosition();
			uint32_t velocity = 15; // meter per second
			double howManySeconds = sqrt((pow((newLocation[0]-m_position.x), 2)+pow((newLocation[1]-m_position.y), 2)+pow((newLocation[2]-m_position.z), 2))/pow(velocity, 2));
			cout << "it will take " << howManySeconds << " seconds to move the node to the new location" << endl;
            mob->SetVelocity(Vector((newLocation[0]-m_position.x)/howManySeconds, (newLocation[1]-m_position.y)/howManySeconds, (newLocation[2]-m_position.z)/howManySeconds));
            Simulator::Schedule(Seconds(howManySeconds), &HwmpProtocol::stopTheNode, this);
          }
        }
      // }
      // else if (myMarker)
      //   cout << +current_me << " claimed but, part of CDS" << endl;
      // else if (!checkEligibility())
      //   cout << +current_me << " claimed but not eligible" << endl;
    }
  }

  // if (current_me == 9 || current_me == 15)
  // {
  //   Vector position = m_mp->GetNode()->GetObject<MobilityModel> () ->GetPosition();
  //   cout << position.x << " " << position.y << " " << position.z << endl;
  // }

  NS_LOG_FUNCTION(this << from << interface);
  Mac48Address origin = lpp.GetOriginAddress();
  NS_ASSERT(origin == from); // Neighbor from which the packet is received is always originator of LPP packet
  uint8_t lppTimeStamp = lpp.GetLppId();
  uint8_t numNeighbors = lpp.GetNumberNeighbors();
  std::pair<Mac48Address, uint8_t> nb_lppreverse;

  // Search for my MAC address in LPP packet
  uint8_t lppReverse = 0; // if my address is not found in the packet, lpp reverse should be 0
  for (uint8_t j = 0; j<numNeighbors; ++j)
    {
      lpp.RemoveFromNeighborsList(nb_lppreverse);
      if (nb_lppreverse.first == GetAddress()) // is it my MAC address?
        {
          lppReverse = nb_lppreverse.second;
          break;
        }
    }
  // Add new or udate existing etx entry for neighbor with MAC address "from".
  // LPP count is updated based on lpp Time Slot indicated in packet.
  // LPP reverse count is updated from list provided in LPP packet.
  m_nbEtx.UpdateNeighborEtx(from, lppTimeStamp, lppReverse);
}
void
HwmpProtocol::ReceiveClaimPkt(IeClaim claimPkt, Mac48Address from, uint32_t interface, Mac48Address fromMp)
{
  // uint8_t current_me = convertMac48AddresstoInt(GetAddress());
  // cout << +current_me << endl;
  // uint8_t current_claimer = convertMac48AddresstoInt(claimPkt.GetOriginAddress());
  if ( claimPkt.IsClaimer() && convertMac48AddresstoInt(GetAddress())!=convertMac48AddresstoInt(claimPkt.GetOriginAddress()) )
  {
    if (m_isClaimer)
    {
      uint8_t current_claimer = convertMac48AddresstoInt(claimPkt.GetOriginAddress());
      cout << "I am " << +convertMac48AddresstoInt(GetAddress()) << " and I got a claim from " << +current_claimer << " at " << Simulator::Now().GetSeconds() << endl;

      Ptr<MobilityModel> mob = m_mp->GetNode()->GetObject<MobilityModel>();
      Vector myPosition = mob->GetPosition();
      double myDistance = pow( (newLocation[0]-myPosition.x), 2 ) + pow( (newLocation[1]-myPosition.y), 2 ) + pow( (newLocation[2]-myPosition.z), 2 );
      myDistance = sqrt(myDistance);

      Vector claimerLocation = claimPkt.GetLocation();
      double claimerDistance = pow( (newLocation[0]-claimerLocation.x), 2 ) + pow( (newLocation[1]-claimerLocation.y), 2 ) + pow( (newLocation[2]-claimerLocation.z), 2 );
      claimerDistance = sqrt(claimerDistance);

      cout << myDistance << " " << claimerDistance << endl;

      storeOtherClaimers.insert(pair<uint8_t, double>(current_claimer, claimerDistance));

      uint8_t counter = 0;
      map<uint8_t, double>::iterator i;
      for (i = storeOtherClaimers.begin(); i != storeOtherClaimers.end(); i++)
      {
        if (myDistance < i->second)
          counter++;
      }
      if (counter == storeOtherClaimers.size())
      {
        cout << "I (" << +convertMac48AddresstoInt(GetAddress()) << ") will move" << endl;
      }

      // if (closestDominator != 0)
      // {
      //   double distanceToClosestDominator = calculateDistance(closestDominator);
      //   if (distanceToClosestDominator<=range)
      //   {
      //     uint8_t theNodeBeingSent = sortedClaimingNodes[0].second;
      //     nodesBeingSent.push_back(theNodeBeingSent);
      //     vector<uint32_t> location;
      //     uint32_t x = newLocation[0]*10;
      //     uint32_t y = newLocation[1]*10;
      //     uint32_t z = newLocation[2]*10;
      //     location.push_back(x);
      //     location.push_back(y);
      //     location.push_back(z);
      //     nodesBeingSentLocations.push_back(location);
      //     cout << "sent the node " << +theNodeBeingSent << endl;
      //     sentTheNodesFlag = true;
      //   }
      //   else if (distanceToClosestDominator>range && distanceToClosestDominator<=2*range)
      //   {
      //     if (sortedClaimingNodes.size()>=2)
      //     {
      //       uint8_t theNodeBeingSent = sortedClaimingNodes[0].second;
      //       nodesBeingSent.push_back(theNodeBeingSent);
      //       vector<uint32_t> location1;
      //       uint32_t a = newLocation[0]*10;
      //       uint32_t b = newLocation[1]*10;
      //       uint32_t c = newLocation[2]*10;
      //       location1.push_back(a);
      //       location1.push_back(b);
      //       location1.push_back(c);
      //       nodesBeingSentLocations.push_back(location1);
      //       cout << "sent the node " << +theNodeBeingSent << endl;
      //
      //       theNodeBeingSent = sortedClaimingNodes[1].second;
      //       nodesBeingSent.push_back(theNodeBeingSent);
      //       vector<uint32_t> location2;
      //       uint32_t x = 10*((newLocation[0]+nodepositions[m_topoId][closestDominator-1][0])/2);
      //       uint32_t y = 10*((newLocation[1]+nodepositions[m_topoId][closestDominator-1][1])/2);
      //       uint32_t z = 10*((newLocation[2]+nodepositions[m_topoId][closestDominator-1][2])/2);
      //       location2.push_back(x);
      //       location2.push_back(y);
      //       location2.push_back(z);
      //       nodesBeingSentLocations.push_back(location2);
      //       cout << "sent the node " << +theNodeBeingSent << endl;
      //       sentTheNodesFlag = true;
      //     }
      //     else
      //       cout << "there are not enough suitable nodes to send" << endl;
      //   }
      // }

    }
  }

  //acceptance criteria:
  std::map<Mac48Address, uint32_t>::const_iterator i = m_hwmpClaimSeqnoDatabase.find (claimPkt.GetOriginAddress ());
  if (i != m_hwmpClaimSeqnoDatabase.end() and ( (i->second - claimPkt.GetOriginatorSeqNumber()) >= 0 ) )
    {
      return;
    }
  m_hwmpClaimSeqnoDatabase[claimPkt.GetOriginAddress ()] = claimPkt.GetOriginatorSeqNumber ();
  NS_LOG_DEBUG ("I am " << GetAddress () << ", Accepted claim packet from address" << from << ", claim pkt:" << claimPkt);
  if (m_isClaimer)
  {
    // Here you process the claimPkt Information Element because it is of interest to other Claimer Nodes
    m_claimersList.push_back (claimPkt);
  }
  // Forward Claim Packet to all interfaces, there might be other claimer node(s):
  NS_LOG_DEBUG ("I am " << GetAddress () << "retransmitting Claim Packet:" << claimPkt);
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->SendClaimPkt (claimPkt);
    }
}
void
HwmpProtocol::stopTheNode()
{
  Ptr<ConstantVelocityMobilityModel> mob = m_mp->GetNode()->GetObject<ConstantVelocityMobilityModel> ();
  mob->SetVelocity(Vector(0, 0, 0));
  cout << "successfully moved" << endl;
}
void
HwmpProtocol::ReceivePerr (std::vector<FailedDestination> destinations, Mac48Address from, uint32_t interface, Mac48Address fromMp)
{
  NS_LOG_FUNCTION (this << from << interface << fromMp);
  //Acceptance cretirea:
  NS_LOG_DEBUG ("I am "<<GetAddress ()<<", received PERR from "<<from);
  std::vector<FailedDestination> retval;
  HwmpRtable::LookupResult result;
  for (unsigned int i = 0; i < destinations.size (); i++)
    {
      result = m_rtable->LookupReactiveExpired (destinations[i].destination);
      if (!(
            (result.retransmitter != from) ||
            (result.ifIndex != interface) ||
            ((int32_t)(result.seqnum - destinations[i].seqnum) > 0)
            ))
        {
          retval.push_back (destinations[i]);
        }
    }
  if (retval.size () == 0)
    {
      return;
    }
  ForwardPathError (MakePathError (retval));
}
void
HwmpProtocol::SendPrep (
  Mac48Address src,
  Mac48Address dst,
  Mac48Address retransmitter,
  uint32_t initMetric,
  uint32_t originatorDsn,
  uint32_t destinationSN,
  uint32_t lifetime,
  uint32_t interface)
{
  IePrep prep;
  prep.SetHopcount (0);
  prep.SetTtl (m_maxTtl);
  prep.SetDestinationAddress (dst);
  prep.SetDestinationSeqNumber (destinationSN);
  prep.SetLifetime (lifetime);
  prep.SetMetric (initMetric);
  prep.SetOriginatorAddress (src);
  prep.SetOriginatorSeqNumber (originatorDsn);
  HwmpProtocolMacMap::const_iterator prep_sender = m_interfaces.find (interface);
  NS_ASSERT (prep_sender != m_interfaces.end ());
  prep_sender->second->SendPrep (prep, retransmitter);
  m_stats.initiatedPrep++;
}
void
HwmpProtocol::SendClaimPkt()
{
  IeClaim claimPkt;
  if (m_isClaimer)
  {
    claimPkt.SetIsClaimer();
  }
  if (m_doIMove)
  {
    claimPkt.SetDoIMove();
  }
  claimPkt.SetOriginAddress(GetAddress());
  claimPkt.SetMarker(myMarker);
  claimPkt.SetRole(myRole);
  Ptr<MobilityModel> mob = m_mp->GetNode()->GetObject<MobilityModel>();
  Vector m_position = mob->GetPosition();
  claimPkt.SetLocation(m_position);
  //Origin Address to be filled by HwmpProtocolMac::SendClaimPkt function
  claimPkt.SetOriginatorSeqNumber(GetNextHwmpSeqno());
  for (HwmpProtocolMacMap::const_iterator claim_sender = m_interfaces.begin(); claim_sender != m_interfaces.end(); claim_sender++)
    {
      claim_sender->second->SendClaimPkt(claimPkt);
    }
  m_claimPktRetry--;
  if (m_claimPktRetry)
  {
    m_claimPktTimer = Simulator::Schedule(MilliSeconds (500), &HwmpProtocol::SendClaimPkt, this);
  }
}
bool
HwmpProtocol::Install (Ptr<MeshPointDevice> mp)
{
  NS_LOG_FUNCTION (this << mp);
  m_mp = mp;
  std::vector<Ptr<NetDevice> > interfaces = mp->GetInterfaces ();
  for (std::vector<Ptr<NetDevice> >::const_iterator i = interfaces.begin (); i != interfaces.end (); i++)
    {
      // Checking for compatible net device
      Ptr<WifiNetDevice> wifiNetDev = (*i)->GetObject<WifiNetDevice> ();
      if (wifiNetDev == 0)
        {
          return false;
        }
      Ptr<MeshWifiInterfaceMac>  mac = wifiNetDev->GetMac ()->GetObject<MeshWifiInterfaceMac> ();
      if (mac == 0)
        {
          return false;
        }
      // Installing plugins:
      Ptr<HwmpProtocolMac> hwmpMac = Create<HwmpProtocolMac> (wifiNetDev->GetIfIndex (), this);
      m_interfaces[wifiNetDev->GetIfIndex ()] = hwmpMac;
      mac->InstallPlugin (hwmpMac);
      //Installing airtime link metric:
      Ptr<AirtimeLinkMetricCalculator> metric = CreateObject <AirtimeLinkMetricCalculator> ();
      mac->SetLinkMetricCallback (MakeCallback (&AirtimeLinkMetricCalculator::CalculateMetric, metric));
    }
  mp->SetRoutingProtocol (this);
  // Mesh point aggregates all installed protocols
  mp->AggregateObject (this);
  m_address = Mac48Address::ConvertFrom (mp->GetAddress ()); // address;
  return true;
}
void
HwmpProtocol::PeerLinkStatus (Mac48Address meshPointAddress, Mac48Address peerAddress, uint32_t interface, bool status)
{
  NS_LOG_FUNCTION (this << meshPointAddress << peerAddress << interface << status);
  if (status)
    {
      return;
    }
  std::vector<FailedDestination> destinations = m_rtable->GetUnreachableDestinations (peerAddress);
  NS_LOG_DEBUG (destinations.size () << " failed destinations for peer address " << peerAddress);
  InitiatePathError (MakePathError (destinations));
}
void
HwmpProtocol::SetNeighboursCallback (Callback<std::vector<Mac48Address>, uint32_t> cb)
{
  m_neighboursCallback = cb;
}
bool
HwmpProtocol::DropDataFrame (uint32_t seqno, Mac48Address source)
{
  NS_LOG_FUNCTION (this << seqno << source);
  if (source == GetAddress ())
    {
      return true;
    }
  std::map<Mac48Address, uint32_t,std::less<Mac48Address> >::const_iterator i = m_lastDataSeqno.find (source);
  if (i == m_lastDataSeqno.end ())
    {
      m_lastDataSeqno[source] = seqno;
    }
  else
    {
      if ((int32_t)(i->second - seqno)  >= 0)
        {
          return true;
        }
      m_lastDataSeqno[source] = seqno;
    }
  return false;
}
HwmpProtocol::PathError
HwmpProtocol::MakePathError (std::vector<FailedDestination> destinations)
{
  NS_LOG_FUNCTION (this);
  PathError retval;
  //HwmpRtable increments a sequence number as written in 11B.9.7.2
  retval.receivers = GetPerrReceivers (destinations);
  if (retval.receivers.size () == 0)
    {
      return retval;
    }
  m_stats.initiatedPerr++;
  for (unsigned int i = 0; i < destinations.size (); i++)
    {
      retval.destinations.push_back (destinations[i]);
      m_rtable->DeleteReactivePath (destinations[i].destination);
      // Notify trace source of routing change
      struct RouteChange rChange;
      rChange.type = "Delete Reactive";
      rChange.destination = destinations[i].destination;
      rChange.seqnum = destinations[i].seqnum;
      m_routeChangeTraceSource (rChange);
    }
  return retval;
}
void
HwmpProtocol::InitiatePathError (PathError perr)
{
  NS_LOG_FUNCTION (this);
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      std::vector<Mac48Address> receivers_for_interface;
      for (unsigned int j = 0; j < perr.receivers.size (); j++)
        {
          if (i->first == perr.receivers[j].first)
            {
              receivers_for_interface.push_back (perr.receivers[j].second);
            }
        }
      i->second->InitiatePerr (perr.destinations, receivers_for_interface);
    }
}
void
HwmpProtocol::ForwardPathError (PathError perr)
{
  NS_LOG_FUNCTION (this);
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      std::vector<Mac48Address> receivers_for_interface;
      for (unsigned int j = 0; j < perr.receivers.size (); j++)
        {
          if (i->first == perr.receivers[j].first)
            {
              receivers_for_interface.push_back (perr.receivers[j].second);
            }
        }
      i->second->ForwardPerr (perr.destinations, receivers_for_interface);
    }
}

std::vector<std::pair<uint32_t, Mac48Address> >
HwmpProtocol::GetPerrReceivers (std::vector<FailedDestination> failedDest)
{
  NS_LOG_FUNCTION (this);
  HwmpRtable::PrecursorList retval;
  for (unsigned int i = 0; i < failedDest.size (); i++)
    {
      HwmpRtable::PrecursorList precursors = m_rtable->GetPrecursors (failedDest[i].destination);
      m_rtable->DeleteReactivePath (failedDest[i].destination);
      // Notify trace source of routing change
      struct RouteChange rChange;
      rChange.type = "Delete Reactive";
      rChange.destination = failedDest[i].destination;
      rChange.seqnum = failedDest[i].seqnum;
      m_routeChangeTraceSource (rChange);
      m_rtable->DeleteProactivePath (failedDest[i].destination);
      // Notify trace source of routing change
      struct RouteChange rChangePro;
      rChangePro.type = "Delete Proactive";
      rChangePro.destination = failedDest[i].destination;
      rChangePro.seqnum = failedDest[i].seqnum;
      m_routeChangeTraceSource (rChangePro);
      for (unsigned int j = 0; j < precursors.size (); j++)
        {
          retval.push_back (precursors[j]);
        }
    }
  //Check if we have duplicates in retval and precursors:
  for (unsigned int i = 0; i < retval.size (); i++)
    {
      for (unsigned int j = i+1; j < retval.size (); j++)
        {
          if (retval[i].second == retval[j].second)
            {
              retval.erase (retval.begin () + j);
            }
        }
    }
  return retval;
}
std::vector<Mac48Address>
HwmpProtocol::GetPreqReceivers (uint32_t interface)
{
  NS_LOG_FUNCTION (this << interface);
  std::vector<Mac48Address> retval;
  if (!m_neighboursCallback.IsNull ())
    {
      retval = m_neighboursCallback (interface);
    }
  if ((retval.size () >= m_unicastPreqThreshold) || (retval.size () == 0))
    {
      retval.clear ();
      retval.push_back (Mac48Address::GetBroadcast ());
    }
  return retval;
}
std::vector<Mac48Address>
HwmpProtocol::GetBroadcastReceivers (uint32_t interface)
{
  NS_LOG_FUNCTION (this << interface);
  std::vector<Mac48Address> retval;
  if (!m_neighboursCallback.IsNull ())
    {
      retval = m_neighboursCallback (interface);
    }
  if ((retval.size () >= m_unicastDataThreshold) || (retval.size () == 0))
    {
      retval.clear ();
      retval.push_back (Mac48Address::GetBroadcast ());
    }
  return retval;
}

bool
HwmpProtocol::QueuePacket (QueuedPacket packet)
{
  NS_LOG_FUNCTION (this);
  if (m_rqueue.size () > m_maxQueueSize)
    {
      return false;
    }
  m_rqueue.push_back (packet);
  return true;
}

HwmpProtocol::QueuedPacket
HwmpProtocol::DequeueFirstPacketByDst (Mac48Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  QueuedPacket retval;
  retval.pkt = 0;
  for (std::vector<QueuedPacket>::iterator i = m_rqueue.begin (); i != m_rqueue.end (); i++)
    {
      if ((*i).dst == dst)
        {
          retval = (*i);
          m_rqueue.erase (i);
          break;
        }
    }
  return retval;
}

HwmpProtocol::QueuedPacket
HwmpProtocol::DequeueFirstPacket ()
{
  NS_LOG_FUNCTION (this);
  QueuedPacket retval;
  retval.pkt = 0;
  if (m_rqueue.size () != 0)
    {
      retval = m_rqueue[0];
      m_rqueue.erase (m_rqueue.begin ());
    }
  return retval;
}

void
HwmpProtocol::ReactivePathResolved (Mac48Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.find (dst);
  if (i != m_preqTimeouts.end ())
    {
      m_routeDiscoveryTimeCallback (Simulator::Now () - i->second.whenScheduled);
    }

  HwmpRtable::LookupResult result = m_rtable->LookupReactive (dst);
  NS_ASSERT (result.retransmitter != Mac48Address::GetBroadcast ());
  //Send all packets stored for this destination
  QueuedPacket packet = DequeueFirstPacketByDst (dst);
  while (packet.pkt != 0)
    {
      //set RA tag for retransmitter:
      HwmpTag tag;
      packet.pkt->RemovePacketTag (tag);
      tag.SetAddress (result.retransmitter);
      packet.pkt->AddPacketTag (tag);
      m_stats.txUnicast++;
      m_stats.txBytes += packet.pkt->GetSize ();
      packet.reply (true, packet.pkt, packet.src, packet.dst, packet.protocol, result.ifIndex);

      packet = DequeueFirstPacketByDst (dst);
    }
}
void
HwmpProtocol::ProactivePathResolved ()
{
  NS_LOG_FUNCTION (this);
  //send all packets to root
  HwmpRtable::LookupResult result = m_rtable->LookupProactive ();
  NS_ASSERT (result.retransmitter != Mac48Address::GetBroadcast ());
  QueuedPacket packet = DequeueFirstPacket ();
  while (packet.pkt != 0)
    {
      //set RA tag for retransmitter:
      HwmpTag tag;
      if (!packet.pkt->RemovePacketTag (tag))
        {
          NS_FATAL_ERROR ("HWMP tag must be present at this point");
        }
      tag.SetAddress (result.retransmitter);
      packet.pkt->AddPacketTag (tag);
      m_stats.txUnicast++;
      m_stats.txBytes += packet.pkt->GetSize ();
      packet.reply (true, packet.pkt, packet.src, packet.dst, packet.protocol, result.ifIndex);

      packet = DequeueFirstPacket ();
    }
}

bool
HwmpProtocol::ShouldSendPreq (Mac48Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  std::map<Mac48Address, PreqEvent>::const_iterator i = m_preqTimeouts.find (dst);
  if (i == m_preqTimeouts.end ())
    {
      m_preqTimeouts[dst].preqTimeout = Simulator::Schedule (
          Time (m_dot11MeshHWMPnetDiameterTraversalTime * 2),
          &HwmpProtocol::RetryPathDiscovery, this, dst, 1);
      m_preqTimeouts[dst].whenScheduled = Simulator::Now ();
      return true;
    }
  return false;
}
void
HwmpProtocol::RetryPathDiscovery (Mac48Address dst, uint8_t numOfRetry)
{
  NS_LOG_FUNCTION (this << dst << (uint16_t) numOfRetry);
  HwmpRtable::LookupResult result = m_rtable->LookupReactive (dst);
  if (result.retransmitter == Mac48Address::GetBroadcast ())
    {
      result = m_rtable->LookupProactive ();
    }
  if (result.retransmitter != Mac48Address::GetBroadcast ())
    {
      std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.find (dst);
      NS_ASSERT (i != m_preqTimeouts.end ());
      m_preqTimeouts.erase (i);
      return;
    }
  if (numOfRetry > m_dot11MeshHWMPmaxPREQretries)
    {
      QueuedPacket packet = DequeueFirstPacketByDst (dst);
      //purge queue and delete entry from retryDatabase
      while (packet.pkt != 0)
        {
          m_stats.totalDropped++;
          packet.reply (false, packet.pkt, packet.src, packet.dst, packet.protocol, HwmpRtable::MAX_METRIC);
          packet = DequeueFirstPacketByDst (dst);
        }
      std::map<Mac48Address, PreqEvent>::iterator i = m_preqTimeouts.find (dst);
      NS_ASSERT (i != m_preqTimeouts.end ());
      m_routeDiscoveryTimeCallback (Simulator::Now () - i->second.whenScheduled);
      m_preqTimeouts.erase (i);
      return;
    }
  numOfRetry++;
  uint32_t originator_seqno = GetNextHwmpSeqno ();
  uint32_t dst_seqno = m_rtable->LookupReactiveExpired (dst).seqnum;
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->RequestDestination (dst, originator_seqno, dst_seqno);
    }
  m_preqTimeouts[dst].preqTimeout = Simulator::Schedule (
      Time ((2 * (numOfRetry + 1)) *  m_dot11MeshHWMPnetDiameterTraversalTime),
      &HwmpProtocol::RetryPathDiscovery, this, dst, numOfRetry);
}
//Proactive PREQ routines:
void
HwmpProtocol::SetRoot ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("ROOT IS: " << m_address);
  m_isRoot = true;
}
void
HwmpProtocol::UnsetRoot ()
{
  NS_LOG_FUNCTION (this);
  m_proactivePreqTimer.Cancel ();
}
void
HwmpProtocol::SendProactivePreq ()
{
  NS_LOG_FUNCTION (this);
  IePreq preq;
  //By default: must answer
  preq.SetHopcount (0);
  preq.SetTTL (m_maxTtl);
  preq.SetLifetime (m_dot11MeshHWMPactiveRootTimeout.GetMicroSeconds () /1024);
  //\attention: do not forget to set originator address, sequence
  //number and preq ID in HWMP-MAC plugin
  preq.AddDestinationAddressElement (true, true, Mac48Address::GetBroadcast (), 0);
  preq.SetOriginatorAddress (GetAddress ());
  preq.SetPreqID (GetNextPreqId ());
  preq.SetOriginatorSeqNumber (GetNextHwmpSeqno ());
  for (HwmpProtocolMacMap::const_iterator i = m_interfaces.begin (); i != m_interfaces.end (); i++)
    {
      i->second->SendPreq (preq);
    }
  m_proactivePreqTimer = Simulator::Schedule (m_dot11MeshHWMPpathToRootInterval, &HwmpProtocol::SendProactivePreq, this);
}
bool
HwmpProtocol::GetDoFlag ()
{
  return m_doFlag;
}
bool
HwmpProtocol::GetRfFlag ()
{
  return m_rfFlag;
}
Time
HwmpProtocol::GetPreqMinInterval ()
{
  return m_dot11MeshHWMPpreqMinInterval;
}
Time
HwmpProtocol::GetPerrMinInterval ()
{
  return m_dot11MeshHWMPperrMinInterval;
}
uint8_t
HwmpProtocol::GetMaxTtl ()
{
  return m_maxTtl;
}
uint32_t
HwmpProtocol::GetNextPreqId ()
{
  m_preqId++;
  return m_preqId;
}
uint32_t
HwmpProtocol::GetNextHwmpSeqno ()
{
  m_hwmpSeqno++;
  return m_hwmpSeqno;
}
uint32_t
HwmpProtocol::GetActivePathLifetime ()
{
  return m_dot11MeshHWMPactivePathTimeout.GetMicroSeconds () / 1024;
}
uint8_t
HwmpProtocol::GetUnicastPerrThreshold ()
{
  return m_unicastPerrThreshold;
}
Mac48Address
HwmpProtocol::GetAddress ()
{
  return m_address;
}
//Statistics:
HwmpProtocol::Statistics::Statistics () :
  txUnicast (0),
  txBroadcast (0),
  txBytes (0),
  droppedTtl (0),
  totalQueued (0),
  totalDropped (0),
  initiatedPreq (0),
  initiatedPrep (0),
  initiatedPerr (0),
  initiatedLpp (0)
{
}
void HwmpProtocol::Statistics::Print (std::ostream & os) const
{
  os << "<Statistics "
  "txUnicast=\"" << txUnicast << "\" "
  "txBroadcast=\"" << txBroadcast << "\" "
  "txBytes=\"" << txBytes << "\" "
  "droppedTtl=\"" << droppedTtl << "\" "
  "totalQueued=\"" << totalQueued << "\" "
  "totalDropped=\"" << totalDropped << "\" "
  "initiatedPreq=\"" << initiatedPreq << "\" "
  "initiatedPrep=\"" << initiatedPrep << "\" "
  "initiatedPerr=\"" << initiatedPerr << "\" "
  "initiatedLpp=\"" << initiatedLpp << "\"/>" << std::endl;
}
void
HwmpProtocol::Report (std::ostream & os)
{
  os << "<Hwmp "
  "address=\"" << m_address << "\"" << std::endl <<
  "maxQueueSize=\"" << m_maxQueueSize << "\"" << std::endl <<
  "Dot11MeshHWMPmaxPREQretries=\"" << (uint16_t)m_dot11MeshHWMPmaxPREQretries << "\"" << std::endl <<
  "Dot11MeshHWMPnetDiameterTraversalTime=\"" << m_dot11MeshHWMPnetDiameterTraversalTime.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPpreqMinInterval=\"" << m_dot11MeshHWMPpreqMinInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPperrMinInterval=\"" << m_dot11MeshHWMPperrMinInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPlppMinInterval=\"" << m_dot11MeshHWMPlppMinInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPactiveRootTimeout=\"" << m_dot11MeshHWMPactiveRootTimeout.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPactivePathTimeout=\"" << m_dot11MeshHWMPactivePathTimeout.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPpathToRootInterval=\"" << m_dot11MeshHWMPpathToRootInterval.GetSeconds () << "\"" << std::endl <<
  "Dot11MeshHWMPrannInterval=\"" << m_dot11MeshHWMPrannInterval.GetSeconds () << "\"" << std::endl <<
  "isRoot=\"" << m_isRoot << "\"" << std::endl <<
  "maxTtl=\"" << (uint16_t)m_maxTtl << "\"" << std::endl <<
  "unicastPerrThreshold=\"" << (uint16_t)m_unicastPerrThreshold << "\"" << std::endl <<
  "unicastPreqThreshold=\"" << (uint16_t)m_unicastPreqThreshold << "\"" << std::endl <<
  "unicastDataThreshold=\"" << (uint16_t)m_unicastDataThreshold << "\"" << std::endl <<
  "doFlag=\"" << m_doFlag << "\"" << std::endl <<
  "rfFlag=\"" << m_rfFlag << "\">" << std::endl;
  m_stats.Print (os);
  m_nbEtx.Print (os);
  //m_rtable->Print (os);
  for (HwmpProtocolMacMap::const_iterator plugin = m_interfaces.begin (); plugin != m_interfaces.end (); plugin++)
    {
      plugin->second->Report (os);
    }
  os << "</Hwmp>" << std::endl;
}
void
HwmpProtocol::ResetStats ()
{
  NS_LOG_FUNCTION (this);
  m_stats = Statistics ();
  for (HwmpProtocolMacMap::const_iterator plugin = m_interfaces.begin (); plugin != m_interfaces.end (); plugin++)
    {
      plugin->second->ResetStats ();
    }
}

int64_t
HwmpProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_coefficient->SetStream (stream);
  return 1;
}

Ptr<HwmpRtable>
HwmpProtocol::GetRoutingTable (void) const
{
  return m_rtable;
}

HwmpProtocol::QueuedPacket::QueuedPacket () :
  pkt (0),
  protocol (0),
  inInterface (0)
{
}
} // namespace dot11s
} // namespace ns3
