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
 *
 *
 * By default this script creates m_xSize * m_ySize square grid topology with
 * IEEE802.11s stack installed at each node with peering management
 * and HWMP protocol.
 * The side of the square cell is defined by m_step parameter.
 * When topology is created, UDP ping is installed to opposite corners
 * by diagonals. packet size of the UDP ping and interval between two
 * successive packets is configurable.
 *
 *  m_xSize * step
 *  |<--------->|
 *   step
 *  |<--->|
 *  * --- * --- * <---Ping sink  _
 *  | \   |   / |                ^
 *  |   \ | /   |                |
 *  * --- * --- * m_ySize * step |
 *  |   / | \   |                |
 *  | /   |   \ |                |
 *  * --- * --- *                _
 *  ^ Ping source
 *
 *  See also MeshTest::Configure to read more about configurable
 *  parameters.
 */

#include "myPacket.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

#include "10nodes/finaltopology.h"
#define numberOfNodes 10

using namespace ns3;

std::string m_csvFileNamePrefix ("Net");
uint8_t appStartDiff = 0; // [s] time difference between start of two following applications
std::string protocol = "ns3::UdpSocketFactory"; // protocol for transport layer
std::string rate ("50kbps"); // application layer data rate
uint32_t packetSize = 128; // Bytes
double netStartupTime = 3.0; // [s] time before any application starts sending data
double simulationDuration = 103.0; // in seconds
uint32_t port = 80;
uint32_t nNodes = 10; // number of nodes

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");

/**
 * \ingroup mesh
 * \brief MeshTest class
 */
class MeshTest
{
public:
  /// Init test
  MeshTest ();
  /**
   * Configure test from command line arguments
   *
   * \param argc command line argument count
   * \param argv command line arguments
   */
  void Configure (int argc, char ** argv);
  /**
   * Run test
   * \returns the test status
   */
  int Run ();
  void StartApplication();
  void ReceivePacket (Ptr<Socket> socket);
  void SchedulePing(uint32_t node, Ptr<Socket> socket);
  void ReportDelay(uint32_t node);

private:
  int       m_xSize; ///< X size
  int       m_ySize; ///< Y size
  double    m_step; ///< step
  double    m_randomStart; ///< random start
  uint32_t  m_nIfaces; ///< number interfaces
  bool      m_chan; ///< channel
  bool      m_pcap; ///< PCAP
  bool      m_ascii; ///< ASCII
  //To implement a topology different than the standard grid:
  bool      m_gridtopology;
  uint64_t  m_topoId;
  /// Hwmp related parameters
  bool      m_etxMetric;
  bool      m_enableLpp;
  int       m_sink;    ///< Sink of data Stream
  uint64_t  m_iDataRate; ///< data rate
  double    m_totalTime; ///< total time
  double    m_packetInterval; ///< packet interval
  uint16_t  m_packetSize; ///< packet size
  double   m_startTime;
  vector<double> m_ETED;
  vector<double> m_sent;
  vector<double> m_received;
  vector<double> m_sent_report;
  vector<double> m_received_report;
  double m_reportFreq;
  uint64_t m_movingNode;
  ofstream report1;
  ofstream report2;

  std::string m_stack; ///< stack
  std::string m_root; ///< root
  /// List of network nodes
  NodeContainer nodes;
  /// List of all mesh point devices
  NetDeviceContainer meshDevices;
  /// Addresses of interfaces:
  Ipv4InterfaceContainer interfaces;
  /// MeshHelper. Report is not static methods
  MeshHelper mesh;
private:
  /// Create nodes and setup their mobility
  void CreateNodes ();
  /// Install internet m_stack on nodes
  void InstallInternetStack ();
  /// Install applications
  void InstallApplication ();
  /// Print mesh devices diagnostics
  void Report ();
};
MeshTest::MeshTest () :
  m_xSize (4),
  m_ySize (5),
  m_step (100.0),
  m_randomStart (0.1),
  m_nIfaces (1),
  m_chan (true),
  m_pcap (false),
  m_ascii (false),
  m_gridtopology (false),
  m_topoId (0),
  m_etxMetric (false),
  m_enableLpp (true),
  m_iDataRate (500),
  m_totalTime (103),
  m_packetInterval (0.03),
  m_packetSize (1024),
  m_startTime (3.0),
  m_ETED (numberOfNodes, 0.0),
  m_sent (numberOfNodes, 0.0),
  m_received (numberOfNodes, 0.0),
  m_sent_report (numberOfNodes, 0.0),
  m_received_report (numberOfNodes, 0.0),
  m_reportFreq (1),
  m_movingNode (0),
  m_stack ("ns3::Dot11sStack"),
  //m_root ("ff:ff:ff:ff:ff:ff")
  m_root ("00:00:00:00:00:01")
{
  m_sink = 2;
}
void
MeshTest::Configure (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.AddValue ("x-size", "Number of nodes in a row grid", m_xSize);
  cmd.AddValue ("y-size", "Number of rows in a grid", m_ySize);
  cmd.AddValue ("step",   "Size of edge in our grid (meters)", m_step);
  // Avoid starting all mesh nodes at the same time (beacons may collide)
  cmd.AddValue ("start",  "Maximum random start delay for beacon jitter (sec)", m_randomStart);
  cmd.AddValue ("time",  "Simulation time (sec)", m_totalTime);
  cmd.AddValue ("packet-interval",  "Interval between packets in UDP ping (sec)", m_packetInterval);
  cmd.AddValue ("packet-size",  "Size of packets in UDP ping (bytes)", m_packetSize);
  cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point", m_nIfaces);
  cmd.AddValue ("channels",   "Use different frequency channels for different interfaces", m_chan);
  cmd.AddValue ("pcap",   "Enable PCAP traces on interfaces", m_pcap);
  cmd.AddValue ("ascii",   "Enable Ascii traces on interfaces", m_ascii);
  cmd.AddValue ("grid", "Choice whether grid or random topology [true]", m_gridtopology);
  cmd.AddValue ("topology", "Number of topology from predefined list [0-67]", m_topoId);
  cmd.AddValue ("stack",  "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
  cmd.AddValue ("root", "Mac address of root mesh point in HWMP", m_root);
  cmd.AddValue ("etx", "Enable use of ETX Metric overriding AirTime Metric [false]", m_etxMetric);
  cmd.AddValue ("lpp", "Enable the Transmission of LPP needed to calculate the ETX Metric [false]", m_enableLpp);
  cmd.AddValue ("movingNode", "Sets which node will move [0]", m_movingNode);

  cmd.Parse (argc, argv);

  NS_LOG_DEBUG ("Grid:" << m_xSize << "*" << m_ySize);
  NS_LOG_DEBUG ("Simulation time: " << m_totalTime << " s");
  if (m_ascii)
    {
      PacketMetadata::Enable ();
    }
}
void
MeshTest::CreateNodes ()
{
  /*
   * Create m_ySize*m_xSize stations to form a grid topology
   */
  nodes.Create (numberOfNodes);
  // Configure YansWifiChannel
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Configure the parameters of the HWMP
  Config::SetDefault ("ns3::dot11s::HwmpProtocol::EtxMetric", BooleanValue (m_etxMetric));
  Config::SetDefault ("ns3::dot11s::HwmpProtocol::LinkProbePacket", BooleanValue (m_enableLpp));
  Config::SetDefault ("ns3::dot11s::HwmpProtocol::TopologyID", UintegerValue (m_topoId));
  Config::SetDefault ("ns3::dot11s::HwmpProtocol::movingNode", UintegerValue (m_movingNode));

  /*
   * Create mesh helper and set stack installer to it
   * Stack installer creates all needed protocols and install them to
   * mesh point device
   */
  mesh = MeshHelper::Default ();
  if (!Mac48Address (m_root.c_str ()).IsBroadcast ())
    {
      mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
    }
  else
    {
      //If root is not set, we do not use "Root" attribute, because it
      //is specified only for 11s
      mesh.SetStackInstaller (m_stack);
    }
  if (m_chan)
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
    }
  else
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
    }
  mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart)));
  // Set number of interfaces - default is single-interface mesh point
  mesh.SetNumberOfInterfaces (m_nIfaces);
  // Install protocols and return container if MeshPointDevices
  meshDevices = mesh.Install (wifiPhy, nodes);
  // Setup mobility - static grid topology
  MobilityHelper mobility;

  if (m_gridtopology)
  {
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (m_step),
                                   "DeltaY", DoubleValue (m_step),
                                   "GridWidth", UintegerValue (m_xSize),
                                   "LayoutType", StringValue ("RowFirst"));
  }
  else
  {
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    for (auto i : nodepositions[m_topoId])
    {
      positionAlloc->Add (Vector(i[0], i[1], i[2]));
    }
    mobility.SetPositionAllocator (positionAlloc);
  }

  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (nodes);
  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));
  if (m_ascii)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("mesh.tr"));
    }
}
void
MeshTest::InstallInternetStack ()
{
  InternetStackHelper internetStack;
  internetStack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (meshDevices);
}
void
MeshTest::InstallApplication ()
{
  Ptr<UniformRandomVariable> var1 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> var2 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> var3 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> var4 = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> var5 = CreateObject<UniformRandomVariable> ();
  double appJitter1 = var1->GetValue (0.0,0.5); // half of a second jitter
  double appJitter2 = var2->GetValue (0.0,0.5); // half of a second jitter
  double appJitter3 = var3->GetValue (0.0,0.5); // half of a second jitter
  double appJitter4 = var4->GetValue (0.0,0.5); // half of a second jitter
  double appJitter5 = var5->GetValue (0.0,0.5); // half of a second jitter

  std::ostringstream oss;
  oss <<  "10.1.1.1";
  InetSocketAddress destinationAddress = InetSocketAddress (Ipv4Address (oss.str().c_str ()), port); // destination address for source apps
  InetSocketAddress sinkReceivingAddress = InetSocketAddress (Ipv4Address::GetAny (), port); // sink nodes receive from any address

  // Source
  StatsSourceHelper sourceAppH (protocol, destinationAddress);
  sourceAppH.SetConstantRate (DataRate (rate));
  sourceAppH.SetAttribute ("PacketSize", UintegerValue(packetSize));
  ApplicationContainer sourceApps1 = sourceAppH.Install (nodes.Get (m_movingNode-1));
  sourceApps1.Start (Seconds (netStartupTime+appStartDiff+appJitter1)); // Every app starts "appStartDiff" seconds after previous one
  sourceApps1.Stop (Seconds (netStartupTime+simulationDuration+appJitter1)); // Every app stops after finishes runnig of "simulationDuration" seconds

  ApplicationContainer sourceApps2 = sourceAppH.Install (nodes.Get (4));
  sourceApps2.Start (Seconds (netStartupTime+appStartDiff+appJitter2)); // Every app starts "appStartDiff" seconds after previous one
  sourceApps2.Stop (Seconds (netStartupTime+simulationDuration+appJitter2)); // Every app stops after finishes runnig of "simulationDuration" seconds

  ApplicationContainer sourceApps3 = sourceAppH.Install (nodes.Get (8));
  sourceApps3.Start (Seconds (netStartupTime+appStartDiff+appJitter3)); // Every app starts "appStartDiff" seconds after previous one
  sourceApps3.Stop (Seconds (netStartupTime+simulationDuration+appJitter3)); // Every app stops after finishes runnig of "simulationDuration" seconds

  ApplicationContainer sourceApps4 = sourceAppH.Install (nodes.Get (7));
  sourceApps4.Start (Seconds (netStartupTime+appStartDiff+appJitter4)); // Every app starts "appStartDiff" seconds after previous one
  sourceApps4.Stop (Seconds (netStartupTime+simulationDuration+appJitter4)); // Every app stops after finishes runnig of "simulationDuration" seconds

  ApplicationContainer sourceApps5 = sourceAppH.Install (nodes.Get (6));
  sourceApps5.Start (Seconds (netStartupTime+appStartDiff+appJitter5)); // Every app starts "appStartDiff" seconds after previous one
  sourceApps5.Stop (Seconds (netStartupTime+simulationDuration+appJitter5)); // Every app stops after finishes runnig of "simulationDuration" seconds

  // Sink
  StatsSinkHelper sink (protocol, sinkReceivingAddress);
  ApplicationContainer sinkApps = sink.Install (nodes.Get (0));
  sinkApps.Start (Seconds (0.0)); // start at the begining and wait for first packet
  sinkApps.Stop (Seconds (netStartupTime+simulationDuration+1)); // stop a bit later then source to receive the last packet
}
int
MeshTest::Run ()
{
  // CreateNodes ();
  // InstallInternetStack ();
  // InstallApplication ();
  // Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);
  //
  // //Flow monitor
  // Ptr<FlowMonitor> flowMonitor;
  // FlowMonitorHelper flowHelper;
  // flowMonitor = flowHelper.InstallAll();
  //
  // Simulator::Stop (Seconds (m_totalTime));
  // Simulator::Run ();
  //
  // flowMonitor->SerializeToXmlFile("MyMeshPerformance.xml", true, true);
  //
  // Simulator::Destroy ();
  // return 0;

  CreateNodes ();
  InstallInternetStack ();
  InstallApplication ();
  // Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  StatsFlows oneRunStats (RngSeedManager::GetRun	(), m_csvFileNamePrefix, true, true); // current RngRun, file name, RunSummary to file, EveryPacket to file
  oneRunStats.SetHistResolution (0.0001); // sets resolution in seconds

  Simulator::Stop (Seconds (m_totalTime));
  Simulator::Run ();

  // Print per flow statistics
   monitor->CheckForLostPackets ();
   Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
   map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

   for (map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
   {
     Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

     if ((t.sourceAddress == Ipv4Address("10.1.1.2") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.3") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.4") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.5") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.6") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.7") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.8") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.9") && t.destinationAddress == Ipv4Address("10.1.1.1"))
       || (t.sourceAddress == Ipv4Address("10.1.1.10") && t.destinationAddress == Ipv4Address("10.1.1.1")))
     {
       std::cout<<"Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress<<"\n";
       std::cout<<"Tx Packets = " << iter->second.txPackets<<"\n";
       std::cout<<"Rx Packets = " << iter->second.rxPackets<<"\n";
       std::cout<<"Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024 /1024  << " Mbps\n";
       cout << (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeLastTxPacket.GetSeconds()) << endl;
       cout << iter->second.delaySum / iter->second.txPackets << endl;
     }
   }

  monitor->SerializeToXmlFile("MyMeshPerformance.xml", true, true);

  oneRunStats.Finalize ();
  Simulator::Destroy ();
  return 0;
}
void
MeshTest::Report ()
{
  unsigned n (0);
  for (NetDeviceContainer::Iterator i = meshDevices.Begin (); i != meshDevices.End (); ++i, ++n)
    {
      std::ostringstream os;
      os << "mp-report-" << n << ".xml";
      std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
      std::ofstream of;
      of.open (os.str ().c_str ());
      if (!of.is_open ())
        {
          std::cerr << "Error: Can't open file " << os.str () << "\n";
          return;
        }
      mesh.Report (*i, of);
      of.close ();
    }
}
int
main (int argc, char *argv[])
{
  MeshTest t;
  //LogComponentEnable("HwmpProtocol", LOG_DEBUG);
  t.Configure (argc, argv);
  return t.Run ();
}
