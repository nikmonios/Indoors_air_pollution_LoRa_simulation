/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/random-waypoint-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/constant-velocity-helper.h"

#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>
#include <ns3/buildings-module.h>
#include <ns3/buildings-propagation-loss-model.h>
#include <time.h>
#include "ns3/netanim-module.h"

//#include "ns3/delay-jitter-estimation.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 700;  //100 per floor, 7 floors
int nGateways = 1;
double radius = 1000; // not used in this code
double simulationTime = 3600 * 24; //=24 hours in seconds

// Channel model
bool realisticChannelModel = true;
int appPeriodSeconds = 0;

// Output control
bool print = true;


int main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("radius", "The radius of the area to simulate", radius);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue ("print", "Whether or not to print various informations", print);
  cmd.Parse (argc, argv);

  /******************
  * Set Data Rates *
  * and SF         *
  ******************/
  // 0 = SF12
  // 1 = SF11
  // 2 = SF10
  // 3 = SF9
  // 4 = SF8
  // 5 = SF7
  int app_payload = 20; //init LoRa payload.

  Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (5));

  appPeriodSeconds = 5*60; // 5 minutes in seconds


  // Set up logging
  //LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_ALL); //useful !!
  //LogComponentEnable("LoraChannel", LOG_LEVEL_ALL); // useful !!

  //LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
  //LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  //LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
  //LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  //LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
  //LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  //LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  //LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
  //LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  //LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);

  //LogComponentEnable("LoraHelper", LOG_LEVEL_ALL); // useful !!
  //LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL); //useful !!

  //LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  //LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
  //LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
  //LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
  //LogComponentEnable("LorawanMacHeader", LOG_LEVEL_INFO);
  //LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
  //LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);

  //LogComponentEnable("NetworkServer", LOG_LEVEL_ALL); //Need it

  //LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
  //LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

  /***********
   *  Setup  *
   ***********/

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // mobility_nodes
  MobilityHelper mobility_nodes; // node will be randomly placed in the XYZ, close to (0,0) - and then will move
  MobilityHelper mobility_gateways; //gateway will be placed on (0,0)

  /* nodes initially will spawn 2 meters around the 0,0 point, randomly*/
  mobility_nodes.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                     "X", StringValue ("5.0"),
                                     "Y", StringValue ("5.0"),
                                     "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=2]"));

  /* nodes will not move from their position */
  mobility_nodes.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  /* the gateways will remain stable */
  /* Set its positions manually */
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  allocator->Add (Vector (50,50,23)); // x_gw = 50, y_gw = 50, z_gw = 23
  mobility_gateways.SetPositionAllocator (allocator);
  mobility_gateways.SetMobilityModel ("ns3::ConstantPositionMobilityModel");


  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (2.3); // was 3.76
  loss->SetReference (1, 42); // was 1, 7.7

  if (realisticChannelModel)
    {
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
          CreateObject<CorrelatedShadowingPropagationLossModel> ();

      // Aggregate shadowing to the logdistance loss
      loss->SetNext (shadowing);

      // Add the effect to the channel propagation loss
      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      shadowing->SetNext (buildingLoss);
    }

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (); // Output filename
  // helper.EnableSimulationTimePrinting ();

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility_nodes model to each node
  mobility_nodes.Install (endDevices);

  srand( (unsigned)time(NULL) );

  int iter_x = 0; //rooms counter x
  int iter_y = 0; //rooms counter y
  int iter_z = 0; //floor counter

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
  {
	  Ptr<ConstantPositionMobilityModel> mob_node = (*j)->GetObject<ConstantPositionMobilityModel> ();
	  Vector position_node = mob_node->GetPosition();

	  position_node.x = (10 * iter_x) + 5;
	  position_node.y = (10 * iter_y) + 5;
	  position_node.z = (3  * iter_z) + 1.2;

	  mob_node->SetPosition(position_node);

	  iter_x++; //inc rooms counter x

	  if(iter_x == 10) //when we have done 10 rooms on x axis, change row
	  {
		  iter_y++; // changer raw
		  iter_x = 0; //reset counter
	  }

	  if(iter_y == 10) //when we have done all 10 rows, we need to change floor
	  {
		  iter_z++; //changed floor
		  iter_x = 0; //reset counter
		  iter_y = 0; //reset counter
	  }
  }

  /* print nodes coords */
  std::ofstream myfile2;
  myfile2.open ("nodes_coords_peladarinos_1.txt"); //name of file that nodes coords will be written
  int index = 0; //init index

  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
  {
	  Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
	  Vector position = mobility->GetPosition ();
	  //Vector node_speed = mobility->GetVelocity();

	  /* write coords of the node now */
	  myfile2<< "index is: " << index << ", Xpos: " << position.x << ", Ypos: "
			  << position.y << ", Zpos: " << position.z << std::endl;
			  //" , X_vel: " << node_speed.x << " , Y_vel: " << node_speed.y << std::endl;

	  /* update index */
	  index++;
  }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    }

  /*********************
   *  Create Gateways  *
   *********************/

  // Create the stationary gateway nodes
  NodeContainer gateways;
  gateways.Create (nGateways);

  //install them on the grid
  mobility_gateways.Install (gateways);

  /* print the coords of the gateays */
  std::ofstream myfile3;
  myfile3.open ("gateways_coords_scenario1_peladarinos.txt"); //name of file that nodes coords will be written
  index = 0; //init index

  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
  {
  	  Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
  	  Vector position = mobility->GetPosition ();

  	  /* write coords of the node now */
  	  myfile3<< "index is: " << index << ", Xpos: " << position.x << ", Ypos: "
  			  << position.y << ", Zpos: " << position.z << std::endl;

  	  /* update index */
  	  index++;
  }


  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /**********************
   *  Handle buildings  *
   **********************/
  Ptr<GridBuildingAllocator>  gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (3));
  gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (100)); // length of building
  gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (100)); // width of building
  gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (7.5));
  gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (7.5));
  gridBuildingAllocator->SetAttribute ("Height", DoubleValue (21)); // height of building
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (10));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (10));
  gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (7));

  gridBuildingAllocator->SetBuildingAttribute ("ExternalWallsType", EnumValue(1)); //0=wood, 1=concrete w windows, 2=concrete no window, 3=brick
  gridBuildingAllocator->SetBuildingAttribute ("Type", EnumValue(2)); //0=residency, 1=office, 2=commercial

  gridBuildingAllocator->SetAttribute ("MinX", DoubleValue (0));
  gridBuildingAllocator->SetAttribute ("MinY", DoubleValue (0));
  //gridBuildingAllocator->Create (2);
  BuildingContainer bContainer = gridBuildingAllocator->Create (2);

  BuildingsHelper::Install (endDevices);
  BuildingsHelper::Install (gateways);

   // Print the buildings
   if (print)
   {
      std::ofstream myfile;
      myfile.open ("peladarinos_Buildings_1.txt");
      std::vector<Ptr<Building>>::const_iterator it;
      int j = 1;
      for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
      {
         Box boundaries = (*it)->GetBoundaries ();
          myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin
                 << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
      }
      myfile.close ();
    }

  /**********************************************
   *  Set up the end device's spreading factor  *
   **********************************************/

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  appHelper.SetPacketSize (app_payload); //max is 255 (uint_8t)
  Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
      "Min", DoubleValue (0), "Max", DoubleValue (10));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);


  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  ////////////////
  // Simulation //
  ////////////////

  //AnimationInterface::SetConstantPosition(endDevices.Get(0), 10,30);
  /*AnimationInterface anim ("my-animation.xml"); // Mandatory
  for (uint32_t i = 0; i < endDevices.GetN (); ++i)
  {
	anim.UpdateNodeDescription (endDevices.Get (i), "NODE"); // Optional
	anim.UpdateNodeColor (endDevices.Get (i), 255, 0, 0); // Optional
  }
  for (uint32_t i = 0; i < gateways.GetN (); ++i)
  {
  	anim.UpdateNodeDescription (gateways.Get (i), "GW"); // Optional
  	anim.UpdateNodeColor (gateways.Get (i), 255, 255, 94); // Optional
  }
  anim.EnablePacketMetadata();
  anim.GetTracePktCount();
*/


  Simulator::Stop (appStopTime);// + Hours (1));

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  ///////////////////////////
  // Print results to file //
  ///////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");

  LoraPacketTracker &tracker = helper.GetPacketTracker ();
  std::cout << tracker.CountMacPacketsGlobally (Seconds (0), appStopTime /*+ Hours (1)*/) << std::endl;
  //std::cout << tracker.CountMacPacketsGloballyCpsr(Seconds (0), appStopTime + Hours (1)) << std::endl;
  std::cout << tracker.PrintPhyPacketsPerGw (Seconds (0), appStopTime /*+ Hours (1)*/, nDevices) << std::endl;
  std::cout << "packets sent " << " received " << " interfered " << " no more receivers " << " under sensitivity " << " lost because TX " << std::endl;

  return 0;
}
