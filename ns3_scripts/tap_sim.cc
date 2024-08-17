#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/network-module.h"
#include "ns3/tap-bridge-module.h"

#include <fstream>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TapCsmaVirtualMachineExample");

int
main(int argc, char* argv[])
{

    int n_car = 2;

    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    NodeContainer nodes;
    nodes.Create(n_car);

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(DataRate("100Mbps")));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(25)));

    NetDeviceContainer devices = csma.Install(nodes);

    TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", StringValue("UseBridge"));

    for (int i = 0; i < n_car; i++)
    {
        std::string device_name = "tap-" + std::to_string(i);
        tapBridge.SetAttribute("DeviceName", StringValue(device_name));
        tapBridge.Install(nodes.Get(i), devices.Get(i));
    }

    NS_LOG_UNCOND("Channel is ready, start your ROS2 simulation!");

    Simulator::Stop(Hours(1.));
    Simulator::Run();   
    Simulator::Destroy();
}
