#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/spectrum-module.h"

#include <vector>
#include <array>
#include <iomanip>
#include <algorithm>

using namespace ns3;
using namespace ns3::lrwpan;

NS_LOG_COMPONENT_DEFINE("AdaptiveChannelSwitch_Final");

// ---------------- CONFIG ----------------
static const double EWMA_ALPHA = 0.65;
static const double HYSTERESIS = 5.0;
static const Time ED_DELAY = MilliSeconds(1);
static const Time SCAN_PERIOD = Seconds(5.0);

static std::vector<uint8_t> PREFERRED = {15, 20, 25};

// ---------------- GLOBAL STATE ----------------
static NetDeviceContainer zigbeeDevices;
static Ptr<LrWpanPhy> scannerPhy;

static std::vector<uint8_t> channels;
static std::array<double, 16> score;
static std::array<uint32_t, 16> samples;

static uint8_t currentChannel = 11;
static uint32_t scanIndex = 0;
static uint8_t scanChannel;

static bool scanning = false;
static Time scanStart;
static bool switched = false;

static Ptr<PacketSink> zigbeeSink;
static Ptr<PacketSink> wifiSink;

// ---------------- UTILS ----------------
inline size_t idx(uint8_t ch) { return ch - 11; }

bool isPreferred(uint8_t ch)
{
    return std::find(PREFERRED.begin(), PREFERRED.end(), ch) != PREFERRED.end();
}

void setChannel(Ptr<LrWpanPhy> phy, uint8_t ch)
{
    Ptr<PhyPibAttributes> pib = Create<PhyPibAttributes>();
    pib->phyCurrentChannel = ch;
    pib->phyCurrentPage = 0;

    phy->PlmeSetAttributeRequest(phyCurrentChannel, pib);
    phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
}

void applyChannel(uint8_t ch)
{
    for (uint32_t i = 0; i < zigbeeDevices.GetN(); i++)
    {
        auto dev = DynamicCast<LrWpanNetDevice>(zigbeeDevices.Get(i));
        setChannel(dev->GetPhy(), ch);
    }

    setChannel(scannerPhy, ch);
    currentChannel = ch;

    NS_LOG_UNCOND("\n[SWITCH] t=" << Simulator::Now().GetSeconds()
        << "s → channel " << (int)ch << "\n");
}

// ---------------- LOGGING ----------------
void PrintSummary()
{
    NS_LOG_UNCOND("\n========== CHANNEL SCAN SUMMARY ==========");

    for (uint8_t ch : channels)
    {
        size_t i = idx(ch);
        if (samples[i] == 0) continue;

        NS_LOG_UNCOND("CH " << std::setw(2) << (int)ch
            << " | Score=" << std::setw(6) << std::setprecision(2) << score[i]
            << " | Samples=" << samples[i]
            << " | Preferred=" << (isPreferred(ch) ? "YES" : "NO"));
    }

    NS_LOG_UNCOND("==========================================\n");
}

// ---------------- DECISION ----------------
void chooseBestChannel()
{
    PrintSummary();

    double bestScore = 1e9;
    uint8_t best = currentChannel;

    for (uint8_t ch : channels)
    {
        size_t i = idx(ch);
        if (samples[i] == 0) continue;

        double s = score[i];

        if (!isPreferred(ch)) s += 8.0;

        if (s < bestScore)
        {
            bestScore = s;
            best = ch;
        }
    }

    size_t ci = idx(currentChannel);
    double curScore = score[ci];

    if ((curScore - bestScore) < HYSTERESIS)
        return;

    NS_LOG_UNCOND("[DECISION]");
    NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
    NS_LOG_UNCOND("Current Channel: " << (int)currentChannel);
    NS_LOG_UNCOND("Selected Channel: " << (int)best);

    if (!switched)
    {
        switched = true;
        NS_LOG_UNCOND("[METRIC] SwitchingLatency="
            << (Simulator::Now() - scanStart).GetSeconds()
            << " sec\n");
    }

    applyChannel(best);
}

// ---------------- SCAN ----------------
void ContinueScan();

void OnEdConfirm(PhyEnumeration status, uint8_t energy)
{
    if (!scanning) return;

    size_t i = idx(scanChannel);
    double sample = energy;

    if (samples[i] == 0)
        score[i] = sample;
    else
        score[i] = EWMA_ALPHA * sample + (1 - EWMA_ALPHA) * score[i];

    samples[i]++;

    NS_LOG_UNCOND("[SCAN] t=" << std::fixed << std::setprecision(3)
        << Simulator::Now().GetSeconds() << "s"
        << " | CH=" << std::setw(2) << (int)scanChannel
        << " | ED=" << std::setw(3) << (int)energy
        << " | EWMA=" << std::setw(6) << std::setprecision(2) << score[i]
        << " | samples=" << samples[i]);

    Simulator::ScheduleNow(&ContinueScan);
}

void ContinueScan()
{
    if (scanIndex >= channels.size())
    {
        scanning = false;
        scanIndex = 0;

        chooseBestChannel();

        Simulator::Schedule(SCAN_PERIOD, [](){
            scanning = true;
            ContinueScan();
        });
        return;
    }

    scanChannel = channels[scanIndex++];
    setChannel(scannerPhy, scanChannel);

    Simulator::Schedule(ED_DELAY,
        &LrWpanPhy::PlmeEdRequest, scannerPhy);
}

void startScan()
{
    scanStart = Simulator::Now();
    scanning = true;

    NS_LOG_UNCOND("\n[INFO] Starting adaptive ED scan\n");
    ContinueScan();
}

// ---------------- THROUGHPUT ----------------
void logThroughput()
{
    static uint64_t prevZ = 0, prevW = 0;
    static Time prevT = Seconds(0);

    Time now = Simulator::Now();

    uint64_t curZ = zigbeeSink->GetTotalRx();
    uint64_t curW = wifiSink->GetTotalRx();

    if (prevT > Seconds(0))
    {
        double dt = (now - prevT).GetSeconds();

        double zRate = (curZ - prevZ) * 8.0 / dt / 1e6;
        double wRate = (curW - prevW) * 8.0 / dt / 1e6;

        NS_LOG_UNCOND("[THROUGHPUT] t=" << now.GetSeconds()
            << "s | Zigbee=" << std::setw(6) << std::setprecision(3) << zRate
            << " Mbps | WiFi=" << std::setw(6) << wRate << " Mbps");
    }

    prevZ = curZ;
    prevW = curW;
    prevT = now;

    Simulator::Schedule(Seconds(1.0), &logThroughput);
}

// ---------------- MAIN ----------------
int main()
{
    for (uint8_t ch = 11; ch <= 26; ch++)
        channels.push_back(ch);

    Ptr<MultiModelSpectrumChannel> channel =
        CreateObject<MultiModelSpectrumChannel>();

    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    channel->AddPropagationLossModel(CreateObject<FriisPropagationLossModel>());

    NodeContainer zigbeeNodes;
    zigbeeNodes.Create(11);

    NodeContainer scannerNode;
    scannerNode.Create(1);

    NodeContainer wifiNodes;
    wifiNodes.Create(7);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.InstallAll();

    // Zigbee
    LrWpanHelper lr;
    lr.SetChannel(channel);

    zigbeeDevices = lr.Install(zigbeeNodes);
    auto scanDev = lr.Install(scannerNode);

    lr.CreateAssociatedPan(zigbeeDevices, 1);

    auto sdev = DynamicCast<LrWpanNetDevice>(scanDev.Get(0));
    scannerPhy = sdev->GetPhy();
    scannerPhy->SetPlmeEdConfirmCallback(MakeCallback(&OnEdConfirm));

    // 6LoWPAN
    SixLowPanHelper six;
    auto lowpan = six.Install(zigbeeDevices);

    // Internet
    InternetStackHelper stack;
    stack.Install(zigbeeNodes);
    stack.Install(wifiNodes);

    // Addressing
    Ipv6AddressHelper ipv6;
    ipv6.SetBase(Ipv6Address("2001:1::"), Ipv6Prefix(64));
    auto iface = ipv6.Assign(lowpan);

    // Zigbee sink
    PacketSinkHelper sinkZ("ns3::UdpSocketFactory",
        Inet6SocketAddress(Ipv6Address::GetAny(), 4000));

    auto sinkApp = sinkZ.Install(zigbeeNodes.Get(0));
    sinkApp.Start(Seconds(1.0));

    zigbeeSink = DynamicCast<PacketSink>(sinkApp.Get(0));

    // Zigbee traffic
    OnOffHelper tx("ns3::UdpSocketFactory",
        Inet6SocketAddress(iface.GetAddress(0,1), 4000));

    tx.SetAttribute("DataRate", StringValue("2Mbps"));
    tx.Install(zigbeeNodes.Get(1)).Start(Seconds(1.2));

    // WiFi
    SpectrumWifiPhyHelper phy;
    phy.SetChannel(channel);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    auto wifiDev = wifi.Install(phy, mac, wifiNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    auto wiface = ipv4.Assign(wifiDev);

    // WiFi sink
    PacketSinkHelper sinkW("ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), 9));

    auto wifiSinkApp = sinkW.Install(wifiNodes.Get(0));
    wifiSinkApp.Start(Seconds(1.0));

    wifiSink = DynamicCast<PacketSink>(wifiSinkApp.Get(0));

    // WiFi traffic
    OnOffHelper wifiTx("ns3::UdpSocketFactory",
        InetSocketAddress(wiface.GetAddress(0), 9));

    wifiTx.SetConstantRate(DataRate("50Mbps"));
    wifiTx.Install(wifiNodes.Get(1)).Start(Seconds(2.0));

    // Start
    Simulator::Schedule(Seconds(2.2), &startScan);
    Simulator::Schedule(Seconds(2.0), &logThroughput);

    Simulator::Stop(Seconds(10.0));
    Simulator::Run();

    NS_LOG_UNCOND("\n========== FINAL SUMMARY ==========");
    NS_LOG_UNCOND("Zigbee Bytes: " << zigbeeSink->GetTotalRx());
    NS_LOG_UNCOND("WiFi Bytes: " << wifiSink->GetTotalRx());
    NS_LOG_UNCOND("Final Channel: " << (int)currentChannel);
    NS_LOG_UNCOND("===================================\n");

    Simulator::Destroy();
}
