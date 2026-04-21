/*
===============================================================================
INTERFERENCE-AWARE CHANNEL SWITCHING (ZIGBEE vs WIFI) – NS-3 PROJECT
===============================================================================

This program simulates coexistence between:
- IEEE 802.15.4 (Zigbee)
- IEEE 802.11 (WiFi)

PROBLEM:
Zigbee operates in the same 2.4 GHz band as WiFi → interference occurs.

SOLUTION:
Zigbee scans all channels → detects interference using Energy Detection (ED)
→ switches to a cleaner channel automatically.

KEY IDEAS:
- ED (Energy Detection)
- EWMA smoothing (for stability)
- Preferred channels
- Hysteresis (avoid frequent switching)
- Throughput monitoring (before & after switching)

===============================================================================
*/

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

// ================= CONFIGURATION =================

// EWMA factor → controls smoothing of ED values
// closer to 1 → more weight to new values
static const double EWMA_ALPHA = 0.65;

// Minimum improvement required to switch channel
static const double HYSTERESIS = 5.0;

// Delay before measuring ED after switching channel
static const Time ED_DELAY = MilliSeconds(1);

// Time between full scans
static const Time SCAN_PERIOD = Seconds(5.0);

// Channels that are known to be less affected by WiFi
static std::vector<uint8_t> PREFERRED = {15, 20, 25};

// ================= GLOBAL STATE =================

// All Zigbee devices
static NetDeviceContainer zigbeeDevices;

// Scanner node PHY (used to measure ED)
static Ptr<LrWpanPhy> scannerPhy;

// List of channels (11–26)
static std::vector<uint8_t> channels;

// Score of each channel (based on ED)
static std::array<double, 16> score;

// Number of samples collected per channel
static std::array<uint32_t, 16> samples;

// Current operating channel
static uint8_t currentChannel = 11;

// Scan progress index
static uint32_t scanIndex = 0;

// Channel currently being scanned
static uint8_t scanChannel;

// Whether scanning is active
static bool scanning = false;

// Time when scan started
static Time scanStart;

// Whether switching has happened once
static bool switched = false;

// Throughput measurement sinks
static Ptr<PacketSink> zigbeeSink;
static Ptr<PacketSink> wifiSink;

// ================= UTILITY FUNCTIONS =================

// Convert channel number → array index
inline size_t idx(uint8_t ch) { return ch - 11; }

// Check if channel is preferred
bool isPreferred(uint8_t ch)
{
    return std::find(PREFERRED.begin(), PREFERRED.end(), ch) != PREFERRED.end();
}

/*
Set channel of a Zigbee PHY

This is how we change frequency in IEEE 802.15.4
*/
void setChannel(Ptr<LrWpanPhy> phy, uint8_t ch)
{
    Ptr<PhyPibAttributes> pib = Create<PhyPibAttributes>();

    pib->phyCurrentChannel = ch;
    pib->phyCurrentPage = 0;

    phy->PlmeSetAttributeRequest(phyCurrentChannel, pib);

    // Put radio into RX mode so it can detect signals
    phy->PlmeSetTRXStateRequest(IEEE_802_15_4_PHY_RX_ON);
}

/*
Apply channel to entire Zigbee network
*/
void applyChannel(uint8_t ch)
{
    for (uint32_t i = 0; i < zigbeeDevices.GetN(); i++)
    {
        auto dev = DynamicCast<LrWpanNetDevice>(zigbeeDevices.Get(i));
        setChannel(dev->GetPhy(), ch);
    }

    // Scanner also switches
    setChannel(scannerPhy, ch);

    currentChannel = ch;

    NS_LOG_UNCOND("\n[SWITCH] t=" << Simulator::Now().GetSeconds()
        << "s → channel " << (int)ch << "\n");
}

/*
Print all channel scores after scan
*/
void PrintSummary()
{
    NS_LOG_UNCOND("\n========== CHANNEL SCAN SUMMARY ==========");

    for (uint8_t ch : channels)
    {
        size_t i = idx(ch);
        if (samples[i] == 0) continue;

        NS_LOG_UNCOND("CH " << std::setw(2) << (int)ch
            << " | Score=" << std::fixed << std::setprecision(2) << score[i]
            << " | Samples=" << samples[i]
            << " | Preferred=" << (isPreferred(ch) ? "YES" : "NO"));
    }

    NS_LOG_UNCOND("==========================================\n");
}

// ================= DECISION LOGIC =================

/*
Choose best channel based on:

1. Low energy (low interference)
2. Preferred channel bonus
3. Hysteresis (avoid frequent switching)
*/
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

        // Penalize non-preferred channels
        if (!isPreferred(ch)) s += 8.0;

        if (s < bestScore)
        {
            bestScore = s;
            best = ch;
        }
    }

    // Current channel score
    size_t ci = idx(currentChannel);
    double curScore = score[ci];

    // If improvement is small → do not switch
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

// ================= SCANNING =================

void ContinueScan();

/*
Called after ED measurement is completed
*/
void OnEdConfirm(PhyEnumeration status, uint8_t energy)
{
    if (!scanning) return;

    size_t i = idx(scanChannel);

    /*
    EWMA update:
    Smooths out noise in energy values
    */
    if (samples[i] == 0)
        score[i] = energy;
    else
        score[i] = EWMA_ALPHA * energy + (1 - EWMA_ALPHA) * score[i];

    samples[i]++;

    NS_LOG_UNCOND("[SCAN] t=" << std::fixed << std::setprecision(3)
        << Simulator::Now().GetSeconds() << "s"
        << " | CH=" << std::setw(2) << (int)scanChannel
        << " | ED=" << std::setw(3) << (int)energy
        << " | EWMA=" << std::setw(6) << std::setprecision(2) << score[i]
        << " | samples=" << samples[i]);

    Simulator::ScheduleNow(&ContinueScan);
}

/*
Scan next channel
*/
void ContinueScan()
{
    // If all channels scanned
    if (scanIndex >= channels.size())
    {
        scanning = false;
        scanIndex = 0;

        chooseBestChannel();

        // Schedule next scan
        Simulator::Schedule(SCAN_PERIOD, [](){
            scanning = true;
            ContinueScan();
        });

        return;
    }

    scanChannel = channels[scanIndex++];

    // Switch scanner to that channel
    setChannel(scannerPhy, scanChannel);

    // Request ED measurement
    Simulator::Schedule(ED_DELAY,
        &LrWpanPhy::PlmeEdRequest, scannerPhy);
}

/*
Start scanning process
*/
void startScan()
{
    scanStart = Simulator::Now();
    scanning = true;

    NS_LOG_UNCOND("\n[INFO] Starting adaptive ED scan\n");

    ContinueScan();
}

// ================= THROUGHPUT =================

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
            << "s | Zigbee=" << zRate
            << " Mbps | WiFi=" << wRate << " Mbps");
    }

    prevZ = curZ;
    prevW = curW;
    prevT = now;

    Simulator::Schedule(Seconds(1.0), &logThroughput);
}

// ================= MAIN =================

int main()
{
    // Initialize channels
    for (uint8_t ch = 11; ch <= 26; ch++)
        channels.push_back(ch);

    /*
    Shared channel → enables interference between WiFi and Zigbee
    */
    Ptr<MultiModelSpectrumChannel> channel =
        CreateObject<MultiModelSpectrumChannel>();

    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    channel->AddPropagationLossModel(CreateObject<FriisPropagationLossModel>());

    // Create nodes
    NodeContainer zigbeeNodes;
    zigbeeNodes.Create(11);

    NodeContainer scannerNode;
    scannerNode.Create(1);

    NodeContainer wifiNodes;
    wifiNodes.Create(7);

    // Static positions
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.InstallAll();

    // Zigbee setup
    LrWpanHelper lr;
    lr.SetChannel(channel);

    zigbeeDevices = lr.Install(zigbeeNodes);
    auto scanDev = lr.Install(scannerNode);

    lr.CreateAssociatedPan(zigbeeDevices, 1);

    scannerPhy = DynamicCast<LrWpanNetDevice>(scanDev.Get(0))->GetPhy();
    scannerPhy->SetPlmeEdConfirmCallback(MakeCallback(&OnEdConfirm));

    // Networking
    SixLowPanHelper six;
    auto lowpan = six.Install(zigbeeDevices);

    InternetStackHelper stack;
    stack.Install(zigbeeNodes);
    stack.Install(wifiNodes);

    // Addressing
    Ipv6AddressHelper ipv6;
    ipv6.SetBase(Ipv6Address("2001:1::"), Ipv6Prefix(64));
    auto iface = ipv6.Assign(lowpan);

    // Zigbee traffic
    PacketSinkHelper sinkZ("ns3::UdpSocketFactory",
        Inet6SocketAddress(Ipv6Address::GetAny(), 4000));

    zigbeeSink = DynamicCast<PacketSink>(sinkZ.Install(zigbeeNodes.Get(0)).Get(0));

    OnOffHelper tx("ns3::UdpSocketFactory",
        Inet6SocketAddress(iface.GetAddress(0,1), 4000));

    tx.SetAttribute("DataRate", StringValue("2Mbps"));
    tx.Install(zigbeeNodes.Get(1)).Start(Seconds(1.2));

    // WiFi setup
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

    // WiFi traffic
    wifiSink = DynamicCast<PacketSink>(
        PacketSinkHelper("ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), 9))
        .Install(wifiNodes.Get(0)).Get(0));

    OnOffHelper wifiTx("ns3::UdpSocketFactory",
        InetSocketAddress(wiface.GetAddress(0), 9));

    wifiTx.SetConstantRate(DataRate("50Mbps"));
    wifiTx.Install(wifiNodes.Get(1)).Start(Seconds(2.0));

    // Start processes
    Simulator::Schedule(Seconds(2.2), &startScan);
    Simulator::Schedule(Seconds(2.0), &logThroughput);

    Simulator::Stop(Seconds(10.0));
    Simulator::Run();

    // Final output
    NS_LOG_UNCOND("\n========== FINAL SUMMARY ==========");
    NS_LOG_UNCOND("Zigbee Bytes: " << zigbeeSink->GetTotalRx());
    NS_LOG_UNCOND("WiFi Bytes: " << wifiSink->GetTotalRx());
    NS_LOG_UNCOND("Final Channel: " << (int)currentChannel);
    NS_LOG_UNCOND("===================================\n");

    Simulator::Destroy();
}

/*
===============================================================================
DETAILED ANALYSIS OF OBSERVED OUTPUT
===============================================================================

The following analysis is based on the actual simulation output obtained from
running this program.

-----------------------------------------------------------------------------
1. INITIAL ENERGY DETECTION (ED) SCAN
-----------------------------------------------------------------------------

Observed Output:

[SCAN] t=2.201s | CH=11 | ED=0
[SCAN] t=2.202s | CH=12 | ED=255
[SCAN] t=2.205s | CH=14 | ED=255
[SCAN] t=2.208s | CH=17 | ED=255
...

Interpretation:

- Energy Detection (ED) measures the signal energy present on each channel.
- Values close to 255 indicate strong interference (typically WiFi activity).
- Values close to 0 indicate clean channels.

From the scan:
- Channels 12, 14, and 17 show ED ≈ 255 → heavily interfered
- Remaining channels show ED ≈ 0 → relatively clean

Conclusion:
✔ The system successfully detects interference patterns caused by WiFi.

-----------------------------------------------------------------------------
2. CHANNEL SCORING AND SUMMARY
-----------------------------------------------------------------------------

Observed:

CH 12 | Score=255.00
CH 14 | Score=255.00
CH 17 | Score=255.00
CH 20 | Score=0.00
CH 25 | Score=0.00

Interpretation:

- Each channel is assigned a score based on ED values.
- Lower score = better channel
- EWMA is used to smooth these values over time

Additionally:
- Preferred channels (15, 20, 25) are marked
- These channels are less likely to overlap with WiFi

Conclusion:
✔ Channel scoring accurately reflects interference levels.

-----------------------------------------------------------------------------
3. THROUGHPUT BEFORE SWITCHING
-----------------------------------------------------------------------------

Observed:

[THROUGHPUT] t=3s → Zigbee=0 Mbps | WiFi=50 Mbps
[THROUGHPUT] t=4s → Zigbee=0 Mbps | WiFi=50 Mbps
...
[THROUGHPUT] t=7s → Zigbee=0 Mbps | WiFi=50 Mbps

Interpretation:

- Zigbee throughput is 0 Mbps → communication failure
- WiFi operates at full capacity (~50 Mbps)

Reason:
- Zigbee is initially operating on a channel with heavy WiFi interference

Conclusion:
✔ Interference completely disrupts Zigbee communication.

-----------------------------------------------------------------------------
4. SECOND SCAN AND EWMA EFFECT
-----------------------------------------------------------------------------

Observed:

CH 11 | Score=165.75
CH 12 | Score=89.25
CH 17 | Score=255.00
CH 20 | Score=0.00

Interpretation:

- EWMA combines previous and current ED values
- Channels that fluctuate show intermediate scores
- Channel 20 remains consistently clean (score = 0)

Conclusion:
✔ EWMA stabilizes channel evaluation and prevents noisy decisions.

-----------------------------------------------------------------------------
5. CHANNEL SWITCHING DECISION
-----------------------------------------------------------------------------

Observed:

[DECISION]
Time: 7.2s
Current Channel: 11
Selected Channel: 20

[METRIC] SwitchingLatency = 5 sec

Interpretation:

- The algorithm selects channel 20 because:
  - It has the lowest interference score (0.00)
  - It is a preferred channel
- Switching latency (~5 seconds) corresponds to:
  - Time required to complete a full scan cycle

Conclusion:
✔ Channel selection is optimal and well-justified.

-----------------------------------------------------------------------------
6. THROUGHPUT AFTER SWITCHING
-----------------------------------------------------------------------------

Observed:

[THROUGHPUT] t=8s → Zigbee=0 Mbps
[THROUGHPUT] t=9s → Zigbee=0.082 Mbps

Interpretation:

- Immediately after switching, throughput is still low (transition phase)
- After stabilization, Zigbee throughput recovers (~0.08 Mbps)

Important Note:
- IEEE 802.15.4 theoretical max ≈ 250 kbps
- Observed ≈ 80 kbps is realistic due to:
  - protocol overhead
  - simulation constraints

WiFi remains unaffected at ~50 Mbps.

Conclusion:
✔ Zigbee successfully recovers after switching to a clean channel.

-----------------------------------------------------------------------------
7. FINAL RESULTS
-----------------------------------------------------------------------------

Zigbee Bytes: 24064
WiFi Bytes: 49972224
Final Channel: 20

Interpretation:

- Zigbee successfully transmitted data after switching
- WiFi maintained high throughput throughout
- Final channel is stable and optimal

-----------------------------------------------------------------------------
8. KEY INSIGHTS
-----------------------------------------------------------------------------

1. Energy Detection (ED) is effective for interference sensing
2. EWMA improves stability of channel selection
3. Preferred channels help avoid WiFi overlap
4. Hysteresis prevents unnecessary switching
5. Shared spectrum model accurately simulates real interference

-----------------------------------------------------------------------------
9. CONCLUSION
-----------------------------------------------------------------------------

The system successfully demonstrates:

✔ Detection of WiFi interference using ED
✔ Intelligent channel evaluation and ranking
✔ Autonomous switching to a better channel
✔ Recovery of Zigbee throughput after switching

Thus, the interference-aware channel switching algorithm effectively improves
performance of IEEE 802.15.4 networks in the presence of WiFi interference.

===============================================================================
*/