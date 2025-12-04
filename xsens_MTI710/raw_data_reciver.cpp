// Build: g++ mti710_force_log.cpp -o mti710_force_log -lusb-1.0 -O2 -pthread

#include <libusb-1.0/libusb.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <arpa/inet.h>

using u8 = unsigned char;
using u16 = unsigned short;
using u32 = unsigned int;
using namespace std::chrono;

volatile bool running = true;
static void sigint_handler(int){ running = false; }

const u16 VID = 0x2639, PID = 0x0017;
const int IFACE0 = 0, IFACE1 = 1;
const u8 EP_OUT = 0x02;
const u8 EP_INT_IN = 0x81;
const u8 EP_BULK_IN = 0x83;
const u8 PRE = 0xFA, BID = 0xFF;

static u8 xsum(const std::vector<u8>& a){
    unsigned int s=0; for (auto b : a) s+=b; return (u8)((256 - (s & 0xFF)) & 0xFF);
}

static bool verify_checksum(const u8* data, size_t len) {
    u8 sum = 0; for(size_t i=1; i<len; i++) sum += data[i]; return sum == 0;
}

static bool send_x(libusb_device_handle* h, u8 mid, const std::vector<u8>& payload){
    std::vector<u8> msg; msg.push_back(PRE); msg.push_back(BID); msg.push_back(mid);
    msg.push_back((u8)payload.size()); msg.insert(msg.end(), payload.begin(), payload.end());
    std::vector<u8> sumfrom(msg.begin()+1, msg.end()); msg.push_back(xsum(sumfrom));
    int xfer=0;
    int rc = libusb_bulk_transfer(h, EP_OUT, msg.data(), (int)msg.size(), &xfer, 500);
    if (rc != 0) std::cerr << "[TX Error] MID=" << std::hex << (int)mid << " rc=" << rc << "\n";
    return rc==0 && xfer==(int)msg.size();
}

static void hexdump(const u8* d, int n){
    for (int i=0;i<n;i++) printf("%02X ", d[i]);
    printf("\n");
}

static float be_f(const u8* b){
    u32 t = (u32)b[3] | ((u32)b[2]<<8) | ((u32)b[1]<<16) | ((u32)b[0]<<24);
    float f; memcpy(&f,&t,4); return f;
}

static u16 be_u16(const u8* b){
    return (u16)(b[1] | (b[0] << 8));
}

static void parse_mtdata2_payload(const u8* payload, size_t plen){
    int idx=0;
    while (idx + 3 <= plen){
        u16 did = be_u16(payload + idx);
        u8 dlen = payload[idx+2];
        
        if (idx + 3 + dlen > plen) break;
        const u8* d = payload + idx + 3;
        
        if (did == 0x2010 && dlen >= 16){
            printf("QUAT: %.4f %.4f %.4f %.4f  ", be_f(d), be_f(d+4), be_f(d+8), be_f(d+12));
        } else if (did == 0x4020 && dlen >= 12){
            printf("ACC: %.3f %.3f %.3f  ", be_f(d), be_f(d+4), be_f(d+8));
        } else if (did == 0x8020 && dlen >= 12){
            printf("GYRO: %.3f %.3f %.3f  ", be_f(d), be_f(d+4), be_f(d+8));
        } else if (did == 0x1020 && dlen >= 2){ 
            printf("CNT: %u  ", be_u16(d));
        }
        idx += 3 + dlen;
    }
    printf("\n");
}

int main(){
    signal(SIGINT, sigint_handler);
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != 0) return 1;

    // --- FORCE RESET ---
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!h){ fprintf(stderr,"Cannot open device (check sudo?)\n"); return 2; }
    
    libusb_set_auto_detach_kernel_driver(h, 1);
    libusb_claim_interface(h, IFACE1);

    fprintf(stderr,"[Init] Sending Restore Defaults...\n");
    send_x(h, 0x0E, {});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    fprintf(stderr,"[Init] Force USB Mode...\n");
    send_x(h, 0x86, std::vector<u8>{0x00});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    fprintf(stderr,"[Init] Resetting Device...\n");
    send_x(h, 0x24, {});
    libusb_release_interface(h, IFACE1);
    libusb_close(h);
    
    fprintf(stderr,"[Init] Waiting 2.5s for reboot...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    // --- RECONNECT & CONFIG ---
    h = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!h){ fprintf(stderr,"Device did not re-open\n"); return 3; }

    libusb_claim_interface(h, IFACE0);
    libusb_claim_interface(h, IFACE1);

    // Wakeup
    u8 wake[3] = {PRE, PRE, PRE};
    int tx=0; libusb_bulk_transfer(h, EP_OUT, wake, 3, &tx, 200);

    send_x(h, 0x30, {}); 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::vector<u8> c0;
    auto add = [&](u16 id, u16 freq){ 
        c0.push_back(id>>8); c0.push_back(id&0xFF);
        c0.push_back(freq>>8); c0.push_back(freq&0xFF);
    };
    add(0x1020, 0xFFFF); 
    add(0x2010, 100);    
    add(0x4020, 100);    
    add(0x8020, 100);    
    send_x(h, 0xC0, c0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    send_x(h, 0x10, {});
    fprintf(stderr,"[Main] Streaming data...\n");

    // --- RING BUFFER LOOP ---
    std::vector<u8> ring;
    ring.reserve(8192);
    u8 buf[4096];

    while (running){
        // Poll Bulk
        int transferred = 0;
        int rc = libusb_bulk_transfer(h, EP_BULK_IN, buf, 4096, &transferred, 50);
        
        if (rc == 0 && transferred > 0) {
            ring.insert(ring.end(), buf, buf + transferred);
            size_t pos = 0;
            while (pos + 5 <= ring.size()){
                if (ring[pos] != PRE || ring[pos+1] != BID) { pos++; continue; }

                u8 mid = ring[pos+2];
                u8 len = ring[pos+3];
                size_t hdr = 4;
                size_t dlen = len;
                if (len == 0xFF) {
                    if (pos + 6 > ring.size()) break;
                    dlen = (ring[pos+4] << 8) | ring[pos+5];
                    hdr = 6;
                }
                
                size_t pkt_len = hdr + dlen + 1; // +CS
                if (pos + pkt_len > ring.size()) break;

                if (verify_checksum(ring.data() + pos, pkt_len)) {
                    if (mid == 0x36) { 
                        parse_mtdata2_payload(ring.data() + pos + hdr, dlen);
                    }
                    pos += pkt_len;
                } else {
                    pos++;
                }
            }
            if (pos > 0) ring.erase(ring.begin(), ring.begin() + pos);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    libusb_release_interface(h, IFACE0);
    libusb_release_interface(h, IFACE1);
    libusb_close(h);
    libusb_exit(ctx);
    return 0;
}
