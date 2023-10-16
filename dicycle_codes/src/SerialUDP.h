#ifndef __SERIAL_UDP_H__
#define __SERIAL_UDP_H__

#include <WiFi.h>
#include <AsyncUDP.h>
#include "utils.h"

class SerialUDP {
public:
    SerialUDP(int port = 8080) : _packet(NULL) {
        _port = port;
    }

    void setup() {
        WiFi.mode(WIFI_STA);
        WiFi.begin("TJ's House", "cafebabe12");
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            LOG(".");
        }
        LOG("\nIP:%s\n", WiFi.localIP().toString().c_str());

        if (_udp.listen(_port)) {
            _udp.onPacket([](void *arg, AsyncUDPPacket packet) {
                LOG("UDP Packet Type: %s, From: %s:%d, To: %s:%d, Length: %d\n", packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast",
                    packet.remoteIP().toString().c_str(), packet.remotePort(), packet.localIP().toString().c_str(), packet.localPort(), packet.length());
                SerialUDP *pThis = (SerialUDP*)arg;
                pThis->set(packet); }, this);
        }
    }

    void set(AsyncUDPPacket p) {
        if (_packet) {
            delete _packet;
        }
        _packet = new AsyncUDPPacket(p);
    }

    int available() {
        return _packet ? _packet->available() : 0;
    }

    int read() {
        return _packet ? _packet->read() : -1;
    }

    void log(const char *format, ...) {
        if (_packet) {
            va_list		argp;
            char		szBuf[1024];

            va_start(argp, format);
            int len = sprintf(szBuf, format, argp);
            va_end(argp);
            _packet->write((uint8_t*)szBuf, len);
        }
    }

private:
    AsyncUDP        _udp;
    int             _port;
    AsyncUDPPacket  *_packet;
};

#endif