#include "libsensors.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace libsensors {

class Sensors::SensorsImpl {
    Sensors* sensors;

  public:
    SensorsImpl(Sensors* sensors) :
        sensors(sensors) {
    }

    ~SensorsImpl() {
    }

    void parse_data(const void* bytes, size_t size) {
        buffer.insert(buffer.end(), (const unsigned char*)bytes, ((const unsigned char*)bytes) + size);

        while (true) {
            size_t consumed = 0;
            std::uint8_t type;
            double timestamp;

            if (!advance(type, consumed)) goto end_parse;
            if (!advance(timestamp, consumed)) goto end_parse;

            switch (type) {
            case 0x00: // image
            {
                std::uint32_t width, height;
                if (!advance(width, consumed)) goto end_parse;
                if (!advance(height, consumed)) goto end_parse;
                if (!try_advance_size(width * height, consumed)) goto end_parse;
                std::vector<std::uint8_t> pixels;
                advance_size(width * height, consumed, pixels);
                sensors->on_image(timestamp, width, height, pixels.data());
            } break;
            case 0x01: // gyroscope
            {
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_gyroscope(timestamp, x, y, z);
            } break;
            case 0x02: // accelerometer
            {
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_accelerometer(timestamp, x, y, z);
            } break;
            case 0x03: // magnetometer
            {
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_magnetometer(timestamp, x, y, z);
            } break;
            case 0x04: // altimeter
            {
                double pressure, elevation;
                if (!advance(pressure, consumed)) goto end_parse;
                if (!advance(elevation, consumed)) goto end_parse;
                sensors->on_altimeter(timestamp, pressure, elevation);
            } break;
            case 0x05: // gps
            {
                double lon, lat, alt, hacc, vacc;
                if (!advance(lon, consumed)) goto end_parse;
                if (!advance(lat, consumed)) goto end_parse;
                if (!advance(alt, consumed)) goto end_parse;
                if (!advance(hacc, consumed)) goto end_parse;
                if (!advance(vacc, consumed)) goto end_parse;
                sensors->on_gps(timestamp, lon, lat, alt, hacc, vacc);
            } break;
            case 0x11: { // attitude reported by device
                double x, y, z, w;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                if (!advance(w, consumed)) goto end_parse;
                sensors->on_attitude(timestamp, x, y, z, w);
            } break;
            case 0x12: { // gravity reported by device
                double x, y, z;
                if (!advance(x, consumed)) goto end_parse;
                if (!advance(y, consumed)) goto end_parse;
                if (!advance(z, consumed)) goto end_parse;
                sensors->on_gravity(timestamp, x, y, z);
            } break;
            default: {
                sensors->on_error("unknown data type.");
            } break;
            }
            if (consumed > 0) {
                buffer.erase(buffer.begin(), buffer.begin() + consumed);
            }
        }

    end_parse:
        return;
    }

  private:
    bool try_advance_size(size_t size, size_t consumed) const {
        if (buffer.size() >= consumed + size) {
            return true;
        } else {
            return false;
        }
    }

    template <typename T>
    bool advance(T& value, size_t& consumed) const {
        if (buffer.size() >= consumed + sizeof(value)) {
            value = *(const T*)(buffer.data() + consumed);
            consumed += sizeof(value);
            return true;
        } else {
            return false;
        }
    }

    bool advance_size(size_t size, size_t& consumed, std::vector<std::uint8_t>& buf) const {
        if (buffer.size() >= consumed + size) {
            buf.resize(size);
            memcpy(buf.data(), buffer.data() + consumed, size);
            consumed += size;
            return true;
        } else {
            sensors->on_error("fatal error: buffer overrun.");
            exit(EXIT_FAILURE);
            return false;
        }
    }

    std::vector<unsigned char> buffer;
};

Sensors::Sensors() {
    pimpl = std::make_unique<SensorsImpl>(this);
}

Sensors::~Sensors() = default;

void Sensors::parse_data(const void* bytes, size_t size) {
    pimpl->parse_data(bytes, size);
}

} // namespace libsensors
