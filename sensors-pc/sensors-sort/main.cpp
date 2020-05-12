#include <cstdio>
#include <array>
#include <vector>
#include <algorithm>
#include <libsensors.h>

struct DataItem {
    uint8_t type;
    double t;
    struct {
        uint32_t width;
        uint32_t height;
        std::vector<uint8_t> buffer;
    } img;
    struct {
        double x;
        double y;
        double z;
    } w;
    struct {
        double x;
        double y;
        double z;
    } a;
    struct {
        double x;
        double y;
        double z;
    } g;
    struct {
        double x;
        double y;
        double z;
        double w;
    } atti;
};

std::vector<DataItem> data;

class SensorsDecoder : public libsensors::Sensors {
  protected:
    void on_image(double t, int width, int height, const unsigned char *bytes) override {
        auto &item = data.emplace_back();
        item.type = 0x00;
        item.t = t;
        item.img.width = width;
        item.img.height = height;
        item.img.buffer.resize(width * height);
        std::memcpy(item.img.buffer.data(), bytes, width * height);
    }

    void on_gyroscope(double t, double x, double y, double z) override {
        auto &item = data.emplace_back();
        item.type = 0x01;
        item.t = t;
        item.w.x = x;
        item.w.y = y;
        item.w.z = z;
    }

    void on_accelerometer(double t, double x, double y, double z) override {
        auto &item = data.emplace_back();
        item.type = 0x02;
        item.t = t;
        item.a.x = x;
        item.a.y = y;
        item.a.z = z;
    }

    void on_attitude(double t, double x, double y, double z, double w) override {
        auto &item = data.emplace_back();
        item.type = 0x11;
        item.t = t;
        item.atti.x = x;
        item.atti.y = y;
        item.atti.z = z;
        item.atti.w = w;
    }

    void on_gravity(double t, double x, double y, double z) override {
        auto &item = data.emplace_back();
        item.type = 0x12;
        item.t = t;
        item.g.x = x;
        item.g.y = y;
        item.g.z = z;
    }
};

void on_image(double t, int width, int height, const unsigned char *bytes) {
    const std::uint8_t type = 0x00;
    fwrite(&type, sizeof(type), 1, stdout);
    fwrite(&t, sizeof(t), 1, stdout);
    const std::uint32_t w = width;
    const std::uint32_t h = height;
    fwrite(&w, sizeof(w), 1, stdout);
    fwrite(&h, sizeof(h), 1, stdout);
    fwrite(bytes, sizeof(std::uint8_t), width * height, stdout);
}

void on_gyroscope(double t, double x, double y, double z) {
    const std::uint8_t type = 0x01;
    fwrite(&type, sizeof(type), 1, stdout);
    fwrite(&t, sizeof(t), 1, stdout);
    fwrite(&x, sizeof(x), 1, stdout);
    fwrite(&y, sizeof(y), 1, stdout);
    fwrite(&z, sizeof(z), 1, stdout);
}

void on_accelerometer(double t, double x, double y, double z) {
    const std::uint8_t type = 0x02;
    fwrite(&type, sizeof(type), 1, stdout);
    fwrite(&t, sizeof(t), 1, stdout);
    fwrite(&x, sizeof(x), 1, stdout);
    fwrite(&y, sizeof(y), 1, stdout);
    fwrite(&z, sizeof(z), 1, stdout);
}

void on_attitude(double t, double x, double y, double z, double w) {
    const std::uint8_t type = 0x11;
    fwrite(&type, sizeof(type), 1, stdout);
    fwrite(&t, sizeof(t), 1, stdout);
    fwrite(&x, sizeof(x), 1, stdout);
    fwrite(&y, sizeof(y), 1, stdout);
    fwrite(&z, sizeof(z), 1, stdout);
    fwrite(&w, sizeof(w), 1, stdout);
}

void on_gravity(double t, double x, double y, double z) {
    const std::uint8_t type = 0x12;
    fwrite(&type, sizeof(type), 1, stdout);
    fwrite(&t, sizeof(t), 1, stdout);
    fwrite(&x, sizeof(x), 1, stdout);
    fwrite(&y, sizeof(y), 1, stdout);
    fwrite(&z, sizeof(z), 1, stdout);
}

int main() {
    SensorsDecoder decoder;
    std::array<unsigned char, 8192> buffer;
    while (true) {
        size_t nread = fread(buffer.data(), 1, buffer.size(), stdin);
        if (nread > 0) {
            decoder.parse_data(buffer.data(), nread);
        } else {
            if (feof(stdin) || ferror(stdin)) {
                break;
            }
        }
    }

    std::sort(data.begin(), data.end(), [](const auto &a, const auto &b) {
        return a.t < b.t;
    });

    for (const auto &item : data) {
        switch (item.type) {
        case 0x00:
            on_image(item.t, item.img.width, item.img.height, item.img.buffer.data());
            break;
        case 0x01:
            on_gyroscope(item.t, item.w.x, item.w.y, item.w.z);
            break;
        case 0x02:
            on_accelerometer(item.t, item.a.x, item.a.y, item.a.z);
            break;
        case 0x11:
            on_attitude(item.t, item.atti.x, item.atti.y, item.atti.z, item.atti.w);
            break;
        case 0x12:
            on_gravity(item.t, item.g.x, item.g.y, item.g.z);
            break;
        default:
            break;
        }
    }

    return 0;
}
