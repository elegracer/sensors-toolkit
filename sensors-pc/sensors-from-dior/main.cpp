#include "dior.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <libsensors.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <string>

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

struct DataItem {
    uint8_t type;
    double t;
    std::string filename;
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

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        exit(-1);
    }
    const std::string path = argv[1];
    CameraCsv cam_csv;
    cam_csv.load(path + "/camera/data.csv");
    ImuCsv imu_csv;
    imu_csv.load(path + "/imu/data.csv");
    AttitudeCsv att_csv;
    att_csv.load(path + "/attitude/data.csv");

    std::vector<DataItem> data(cam_csv.items.size() + imu_csv.items.size() * 2 + att_csv.items.size() * 2);
    size_t i = 0;
    for (auto &item : cam_csv.items) {
        data[i].type = 0x00;
        data[i].t = item.t;
        data[i].filename = item.filename;
        ++i;
    }
    for (auto &item : imu_csv.items) {
        data[i].type = 0x01;
        data[i].t = item.t;
        data[i].w = {item.w.x, item.w.y, item.w.z};
        ++i;
        data[i].type = 0x02;
        data[i].t = item.t;
        data[i].a = {item.a.x, item.a.y, item.a.z};
        ++i;
    }
    for (auto &item : att_csv.items) {
        data[i].type = 0x11;
        data[i].t = item.t;
        data[i].atti = {item.atti.x, item.atti.y, item.atti.z, item.atti.w};
        ++i;
        data[i].type = 0x12;
        data[i].t = item.t;
        data[i].g = {item.g.x, item.g.y, item.g.z};
        ++i;
    }

    std::stable_sort(data.begin(), data.end(), [](const auto &a, const auto &b) {
        return a.t < b.t;
    });

    cv::Mat img;
    for (auto &item : data) {
        switch (item.type) {
        case 0x00:
            img = cv::imread(path + "/camera/images/" + item.filename, cv::IMREAD_GRAYSCALE);
            on_image(item.t, img.cols, img.rows, img.data);
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
            std::cerr << "unsupported data type" << std::endl;
            exit(-2);
        }
    }

    return 0;
}
