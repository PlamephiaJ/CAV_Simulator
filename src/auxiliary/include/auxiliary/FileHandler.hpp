/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 19:10:46
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 11:39:16
 * @Description: File handler header file.
 */

#ifndef FILEHANDLER_HPP
#define FILEHANDLER_HPP

#include "geometry_msgs/msg/point.hpp"

#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>

struct PosAndSpeed
{
    geometry_msgs::msg::Point position;
    double speed;
};


class FileHandler {
protected:
    std::string file_path;
public:
    FileHandler(const std::string& file_path);
    virtual ~FileHandler() {}
    const std::string& get_file_path() const;
    // virtual bool read(const std::string& filename) = 0;
    // virtual bool write(const std::string& filename, const std::string& data) = 0;
};

class CSVHandler : public FileHandler {
public:
    CSVHandler(const std::string& file_path);
    virtual ~CSVHandler() {}
    // virtual bool read(const std::string& filename) override;
    // virtual bool write(const std::string& filename, const std::string& data) override { return false; }
    std::vector<geometry_msgs::msg::Point> read_waypoint_list_from_csv();
    std::vector<PosAndSpeed> read_waypoint_and_speed_list_from_csv();
};

#endif