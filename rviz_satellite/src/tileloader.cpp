/*
 * TileLoader.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include "tileloader.h"

#include <QVariant>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QImageReader>
#include <stdexcept>
#include <ros/ros.h>
#include <ros/package.h>
#include <functional> // for std::hash



bool TileLoader::MapTile::hasImage() const { 
  return !image_.isNull(); }

TileLoader::TileLoader( double latitude, double longitude, QObject *parent)
    : QObject(parent), latitude_(latitude), longitude_(longitude)
 {

  const std::string package_path = ros::package::getPath("rviz_satellite");
  if (package_path.empty()) {
    throw std::runtime_error("package 'rviz_satellite' not found");
  }

  double x=11510.0, y=7304.0; 
  latLonToTileCoords(latitude_, longitude_, 14, x, y);
  center_tile_x_ = std::floor(x);
  center_tile_y_ = std::floor(y);
  //  fractional component
  origin_offset_x_ = x - center_tile_x_;
  origin_offset_y_ = y - center_tile_y_;
}

bool TileLoader::insideCentreTile(double lat, double lon) const {
   double x=11510.0, y=7304.0; 
  latLonToTileCoords(lat, lon, 14, x, y);
  return (std::floor(x) == center_tile_x_ && std::floor(y) == center_tile_y_);
}

void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
  abort();
   double x=11510.0, y=7304.0; 
  ROS_INFO("loading around tile=(%d,%d)", center_tile_x_, center_tile_y_ );
  const std::string package_path = ros::package::getPath("rviz_satellite");
  QImage image=QImage(QDir::cleanPath(QString::fromStdString(package_path) + QDir::separator() +
                      QString("tiles") + QDir::separator() +
                      QString("insti.jpg")));
  ROS_INFO("Tile Loaded");
  tiles_.push_back(MapTile(x, y, 14, image));
   ROS_INFO("Tile Pushed");
      
}


double TileLoader::resolution() const {
  return zoomToResolution(latitude_, 14);
}

/// @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
/// For explanation of these calculations.
void TileLoader::latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                    double &x, double &y) {
  if (zoom > 31) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) +
                                " too high");
  } else if (lat < -85.0511 || lat > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(lat) + " invalid");
  } else if (lon < -180 && lon > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(lon) +
                                " invalid");
  }

  const double rho = M_PI / 180;
  const double lat_rad = lat * rho;

  unsigned int n = (1 << zoom);
  x = n * ((lon + 180) / 360.0);
  y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) /
      2;
  ROS_DEBUG_STREAM( "Center tile coords: " << x << ", " << y );
}

double TileLoader::zoomToResolution(double lat, unsigned int zoom) {
  const double lat_rad = lat * M_PI / 180;
  return 156543.034 * std::cos(lat_rad) / (1 << zoom);
}

void TileLoader::abort() {
  tiles_.clear();
}
