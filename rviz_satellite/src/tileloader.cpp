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
#include <boost/regex.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <functional> // for std::hash



bool TileLoader::MapTile::hasImage() const { return !image_.isNull(); }

TileLoader::TileLoader(double latitude,
                       double longitude, unsigned int blocks,unsigned int zoom,
                       QObject *parent)
    : QObject(parent), latitude_(latitude), longitude_(longitude), zoom_(zoom),
      blocks_(blocks) 
{
  assert(blocks_ >= 0);

  const std::string package_path = ros::package::getPath("rviz_satellite");
  if (package_path.empty()) {
    throw std::runtime_error("package 'rviz_satellite' not found");
  }

  //  calculate center tile coordinates
  center_tile_x_ = 11510;
  center_tile_y_ = 7304;
  //  fractional component
  origin_offset_x_ = 0;
  origin_offset_y_ = 0;
}

bool TileLoader::insideCentreTile(double lat, double lon) const {
  double x=11510.0, y=7304.0; 
  latLonToTileCoords(lat, lon, 14, x, y);
  return (std::floor(x) == center_tile_x_ && std::floor(y) == center_tile_y_);
}

void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
  abort();
  ROS_DEBUG("loading %d blocks around tile=(%d,%d)", blocks_, center_tile_x_, center_tile_y_ );
  QImage image=QImage(":/tiles/insti.jpg");
  tiles_.push_back(MapTile(11510, 7304, 14, image));
  //  determine what range of tiles we can load
  /*
  const int min_x = std::max(0, center_tile_x_ - blocks_);
  const int min_y = std::max(0, center_tile_y_ - blocks_);
  const int max_x = std::min(maxTiles(), center_tile_x_ + blocks_);
  const int max_y = std::min(maxTiles(), center_tile_y_ + blocks_);

  //  initiate requests
  for (int y = min_y; y <= max_y; y++) {
    for (int x = min_x; x <= max_x; x++) {
      // Generate filename
      const QString full_path = cachedPathForTile(x, y, zoom_);

    }
  }*/

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


int TileLoader::maxTiles() const { return (1 << zoom_) - 1; }

