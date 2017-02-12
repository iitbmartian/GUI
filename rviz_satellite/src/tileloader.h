/*
 * TileLoader.h
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#ifndef TILELOADER_H
#define TILELOADER_H

#include <QObject>
#include <QImage>
#include <QString>
#include <vector>
#include <memory>

class TileLoader : public QObject {
  Q_OBJECT
public:
  class MapTile {
  public:
      
    MapTile(int x, int y, int z, QImage & image)
      : x_(x), y_(y), z_(z), image_(image) {}

    /// X tile coordinate.
    int x() const { return x_; }

    /// Y tile coordinate.
    int y() const { return y_; }
      
    /// Z tile zoom value.
    int z() const { return z_; }

   
    /// Has a tile successfully loaded?
    bool hasImage() const;

    /// Image associated with this tile.
    const QImage &image() const { return image_; }
    void setImage(const QImage &image) { image_ = image; }

  private:
    int x_;
    int y_;
    int z_;
    QImage image_;
  };

  explicit TileLoader(double latitude,double longitude, QObject *parent = 0);

  /// Start loading tiles asynchronously.
  void start();

  /// Meters/pixel of the tiles.
  double resolution() const;

  /// X index of central tile.
  int centerTileX() const { return center_tile_x_; }

  /// Y index of central tile.
  int centerTileY() const { return center_tile_y_; }

  /// Fraction of a tile to offset the origin (X).
  double originOffsetX() const { return origin_offset_x_; }

  /// Fraction of a tile to offset the origin (Y).
  double originOffsetY() const { return origin_offset_y_; }

  /// Test if (lat,lon) falls inside centre tile.
  bool insideCentreTile(double lat, double lon) const;

  /// Convert lat/lon to a tile index with mercator projection.
  static void latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                 double &x, double &y);

  /// Convert latitude and zoom level to ground resolution.
  static double zoomToResolution(double lat, unsigned int zoom);

  /// Current set of tiles.
  const std::vector<MapTile> &tiles() const { return tiles_; }

  /// Cancel all current requests.
  void abort();


public slots:

private slots:


private:


  double latitude_;
  double longitude_;
  int center_tile_x_;
  int center_tile_y_;
  double origin_offset_x_;
  double origin_offset_y_;

  QString cache_path_;


  std::vector<MapTile> tiles_;
};

#endif // TILELOADER_H
