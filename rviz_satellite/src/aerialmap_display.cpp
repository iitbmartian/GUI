/*
 * AerialMapDisplay.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include <QtGlobal>
#include <QImage>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreImageCodec.h>
#include <OGRE/OgreVector3.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "aerialmap_display.h"

#define FRAME_CONVENTION_XYZ_ENU (0)  //  X -> East, Y -> North
#define FRAME_CONVENTION_XYZ_NED (1)  //  X -> North, Y -> East
#define FRAME_CONVENTION_XYZ_NWU (2)  //  X -> North, Y -> West


Ogre::TexturePtr textureFromImage(const QImage &image,
                                  const std::string &name) {
  //  convert to 24bit rgb
  QImage converted = image.convertToFormat(QImage::Format_RGB888).mirrored();

  //  create texture
  Ogre::TexturePtr texture;
  Ogre::DataStreamPtr data_stream;
  data_stream.bind(new Ogre::MemoryDataStream((void *)converted.constBits(),
                                              converted.byteCount()));

  const Ogre::String res_group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::TextureManager &texture_manager = Ogre::TextureManager::getSingleton();
  //  swap byte order when going from QImage to Ogre
  texture = texture_manager.loadRawData(name, res_group, data_stream,
                                        converted.width(), converted.height(),
                                        Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
  return texture;
}

namespace rviz {

AerialMapDisplay::AerialMapDisplay()
    : Display(), map_id_(0), scene_id_(0), dirty_(false),
      received_msg_(false) {

  static unsigned int map_ids = 0;
  map_id_ = map_ids++; //  global counter of map ids

  topic_property_ = new RosTopicProperty(
      "Topic", "", QString::fromStdString(
                       ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
      "nav_msgs::Odometry topic to subscribe to.", this, SLOT(updateTopic()));

  frame_property_ = new TfFrameProperty("Robot frame", "world",
                                        "TF frame for the moving robot.", this,
                                        0, false, SLOT(updateFrame()), this);

  frame_convention_property_ =
      new EnumProperty("Frame Convention", "XYZ -> ENU",
                       "Convention for mapping cartesian frame to the compass",
                       this, SLOT(updateFrameConvention()));
  frame_convention_property_->addOptionStd("XYZ -> ENU",
                                           FRAME_CONVENTION_XYZ_ENU);
  frame_convention_property_->addOptionStd("XYZ -> NED",
                                           FRAME_CONVENTION_XYZ_NED);
  frame_convention_property_->addOptionStd("XYZ -> NWU",
                                           FRAME_CONVENTION_XYZ_NWU);

}

AerialMapDisplay::~AerialMapDisplay() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::onInitialize() {
  frame_property_->setFrameManager(context_->getFrameManager());
}

void AerialMapDisplay::onEnable() { subscribe(); }

void AerialMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }

  if (!topic_property_->getTopic().isEmpty()) {
    try {
      ROS_INFO("Subscribing to %s","/mavros/global_position/raw/fix");
      coord_sub_ =
          update_nh_.subscribe("/mavros/global_position/raw/fix", 1,
                               &AerialMapDisplay::navFixCallback, this);

      setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception &e) {
      setStatus(StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void AerialMapDisplay::unsubscribe() {
  coord_sub_.shutdown();
  ROS_INFO("Unsubscribing.");
}


void AerialMapDisplay::updateFrame() {
  ROS_INFO_STREAM("Changing robot frame to " << frame_property_->getFrameStd());
  transformAerialMap();
}


void AerialMapDisplay::updateTopic() {
  unsubscribe();
  clear();
  subscribe();
}

void AerialMapDisplay::clear() {
  setStatus(StatusProperty::Warn, "Message", "No map received");
  clearGeometry();
  //  the user has cleared here
  received_msg_ = false;
  //  cancel current imagery, if any
  loader_.reset();
}

void AerialMapDisplay::clearGeometry() {
  for (MapObject &obj : objects_) {
    //  destroy object
    scene_node_->detachObject(obj.object);
    scene_manager_->destroyManualObject(obj.object);
    //  destroy texture
    if (!obj.texture.isNull()) {
      Ogre::TextureManager::getSingleton().remove(obj.texture->getName());
    }
    //  destroy material
    if (!obj.material.isNull()) {
      Ogre::MaterialManager::getSingleton().remove(obj.material->getName());
    }
  }
  objects_.clear();
}

void AerialMapDisplay::update(float, float) {
  //  creates all geometry, if necessary
  assembleScene();
  //  draw
  context_->queueRender();
}

void AerialMapDisplay::navFixCallback(const sensor_msgs::NavSatFixConstPtr &msg) {
  // If the new (lat,lon) falls into a different tile then we have some
  // reloading to do.
  if (!received_msg_ ||
      (loader_ && !loader_->insideCentreTile(msg->latitude, msg->longitude) )) {
    ref_fix_ = *msg;
    ROS_INFO("Reference point set to: %.12f, %.12f", ref_fix_.latitude,
             ref_fix_.longitude);
    setStatus(StatusProperty::Warn, "Message", "Loading map tiles.");

    //  re-load imagery
    received_msg_ = true;
    loadImagery();
    transformAerialMap();
  }
}

void AerialMapDisplay::loadImagery() {
  //  cancel current imagery, if any
  loader_.reset();
  
  if (!received_msg_) {
    //  no message received from publisher
    return;
  }
  
  try {
    loader_.reset(new TileLoader(ref_fix_.latitude,
                                 ref_fix_.longitude, this));
  } catch (std::exception &e) {
    setStatus(StatusProperty::Error, "Message", QString(e.what()));
    return;
  }

  loader_->start();
}

void AerialMapDisplay::assembleScene() {
  
  if (!loader_) {
    return; //  no tiles loaded, don't do anything
  }
  
  //  get rid of old geometry, we will re-build this
  clearGeometry();
  
  //  iterate over all tiles and create an object for each of them
  for (const TileLoader::MapTile &tile : loader_->tiles()) {
    // NOTE(gareth): We invert the y-axis so that positive y corresponds
    // to north. We are in XYZ->ENU convention here.
    const int w = tile.image().width();
    const int h = tile.image().height();
    const double tile_w = w * loader_->resolution();
    const double tile_h = h * loader_->resolution();

    // Shift back such that (0, 0) corresponds to the exact latitude and
    // longitude the tile loader requested.
    // This is the local origin, in the frame of the map node.
    const double origin_x = -loader_->originOffsetX() * tile_w;
    const double origin_y = -(1 - loader_->originOffsetY()) * tile_h;

    // determine location of this tile, flipping y in the process
    const double x = (tile.x() - loader_->centerTileX()) * tile_w + origin_x;
    const double y = -(tile.y() - loader_->centerTileY()) * tile_h + origin_y;
    //  don't re-use any ids
    const std::string name_suffix =
        std::to_string(tile.x()) + "_" + std::to_string(tile.y()) + "_" +
        std::to_string(map_id_) + "_" + std::to_string(scene_id_);

    if (tile.hasImage()) {
      //  one material per texture
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
          "material_" + name_suffix,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(false);
      material->setDepthBias(-16.0f, 0.0f);
      material->setCullingMode(Ogre::CULL_NONE);
      material->setDepthWriteEnabled(false);

      //  create textureing unit
      Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
      Ogre::TextureUnitState *tex_unit = NULL;
      if (pass->getNumTextureUnitStates() > 0) {
        tex_unit = pass->getTextureUnitState(0);
      } else {
        tex_unit = pass->createTextureUnitState();
      }

      //  only add if we have a texture for it
      Ogre::TexturePtr texture =
          textureFromImage(tile.image(), "texture_" + name_suffix);

      tex_unit->setTextureName(texture->getName());
      tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

      //  create an object
      const std::string obj_name = "object_" + name_suffix;
      Ogre::ManualObject *obj = scene_manager_->createManualObject(obj_name);
      scene_node_->attachObject(obj);


      //  create a quad for this tile
      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      //  bottom left
      obj->position(x, y, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(x + tile_w, y + tile_h, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top left
      obj->position(x, y + tile_h, 0.0f);
      obj->textureCoord(0.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      //  bottom left
      obj->position(x, y, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // bottom right
      obj->position(x + tile_w, y, 0.0f);
      obj->textureCoord(1.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(x + tile_w, y + tile_h, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      obj->end();

  
      MapObject object;
      object.object = obj;
      object.texture = texture;
      object.material = material;
      objects_.push_back(object);
    }
  }
  scene_id_++;
}


// TODO(gareth): We are technically ignoring the orientation from the
// frame manager here - does this make sense?
void AerialMapDisplay::transformAerialMap() {
  // pass in identity to get pose of robot wrt to the fixed frame
  // the map will be shifted so as to compensate for the center tile shifting
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  
  const std::string frame = frame_property_->getFrameStd();
  Ogre::Vector3 position{0, 0, 0};
  Ogre::Quaternion orientation{1, 0, 0, 0};

  // get the transform at the time we received the reference lat and lon
  if (!context_->getFrameManager()->transform(frame, ref_fix_.header.stamp, pose,
                                              position, orientation)) {
    ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
              qPrintable(getName()), frame.c_str(), qPrintable(fixed_frame_));

    setStatus(StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame) + "] to [" +
                  fixed_frame_ + "]");

    // set the transform to identity on failure
    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;
  } else {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }
  if (position.isNaN() || orientation.isNaN()) {
    // this can occur if an invalid TF is published. Set to identiy so OGRE does
    // not throw an assertion
    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;
    ROS_ERROR("rviz_satellite received invalid transform, setting to identity");
  }

  // Here we assume that the fixed/world frame is at altitude=0
  // force aerial imagery on ground
  position.z = 0;
  scene_node_->setPosition(position);
  
  const int convention = frame_convention_property_->getOptionInt();
  if (convention == FRAME_CONVENTION_XYZ_ENU) {
    // ENU corresponds to our default drawing method
    scene_node_->setOrientation(Ogre::Quaternion::IDENTITY);
  } else if (convention == FRAME_CONVENTION_XYZ_NED) {
    // NOTE(gareth): XYZ->NED will cause the map to appear reversed when viewed
    // from ve (from +z).
    // clang-format off
    const Ogre::Matrix3 xyz_R_ned(0, 1, 0,
                                  1, 0, 0,
                                  0, 0,-1);
    // clang-format on
    scene_node_->setOrientation(xyz_R_ned.Transpose());
  } else if (convention == FRAME_CONVENTION_XYZ_NWU) {
    // clang-format off
    const Ogre::Matrix3 xyz_R_nwu(0,-1, 0,
                                  1, 0, 0,
                                  0, 0, 1);
    // clang-format on
    scene_node_->setOrientation(xyz_R_nwu.Transpose());
  } else {
    ROS_ERROR_STREAM("Invalid convention code: " << convention);
  }
}

void AerialMapDisplay::fixedFrameChanged() { transformAerialMap(); }

void AerialMapDisplay::reset() {
  Display::reset();
  //  unsub,clear,resub
  updateTopic();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz::Display)
