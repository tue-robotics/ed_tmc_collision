#include "tmc_collision_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <geolib/Box.h>
#include <geolib/Mesh.h>
#include <geolib/Shape.h>
#include <geolib/ros/msg_conversions.h>

#include <geometry_msgs/Pose.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <tmc_manipulation_msgs/CollisionEnvironment.h>
#include <tmc_manipulation_msgs/CollisionObject.h>
#include <tmc_manipulation_msgs/CollisionObjectOperation.h>


// ----------------------------------------------------------------------------------------------------

void convert(const geo::Mesh& m, tmc_geometric_shapes_msgs::Shape& msg)
{
    msg.type = tmc_geometric_shapes_msgs::Shape::MESH;
    const std::vector<geo::Vector3>& points = m.getPoints();
    const std::vector<geo::TriangleI>& triangles = m.getTriangleIs();

    for (std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        geometry_msgs::Point point;
        convert(*it, point);
        msg.vertices.push_back(point);
    }

    for (std::vector<geo::TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
    {
        msg.triangles.push_back(it->i1_);
        msg.triangles.push_back(it->i2_);
        msg.triangles.push_back(it->i3_);
    }
}

// ----------------------------------------------------------------------------------------------------

TMCCollisionPlugin::TMCCollisionPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

TMCCollisionPlugin::~TMCCollisionPlugin()
{
    srv_get_collision_environment_.shutdown();
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::configure(tue::Configuration /*config*/)
{
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::initialize()
{
    ros::NodeHandle nh("~/tmc_collision");

    ros::AdvertiseServiceOptions opt_get_collision_environment =
            ros::AdvertiseServiceOptions::create<ed_tmc_collision_msgs::GetCollisionEnvironment>(
                "get_collision_environment", boost::bind(&TMCCollisionPlugin::srvGetCollisionEnvironment, this, _1, _2), ros::VoidPtr(), &cb_queue_);
    srv_get_collision_environment_ = nh.advertiseService(opt_get_collision_environment);
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& /*req*/)
{
    world_ = &world;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool TMCCollisionPlugin::srvGetCollisionEnvironment(const ed_tmc_collision_msgs::GetCollisionEnvironment::Request& /*req*/, ed_tmc_collision_msgs::GetCollisionEnvironment::Response& res)
{
    ROS_INFO("[ED TMC Collision] Generating collision environment");
    tmc_manipulation_msgs::CollisionEnvironment& msg = res.collision_environment;
    uint object_id = 1; // 0 is invalid
    for(ed::WorldModel::const_iterator it = world_->begin(); it != world_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->id() != "dinner_table" || !e->has_pose() || !e->shape() || e->existenceProbability() < 0.95 || e->hasFlag("self") || e->id() == "floor")
            continue;

        tmc_geometric_shapes_msgs::Shape shape_msg;
        const geo::BoxConstPtr box = std::dynamic_pointer_cast<const geo::Box>(e->shape());
        if (box)
        {
            // Do box stuff
            shape_msg.type = tmc_geometric_shapes_msgs::Shape::BOX;
            shape_msg.dimensions.reserve(3);
            const geo::Vector3& size = box->getSize();
            shape_msg.dimensions.push_back(size.x);
            shape_msg.dimensions.push_back(size.y);
            shape_msg.dimensions.push_back(size.z);
        }
        else
        {
            // Do mesh stuff
            const geo::Mesh& mesh = e->shape()->getMesh();
            convert(mesh, shape_msg);
        }

        tmc_manipulation_msgs::CollisionObject object_msg;
        object_msg.shapes.push_back(shape_msg);

        // Object is defined in own frame. Shape has identity pose relative to this frame.
        object_msg.poses.resize(1);
        object_msg.poses.back().orientation.w = 1.;

        object_msg.operation.operation = tmc_manipulation_msgs::CollisionObjectOperation::ADD;
        object_msg.id.object_id = object_id++;
        object_msg.id.name = e->id().str();
        object_msg.header.frame_id = e->id().str();
        object_msg.header.stamp = ros::Time::now();
        msg.known_objects.push_back(object_msg);

        // ToDo: Change to requested frame. Requires pose transformations
        msg.poses.push_back(geometry_msgs::Pose());
        geo::convert(e->pose(), msg.poses.back());
    }

    return true;
}

ED_REGISTER_PLUGIN(TMCCollisionPlugin)