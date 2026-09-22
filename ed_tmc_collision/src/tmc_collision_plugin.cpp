#include "tmc_collision_plugin.h"

#include <boost/filesystem/operations.hpp>

#include <ed/error_context.h>
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <geolib/Box.h>
#include <geolib/io/export.h>
#include <geolib/Mesh.h>
#include <geolib/Shape.h>
#include <geolib/ros/msg_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.hpp>

#include <tmc_geometric_shapes_msgs/msg/shape.hpp>
#include <tmc_manipulation_msgs/msg/collision_environment.hpp>
#include <tmc_manipulation_msgs/msg/collision_object.hpp>
#include <tmc_manipulation_msgs/msg/collision_object_operation.hpp>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <tuple>
#include <string>

using boost::filesystem::create_directories;
using boost::filesystem::remove_all;
using boost::filesystem::temp_directory_path;
using boost::filesystem::unique_path;


// ----------------------------------------------------------------------------------------------------

TMCCollisionPlugin::TMCCollisionPlugin() : mesh_file_directory_(temp_directory_path()/=unique_path("ed_tmc_collision-%%%%%")), http_server_(nullptr)
{
    ed::ErrorContext errc("tmc_collision", "constructor");
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("tmc_collision"), "mesh_file_directory_: " << mesh_file_directory_);
    create_directories(mesh_file_directory_);
}

// ----------------------------------------------------------------------------------------------------

TMCCollisionPlugin::~TMCCollisionPlugin()
{
    ed::ErrorContext errc("destructor");
    srv_get_collision_environment_.reset();
    if (http_server_)
        http_server_->stop();

    remove_all(mesh_file_directory_);
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::configure(tue::Configuration config)
{
    ed::ErrorContext errc("configure");
    uint threads = 1;
    int port = 8080;
    std::string address, server_address;
    if (config.readGroup("http_server", tue::config::REQUIRED))
    {
        config.value("address", address, tue::config::REQUIRED); // Used in msg; Can be hostname or IP
        config.value("server_address", server_address, tue::config::OPTIONAL); // Used for the server; Needs to be IP
        config.value("threads", reinterpret_cast<int&>(threads), tue::config::OPTIONAL);
        config.value("port", port, tue::config::OPTIONAL);
        config.endGroup();
    }
    msg_server_prefix_ = "http://" + address + ":" + std::to_string(port) + "/";

    RCLCPP_INFO_STREAM(rclcpp::get_logger("tmc_collision"), "Starting HTTP server:\naddress: " << server_address << "\nport: " << port << "\ndoc_root: " << mesh_file_directory_ << "\nthreads: " << threads);
    if (!server_address.empty())
        http_server_ = std::make_unique<HTTPServer>(mesh_file_directory_.string(), static_cast<unsigned short>(port), net::ip::make_address(server_address));
    else
        http_server_ = std::make_unique<HTTPServer>(mesh_file_directory_.string(), static_cast<unsigned short>(port));
    http_server_->run(threads);
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::initialize()
{
    ed::ErrorContext errc("initialize");
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_get_collision_environment_ = node_->create_service<ed_tmc_collision_interfaces::srv::GetCollisionEnvironment>(
        "~/tmc_collision/get_collision_environment",
        std::bind(&TMCCollisionPlugin::srvGetCollisionEnvironment, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        cb_group_);
    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& /*req*/)
{
    ed::ErrorContext errc("process");
    world_ = &world;
    executor_.spin_some();
}

// ----------------------------------------------------------------------------------------------------

void TMCCollisionPlugin::srvGetCollisionEnvironment(
    const std::shared_ptr<ed_tmc_collision_interfaces::srv::GetCollisionEnvironment::Request> req,
    std::shared_ptr<ed_tmc_collision_interfaces::srv::GetCollisionEnvironment::Response> res)
{
    ed::ErrorContext errc("srvGetCollisionEnvironment");
    RCLCPP_INFO(rclcpp::get_logger("tmc_collision"), "[ED TMC Collision] Generating collision environment");

    geometry_msgs::msg::TransformStamped transform_msg;
    try
    {
        transform_msg = tf_buffer_->lookupTransform(req->frame_id, "map", tf2::TimePointZero);
    }
    catch(tf2::TransformException& exc)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("tmc_collision"), "Could not lookup the tranform from 'map' to '" << req->frame_id << "'\n" << exc.what());
        return;
    }
    geo::Transform transform;
    geo::convert(transform_msg.transform, transform);

    tmc_manipulation_msgs::msg::CollisionEnvironment& msg = res->collision_environment;
    uint object_id = 1; // 0 is invalid
    for (ed::WorldModel::const_iterator it = world_->begin(); it != world_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        const ed::UUID& id = e->id();

        // General filtering
        if (!e->has_pose() || !e->collision() || e->existenceProbability() < 0.95 || e->hasFlag("self") || (id.str().size() >= 5 && id.str().substr(0, 5) == "floor"))
            continue;

        const decltype(req->required_entities)& req_entities = req->required_entities;
        bool required_as_wall = req->include_walls && id.str().substr(0, 4) == "wall";
        bool required_by_id = std::find(req_entities.begin(), req_entities.end(), id.str()) == req_entities.end();
        bool required_by_distance = std::isinf(req->entity_range) || true; // ToDo: add distance calculations

        if (!required_as_wall && !required_by_id && !required_by_distance)
            continue;

        tmc_geometric_shapes_msgs::msg::Shape shape_msg;
        const geo::BoxConstPtr box = std::dynamic_pointer_cast<const geo::Box>(e->collision());
        if (box)
        {
            // Do box stuff
            shape_msg.type = tmc_geometric_shapes_msgs::msg::Shape::BOX;
            shape_msg.dimensions.reserve(3);
            const geo::Vector3& size = box->getSize();
            shape_msg.dimensions.push_back(size.x);
            shape_msg.dimensions.push_back(size.y);
            shape_msg.dimensions.push_back(size.z);
        }
        else
        {
            // Do mesh stuff
            MeshFileEntry& entry = mesh_file_cache_[id.str()]; // Accesses or inserts element
            std::string& mesh_file = entry.mesh_file;
            if (entry.collision_revision != e->collisionRevision())
            {
                if (mesh_file.empty())
                {
                    mesh_file = id.str() + ".stl";
                    std::replace(mesh_file.begin(), mesh_file.end(), '/', '_');
                }

                if (!geo::io::writeMeshFile(boost::filesystem::path(mesh_file_directory_).append(mesh_file).string(), *e->collision(), "stlb")) // Only binary STL is accepted by TMC
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("tmc_collision"), "Could not write shape of entity '" << id << "' to file '" << mesh_file << "'");
                    continue;
                }
                entry.collision_revision = e->collisionRevision();
            }
            shape_msg.type = tmc_geometric_shapes_msgs::msg::Shape::MESH;
            shape_msg.stl_file_name = msg_server_prefix_ + mesh_file;
        }

        tmc_manipulation_msgs::msg::CollisionObject object_msg;
        object_msg.shapes.push_back(shape_msg);

        // Object is defined in own frame. Shape has identity pose relative to this frame.
        object_msg.poses.resize(1);
        object_msg.poses.back().orientation.w = 1.;

        object_msg.operation.operation = tmc_manipulation_msgs::msg::CollisionObjectOperation::ADD;
        object_msg.id.object_id = object_id++;
        object_msg.id.name = e->id().str();
        object_msg.header.frame_id = e->id().str();
        object_msg.header.stamp = node_->get_clock()->now().to_msg();
        msg.known_objects.push_back(object_msg);

        msg.poses.push_back(geometry_msgs::msg::Pose());
        geo::convert(transform * e->pose(), msg.poses.back());
    }

    msg.header.frame_id = req->frame_id;
    msg.header.stamp = node_->get_clock()->now().to_msg();
}

ED_REGISTER_PLUGIN(TMCCollisionPlugin)
