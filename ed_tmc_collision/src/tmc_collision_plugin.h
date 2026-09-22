#ifndef ed_tmc_collision_plugin_h_
#define ed_tmc_collision_plugin_h_

#include "http_server.h"

#include <boost/filesystem/path.hpp>

#include <ed/plugin.h>
#include <ed/types.h>

#include <ed_tmc_collision_interfaces/srv/get_collision_environment.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>

struct MeshFileEntry
{
public:
    unsigned long collision_revision;
    std::string mesh_file;
};


/**
 * @brief The TMCCollisionPlugin class
 */
class TMCCollisionPlugin : public ed::Plugin
{

public:

    /**
     * @brief constructor
     */
    TMCCollisionPlugin();

    /**
     * @brief destructor
     */
    virtual ~TMCCollisionPlugin();

    /**
     * @brief configure
     * @param config
     */
    void configure(tue::Configuration config);

    /**
     * @brief initialize
     */
    void initialize();

    /**
     * @brief process
     * @param world
     * @param req
     */
    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    // --------------------

private:

    /**
     * @brief Get a TMC collision environement based on entities objects in ED
     * @param req service request
     * @param res service result
     * @return bool Success
     */
    void srvGetCollisionEnvironment(
        const std::shared_ptr<ed_tmc_collision_interfaces::srv::GetCollisionEnvironment::Request> req,
        std::shared_ptr<ed_tmc_collision_interfaces::srv::GetCollisionEnvironment::Response> res);

    // Services
    rclcpp::Service<ed_tmc_collision_interfaces::srv::GetCollisionEnvironment>::SharedPtr srv_get_collision_environment_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    const ed::WorldModel* world_;

    /**
     * @brief mesh_file_directory_ Folder where mesh files are stored
     */
    const boost::filesystem::path mesh_file_directory_;

    std::string msg_server_prefix_;

    std::unordered_map<std::string, MeshFileEntry> mesh_file_cache_;

    std::unique_ptr<HTTPServer> http_server_;

};

#endif
