#ifndef ed_tmc_collision_plugin_h_
#define ed_tmc_collision_plugin_h_

#include "http_server.h"

#include <boost/filesystem/path.hpp>

#include <ed/plugin.h>
#include <ed/types.h>

#include <ed_tmc_collision_msgs/GetCollisionEnvironment.h>

#include <ros/callback_queue.h>

#include <memory>
#include <string>
#include <unordered_map>

struct MeshFileEntry
{
public:
    unsigned long shape_revision;
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
    bool srvGetCollisionEnvironment(const ed_tmc_collision_msgs::GetCollisionEnvironment::Request& req, ed_tmc_collision_msgs::GetCollisionEnvironment::Response& res);

    // Services
    ros::ServiceServer srv_get_collision_environment_;
    ros::CallbackQueue cb_queue_;

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
