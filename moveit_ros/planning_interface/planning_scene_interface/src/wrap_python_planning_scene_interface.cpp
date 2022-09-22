/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

namespace moveit
{
namespace planning_interface
{
class PlanningSceneInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public PlanningSceneInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterfaceWrapper(const std::string& ns = "")
    : py_bindings_tools::ROScppInitializer(), PlanningSceneInterface(ns)
  {
  }

  bool applyCollisionObjectPython(const py_bindings_tools::ByteString& object_str, const py_bindings_tools::ByteString& color_str)
  {
    moveit_msgs::CollisionObject object_msg;
    std_msgs::ColorRGBA color_msg;
    py_bindings_tools::deserializeMsg(object_str, object_msg);
    py_bindings_tools::deserializeMsg(color_str, color_msg);
    return applyCollisionObject(object_msg, color_msg);
  }

  bool applyCollisionObjectsPython(const bp::list& object_list, const bp::list& color_list)
  {
    std::vector<moveit_msgs::CollisionObject> object_msgs;
    std::vector<moveit_msgs::ObjectColor> color_msgs;
    std::vector<py_bindings_tools::ByteString> object_msgs_str = py_bindings_tools::typeFromList<py_bindings_tools::ByteString>(object_list);
    std::vector<py_bindings_tools::ByteString> color_msgs_str = py_bindings_tools::typeFromList<py_bindings_tools::ByteString>(color_list);
    moveit_msgs::CollisionObject object_msg;
    moveit_msgs::ObjectColor color_msg;
    for (std::size_t i = 0; i < object_msgs_str.size(); ++i) {
      py_bindings_tools::deserializeMsg(object_msgs_str.at(i), object_msg);
      py_bindings_tools::deserializeMsg(color_msgs_str.at(i), color_msg);
      object_msgs.push_back(object_msg);
      color_msgs.push_back(color_msg);
    }
    return applyCollisionObjects(object_msgs, color_msgs);
  }

  bp::list getKnownObjectNamesPython(bool with_type = false)
  {
    return py_bindings_tools::listFromString(getKnownObjectNames(with_type));
  }

  bp::list getKnownObjectNamesInROIPython(double minx, double miny, double minz, double maxx, double maxy, double maxz,
                                          bool with_type = false)
  {
    return py_bindings_tools::listFromString(getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type));
  }

  bp::dict getObjectPosesPython(const bp::list& object_ids)
  {
    std::map<std::string, geometry_msgs::Pose> ops = getObjectPoses(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, py_bindings_tools::ByteString> ser_ops;
    for (std::map<std::string, geometry_msgs::Pose>::const_iterator it = ops.begin(); it != ops.end(); ++it)
      ser_ops[it->first] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_ops);
  }

  bp::dict getObjectsPython(const bp::list& object_ids)
  {
    std::map<std::string, moveit_msgs::CollisionObject> objs =
        getObjects(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, py_bindings_tools::ByteString> ser_objs;
    for (std::map<std::string, moveit_msgs::CollisionObject>::const_iterator it = objs.begin(); it != objs.end(); ++it)
      ser_objs[it->first] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_objs);
  }

  bp::dict getAttachedObjectsPython(const bp::list& object_ids)
  {
    std::map<std::string, moveit_msgs::AttachedCollisionObject> aobjs =
        getAttachedObjects(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, py_bindings_tools::ByteString> ser_aobjs;
    for (std::map<std::string, moveit_msgs::AttachedCollisionObject>::const_iterator it = aobjs.begin();
         it != aobjs.end(); ++it)
      ser_aobjs[it->first] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_aobjs);
  }

  bool applyPlanningScenePython(const py_bindings_tools::ByteString& ps_str)
  {
    moveit_msgs::PlanningScene ps_msg;
    py_bindings_tools::deserializeMsg(ps_str, ps_msg);
    return applyPlanningScene(ps_msg);
  }
};

static void wrap_planning_scene_interface()
{
  bp::class_<PlanningSceneInterfaceWrapper> planning_scene_class("PlanningSceneInterface",
                                                                 bp::init<bp::optional<std::string>>());

  planning_scene_class.def("apply_collision_object", &PlanningSceneInterfaceWrapper::applyCollisionObjectPython);
  planning_scene_class.def("apply_collision_objects", &PlanningSceneInterfaceWrapper::applyCollisionObjectsPython);
  planning_scene_class.def("get_known_object_names", &PlanningSceneInterfaceWrapper::getKnownObjectNamesPython);
  planning_scene_class.def("get_known_object_names_in_roi",
                           &PlanningSceneInterfaceWrapper::getKnownObjectNamesInROIPython);
  planning_scene_class.def("get_object_poses", &PlanningSceneInterfaceWrapper::getObjectPosesPython);
  planning_scene_class.def("get_objects", &PlanningSceneInterfaceWrapper::getObjectsPython);
  planning_scene_class.def("get_attached_objects", &PlanningSceneInterfaceWrapper::getAttachedObjectsPython);
  planning_scene_class.def("apply_planning_scene", &PlanningSceneInterfaceWrapper::applyPlanningScenePython);
}
}  // namespace planning_interface
}  // namespace moveit

BOOST_PYTHON_MODULE(_moveit_planning_scene_interface)
{
  using namespace moveit::planning_interface;
  wrap_planning_scene_interface();
}

/** @endcond */
