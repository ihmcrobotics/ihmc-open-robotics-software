package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;


/**
 * This class defines (ansd implements) some basic parameter methods that apply to a range of 
 * lower body locomotion controllers.
 *
 */
public class AbstractHighLevelControllerParameters
{
   /**
    * Returns a list of joint control gains for groups of joints.
    * <p>
    * Each {@link GroupParameter} contains gains for one joint group:</br>
    *  - The name of the joint group that the gain is used for (e.g. Arms).</br>
    *  - The gains for the joint group.</br>
    *  - The names of all rigid bodies in the joint group.
    * </p>
    * If a joint is not contained in the list, jointspace control is not supported
    * for that joint.
    *
    * @return list containing jointspace PID gains and the corresponding joints
    */
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      return new ArrayList<>();
   }

   /**
    * Returns a list of taskspace orientation control gains for groups of bodies.
    * <p>
    * Each {@link GroupParameter} contains gains for one body group:</br>
    *  - The name of the body group that the gain is used for (e.g. Hands).</br>
    *  - The gains for the body group.</br>
    *  - The names of all rigid bodies in the body group.
    * </p>
    * If a body is not contained in the list, taskspace orientation or pose control is not
    * supported for that rigid body. These gains will be used by the controller for tracking
    * taskspace orientation trajectories (or the orientation part of a pose trajectory) for a
    * rigid body.
    *
    * @return list containing orientation PID gains and the corresponding rigid bodies
    */
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspaceOrientationControlGains()
   {
      return new ArrayList<>();
   }

   /**
    * Returns a list of taskspace position control gains for groups of bodies.
    * <p>
    * Each {@link GroupParameter} contains gains for one body group:</br>
    *  - The name of the body group that the gain is used for (e.g. Hands).</br>
    *  - The gains for the body group.</br>
    *  - The names of all rigid bodies in the body group.
    * </p>
    * If a body is not contained in the list, taskspace position or pose control is not
    * supported for that rigid body. These gains will be used by the controller for tracking
    * taskspace position trajectories (or the position part of a pose trajectory) for a
    * rigid body.
    *
    * @return list containing position PID gains and the corresponding rigid bodies
    */
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspacePositionControlGains()
   {
      return new ArrayList<>();
   }

   /**
    * Returns a map with default control modes for each rigid body.
    * <p>
    * The key of the map is the rigid body name as defined in the joint map. Possible
    * control modes are defined in {@link RigidBodyControlMode}. By default (if a body
    * is not contained in the map) {@link RigidBodyControlMode#JOINTSPACE} will be used
    * for the body. In some cases (e.g. the chest) it makes more sense to use the default
    * mode {@link RigidBodyControlMode#TASKSPACE}.
    * </p>
    *
    * @return the default control mode of the body
    */
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      return new HashMap<>();
   }

   /**
    * The map returned contains the default home joint angles. The key of the map is the joint name
    * as defined in the robot joint map.
    *
    * @return map containing home joint angles by joint name
    */
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      return new TObjectDoubleHashMap<String>();
   }

   /**
    * The map returned contains the default rigid body poses in their respective base frame. For example, if the base
    * frame of the chest body is the pelvis z-up frame this should contain the home pose of the chest in that frame.
    * If the particular body does not support full pose control but only orientation control the position part of the
    * pose will be disregarded.
    * <p>
    * The key of the map is the name of the rigid body that can be obtained with {@link RigidBody#getName()}. If a
    * body is not contained in this map but a default control mode of {@link RigidBodyControlMode#TASKSPACE} is not
    * supported for that body.
    *
    * @return map containing home pose in base frame by body name
    */
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      return new HashMap<String, Pose3D>();
   }
}
