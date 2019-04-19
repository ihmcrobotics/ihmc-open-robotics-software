package us.ihmc.robotModels;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public interface FullRobotModel
{
   /** Returns the specific the joint names of this robot. See {@link RobotSpecificJointNames}. */
   RobotSpecificJointNames getRobotSpecificJointNames();

   /** Update all the {@link ReferenceFrame}s attached to this robot. */
   void updateFrames();

   /** Returns the {@link ReferenceFrame} attached to the elevator (see {@link FullHumanoidRobotModel#getElevator()}).*/
   MovingReferenceFrame getElevatorFrame();

   /**
    * Returns the root joint of this robot. It is a six degrees of freedom joint.
    */
   FloatingJointBasics getRootJoint();

   /**
    * Returns the elevator of this robot.
    * The elevator is a virtual {@link RigidBodyBasics} that has no size nor mass, and is fixed in world (it does not move at all).
    * It is the only predecessor of the root joint.
    * It only has an acceleration that is equal to the opposite of the gravity.
    * It is mainly to easily propagates the effect of gravity to all the robot's joints.
    * @return
    */
   RigidBodyBasics getElevator();

   /**
    * Return the {@link OneDoFJointBasics} describing the the corresponding spine joint.
    * @param spineJointName Refers to the joint's name.
    */
   OneDoFJointBasics getSpineJoint(SpineJointName spineJointName);

   RigidBodyBasics getEndEffector(Enum<?> segmentEnum);

   /**
    * Return the {@link OneDoFJointBasics} describing the the corresponding leg joint.
    * @param robotSide Refers to which leg the joint belongs to (assuming there is only one left and one right leg).
    * @param neckJointName Refers to the joint's name.
    */
   OneDoFJointBasics getNeckJoint(NeckJointName neckJointName);

   
   /**
    * 
    * @return the list of lidar sensor names in this robot model
    */
   default List<String> getLidarSensorNames()
   {
      return Collections.emptyList();
   }
   
   /**
    * 
    * @return the list of camera sensor names in this robot model
    */
   default List<String> getCameraSensorNames()
   {
      return Collections.emptyList();
   }
   
   /**
    * Returns all the lidar joints existing on this robot.
    * @param lidarName TODO
    * @return TODO
    */
   JointBasics getLidarJoint(String lidarName);

   ReferenceFrame getLidarBaseFrame(String name);

   RigidBodyTransform getLidarBaseToSensorTransform(String name);

   ReferenceFrame getCameraFrame(String name);

   /**
    * Returns the {@link RigidBodyBasics} describing the root link of this robot.
    * In the current framework (on the day: 2/28/2018), the root link is the the first successor of the root joint.
    */
   RigidBodyBasics getRootBody();

   /**
    * Returns the {@link RigidBodyBasics} describing the head of this robot.
    * It is located right after the neck joints.
    */
   RigidBodyBasics getHead();

   ReferenceFrame getHeadBaseFrame();

   /** Returns all the one DoF joints that this robot has. */
   OneDoFJointBasics[] getOneDoFJoints();

   Map<String, OneDoFJointBasics> getOneDoFJointsAsMap();

   void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack);

   /** Returns all one DoF joints, excluding joints that do not exist in the controller. */
   OneDoFJointBasics[] getControllableOneDoFJoints();

   /**
    *  Gets all the one DoF joints that this robot has.
    *
    *  @param oneDoFJointsToPack {@code List<OneDoFJoint>} that will be packed will all the one DoF joints.
    */
   void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack);

   /**
    * Gets the one DoF joint with the given name.
    *
    * @param name Name of the OneDoFJoint to return.
    * @return
    */
   OneDoFJointBasics getOneDoFJointByName(String name);

   /**
    *  Gets all one DoF joints, excluding joints that do not exist in the controller.
    *
    *  @param oneDoFJointsToPack {@code List<OneDoFJoint>} that will be packed will the controllable one DoF joints.
    */
   void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack);

   /**
    * Returns all the IMUDefinitions corresponding to each IMU attached to this robot.
    * @return
    */
   IMUDefinition[] getIMUDefinitions();

   /**
    * Returns all the ForceSensorDefinitions corresponding to each force sensor attached to this robot.
    * @return
    */
   ForceSensorDefinition[] getForceSensorDefinitions();

   double getTotalMass();
}