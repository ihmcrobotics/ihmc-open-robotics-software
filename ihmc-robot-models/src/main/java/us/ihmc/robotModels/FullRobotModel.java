package us.ihmc.robotModels;

import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
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
   FloatingInverseDynamicsJoint getRootJoint();

   /**
    * Returns the elevator of this robot.
    * The elevator is a virtual {@link RigidBody} that has no size nor mass, and is fixed in world (it does not move at all).
    * It is the only predecessor of the root joint.
    * It only has an acceleration that is equal to the opposite of the gravity.
    * It is mainly to easily propagates the effect of gravity to all the robot's joints.
    * @return
    */
   RigidBody getElevator();

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding spine joint.
    * @param spineJointName Refers to the joint's name.
    */
   OneDoFJoint getSpineJoint(SpineJointName spineJointName);

   RigidBody getEndEffector(Enum<?> segmentEnum);

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding leg joint.
    * @param robotSide Refers to which leg the joint belongs to (assuming there is only one left and one right leg).
    * @param neckJointName Refers to the joint's name.
    */
   OneDoFJoint getNeckJoint(NeckJointName neckJointName);

   /**
    * Returns all the lidar joints existing on this robot.
    * @param lidarName TODO
    * @return TODO
    */
   InverseDynamicsJoint getLidarJoint(String lidarName);

   ReferenceFrame getLidarBaseFrame(String name);

   RigidBodyTransform getLidarBaseToSensorTransform(String name);

   ReferenceFrame getCameraFrame(String name);

   /**
    * Returns the {@link RigidBody} describing the root link of this robot.
    * In the current framework (on the day: 2/28/2018), the root link is the the first successor of the root joint.
    */
   RigidBody getRootBody();

   /**
    * Returns the {@link RigidBody} describing the head of this robot.
    * It is located right after the neck joints.
    */
   RigidBody getHead();

   ReferenceFrame getHeadBaseFrame();

   /** Returns all the one DoF joints that this robot has. */
   OneDoFJoint[] getOneDoFJoints();

   Map<String, OneDoFJoint> getOneDoFJointsAsMap();

   void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, List<OneDoFJoint> oneDoFJointsToPack);

   /** Returns all one DoF joints, excluding joints that do not exist in the controller. */
   OneDoFJoint[] getControllableOneDoFJoints();

   /**
    *  Gets all the one DoF joints that this robot has.
    *
    *  @param oneDoFJointsToPack {@code List<OneDoFJoint>} that will be packed will all the one DoF joints.
    */
   void getOneDoFJoints(List<OneDoFJoint> oneDoFJointsToPack);

   /**
    * Gets the one DoF joint with the given name.
    *
    * @param name Name of the OneDoFJoint to return.
    * @return
    */
   OneDoFJoint getOneDoFJointByName(String name);

   /**
    *  Gets all one DoF joints, excluding joints that do not exist in the controller.
    *
    *  @param oneDoFJointsToPack {@code List<OneDoFJoint>} that will be packed will the controllable one DoF joints.
    */
   void getControllableOneDoFJoints(List<OneDoFJoint> oneDoFJointsToPack);

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

   /**
    * Returns all the ContactSensorDefinitions corresponding to each contact sensor attached to this robot.
    */
   ContactSensorDefinition[] getContactSensorDefinitions();


   double getTotalMass();
}