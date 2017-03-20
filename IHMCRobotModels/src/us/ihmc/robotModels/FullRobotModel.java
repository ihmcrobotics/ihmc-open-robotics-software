package us.ihmc.robotModels;

import java.util.ArrayList;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public interface FullRobotModel
{

   /** Returns the specific the joint names of this robot. See {@link RobotSpecificJointNames}. */
   public abstract RobotSpecificJointNames getRobotSpecificJointNames();

   /** Update all the {@link ReferenceFrame}s attached to this robot. */
   public abstract void updateFrames();

   /** Same as {@link ReferenceFrame#getWorldFrame()}. */
   public abstract ReferenceFrame getWorldFrame();

   /** Returns the {@link ReferenceFrame} attached to the elevator (see {@link FullHumanoidRobotModel#getElevator()}).*/
   public abstract ReferenceFrame getElevatorFrame();

   /**
    * Returns the root joint of this robot. It is a six degrees of freedom joint.
    */
   public abstract FloatingInverseDynamicsJoint getRootJoint();

   /**
    * Returns the elevator of this robot.
    * The elevator is a virtual {@link RigidBody} that has no size nor mass, and is fixed in world (it does not move at all).
    * It is the only predecessor of the root joint.
    * It only has an acceleration that is equal to the opposite of the gravity.
    * It is mainly to easily propagates the effect of gravity to all the robot's joints.
    * @return
    */
   public abstract RigidBody getElevator();

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding spine joint.
    * @param spineJointName Refers to the joint's name.
    */
   public abstract OneDoFJoint getSpineJoint(SpineJointName spineJointName);

   public abstract RigidBody getEndEffector(Enum<?> segmentEnum);

   /**
    * Return the {@link OneDoFJoint} describing the the corresponding leg joint.
    * @param robotSide Refers to which leg the joint belongs to (assuming there is only one left and one right leg).
    * @param neckJointName Refers to the joint's name.
    */
   public abstract OneDoFJoint getNeckJoint(NeckJointName neckJointName);

   /**
    * Returns all the lidar joints existing on this robot.
    * @param lidarName TODO
    * @return TODO
    */
   public abstract InverseDynamicsJoint getLidarJoint(String lidarName);

   public abstract ReferenceFrame getLidarBaseFrame(String name);

   public abstract RigidBodyTransform getLidarBaseToSensorTransform(String name);

   public abstract ReferenceFrame getCameraFrame(String name);

   /**
    * Returns the {@link RigidBody} describing the pelvis of this robot.
    * In the current framework (on the day: 11/18/2014), the pelvis is the the first successor of the root joint.
    */
   public abstract RigidBody getPelvis();

   /**
    * Returns the {@link RigidBody} describing the chest (or trunk) of this robot.
    * The chest is considered to be located right after the spine joints, and right before the arms and head.
    */
   public abstract RigidBody getChest();

   /**
    * Returns the {@link RigidBody} describing the head of this robot.
    * It is located right after the neck joints.
    */
   public abstract RigidBody getHead();

   public abstract ReferenceFrame getHeadBaseFrame();

   /** Returns all the one DoF joints that this robot has. */
   public abstract OneDoFJoint[] getOneDoFJoints();

   public abstract Map<String, OneDoFJoint> getOneDoFJointsAsMap();

   public abstract void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack);

   /** Returns all one DoF joints, excluding joints that do not exist in the controller. */
   public abstract OneDoFJoint[] getControllableOneDoFJoints();

   /**
    *  Gets all the one DoF joints that this robot has.
    *
    *  @param oneDoFJointsToPack {@code ArrayList<OneDoFJoint>} that will be packed will all the one DoF joints.
    */
   public abstract void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack);

   /**
    * Gets the one DoF joint with the given name.
    *
    * @param name Name of the OneDoFJoint to return.
    * @return
    */
   public abstract OneDoFJoint getOneDoFJointByName(String name);

   /**
    *  Gets all one DoF joints, excluding joints that do not exist in the controller.
    *
    *  @param oneDoFJointsToPack {@code ArrayList<OneDoFJoint>} that will be packed will the controllable one DoF joints.
    */
   public abstract void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack);

   /**
    * Returns all the IMUDefinitions corresponding to each IMU attached to this robot.
    * @return
    */
   public abstract IMUDefinition[] getIMUDefinitions();

   /**
    * Returns all the ForceSensorDefinitions corresponding to each force sensor attached to this robot.
    * @return
    */
   public abstract ForceSensorDefinition[] getForceSensorDefinitions();

   /**
    * Returns all the ContactSensorDefinitions corresponding to each contact sensor attached to this robot.
    */
   public abstract ContactSensorDefinition[] getContactSensorDefinitions();


   public abstract double getTotalMass();
}