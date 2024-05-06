package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

import java.util.Arrays;

public interface KSTOutputDataReadOnly
{
   int getJointNameHash();

   boolean hasAccelerationData();

   /**
    * Returns the position of the root joint expressed in world frame.
    *
    * @return the position of the root joint.
    */
   Point3DReadOnly getRootJointPosition();

   /**
    * Returns the orientation of the root joint expressed in world frame.
    *
    * @return the orientation of the root joint.
    */
   QuaternionReadOnly getRootJointOrientation();

   /**
    * Returns the linear velocity of the root joint expressed in the local frame of the root joint.
    *
    * @return the linear velocity of the root joint.
    */
   default Vector3DReadOnly getRootJointLinearVelocity()
   {
      return null;
   }

   /**
    * Returns the angular velocity of the root joint expressed in the local frame of the root joint.
    *
    * @return the angular velocity of the root joint.
    */
   default Vector3DReadOnly getRootJointAngularVelocity()
   {
      return null;
   }

   /**
    * Returns the linear acceleration of the root joint expressed in the local frame of the root joint.
    *
    * @return the linear acceleration of the root joint.
    */
   default Vector3DReadOnly getRootJointLinearAcceleration()
   {
      return null;
   }

   /**
    * Returns the angular acceleration of the root joint expressed in the local frame of the root joint.
    *
    * @return the angular acceleration of the root joint.
    */
   default Vector3DReadOnly getRootJointAngularAcceleration()
   {
      return null;
   }

   /**
    * Returns the number of 1-DoF joints in the robot.
    *
    * @return the number of 1-DoF joints.
    */
   int getNumberOfJoints();

   double getJointPosition(int jointIndex);

   default double getJointVelocity(int jointIndex)
   {
      return 0;
   }

   default double getJointAcceleration(int jointIndex)
   {
      return 0;
   }

   default void updateRobot(FloatingJointBasics rootJointToUpdate, OneDoFJointBasics[] jointsToUpdate)
   {

      if (getJointNameHash() != Arrays.hashCode(jointsToUpdate))
         throw new RuntimeException("The robots are different.");

      rootJointToUpdate.getJointPose().getPosition().set(getRootJointPosition());
      rootJointToUpdate.getJointPose().getOrientation().set(getRootJointOrientation());
      rootJointToUpdate.getJointTwist().getLinearPart().set(getRootJointLinearVelocity());
      rootJointToUpdate.getJointTwist().getAngularPart().set(getRootJointAngularVelocity());

      for (int i = 0; i < jointsToUpdate.length; i++)
      {
         jointsToUpdate[i].setQ(getJointPosition(i));
         jointsToUpdate[i].setQd(getJointVelocity(i));
      }

      if (hasAccelerationData())
      {
         rootJointToUpdate.getJointAcceleration().getLinearPart().set(getRootJointLinearAcceleration());
         rootJointToUpdate.getJointAcceleration().getAngularPart().set(getRootJointAngularAcceleration());

         for (int i = 0; i < jointsToUpdate.length; i++)
         {
            jointsToUpdate[i].setQdd(getJointAcceleration(i));
         }
      }
   }

   default void checkCompatibility(KSTOutputDataReadOnly other)
   {
      if (getJointNameHash() != other.getJointNameHash())
         throw new IllegalArgumentException("Joint name hash does not match, cannot set data from different robot.");

      if (getNumberOfJoints() != other.getNumberOfJoints())
         throw new IllegalArgumentException("Number of joints does not match, cannot set data from different robot.");
   }

   static KSTOutputDataReadOnly wrap(KinematicsToolboxOutputStatus outputStatus)
   {
      return new KSTOutputDataReadOnly()
      {
         @Override
         public int getJointNameHash()
         {
            return outputStatus.getJointNameHash();
         }

         @Override
         public boolean hasAccelerationData()
         {
            return false;
         }

         @Override
         public Point3DReadOnly getRootJointPosition()
         {
            return outputStatus.getDesiredRootPosition();
         }

         @Override
         public QuaternionReadOnly getRootJointOrientation()
         {
            return outputStatus.getDesiredRootOrientation();
         }

         @Override
         public Vector3DReadOnly getRootJointLinearVelocity()
         {
            return outputStatus.getDesiredRootLinearVelocity();
         }

         @Override
         public Vector3DReadOnly getRootJointAngularVelocity()
         {
            return outputStatus.getDesiredRootAngularVelocity();
         }

         @Override
         public int getNumberOfJoints()
         {
            return outputStatus.getDesiredJointAngles().size();
         }

         @Override
         public double getJointPosition(int jointIndex)
         {
            return outputStatus.getDesiredJointAngles().get(jointIndex);
         }

         @Override
         public double getJointVelocity(int jointIndex)
         {
            return outputStatus.getDesiredJointVelocities().get(jointIndex);
         }
      };
   }
}
