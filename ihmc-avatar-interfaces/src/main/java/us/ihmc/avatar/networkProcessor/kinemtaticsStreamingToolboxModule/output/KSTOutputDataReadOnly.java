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

   Point3DReadOnly getRootJointPosition();

   QuaternionReadOnly getRootJointOrientation();

   default Vector3DReadOnly getRootJointLinearVelocity()
   {
      return null;
   }

   default Vector3DReadOnly getRootJointAngularVelocity()
   {
      return null;
   }

   default Vector3DReadOnly getRootJointLinearAcceleration()
   {
      return null;
   }

   default Vector3DReadOnly getRootJointAngularAcceleration()
   {
      return null;
   }

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
      int jointNameHash = Arrays.hashCode(jointsToUpdate);

      if (jointNameHash != getJointNameHash())
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
