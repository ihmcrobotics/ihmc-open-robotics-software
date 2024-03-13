package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

import java.util.Arrays;

public interface KSTOutputDataBasics extends KSTOutputDataReadOnly
{
   @Override
   Point3DBasics getRootJointPosition();

   @Override
   QuaternionBasics getRootJointOrientation();

   @Override
   default Vector3DBasics getRootJointLinearVelocity()
   {
      return null;
   }

   @Override
   default Vector3DBasics getRootJointAngularVelocity()
   {
      return null;
   }

   @Override
   default Vector3DBasics getRootJointLinearAcceleration()
   {
      return null;
   }

   @Override
   default Vector3DBasics getRootJointAngularAcceleration()
   {
      return null;
   }

   void setJointPosition(int jointIndex, double position);

   default void setJointVelocity(int jointIndex, double velocity)
   {
   }

   default void setJointAcceleration(int jointIndex, double acceleration)
   {
   }

   default void set(KSTOutputDataReadOnly other)
   {
      checkCompatibility(other);

      getRootJointPosition().set(other.getRootJointPosition());
      getRootJointOrientation().set(other.getRootJointOrientation());
      getRootJointLinearVelocity().set(other.getRootJointLinearVelocity());
      getRootJointAngularVelocity().set(other.getRootJointAngularVelocity());

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         setJointPosition(i, other.getJointPosition(i));
         setJointVelocity(i, other.getJointVelocity(i));
      }

      if (hasAccelerationData())
      {
         if (other.hasAccelerationData())
         {
            getRootJointLinearAcceleration().set(other.getRootJointLinearAcceleration());
            getRootJointAngularAcceleration().set(other.getRootJointAngularAcceleration());

            for (int i = 0; i < getNumberOfJoints(); i++)
            {
               setJointAcceleration(i, other.getJointAcceleration(i));
            }
         }
         else
         {
            setAccelerationToZero();
         }
      }
   }

   default void setConfiguration(KSTOutputDataReadOnly other)
   {
      checkCompatibility(other);

      getRootJointPosition().set(other.getRootJointPosition());
      getRootJointOrientation().set(other.getRootJointOrientation());

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         setJointPosition(i, other.getJointPosition(i));
      }
   }

   default void interpolate(KSTOutputDataReadOnly end, double alpha)
   {
      interpolate(this, end, alpha);
   }

   default void interpolate(KSTOutputDataReadOnly start, KSTOutputDataReadOnly end, double alpha)
   {
      checkCompatibility(start);
      checkCompatibility(end);

      getRootJointPosition().interpolate(start.getRootJointPosition(), end.getRootJointPosition(), alpha);
      getRootJointOrientation().interpolate(start.getRootJointOrientation(), end.getRootJointOrientation(), alpha);
      getRootJointLinearVelocity().interpolate(start.getRootJointLinearVelocity(), end.getRootJointLinearVelocity(), alpha);
      getRootJointAngularVelocity().interpolate(start.getRootJointAngularVelocity(), end.getRootJointAngularVelocity(), alpha);

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         setJointPosition(i, EuclidCoreTools.interpolate(start.getJointPosition(i), end.getJointPosition(i), alpha));
         setJointVelocity(i, EuclidCoreTools.interpolate(start.getJointVelocity(i), end.getJointVelocity(i), alpha));
      }

      if (hasAccelerationData())
      {
         boolean hasStartAcceleration = start.hasAccelerationData();
         boolean hasEndAcceleration = end.hasAccelerationData();

         if (!hasStartAcceleration && !hasEndAcceleration)
         {
            setAccelerationToZero();
         }
         else
         {
            Vector3DReadOnly startLinearRootAcceleration = hasStartAcceleration ? start.getRootJointLinearAcceleration() : EuclidCoreTools.zeroVector3D;
            Vector3DReadOnly startAngularRootAcceleration = hasStartAcceleration ? start.getRootJointAngularAcceleration() : EuclidCoreTools.zeroVector3D;

            Vector3DReadOnly endLinearRootAcceleration = hasEndAcceleration ? end.getRootJointLinearAcceleration() : EuclidCoreTools.zeroVector3D;
            Vector3DReadOnly endAngularRootAcceleration = hasEndAcceleration ? end.getRootJointAngularAcceleration() : EuclidCoreTools.zeroVector3D;

            getRootJointLinearAcceleration().interpolate(startLinearRootAcceleration, endLinearRootAcceleration, alpha);
            getRootJointAngularAcceleration().interpolate(startAngularRootAcceleration, endAngularRootAcceleration, alpha);

            for (int i = 0; i < getNumberOfJoints(); i++)
            {
               double qdd_start = hasStartAcceleration ? start.getJointAcceleration(i) : 0.0;
               double qdd_end = hasEndAcceleration ? end.getJointAcceleration(i) : 0.0;
               double qdd = EuclidCoreTools.interpolate(qdd_start, qdd_end, alpha);
               setJointAcceleration(i, qdd);
            }
         }
      }
   }

   default void setAccelerationToZero()
   {
      getRootJointLinearAcceleration().setToZero();
      getRootJointAngularAcceleration().setToZero();

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         setJointAcceleration(i, 0.0);
      }
   }

   default void setFromRobot(FloatingJointReadOnly rootJoint, OneDoFJointReadOnly[] joints)
   {
      if (getJointNameHash() != Arrays.hashCode(joints))
         throw new IllegalArgumentException("Joint name hash does not match, cannot set data from different robot.");

      getRootJointPosition().set(rootJoint.getJointPose().getPosition());
      getRootJointOrientation().set(rootJoint.getJointPose().getOrientation());
      getRootJointLinearVelocity().set(rootJoint.getJointTwist().getLinearPart());
      getRootJointAngularVelocity().set(rootJoint.getJointTwist().getAngularPart());

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         setJointPosition(i, joints[i].getQ());
         setJointVelocity(i, joints[i].getQd());
      }

      if (hasAccelerationData())
      {
         getRootJointLinearAcceleration().set(rootJoint.getJointAcceleration().getLinearPart());
         getRootJointAngularAcceleration().set(rootJoint.getJointAcceleration().getAngularPart());

         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            setJointAcceleration(i, joints[i].getQdd());
         }
      }
   }

   static KSTOutputDataBasics wrap(KinematicsToolboxOutputStatus outputStatus)
   {
      return new KSTOutputDataBasics()
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
         public Point3DBasics getRootJointPosition()
         {
            return outputStatus.getDesiredRootPosition();
         }

         @Override
         public QuaternionBasics getRootJointOrientation()
         {
            return outputStatus.getDesiredRootOrientation();
         }

         @Override
         public Vector3DBasics getRootJointLinearVelocity()
         {
            return outputStatus.getDesiredRootLinearVelocity();
         }

         @Override
         public Vector3DBasics getRootJointAngularVelocity()
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

         @Override
         public void setJointPosition(int jointIndex, double position)
         {
            outputStatus.getDesiredJointAngles().set(jointIndex, (float) position);
         }

         @Override
         public void setJointVelocity(int jointIndex, double velocity)
         {
            outputStatus.getDesiredJointVelocities().set(jointIndex, (float) velocity);
         }
      };
   }
}
