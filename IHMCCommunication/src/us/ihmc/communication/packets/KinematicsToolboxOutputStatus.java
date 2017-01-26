package us.ihmc.communication.packets;

import java.util.List;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class KinematicsToolboxOutputStatus extends StatusPacket<KinematicsToolboxOutputStatus>
{
   public int jointNameHash;
   public float[] desiredJointAngles;

   public Vector3f desiredRootTranslation = new Vector3f();
   public Quat4f desiredRootOrientation = new Quat4f();

   /** Below 5.0e-3 seems to represent a good solution. */
   public double solutionQuality = Double.NaN;

   public KinematicsToolboxOutputStatus(Random random)
   {
      jointNameHash = random.nextInt(10000);

      int size = Math.abs(random.nextInt(1000));

      desiredJointAngles = new float[size];
      for (int i = 0; i < desiredJointAngles.length; i++)
      {
         desiredJointAngles[i] = random.nextFloat();
      }

      desiredRootTranslation = RandomTools.generateRandomVector3f(random);
      desiredRootOrientation = RandomTools.generateRandomQuaternion4f(random);
   }

   public KinematicsToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public KinematicsToolboxOutputStatus(OneDoFJoint[] joints)
   {
      desiredJointAngles = new float[joints.length];
      jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(joints);
   }

   @Override
   public void set(KinematicsToolboxOutputStatus other)
   {
      if (desiredJointAngles == null)
      {
         desiredJointAngles = new float[other.desiredJointAngles.length];
         jointNameHash = other.jointNameHash;
      }
      else
      {
         if (other.desiredJointAngles.length != desiredJointAngles.length)
            throw new RuntimeException("Array size does not match");

         if (jointNameHash != other.jointNameHash)
            throw new RuntimeException("Joint name hashes are different.");
      }

      for (int i = 0; i < desiredJointAngles.length; i++)
         desiredJointAngles[i] = other.desiredJointAngles[i];

      desiredRootTranslation.set(other.desiredRootTranslation);
      desiredRootOrientation.set(other.desiredRootOrientation);

      uniqueId = other.uniqueId;
      destination = other.destination;
      solutionQuality = other.solutionQuality;
   }

   public void setDesiredJointState(FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] newJointData)
   {
      if (newJointData.length != desiredJointAngles.length)
         throw new RuntimeException("Array size does not match");

      for (int i = 0; i < desiredJointAngles.length; i++)
         desiredJointAngles[i] = (float) newJointData[i].getqDesired();

      rootJoint.getTranslation(desiredRootTranslation);
      rootJoint.getRotation(desiredRootOrientation);
   }

   public void setDesiredJointState(List<OneDoFJoint> newJointData)
   {
      if (newJointData.size() != desiredJointAngles.length)
         throw new RuntimeException("Array size does not match");

      for (int i = 0; i < desiredJointAngles.length; i++)
         desiredJointAngles[i] = (float) newJointData.get(i).getqDesired();
   }

   public void setRootTranslation(Vector3d rootTranslation)
   {
      this.desiredRootTranslation.set(rootTranslation);
   }

   public void setRootOrientation(Quat4d rootOrientation)
   {
      this.desiredRootOrientation.set(rootOrientation);
   }

   public void setSolutionQuality(double solutionQuality)
   {
      this.solutionQuality = solutionQuality;
   }

   public float[] getJointAngles()
   {
      return desiredJointAngles;
   }

   public Vector3f getPelvisTranslation()
   {
      return desiredRootTranslation;
   }

   public Quat4f getPelvisOrientation()
   {
      return desiredRootOrientation;
   }

   public double getSolutionQuality()
   {
      return solutionQuality;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxOutputStatus other, double epsilon)
   {
      if (!desiredRootTranslation.epsilonEquals(other.desiredRootTranslation, 1e-3f))
      {
         return false;
      }

      if (!RotationTools.quaternionEpsilonEquals(desiredRootOrientation, other.desiredRootOrientation, 1e-3f))
      {
         return false;
      }

      if (!MathTools.epsilonEquals(solutionQuality, other.solutionQuality, epsilon))
         return false;

      for (int i = 0; i < desiredJointAngles.length; i++)
      {
         if (Math.abs(desiredJointAngles[i] - other.desiredJointAngles[i]) > epsilon)
         {
            System.out.println(i);
            System.out.println("Diff: " + Math.abs(desiredJointAngles[i] - other.desiredJointAngles[i]) + ", this: " + desiredJointAngles[i] + ", other: " + other.desiredJointAngles[i]);
            return false;
         }
      }

      return true;
   }
}
