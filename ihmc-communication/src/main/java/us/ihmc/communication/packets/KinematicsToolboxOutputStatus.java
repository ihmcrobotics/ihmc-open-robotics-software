package us.ihmc.communication.packets;

import java.util.Random;

import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class KinematicsToolboxOutputStatus extends StatusPacket<KinematicsToolboxOutputStatus>
{
   public int jointNameHash;
   public float[] desiredJointAngles;

   public Vector3D32 desiredRootTranslation = new Vector3D32();
   public Quaternion32 desiredRootOrientation = new Quaternion32();

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

      desiredRootTranslation = RandomGeometry.nextVector3D32(random);
      desiredRootOrientation = RandomGeometry.nextQuaternion32(random);
   }

   public KinematicsToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public KinematicsToolboxOutputStatus(KinematicsToolboxOutputStatus other)
   {
      set(other);
   }

   public KinematicsToolboxOutputStatus(OneDoFJoint[] joints)
   {
      desiredJointAngles = new float[joints.length];
      jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(joints);
   }

   public KinematicsToolboxOutputStatus(FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] newJointData, boolean useQDesired)
   {
      desiredJointAngles = new float[newJointData.length];
      jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(newJointData);
      setDesiredJointState(rootJoint, newJointData, useQDesired);
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

   public void getDesiredJointState(FloatingInverseDynamicsJoint rootJointToUpdate, OneDoFJoint[] jointsToUpdate)
   {
      if (jointsToUpdate.length != desiredJointAngles.length)
         throw new RuntimeException("Array size does not match");
      int jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(jointsToUpdate);

      if (jointNameHash != this.jointNameHash)
         throw new RuntimeException("The robots are different.");

      for (int i = 0; i < desiredJointAngles.length; i++)
         jointsToUpdate[i].setQ(desiredJointAngles[i]);

      rootJointToUpdate.setPosition(desiredRootTranslation);
      rootJointToUpdate.setRotation(desiredRootOrientation);
   }

   public void setDesiredJointState(FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] newJointData, boolean useQDesired)
   {
      if (newJointData.length != desiredJointAngles.length)
         throw new RuntimeException("Array size does not match");
      int jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(newJointData);

      if (jointNameHash != this.jointNameHash)
         throw new RuntimeException("The robots are different.");

      if (useQDesired)
      {
         for (int i = 0; i < desiredJointAngles.length; i++)
         {
            desiredJointAngles[i] = (float) newJointData[i].getqDesired();
         }
      }
      else
      {
         for (int i = 0; i < desiredJointAngles.length; i++)
         {
            desiredJointAngles[i] = (float) newJointData[i].getQ();
         }
      }

      if (rootJoint != null)
      {
         rootJoint.getTranslation(desiredRootTranslation);
         rootJoint.getRotation(desiredRootOrientation);
      }
   }

   public void setRootTranslation(Vector3DReadOnly rootTranslation)
   {
      this.desiredRootTranslation.set(rootTranslation);
   }

   public void setRootOrientation(QuaternionReadOnly rootOrientation)
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

   public Vector3D32 getPelvisTranslation()
   {
      return desiredRootTranslation;
   }

   public Quaternion32 getPelvisOrientation()
   {
      return desiredRootOrientation;
   }

   public double getSolutionQuality()
   {
      return solutionQuality;
   }

   public static KinematicsToolboxOutputStatus interpolateOutputStatus(KinematicsToolboxOutputStatus outputStatusOne,
                                                                       KinematicsToolboxOutputStatus outputStatusTwo, double alpha)
   {
      if (outputStatusOne.jointNameHash != outputStatusTwo.jointNameHash)
         throw new RuntimeException("Output status are not compatible.");

      KinematicsToolboxOutputStatus interplateOutputStatus = new KinematicsToolboxOutputStatus();

      interplateOutputStatus.desiredJointAngles = new float[outputStatusOne.desiredJointAngles.length];

      for (int i = 0; i < interplateOutputStatus.desiredJointAngles.length; i++)
         interplateOutputStatus.desiredJointAngles[i] = (float) TupleTools.interpolate(outputStatusOne.getJointAngles()[i], outputStatusTwo.getJointAngles()[i],
                                                                                       alpha);

      interplateOutputStatus.desiredRootTranslation.interpolate(outputStatusOne.getPelvisTranslation(), outputStatusTwo.getPelvisTranslation(), alpha);
      interplateOutputStatus.desiredRootOrientation.interpolate(outputStatusOne.getPelvisOrientation(), outputStatusTwo.getPelvisOrientation(), alpha);

      interplateOutputStatus.jointNameHash = outputStatusOne.jointNameHash;

      return interplateOutputStatus;
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
            System.out.println("Diff: " + Math.abs(desiredJointAngles[i] - other.desiredJointAngles[i]) + ", this: " + desiredJointAngles[i] + ", other: "
                  + other.desiredJointAngles[i]);
            return false;
         }
      }

      return true;
   }
}
