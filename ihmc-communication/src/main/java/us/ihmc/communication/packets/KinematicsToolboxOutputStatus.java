package us.ihmc.communication.packets;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.ArrayTools;

public class KinematicsToolboxOutputStatus extends Packet<KinematicsToolboxOutputStatus>
{
   public int jointNameHash;
   public float[] desiredJointAngles;

   public Vector3D32 desiredRootTranslation = new Vector3D32();
   public Quaternion32 desiredRootOrientation = new Quaternion32();

   /** Below 5.0e-3 seems to represent a good solution. */
   public double solutionQuality = Double.NaN;

   public KinematicsToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public KinematicsToolboxOutputStatus(KinematicsToolboxOutputStatus other)
   {
      set(other);
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
      setPacketInformation(other);
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
      float[] jointAngles1 = outputStatusOne.getJointAngles();
      float[] jointAngles2 = outputStatusTwo.getJointAngles();

      for (int i = 0; i < interplateOutputStatus.desiredJointAngles.length; i++)
      {
         interplateOutputStatus.desiredJointAngles[i] = (float) EuclidCoreTools.interpolate(jointAngles1[i], jointAngles2[i], alpha);
      }

      Vector3D32 rootTranslation1 = outputStatusOne.getPelvisTranslation();
      Vector3D32 rootTranslation2 = outputStatusTwo.getPelvisTranslation();
      Quaternion32 rootOrientation1 = outputStatusOne.getPelvisOrientation();
      Quaternion32 rootOrientation2 = outputStatusTwo.getPelvisOrientation();

      interplateOutputStatus.desiredRootTranslation.interpolate(rootTranslation1, rootTranslation2, alpha);
      interplateOutputStatus.desiredRootOrientation.interpolate(rootOrientation1, rootOrientation2, alpha);

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

      if (!ArrayTools.deltaEquals(desiredJointAngles, other.desiredJointAngles, (float) epsilon))
         return false;

      return true;
   }
}
