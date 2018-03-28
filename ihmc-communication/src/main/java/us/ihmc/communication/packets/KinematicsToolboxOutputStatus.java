package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotics.geometry.RotationTools;

public class KinematicsToolboxOutputStatus extends Packet<KinematicsToolboxOutputStatus>
{
   public int jointNameHash;
   public TFloatArrayList desiredJointAngles = new TFloatArrayList();

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
      jointNameHash = other.jointNameHash;
      MessageTools.copyData(other.desiredJointAngles, desiredJointAngles);

      desiredRootTranslation.set(other.desiredRootTranslation);
      desiredRootOrientation.set(other.desiredRootOrientation);

      uniqueId = other.uniqueId;
      destination = other.destination;
      solutionQuality = other.solutionQuality;
      setPacketInformation(other);
   }

   public void setSolutionQuality(double solutionQuality)
   {
      this.solutionQuality = solutionQuality;
   }

   public TFloatArrayList getDesiredJointAngles()
   {
      return desiredJointAngles;
   }

   public Vector3D32 getDesiredRootTranslation()
   {
      return desiredRootTranslation;
   }

   public Quaternion32 getDesiredRootOrientation()
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

      if (desiredJointAngles.size() != other.desiredJointAngles.size())
         return false;
      for (int i = 0; i < desiredJointAngles.size(); i++)
      {
         if (!MathTools.epsilonEquals(desiredJointAngles.get(i), other.desiredJointAngles.get(i), epsilon))
            return false;
      }

      return true;
   }
}
