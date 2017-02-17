package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;

public class WrenchDistributorTools
{
   public static void computeWrench(Wrench groundReactionWrenchToPack, FrameVector force, FramePoint2d cop, double normalTorque)
   {
      ReferenceFrame referenceFrame = cop.getReferenceFrame();
      force.changeFrame(referenceFrame);

      // r x f + tauN
      Vector3D torque = new Vector3D();
      torque.setX(cop.getY() * force.getZ());
      torque.setY(-cop.getX() * force.getZ());
      torque.setZ(cop.getX() * force.getY() - cop.getY() * force.getX() + normalTorque);

      groundReactionWrenchToPack.set(referenceFrame, force.getVector(), torque);
   }

   public static FramePoint computePseudoCMP3d(FramePoint centerOfMass, FramePoint2d cmp, double fZ, double totalMass, double omega0)
   {
      FramePoint pseudoCMP3d = new FramePoint();

      computePseudoCMP3d(pseudoCMP3d, centerOfMass, cmp, fZ, totalMass, omega0);

      return pseudoCMP3d;
   }

   public static void computePseudoCMP3d(FramePoint pseudoCMP3dToPack, FramePoint centerOfMass, FramePoint2d cmp, double fZ, double totalMass, double omega0)
   {
      double zCMP = centerOfMass.getZ() - fZ / (totalMass * MathTools.square(omega0));
      pseudoCMP3dToPack.setIncludingFrame(cmp.getReferenceFrame(), cmp.getX(), cmp.getY(), 0.0);
      pseudoCMP3dToPack.changeFrame(centerOfMass.getReferenceFrame());
      pseudoCMP3dToPack.setZ(zCMP);
   }

   public static FrameVector computeForce(FramePoint centerOfMass, FramePoint cmp, double fZ)
   {
      FrameVector force = new FrameVector(centerOfMass);

      computeForce(force, centerOfMass, cmp, fZ);

      return force;
   }

   public static void computeForce(FrameVector forceToPack, FramePoint centerOfMass, FramePoint cmp, double fZ)
   {
      cmp.changeFrame(centerOfMass.getReferenceFrame());
      forceToPack.setIncludingFrame(centerOfMass);
      forceToPack.sub(cmp);
      forceToPack.scale(fZ / forceToPack.getZ());
   }

   public static void getSupportVectors(List<FrameVector> normalizedSupportVectorsToPack, double mu, ReferenceFrame contactPlaneFrame)
   {
      int numberOfSupportVectors = normalizedSupportVectorsToPack.size();
      double angleIncrement = 2.0 * Math.PI / ((double) numberOfSupportVectors);

      for (int i = 0; i < numberOfSupportVectors; i++)
      {
         double angle = i * angleIncrement;
         getSupportVector(normalizedSupportVectorsToPack.get(i), angle, mu, contactPlaneFrame);
      }
   }

   public static void getSupportVector(FrameVector normalizedSupportVectorToPack, double angle, double mu, ReferenceFrame contactPlaneFrame)
   {
      double x = mu * Math.cos(angle);
      double y = mu * Math.sin(angle);
      double z = 1.0;

      normalizedSupportVectorToPack.setIncludingFrame(contactPlaneFrame, x, y, z);
      normalizedSupportVectorToPack.normalize();
   }

   public static void computeSupportVectorMatrixBlock(DenseMatrix64F supportVectorMatrixBlock, ArrayList<FrameVector> normalizedSupportVectors,
         ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < normalizedSupportVectors.size(); i++)
      {
         FrameVector normalizedSupportVector = normalizedSupportVectors.get(i);
         normalizedSupportVector.changeFrame(referenceFrame);
         normalizedSupportVector.getVector().get(0, i, supportVectorMatrixBlock);
      }
   }

   public static double computeFz(double totalMass, double gravityZ, double centerOfMassHeightAcceleration)
   {
      double fZ = totalMass * (gravityZ + centerOfMassHeightAcceleration);
      return fZ;
   }
}
