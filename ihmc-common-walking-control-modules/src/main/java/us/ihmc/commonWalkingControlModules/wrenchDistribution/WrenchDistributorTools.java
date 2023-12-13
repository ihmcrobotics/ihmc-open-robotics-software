package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.Wrench;

public class WrenchDistributorTools
{
   public static void computeWrench(Wrench groundReactionWrenchToPack, FrameVector3D force, FramePoint2DReadOnly cop, double normalTorque)
   {
      ReferenceFrame referenceFrame = cop.getReferenceFrame();
      force.changeFrame(referenceFrame);

      // r x f + tauN
      Vector3D torque = new Vector3D();
      torque.setX(cop.getY() * force.getZ());
      torque.setY(-cop.getX() * force.getZ());
      torque.setZ(cop.getX() * force.getY() - cop.getY() * force.getX() + normalTorque);

      groundReactionWrenchToPack.setIncludingFrame(referenceFrame, torque, force);
   }

   public static FramePoint3D computePseudoCMP3d(FramePoint3DReadOnly centerOfMass, FramePoint2DReadOnly cmp, double fZ, double totalMass, double omega0)
   {
      FramePoint3D pseudoCMP3d = new FramePoint3D();

      computePseudoCMP3d(pseudoCMP3d, centerOfMass, cmp, fZ, totalMass, omega0);

      return pseudoCMP3d;
   }

   /**
    * <img src=
    * "https://latex.codecogs.com/png.image?e_{CMP}=\begin{bmatrix}x_{CMP}\\y_{CMP}\\z_{CoM}-\frac{f_z}{m
    * \omega_0^2}\end{bmatrix}"/><br>
    * Where:<br>
    * <img src= "https://latex.codecogs.com/png.image?\omega_0=\sqrt{\frac{g}{z_0}}"/>
    */
   public static void computePseudoCMP3d(FramePoint3DBasics pseudoCMP3dToPack,
                                         FramePoint3DReadOnly centerOfMass,
                                         FramePoint2DReadOnly cmp,
                                         double fZ,
                                         double totalMass,
                                         double omega0)
   {
      double zCMP = centerOfMass.getZ() - fZ / (totalMass * MathTools.square(omega0));
      pseudoCMP3dToPack.setIncludingFrame(cmp.getReferenceFrame(), cmp.getX(), cmp.getY(), 0.0);
      pseudoCMP3dToPack.changeFrame(centerOfMass.getReferenceFrame());
      pseudoCMP3dToPack.setZ(zCMP);
   }

   public static FrameVector3D computeForce(FramePoint3DReadOnly centerOfMass, FramePoint3D cmp, double fZ)
   {
      FrameVector3D force = new FrameVector3D(centerOfMass);

      computeForce(force, centerOfMass, cmp, fZ);

      return force;
   }

   public static void computeForce(FrameVector3D forceToPack, FramePoint3DReadOnly centerOfMass, FramePoint3DBasics cmp, double fZ)
   {
      cmp.changeFrame(centerOfMass.getReferenceFrame());
      forceToPack.setIncludingFrame(centerOfMass);
      forceToPack.sub(cmp);
      forceToPack.scale(fZ / forceToPack.getZ());
   }

   public static void getSupportVectors(List<FrameVector3D> normalizedSupportVectorsToPack, double mu, ReferenceFrame contactPlaneFrame)
   {
      int numberOfSupportVectors = normalizedSupportVectorsToPack.size();
      double angleIncrement = 2.0 * Math.PI / ((double) numberOfSupportVectors);

      for (int i = 0; i < numberOfSupportVectors; i++)
      {
         double angle = i * angleIncrement;
         getSupportVector(normalizedSupportVectorsToPack.get(i), angle, mu, contactPlaneFrame);
      }
   }

   public static void getSupportVector(FrameVector3D normalizedSupportVectorToPack, double angle, double mu, ReferenceFrame contactPlaneFrame)
   {
      double x = mu * Math.cos(angle);
      double y = mu * Math.sin(angle);
      double z = 1.0;

      normalizedSupportVectorToPack.setIncludingFrame(contactPlaneFrame, x, y, z);
      normalizedSupportVectorToPack.normalize();
   }

   public static void computeSupportVectorMatrixBlock(DMatrixRMaj supportVectorMatrixBlock,
                                                      ArrayList<FrameVector3D> normalizedSupportVectors,
                                                      ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < normalizedSupportVectors.size(); i++)
      {
         FrameVector3D normalizedSupportVector = normalizedSupportVectors.get(i);
         normalizedSupportVector.changeFrame(referenceFrame);
         normalizedSupportVector.get(0, i, supportVectorMatrixBlock);
      }
   }

   public static double computeFz(double totalMass, double gravityZ, double centerOfMassHeightAcceleration)
   {
      double fZ = totalMass * (gravityZ + centerOfMassHeightAcceleration);
      return fZ;
   }
}
