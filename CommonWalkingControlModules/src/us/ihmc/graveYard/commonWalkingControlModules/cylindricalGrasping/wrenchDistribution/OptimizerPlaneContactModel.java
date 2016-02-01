package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public class OptimizerPlaneContactModel implements OptimizerContactModel
{
   private double rhoMin;
   private static final int VECTORS = 4;
   private static final int MAXPOINTS = 6; //TODO Inconsistent with solver parameters
   private int numberOfPointsInContact = MAXPOINTS;
   private static final int MAX_RHO_SIZE = MAXPOINTS * VECTORS;
   private static final double ANGLE_INCREMENT = 2 * Math.PI / (VECTORS);
   private double mu = 0.3;
   private double wRho;
   private final DenseMatrix64F[] rhoQ = new DenseMatrix64F[MAX_RHO_SIZE];
   private final SpatialForceVector tempForceVector = new SpatialForceVector();
   private final Matrix3d tempTransformLinearPart = new Matrix3d();
   private final Vector3d tempLinearPart = new Vector3d();
   private final Vector3d tempArm = new Vector3d();
   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempContactNormalVector = new FrameVector();

   public OptimizerPlaneContactModel()
   {
      for (int i = 0; i < MAXPOINTS; i++)
      {
         for (int j = 0; j < VECTORS; j++)
         {
            int rhoPosition = i * VECTORS + j;
            rhoQ[rhoPosition] = new DenseMatrix64F(6, 1);
         }
      }
   }

   private final AxisAngle4d normalContactVectorRotation = new AxisAngle4d();

   public void setup(PlaneContactState plane, double wRho, double rhoMin)
   {
      if (!plane.inContact())
         return;

      this.mu = plane.getCoefficientOfFriction();
      numberOfPointsInContact = plane.getNumberOfContactPointsInContact();
      plane.getContactNormalFrameVector(tempContactNormalVector);
      tempContactNormalVector.changeFrame(plane.getPlaneFrame());
      tempContactNormalVector.normalize();
      GeometryTools.getRotationBasedOnNormal(normalContactVectorRotation, tempContactNormalVector.getVector());

      if (numberOfPointsInContact > MAXPOINTS)
      {
         throw new RuntimeException("Unhandled number of contact points: " + numberOfPointsInContact);
      }

      int i = -1;

      for (ContactPointInterface contactPoint : plane.getContactPoints())
      {
         if (!contactPoint.isInContact())
            continue;

         i++;

         contactPoint.getPosition(tempFramePoint);

         for (int j = 0; j < VECTORS; j++)
         {
            int rhoPosition = i * VECTORS + j;

            double angle = j * ANGLE_INCREMENT;

            // Compute the linear part considering a normal contact vector pointing up
            tempLinearPart.set(Math.cos(angle) * mu, Math.sin(angle) * mu, 1);

            // Transforming the result to consider the actual normal contact vector
            tempTransformLinearPart.set(normalContactVectorRotation);
            tempTransformLinearPart.transform(tempLinearPart);
            tempLinearPart.normalize();

            tempArm.set(tempFramePoint.getX(), tempFramePoint.getY(), tempFramePoint.getZ());
            tempForceVector.setUsingArm(plane.getPlaneFrame(), tempLinearPart, tempArm);
            tempForceVector.changeFrame(plane.getFrameAfterParentJoint());

            tempForceVector.packMatrix(rhoQ[rhoPosition]);
         }
      }

      this.wRho = wRho;
      this.rhoMin = rhoMin;
   }

   @Deprecated
   public void setup(double coefficientOfFriction, List<FramePoint> contactPoints, FrameVector normalContactVector, ReferenceFrame endEffectorFrame,
                     double wRho, double rhoMin)
   {
      this.mu = coefficientOfFriction;
      numberOfPointsInContact = contactPoints.size();
      tempContactNormalVector.setIncludingFrame(normalContactVector);
      tempContactNormalVector.changeFrame(contactPoints.get(0).getReferenceFrame());
      tempContactNormalVector.normalize();
      GeometryTools.getRotationBasedOnNormal(normalContactVectorRotation, tempContactNormalVector.getVector());

      if (numberOfPointsInContact > MAXPOINTS)
      {
         throw new RuntimeException("Unhandled number of contact points: " + numberOfPointsInContact);
      }

      for (int i = 0; i < numberOfPointsInContact; i++)
      {
         tempFramePoint.setIncludingFrame(contactPoints.get(i));

         for (int j = 0; j < VECTORS; j++)
         {
            int rhoPosition = i * VECTORS + j;

            double angle = j * ANGLE_INCREMENT;

            // Compute the linear part considering a normal contact vector pointing up
            tempLinearPart.set(Math.cos(angle) * mu, Math.sin(angle) * mu, 1);

            // Transforming the result to consider the actual normal contact vector
            tempTransformLinearPart.set(normalContactVectorRotation);
            tempTransformLinearPart.transform(tempLinearPart);
            tempLinearPart.normalize();

            tempArm.set(tempFramePoint.getX(), tempFramePoint.getY(), tempFramePoint.getZ());
            tempForceVector.setUsingArm(tempFramePoint.getReferenceFrame(), tempLinearPart, tempArm);
            tempForceVector.changeFrame(endEffectorFrame);

            tempForceVector.packMatrix(rhoQ[rhoPosition]);
         }
      }

      this.wRho = wRho;
      this.rhoMin = rhoMin;
   }

   
   public int getRhoSize()
   {
      return VECTORS * numberOfPointsInContact;
   }

   
   public int getPhiSize()
   {
      return 0;
   }

   
   public double getRhoMin(int i)
   {
      return rhoMin;
   }

   
   public double getPhiMin(int i)
   {
      return 0;
   }

   
   public double getPhiMax(int i)
   {
      return 0;
   }

   
   public void packQRhoBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
      spatialForceVector.set(referenceFrame, rhoQ[i]);
   }

   
   public void packQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
   }

   
   public double getWPhi()
   {
      return Double.NaN;
   }

   
   public double getWRho()
   {
      return wRho;
   }

}
