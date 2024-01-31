package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.ejml.data.DMatrix1Row;

import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ICPControllerHelper
{
   private static final double epsilonZeroICPVelocity = MathTools.square(1e-5);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Vector2dZUpFrame icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

   private final FrameMatrix3D frameMatrix3D = new FrameMatrix3D();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FrameVector2D tempVector = new FrameVector2D();

   public double transformGainsFromDynamicsFrame(DMatrix1Row feedbackGainsToPack,
                                                 FrameVector2DReadOnly desiredICPVelocity,
                                                 double parallelGain,
                                                 double orthogonalGain)
   {
      return transformFromDynamicsFrame(feedbackGainsToPack, desiredICPVelocity, parallelGain + 1.0, orthogonalGain + 1.0);
   }

   public double transformFromDynamicsFrame(DMatrix1Row valuesToPack, FrameVector2DReadOnly desiredICPVelocity, double parallelValue, double orthogonalValue)
   {
      if (!isStationary(desiredICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);

         transformValues(valuesToPack, parallelValue, orthogonalValue, icpVelocityDirectionFrame, worldFrame);

         return EuclidCoreTools.norm(parallelValue, orthogonalValue);
      }
      else
      {
         valuesToPack.set(0, 0, orthogonalValue);
         valuesToPack.set(0, 1, 0.0);
         valuesToPack.set(1, 0, 0.0);
         valuesToPack.set(1, 1, orthogonalValue);

         return Math.sqrt(2.0 * orthogonalValue * orthogonalValue);
      }
   }

   public void getTransformToDynamicsFrame(DMatrix1Row transformToPack, FrameVector2DReadOnly desiredICPVelocity)
   {
      if (!isStationary(desiredICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         RigidBodyTransform transformToWorld = icpVelocityDirectionFrame.getTransformToRoot();
         transformToWorld.invert();

         transformToPack.set(0, 0, transformToWorld.getRotation().getElement(0, 0));
         transformToPack.set(0, 1, transformToWorld.getRotation().getElement(0, 1));
         transformToPack.set(1, 0, transformToWorld.getRotation().getElement(1, 0));
         transformToPack.set(1, 1, transformToWorld.getRotation().getElement(1, 1));
      }
      else
      {
         CommonOps_DDRM.setIdentity(transformToPack);
      }
   }

   public static boolean isStationary(FrameVector2DReadOnly desiredICPVelocity)
   {
      return desiredICPVelocity.normSquared() < MathTools.square(epsilonZeroICPVelocity);
   }

   // FIXME this is wrong.
   public void transformFromDynamicsFrame(FixedFrameVector2DBasics valuesToPack,
                                          FrameVector2DReadOnly desiredICPVelocity,
                                          double parallelValue,
                                          double orthogonalValue)
   {

      if (!isStationary(desiredICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         icpVelocityDirectionFrame.getTransformToDesiredFrame(tempTransform, worldFrame);

         tempVector.setIncludingFrame(icpVelocityDirectionFrame, parallelValue, orthogonalValue);
         tempVector.changeFrameAndProjectToXYPlane(valuesToPack.getReferenceFrame());
         if (Double.isNaN(tempVector.getX()))
            valuesToPack.setX(Double.POSITIVE_INFINITY);
         else
            valuesToPack.setX(tempVector.getX());
         if (Double.isNaN(tempVector.getY()))
            valuesToPack.setY(Double.POSITIVE_INFINITY);
         else
            valuesToPack.setY(tempVector.getY());
      }
      else
      {
         valuesToPack.setX(orthogonalValue);
         valuesToPack.setY(orthogonalValue);
      }
   }

   private void transformValues(DMatrix1Row valuesToPack, double xValue, double yValue, ReferenceFrame currentFrame, ReferenceFrame desiredFrame)
   {
      frameMatrix3D.setToZero(currentFrame);
      frameMatrix3D.setM00(xValue);
      frameMatrix3D.setM11(yValue);
      frameMatrix3D.changeFrame(desiredFrame);

      valuesToPack.set(0, 0, frameMatrix3D.getElement(0, 0));
      valuesToPack.set(0, 1, frameMatrix3D.getElement(0, 1));
      valuesToPack.set(1, 0, frameMatrix3D.getElement(1, 0));
      valuesToPack.set(1, 1, frameMatrix3D.getElement(1, 1));
   }

   public static class Vector2dZUpFrame extends ReferenceFrame
   {
      private final FrameVector2D xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2D(parentFrame);
      }

      public void setXAxis(FrameTuple2DReadOnly xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(getParent());
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumns(x, y, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
