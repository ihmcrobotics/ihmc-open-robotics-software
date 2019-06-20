package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import org.ejml.data.D1Matrix64F;
import org.ejml.data.RowD1Matrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ICPOptimizationControllerHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Vector2dZUpFrame icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

   private final FrameMatrix3D frameMatrix3D = new FrameMatrix3D();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();


   public double transformGainsFromDynamicsFrame(RowD1Matrix64F feedbackGainsToPack, FixedFrameVector2DBasics desiredICPVelocity, double parallelGain,
                                               double orthogonalGain)
   {
      return transformFromDynamicsFrame(feedbackGainsToPack, desiredICPVelocity, parallelGain + 1.0, orthogonalGain + 1.0);
   }

   public double transformFromDynamicsFrame(RowD1Matrix64F valuesToPack, FixedFrameVector2DBasics desiredICPVelocity, double parallelValue,
                                          double orthogonalValue)
   {
      double epsilonZeroICPVelocity = 1e-5;

      if (desiredICPVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);

         transformValues(valuesToPack, parallelValue, orthogonalValue, icpVelocityDirectionFrame, worldFrame);

         return Math.sqrt(parallelValue * parallelValue + orthogonalValue * orthogonalValue);
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

   private final FrameVector2D tempVector = new FrameVector2D();
   public void transformFromDynamicsFrame(FixedFrameVector2DBasics valuesToPack, FixedFrameVector2DBasics desiredICPVelocity, double parallelValue,
                                          double orthogonalValue)
   {
      double epsilonZeroICPVelocity = 1e-5;

      if (desiredICPVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         icpVelocityDirectionFrame.getTransformToDesiredFrame(tempTransform, worldFrame);

         tempVector.setIncludingFrame(icpVelocityDirectionFrame, parallelValue, orthogonalValue);
         tempVector.changeFrameAndProjectToXYPlane(valuesToPack.getReferenceFrame());
         valuesToPack.set(tempVector);
      }
      else
      {
         valuesToPack.setX(orthogonalValue);
         valuesToPack.setY(orthogonalValue);
      }
   }

   public void transformToWorldFrame(D1Matrix64F weightsToPack, double xValue, double yValue, ReferenceFrame frame)
   {
      transformValues(weightsToPack, xValue, yValue, frame, worldFrame);
   }

   private void transformValues(D1Matrix64F valuesToPack, double xValue, double yValue, ReferenceFrame currentFrame, ReferenceFrame desiredFrame)
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

   private class Vector2dZUpFrame extends ReferenceFrame
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

      public void setXAxis(FixedFrameTuple2DBasics xAxis)
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
