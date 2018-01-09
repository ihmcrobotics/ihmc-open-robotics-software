package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICPOptimizationControllerHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Vector2dZUpFrame icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

   private final Matrix3D matrix = new Matrix3D();
   private final Matrix3D matrixTransformed = new Matrix3D();

   private final RigidBodyTransform transformTranspose = new RigidBodyTransform();
   private final RotationMatrix rotation = new RotationMatrix();
   private final RotationMatrix rotationTranspose = new RotationMatrix();

   public void transformFromDynamicsFrame(FrameVector2D feedbackGainsToPack, FrameVector2D desiredICPVelocity, YoDouble parallelGain, YoDouble orthogonalGain)
   {
      transformFromDynamicsFrame(feedbackGainsToPack, desiredICPVelocity, parallelGain.getDoubleValue(), orthogonalGain.getDoubleValue());
   }

   public void transformFromDynamicsFrame(FrameVector2D feedbackGainsToPack, FrameVector2D desiredICPVelocity, double parallelGain, double orthogonalGain)
   {
      double epsilonZeroICPVelocity = 1e-5;

      if (desiredICPVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         RigidBodyTransform transform = icpVelocityDirectionFrame.getTransformToWorldFrame();

         transformValues(feedbackGainsToPack, 1.0 + parallelGain, 1.0 + orthogonalGain, transform);
      }
      else
      {
         feedbackGainsToPack.setToZero(worldFrame);
         feedbackGainsToPack.set(1.0 + orthogonalGain, 1.0 + orthogonalGain);
      }
   }


   public void transformToWorldFrame(FrameVector2D weightsToPack, YoDouble xWeight, YoDouble yWeight, ReferenceFrame frame)
   {
      transformValues(weightsToPack, xWeight.getValue(), yWeight.getValue(), frame.getTransformToWorldFrame());
   }

   private void transformValues(FrameTuple2D valuesToPack, double xValue, double yValue, RigidBodyTransform transformToDesiredFrame)
   {
      transformToDesiredFrame.getRotation(rotation);
      rotationTranspose.set(rotation);
      rotation.transpose();
      transformTranspose.setRotation(rotationTranspose);

      matrix.setToZero();
      matrix.setElement(0, 0, xValue);
      matrix.setElement(1, 1, yValue);

      matrixTransformed.set(rotation);
      matrixTransformed.multiply(matrix);
      matrixTransformed.multiply(rotationTranspose);

      valuesToPack.setToZero(worldFrame);
      valuesToPack.setX(matrixTransformed.getElement(0, 0));
      valuesToPack.setY(matrixTransformed.getElement(1, 1));
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

      public void setXAxis(FrameVector2D xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
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
