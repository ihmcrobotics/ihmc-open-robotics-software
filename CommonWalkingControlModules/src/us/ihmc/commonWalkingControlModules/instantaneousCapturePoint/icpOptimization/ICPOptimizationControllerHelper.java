package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class ICPOptimizationControllerHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Vector2dZUpFrame icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

   private static final Matrix3d matrix = new Matrix3d();
   private static final Matrix3d matrixTransformed = new Matrix3d();

   public static void transformFeedbackGains(FrameVector2d feedbackGainsToPack, FrameVector2d desiredICPVelocity, DoubleYoVariable parallelGain, DoubleYoVariable orthogonalGain)
   {
      double epsilonZeroICPVelocity = 1e-5;

      if (desiredICPVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         RigidBodyTransform transform = icpVelocityDirectionFrame.getTransformToWorldFrame();

         transform.getRotation(rotation);
         rotationTranspose.set(rotation);
         rotationTranspose.transpose();
         transformTranspose.setRotation(rotationTranspose);

         matrix.setZero();
         matrix.setElement(0, 0, 1.0 + parallelGain.getDoubleValue());
         matrix.setElement(1, 1, 1.0 + orthogonalGain.getDoubleValue());

         matrixTransformed.set(rotation);
         matrixTransformed.mul(matrix);
         matrixTransformed.mul(rotationTranspose);

         feedbackGainsToPack.setToZero(worldFrame);
         feedbackGainsToPack.setX(matrixTransformed.getElement(0, 0));
         feedbackGainsToPack.setY(matrixTransformed.getElement(1, 1));
      }
      else
      {
         feedbackGainsToPack.setToZero(worldFrame);
         feedbackGainsToPack.set(1.0 + orthogonalGain.getDoubleValue(), 1.0 + orthogonalGain.getDoubleValue());
      }
   }

   private static final RigidBodyTransform transformTranspose = new RigidBodyTransform();
   private static final Matrix3d rotation = new Matrix3d();
   private static final Matrix3d rotationTranspose = new Matrix3d();

   public static void transformWeightsToWorldFrame(FrameVector2d weightsToPack, DoubleYoVariable xWeight, DoubleYoVariable yWeight, ReferenceFrame frame)
   {
      RigidBodyTransform transformToWorldFrame = frame.getTransformToWorldFrame();

      transformToWorldFrame.getRotation(rotation);
      rotationTranspose.set(rotation);
      rotation.transpose();
      transformTranspose.setRotation(rotationTranspose);

      matrix.setZero();
      matrix.setElement(0, 0, xWeight.getDoubleValue());
      matrix.setElement(1, 1, yWeight.getDoubleValue());

      matrixTransformed.set(rotation);
      matrixTransformed.mul(matrix);
      matrixTransformed.mul(rotationTranspose);

      weightsToPack.setToZero(worldFrame);
      weightsToPack.setX(matrixTransformed.getElement(0, 0));
      weightsToPack.setY(matrixTransformed.getElement(1, 1));

   }

   private static class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3d x = new Vector3d();
      private final Vector3d y = new Vector3d();
      private final Vector3d z = new Vector3d();
      private final Matrix3d rotation = new Matrix3d();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
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

         rotation.setColumn(0, x);
         rotation.setColumn(1, y);
         rotation.setColumn(2, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
