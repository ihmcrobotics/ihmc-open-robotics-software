package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPOptimizationControllerHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Vector2dZUpFrame icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

   private static final Matrix3D matrix = new Matrix3D();
   private static final Matrix3D matrixTransformed = new Matrix3D();

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

         matrix.setToZero();
         matrix.setElement(0, 0, 1.0 + parallelGain.getDoubleValue());
         matrix.setElement(1, 1, 1.0 + orthogonalGain.getDoubleValue());

         matrixTransformed.set(rotation);
         matrixTransformed.multiply(matrix);
         matrixTransformed.multiply(rotationTranspose);

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
   private static final RotationMatrix rotation = new RotationMatrix();
   private static final RotationMatrix rotationTranspose = new RotationMatrix();

   public static void transformWeightsToWorldFrame(FrameVector2d weightsToPack, DoubleYoVariable xWeight, DoubleYoVariable yWeight, ReferenceFrame frame)
   {
      RigidBodyTransform transformToWorldFrame = frame.getTransformToWorldFrame();

      transformToWorldFrame.getRotation(rotation);
      rotationTranspose.set(rotation);
      rotation.transpose();
      transformTranspose.setRotation(rotationTranspose);

      matrix.setToZero();
      matrix.setElement(0, 0, xWeight.getDoubleValue());
      matrix.setElement(1, 1, yWeight.getDoubleValue());

      matrixTransformed.set(rotation);
      matrixTransformed.multiply(matrix);
      matrixTransformed.multiply(rotationTranspose);

      weightsToPack.setToZero(worldFrame);
      weightsToPack.setX(matrixTransformed.getElement(0, 0));
      weightsToPack.setY(matrixTransformed.getElement(1, 1));

   }

   private static class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

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

         rotation.setColumns(x, y, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
