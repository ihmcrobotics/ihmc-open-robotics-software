package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import org.ejml.data.D1Matrix64F;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class ICPOptimizationControllerHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Vector2dZUpFrame icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

   private final Matrix3D matrix = new Matrix3D();
   private final Matrix3D matrixTransformed = new Matrix3D();

   private final RotationMatrix rotation = new RotationMatrix();
   private final RotationMatrix rotationTranspose = new RotationMatrix();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void transformGainsFromDynamicsFrame(RowD1Matrix64F feedbackGainsToPack, FixedFrameVector2DBasics desiredICPVelocity, double parallelGain,
                                               double orthogonalGain)
   {
      transformFromDynamicsFrame(feedbackGainsToPack, desiredICPVelocity, parallelGain + 1.0, orthogonalGain + 1.0);
   }

   public void transformGainsFromDynamicsFrame(FrameVector2DBasics feedbackGainsToPack, FixedFrameVector2DBasics desiredICPVelocity, double parallelGain,
                                               double orthogonalGain)
   {
      transformFromDynamicsFrame(feedbackGainsToPack, desiredICPVelocity, parallelGain + 1.0, orthogonalGain + 1.0);
   }

   public void transformFromDynamicsFrame(RowD1Matrix64F valuesToPack, FixedFrameVector2DBasics desiredICPVelocity, double parallelValue,
                                          double orthogonalValue)
   {
      double epsilonZeroICPVelocity = 1e-5;

      if (desiredICPVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredICPVelocity);
         icpVelocityDirectionFrame.getTransformToDesiredFrame(tempTransform, worldFrame);

         transformValues(valuesToPack, parallelValue, orthogonalValue, tempTransform);
      }
      else
      {
         valuesToPack.set(0, 0, orthogonalValue);
         valuesToPack.set(0, 1, 0.0);
         valuesToPack.set(1, 0, 0.0);
         valuesToPack.set(1, 1, orthogonalValue);
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
      frame.getTransformToDesiredFrame(tempTransform, worldFrame);
      transformValues(weightsToPack, xValue, yValue, tempTransform);
   }

   public void transformToWorldFrame(FixedFrameVector2DBasics weightsToPack, double xValue, double yValue, ReferenceFrame frame)
   {
      frame.getTransformToDesiredFrame(tempTransform, worldFrame);
      tempVector.setIncludingFrame(frame, xValue, yValue);
      tempVector.changeFrame(weightsToPack.getReferenceFrame());
      weightsToPack.set(tempVector);
//      transformValues(weightsToPack, xValue, yValue, tempTransform);
   }

   private void transformValues(D1Matrix64F valuesToPack, double xValue, double yValue, RigidBodyTransform transformToDesiredFrame)
   {
      transformToDesiredFrame.getRotation(rotation);
      rotationTranspose.set(rotation);
      rotationTranspose.transpose();

      matrix.setToZero();
      matrix.setElement(0, 0, xValue);
      matrix.setElement(1, 1, yValue);

      matrixTransformed.set(rotation);
      matrixTransformed.multiply(matrix);
      matrixTransformed.multiply(rotationTranspose);

      valuesToPack.set(0, 0, matrixTransformed.getM00());
      valuesToPack.set(0, 1, matrixTransformed.getM01());
      valuesToPack.set(1, 0, matrixTransformed.getM10());
      valuesToPack.set(1, 1, matrixTransformed.getM11());
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
