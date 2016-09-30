package us.ihmc.robotics.controllers;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameMatrix3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

public class EuclideanTangentialDampingCalculator
{
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame bodyFrameTangentToControl;
   private final FrameVector positionError = new FrameVector();

   private final DoubleYoVariable maxProportionalError;

   private final TangentialDampingGains tangentialDampingGains;

   private final FrameMatrix3D transformedGains;

   public EuclideanTangentialDampingCalculator(String prefix, ReferenceFrame bodyFrame, TangentialDampingGains tangentialDampingGains, DoubleYoVariable maxProportionalError)
   {
      this.bodyFrame = bodyFrame;
      this.maxProportionalError = maxProportionalError;
      this.tangentialDampingGains = tangentialDampingGains;

      bodyFrameTangentToControl = new ReferenceFrame(prefix + "BodyFrameTangentToControl", bodyFrame)
      {
         private static final long serialVersionUID = 8992154939350877111L;
         private final AxisAngle4d rotationToControlFrame = new AxisAngle4d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            positionError.changeFrame(bodyFrame);
            GeometryTools.getRotationBasedOnNormal(rotationToControlFrame, positionError.getVector());

            transformToParent.setRotationAndZeroTranslation(rotationToControlFrame);
         }
      };

      transformedGains = new FrameMatrix3D(bodyFrame);
   }

   public void compute(YoFrameVector positionError, Matrix3d derivativeGainsToPack)
   {
      this.positionError.setIncludingFrame(positionError.getFrameTuple());
      bodyFrameTangentToControl.update();

      double alpha = computeDampingReductionRatioParallelToMotion(positionError.getX());

      transformedGains.setToZero(bodyFrame);
      transformedGains.set(derivativeGainsToPack);
      transformedGains.changeFrame(bodyFrameTangentToControl);
      transformedGains.setElement(0, 0, alpha * transformedGains.getElement(0, 0));
      transformedGains.setElement(1, 0, alpha * transformedGains.getElement(1, 0));
      transformedGains.setElement(2, 0, alpha * transformedGains.getElement(2, 0));

      transformedGains.changeFrame(bodyFrame);

      transformedGains.getMatrix(derivativeGainsToPack);
   }

   private double computeDampingReductionRatioParallelToMotion(double parallelError)
   {
      double maxError = maxProportionalError.getDoubleValue();
      if (Double.isInfinite(maxError))
         maxError = 1.0;

      double reductionRatio = MathTools.clipToMinMax(tangentialDampingGains.getKdReductionRatio(), 0.0, 1.0);
      double deadband = tangentialDampingGains.getParallelDampingDeadband();

      if (Double.isInfinite(deadband) || deadband > maxError || (Math.abs(parallelError) < deadband))
      {
         return 1.0;
      }
      else
      {
         double slope = (1.0 - reductionRatio) / (maxError - deadband);

         return 1.0 - slope * (Math.abs(parallelError) - deadband);
      }
   }
}
