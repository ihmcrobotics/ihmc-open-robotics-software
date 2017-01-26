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

/**
 * Modifies the derivative gain matrix to reduce the amount of derivative action along the main line of action for the feedback controller when there
 * is a large tracking error
 */
public class EuclideanTangentialDampingCalculator
{
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame bodyFrameTangentToControl;
   private final FrameVector positionError = new FrameVector();

   private final TangentialDampingGains tangentialDampingGains;

   private final FrameMatrix3D transformedGains;

   /**
    * @param prefix prefix to append to the reference frame name
    * @param bodyFrame frame to perform control in
    * @param tangentialDampingGains gains to use for the reduction
    */
   public EuclideanTangentialDampingCalculator(String prefix, final ReferenceFrame bodyFrame, TangentialDampingGains tangentialDampingGains)
   {
      this.bodyFrame = bodyFrame;
      this.tangentialDampingGains = tangentialDampingGains;

      bodyFrameTangentToControl = new ReferenceFrame(prefix + "BodyFrameTangentToControl", bodyFrame)
      {
         private static final long serialVersionUID = 8992154939350877111L;
         private final AxisAngle4d rotationToControlFrame = new AxisAngle4d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            positionError.changeFrame(bodyFrame);

            GeometryTools.getAxisAngleFromZUpToVector(positionError.getVector(), rotationToControlFrame);

            transformToParent.setRotationAndZeroTranslation(rotationToControlFrame);
         }
      };

      transformedGains = new FrameMatrix3D(bodyFrame);
   }

   /**
    * Computes the new derivative gains to use that reduces the amount of damping in the direction with the most position error.
    * This prevents a high desired correcting velocity from being penalized when it deviates from the objective motion.
    * @param positionError current position error being used by the feedback control
    * @param derivativeGainsToPack derivative gain matrix to use with less damping in the main direction of motion
    */
   public void compute(YoFrameVector positionError, Matrix3d derivativeGainsToPack)
   {
      this.positionError.setIncludingFrame(positionError.getFrameTuple());

      if (positionError.length() > tangentialDampingGains.getParallelDampingDeadband())
      {
         bodyFrameTangentToControl.update();

         double alpha = computeDampingReductionRatioParallelToMotion(positionError.length());

         transformedGains.setToZero(bodyFrame);
         transformedGains.set(derivativeGainsToPack);
         transformedGains.changeFrame(bodyFrameTangentToControl);
         transformedGains.setElement(0, 0, alpha * transformedGains.getElement(0, 0));
         transformedGains.setElement(1, 0, alpha * transformedGains.getElement(1, 0));
         transformedGains.setElement(2, 0, alpha * transformedGains.getElement(2, 0));

         transformedGains.changeFrame(bodyFrame);
         transformedGains.getMatrix(derivativeGainsToPack);
      }
   }

   private double computeDampingReductionRatioParallelToMotion(double parallelError)
   {
      double reductionRatio = MathTools.clipToMinMax(tangentialDampingGains.getKdReductionRatio(), 0.0, 1.0);
      double deadband = tangentialDampingGains.getParallelDampingDeadband();
      double maxError = tangentialDampingGains.getPositionErrorForMinimumKd();

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
