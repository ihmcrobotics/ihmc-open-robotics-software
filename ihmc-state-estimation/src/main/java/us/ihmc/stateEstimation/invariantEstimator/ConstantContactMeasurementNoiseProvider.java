package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Default {@link ContactMeasurementNoiseProvider}: the historical constant isotropic diagonal
 * built from {@code contactMeasurementVariance}. Reproduces the pre-seam behavior exactly and is
 * the fallback whenever the joint-level pre-filter offers no covariance.
 */
public class ConstantContactMeasurementNoiseProvider implements ContactMeasurementNoiseProvider
{
   private final Matrix3D constantCovariance = new Matrix3D();

   public ConstantContactMeasurementNoiseProvider(double contactMeasurementVariance)
   {
      constantCovariance.setToZero();
      constantCovariance.setM00(contactMeasurementVariance);
      constantCovariance.setM11(contactMeasurementVariance);
      constantCovariance.setM22(contactMeasurementVariance);
   }

   @Override
   public void packContactCovariance(RobotSide side, Matrix3D covarianceToPack)
   {
      covarianceToPack.set(constantCovariance);
   }
}
