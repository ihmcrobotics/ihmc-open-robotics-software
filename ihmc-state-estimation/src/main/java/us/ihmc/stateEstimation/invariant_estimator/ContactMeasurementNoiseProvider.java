package us.ihmc.stateEstimation.invariant_estimator;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Supplies the 3x3 body(pelvis)-frame contact FK measurement covariance for one side, queried once
 * per contact per tick before the contact-probability inflation is applied on top.
 *
 * <p>The two knobs compose: this provider captures joint-sensing uncertainty (constant floor, or
 * J Sigma_q J^T routed from a joint-level pre-filter with covariance); the probability-based
 * inflation in the update loop captures contact-state uncertainty.</p>
 */
public interface ContactMeasurementNoiseProvider
{
   /**
    * Packs the per-contact measurement covariance in the body (pelvis) frame.
    *
    * @param side             which foot's contact measurement.
    * @param covarianceToPack the 3x3 covariance. Modified.
    */
   void packContactCovariance(RobotSide side, Matrix3D covarianceToPack);
}
