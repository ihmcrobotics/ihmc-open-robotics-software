package us.ihmc.stateEstimation.jointLevel;

import java.util.Collection;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Test-only bridge that exposes the package-private {@link JointLevelKFPreFilter} constructor to tests in
 * other packages (e.g. the combined estimator test in {@code us.ihmc.stateEstimation.invariant_estimator}).
 * This lives in the {@code jointLevel} package so it can reach the constructor without going through
 * {@link JointLevelKFPreFilter#createForKinematicsEstimator} (which needs a full {@code StateEstimatorParameters}).
 * Not for production use.
 */
public final class JointLevelKFTestSupport
{
   private JointLevelKFTestSupport()
   {
   }

   public static JointLevelKFPreFilter newPreFilter(SensorOutputMapReadOnly sensorMap,
                                                    List<IMUBasedJointStateEstimatorParameters> pairParameters,
                                                    Collection<RigidBodyBasics> feet,
                                                    double dt,
                                                    YoRegistry parentRegistry)
   {
      return new JointLevelKFPreFilter(sensorMap, pairParameters, feet, dt, parentRegistry);
   }

   /** Variant with a robot model root body, enabling the mass-matrix process noise Qa = sigma_tau^2 M(q)^-2. */
   public static JointLevelKFPreFilter newPreFilter(SensorOutputMapReadOnly sensorMap,
                                                    List<IMUBasedJointStateEstimatorParameters> pairParameters,
                                                    Collection<RigidBodyBasics> feet,
                                                    RigidBodyBasics rootBody,
                                                    double dt,
                                                    YoRegistry parentRegistry)
   {
      return new JointLevelKFPreFilter(sensorMap, pairParameters, feet, rootBody, dt, parentRegistry);
   }
}
