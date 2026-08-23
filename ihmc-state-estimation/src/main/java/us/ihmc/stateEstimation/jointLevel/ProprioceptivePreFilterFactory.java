package us.ihmc.stateEstimation.jointLevel;

import org.apache.commons.lang3.NotImplementedException;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Collection;
import java.util.List;

/**
 * Construction-time dispatch for the joint-level pre-filter: consults
 * {@link StateEstimatorParameters#getJointLevelEstimatorType()} once and builds the corresponding
 * {@link ProprioceptivePreFilter}. This is the only class that knows all implementations; consumers
 * know only the interface, and each implementation owns its own assembly (its static factory).
 */
public final class ProprioceptivePreFilterFactory
{
   private ProprioceptivePreFilterFactory()
   {
   }

   /**
    * @param estimatorRootBody root body (elevator) of the estimator's robot model. Used by the JOINT_KF
    *                          implementation for the mass-matrix-induced process noise Qa = sigma_tau^2 M(q)^-2;
    *                          may be null, degrading that filter to its scalar-CWNA process noise. Ignored by
    *                          the other implementations.
    */
   public static ProprioceptivePreFilter create(SensorOutputMapReadOnly sensorOutputMap,
                                                StateEstimatorParameters stateEstimatorParameters,
                                                List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                                Collection<RigidBodyBasics> feet,
                                                RigidBodyBasics estimatorRootBody,
                                                double gravitationalAcceleration,
                                                BooleanProvider cancelGravityFromAccelerationMeasurement,
                                                double estimatorDT,
                                                YoRegistry parentRegistry)
   {
      return create(sensorOutputMap,
                    stateEstimatorParameters,
                    imuProcessedOutputs,
                    feet,
                    estimatorRootBody,
                    gravitationalAcceleration,
                    cancelGravityFromAccelerationMeasurement,
                    Double.NaN,
                    estimatorDT,
                    parentRegistry);
   }

   /**
    * @param sigmaTauOverride JOINT_KF-only boot-time override (N*m) for {@code jointKFParam_sigmaTau}; NaN
    *                         leaves the default. Ignored by the other {@code JointLevelEstimatorType}s. See
    *                         {@link JointLevelKFPreFilter}'s 11-arg constructor javadoc for why this has to be
    *                         a construction-time argument rather than a post-construction YoDouble edit --
    *                         useful for an offline NIS-based retuning sweep, where each candidate sigma_tau
    *                         needs a fresh filter instance built from a fresh log replay.
    */
   public static ProprioceptivePreFilter create(SensorOutputMapReadOnly sensorOutputMap,
                                                StateEstimatorParameters stateEstimatorParameters,
                                                List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                                Collection<RigidBodyBasics> feet,
                                                RigidBodyBasics estimatorRootBody,
                                                double gravitationalAcceleration,
                                                BooleanProvider cancelGravityFromAccelerationMeasurement,
                                                double sigmaTauOverride,
                                                double estimatorDT,
                                                YoRegistry parentRegistry)
   {
      // Switch expression, deliberately without a default arm: adding a new enum value makes this a
      // compile error here instead of a silent fallthrough.
      return switch (stateEstimatorParameters.getJointLevelEstimatorType())
      {
         case NONE -> new PassThroughPreFilter();
         case ALPHA_COMPLEMENTARY -> AlphaComplementaryPreFilter.createForKinematicsEstimator(sensorOutputMap,
                                                                                              stateEstimatorParameters,
                                                                                              imuProcessedOutputs,
                                                                                              feet,
                                                                                              gravitationalAcceleration,
                                                                                              cancelGravityFromAccelerationMeasurement,
                                                                                              estimatorDT,
                                                                                              parentRegistry);
         //TODO: Robert wants to move away from factories, might be worth just wiring this into ProprioceptivePrefilter anyways...
         case JOINT_KF -> JointLevelKFPreFilter.createForKinematicsEstimator(sensorOutputMap,
                                                                             stateEstimatorParameters,
                                                                             imuProcessedOutputs,
                                                                             feet,
                                                                             estimatorRootBody,
                                                                             gravitationalAcceleration,
                                                                             cancelGravityFromAccelerationMeasurement,
                                                                             sigmaTauOverride,
                                                                             estimatorDT,
                                                                             parentRegistry);
      };
   }
}
