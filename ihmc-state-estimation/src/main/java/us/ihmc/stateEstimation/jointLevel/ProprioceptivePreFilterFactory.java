package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Collection;
import java.util.List;

public class ProprioceptivePreFilterFactory
{
   private ProprioceptivePreFilterFactory()
   {

   }

   //TODO: fix void return
   public static void create(SensorOutputMapReadOnly sensorOutputMap,
                                                StateEstimatorParameters stateEstimatorParameters,
                                                List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                                Collection<? extends RigidBodyBasics> feet,
                                                double gravitationalAcceleration,
                                                BooleanProvider cancelGravityFromAccelerationMeasurement,
                                                double estimatorDT,
                                                YoRegistry parentRegistry)
   {
//      return switch (stateEstimatorParameters.getJointLevelEstimatorType())
//      {
//         case ALPHA_COMPLEMENTARY -> AlphaComplementaryPreFilter.createForKinematicsEstimator();
//         case JOINT_KF -> throw new UnsupportedOperationException("JointLevelKFPreFilter not yet added.");
//      };
   }
}
