/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Creates an alpha filter defined by:
 * qdFiltered = alpha*qd_from_IMU_estimate + (1-alpha)*qdFromEncoders
 * In which qdFiltered is defined as the filtered version of:
 * qdFromEncoders = {qd_WaistRotator, qd_WaistExtensor, qd_WaistLateralExtensor}
 * call getEncoderVelocityEstimates for output of the filter.
 */
public class IMUBasedJointStateEstimator
{
   private final IMUBasedJointVelocityEstimator velocityEstimator;
   private final DoubleProvider velocityBreakFrequency;
   private final DoubleProvider positionBreakFrequency;
   private final GeometricJacobian jacobian;
   private final SensorOutputMapReadOnly sensorMap;
   private final Map<OneDoFJoint, BacklashProcessingYoVariable> jointVelocities = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositions = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositionsFromIMUOnly = new LinkedHashMap<>();
   private final OneDoFJoint[] joints;

   private final double estimatorDT;

   public IMUBasedJointStateEstimator(IMUSensorReadOnly pelvisIMU, IMUSensorReadOnly chestIMU, SensorOutputMapReadOnly sensorMap,
                                      StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry registry)
   {
      this.sensorMap = sensorMap;
      jacobian = new GeometricJacobian(pelvisIMU.getMeasurementLink(), chestIMU.getMeasurementLink(), chestIMU.getMeasurementLink().getBodyFixedFrame());
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      this.velocityEstimator = new IMUBasedJointVelocityEstimator(jacobian, pelvisIMU, chestIMU, registry);

      String namePrefix = "imuBasedJointVelocityEstimator";
      velocityBreakFrequency = new DoubleParameter(namePrefix + "AlphaFuseVelocity", registry, stateEstimatorParameters.getBreakFrequencyForSpineJointVelocityEstimation());
      positionBreakFrequency = new DoubleParameter(namePrefix + "AlphaFusePosition", registry, stateEstimatorParameters.getBreakFrequencyForSpineJointPositionEstimation());

      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();

      DoubleProvider slopTime = new DoubleParameter(namePrefix + "SlopTime", registry, stateEstimatorParameters.getIMUJointVelocityEstimationBacklashSlopTime());
      for (OneDoFJoint joint : joints)
      {
         jointVelocities.put(joint, new BacklashProcessingYoVariable("qd_" + joint.getName() + "_FusedWithIMU", "", estimatorDT, slopTime, registry));

         jointPositionsFromIMUOnly.put(joint, new YoDouble("q_" + joint.getName() + "_IMUBased", registry));
         jointPositions.put(joint, new YoDouble("q_" + joint.getName() + "_FusedWithIMU", registry));
      }
   }

   public void compute()
   {
      velocityEstimator.compute();

      double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(velocityBreakFrequency.getValue(), estimatorDT);
      double alphaPosition = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(positionBreakFrequency.getValue(), estimatorDT);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         double qd_sensorMap = sensorMap.getJointVelocityProcessedOutput(joint);
         double qd_IMU = velocityEstimator.getEstimatedJointVelocity(i);
         double qd_fused = (1.0 - alphaVelocity) * qd_sensorMap + alphaVelocity * qd_IMU;

         jointVelocities.get(joint).update(qd_fused);

         double q_sensorMap = sensorMap.getJointPositionProcessedOutput(joint);
         double q_IMU = jointPositions.get(joint).getDoubleValue() + estimatorDT * qd_IMU; // is qd_IMU or qd_fused better here?
         double q_fused = (1.0 - alphaPosition) * q_sensorMap + alphaPosition * q_IMU;

         jointPositionsFromIMUOnly.get(joint).set(q_IMU);
         jointPositions.get(joint).set(q_fused);
      }
   }

   public double getEstimatedJointVelocitiy(OneDoFJoint joint)
   {
      BacklashProcessingYoVariable estimatedJointVelocity = jointVelocities.get(joint);
      if (estimatedJointVelocity != null)
         return estimatedJointVelocity.getDoubleValue();
      else
         return Double.NaN;
   }

   public double getEstimatedJointPosition(OneDoFJoint joint)
   {
      YoDouble estimatedJointPosition = jointPositions.get(joint);
      if (estimatedJointPosition != null)
         return estimatedJointPosition.getDoubleValue();
      else
         return Double.NaN;
   }
}
