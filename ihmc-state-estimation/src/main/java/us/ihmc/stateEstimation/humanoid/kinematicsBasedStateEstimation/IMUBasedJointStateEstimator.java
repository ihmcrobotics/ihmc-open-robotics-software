/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import gnu.trove.impl.Constants;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFusedYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Creates an alpha filter defined by: qdFiltered = alpha*qd_from_IMU_estimate +
 * (1-alpha)*qdFromEncoders In which qdFiltered is defined as the filtered version of:
 * qdFromEncoders = {qd_WaistRotator, qd_WaistExtensor, qd_WaistLateralExtensor} call
 * getEncoderVelocityEstimates for output of the filter.
 */
public class IMUBasedJointStateEstimator
{
   private final YoRegistry registry;
   private final BooleanProvider enableOutput;
   private final IMUBasedJointVelocityEstimator velocityEstimator;
   private final DoubleProvider velocityBreakFrequency;
   private final DoubleProvider positionBreakFrequency;
   private final GeometricJacobian jacobian;
   private final SensorOutputMapReadOnly sensorMap;

   private final OneDoFJointBasics[] joints;
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(3, Constants.DEFAULT_LOAD_FACTOR, -1);
   private final BacklashProcessingYoVariable[] jointVelocities;
   private final YoDouble[] jointPositions;
   private final YoDouble[] jointPositionsFromIMUOnly;

   private final double estimatorDT;

   private final ButterworthFusedYoVariable[] newJointPositions;
   private final YoDouble butterAlphaLeak;
   private final YoDouble[] newIMUPositions;
   private final ButterworthFusedYoVariable[] newJointVelocities;

   private final YoBoolean useButterworthFuseFilter;

   public IMUBasedJointStateEstimator(double estimatorDT,
                                      IMUSensorReadOnly parentIMU,
                                      IMUSensorReadOnly childIMU,
                                      SensorOutputMapReadOnly sensorMap,
                                      IMUBasedJointStateEstimatorParameters parameters,
                                      YoRegistry parentRegistry)
   {
      this.estimatorDT = estimatorDT;
      this.sensorMap = sensorMap;

      String name = parameters.getEstimatorName() + getClass().getSimpleName();
      registry = new YoRegistry(name);

      enableOutput = new BooleanParameter(name + "EnableOutput", registry, parameters.isOuputEnabled());

      jacobian = new GeometricJacobian(parentIMU.getMeasurementLink(), childIMU.getMeasurementLink(), childIMU.getMeasurementLink().getBodyFixedFrame());
      joints = MultiBodySystemTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJointBasics.class);
      this.velocityEstimator = new IMUBasedJointVelocityEstimator(jacobian, parentIMU, childIMU, registry);

      velocityBreakFrequency = new DoubleParameter(name + "AlphaFuseVelocity", registry, parameters.getBreakFrequencyForVelocityEstimation());
      positionBreakFrequency = new DoubleParameter(name + "AlphaFusePosition", registry, parameters.getBreakFrequencyForPositionEstimation());

      DoubleProvider slopTime = new DoubleParameter(name + "SlopTime", registry, parameters.getVelocityEstimationBacklashSlopTime());

      useButterworthFuseFilter = new YoBoolean(name + "UseButterworthFuseFilter", registry);
      useButterworthFuseFilter.set(true);

      jointVelocities = new BacklashProcessingYoVariable[joints.length];
      jointPositionsFromIMUOnly = new YoDouble[joints.length];
      jointPositions = new YoDouble[joints.length];

      newIMUPositions = new YoDouble[joints.length];
      newJointPositions = new ButterworthFusedYoVariable[joints.length];
      newJointVelocities = new ButterworthFusedYoVariable[joints.length];
      DoubleProvider butterAlphaPosition = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(velocityBreakFrequency.getValue(), estimatorDT);
      DoubleProvider butterAlphaVelocity = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(positionBreakFrequency.getValue(), estimatorDT);
      butterAlphaLeak = new YoDouble(name + "fooIMUPositionLeak", registry);
      butterAlphaLeak.set(1.0e-4);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         jointToIndexMap.put(joint, i);
         jointVelocities[i] = new BacklashProcessingYoVariable("qd_" + joint.getName() + "_FusedWithIMU", "", estimatorDT, slopTime, registry);
         jointPositionsFromIMUOnly[i] = new YoDouble("q_" + joint.getName() + "_IMUBased", registry);
         jointPositions[i] = new YoDouble("q_" + joint.getName() + "_FusedWithIMU", registry);

         newIMUPositions[i] = new YoDouble("foo_q_imu_" + joint.getName(), registry);
         newJointPositions[i] = new ButterworthFusedYoVariable("foo_q_" + joint.getName(), registry, butterAlphaPosition, null, null);
         newJointVelocities[i] = new ButterworthFusedYoVariable("foo_qd_" + joint.getName(), registry, butterAlphaVelocity, null, null);
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      velocityEstimator.compute();

      double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(velocityBreakFrequency.getValue(), estimatorDT);
      double alphaPosition = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(positionBreakFrequency.getValue(), estimatorDT);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         OneDoFJointStateReadOnly jointSensorOutput = sensorMap.getOneDoFJointOutput(joint);

         double qd_sensorMap = jointSensorOutput.getVelocity();
         double qd_IMU = velocityEstimator.getEstimatedJointVelocity(i);

         double q_sensorMap = jointSensorOutput.getPosition();

         if (!useButterworthFuseFilter.getValue())
         {
            double qd_fused = (1.0 - alphaVelocity) * qd_sensorMap + alphaVelocity * qd_IMU;
            jointVelocities[i].update(qd_fused);
            
            double q_IMU = jointPositions[i].getDoubleValue() + estimatorDT * qd_IMU; // is qd_IMU or qd_fused better here?
            double q_fused = (1.0 - alphaPosition) * q_sensorMap + alphaPosition * q_IMU;
            
            jointPositionsFromIMUOnly[i].set(q_IMU);
            jointPositions[i].set(q_fused);
         }
         else
         {
            newJointVelocities[i].update(qd_sensorMap, qd_IMU);
            newIMUPositions[i].mul(1.0 - butterAlphaLeak.getValue());
            newIMUPositions[i].add(newJointVelocities[i].getValue() * estimatorDT);
            newJointPositions[i].update(q_sensorMap, newIMUPositions[i].getValue());

            jointVelocities[i].update(newJointVelocities[i].getValue());
            jointPositions[i].set(newJointPositions[i].getValue());
         }
      }
   }

   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return jointToIndexMap.containsKey(joint);
   }

   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      if (!enableOutput.getValue())
         return Double.NaN;

      int jointIndex = jointToIndexMap.get(joint);

      if (jointIndex == -1)
         return Double.NaN;

      return jointVelocities[jointIndex].getDoubleValue();
   }

   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      if (!enableOutput.getValue())
         return Double.NaN;

      int jointIndex = jointToIndexMap.get(joint);

      if (jointIndex == -1)
         return Double.NaN;

      return jointPositions[jointIndex].getDoubleValue();
   }
}
