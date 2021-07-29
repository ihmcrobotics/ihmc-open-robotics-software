/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.filters.ButterworthFilteredYoVariable;
import us.ihmc.robotics.math.filters.ButterworthFusedYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
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
   private final GeometricJacobian jacobian;
   private final SensorOutputMapReadOnly sensorMap;

   private final DoubleParameter velocityBreakFrequency;
   private final DoubleParameter positionBreakFrequency;

   private boolean velocityAlphaDirty = true;
   private double velocityAlpha = 0.0;
   private boolean positionAlphaDirty = true;
   private double positionAlpha = 0.0;

   private final OneDoFJointBasics[] joints;
   private final ButterworthFusedYoVariable[] jointVelocities;
   private final ButterworthFusedYoVariable[] jointPositions;
   private final YoDouble[] jointPositionsFromIMUOnly;

   private final double estimatorDT;

   private final YoDouble imuOnlyPositionLeak;

   private final Map<OneDoFJointBasics, OneDoFJointStateReadOnly> jointOutputStates = new HashMap<>();

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

      velocityBreakFrequency = new DoubleParameter(name + "FuseVelocityBreakFreq", registry, parameters.getBreakFrequencyForVelocityEstimation());
      positionBreakFrequency = new DoubleParameter(name + "FusePositionBreakFreq", registry, parameters.getBreakFrequencyForPositionEstimation());
      velocityBreakFrequency.addListener(p -> velocityAlphaDirty = true);
      positionBreakFrequency.addListener(p -> positionAlphaDirty = true);

      jointVelocities = new ButterworthFusedYoVariable[joints.length];
      jointPositionsFromIMUOnly = new YoDouble[joints.length];
      jointPositions = new ButterworthFusedYoVariable[joints.length];

      imuOnlyPositionLeak = new YoDouble(name + "IMUOnlyPositionLeak", registry);
      imuOnlyPositionLeak.set(1.0e-4);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         String jointName = joint.getName();
         jointPositionsFromIMUOnly[i] = new YoDouble("q_" + jointName + "_IMUBased", registry);
         ButterworthFusedYoVariable jointVelocity = new ButterworthFusedYoVariable("qd_" + jointName + "_FusedWithIMU", registry, () -> velocityAlpha);
         ButterworthFusedYoVariable jointPosition = new ButterworthFusedYoVariable("q_" + jointName + "_FusedWithIMU", registry, () -> positionAlpha);
         jointVelocities[i] = jointVelocity;
         jointPositions[i] = jointPosition;

         jointOutputStates.put(joint,
                               OneDoFJointStateReadOnly.createFromSuppliers(jointName,
                                                                            () -> jointPosition.getValue(),
                                                                            () -> jointVelocity.getValue(),
                                                                            () -> Double.NaN,
                                                                            () -> Double.NaN,
                                                                            () -> true));
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      updateAlphas();

      velocityEstimator.compute();

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         OneDoFJointStateReadOnly jointSensorOutput = sensorMap.getOneDoFJointOutput(joint);

         double qd_sensorMap = jointSensorOutput.getVelocity();
         double qd_IMU = velocityEstimator.getEstimatedJointVelocity(i);
         double q_sensorMap = jointSensorOutput.getPosition();

         jointVelocities[i].update(qd_sensorMap, qd_IMU);
         jointPositionsFromIMUOnly[i].mul(1.0 - imuOnlyPositionLeak.getValue());
         jointPositionsFromIMUOnly[i].add(jointVelocities[i].getValue() * estimatorDT);
         jointPositions[i].update(q_sensorMap, jointPositionsFromIMUOnly[i].getValue());
      }
   }

   private void updateAlphas()
   {
      if (velocityAlphaDirty)
      {
         velocityAlphaDirty = false;
         velocityAlpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(velocityBreakFrequency.getValue(), estimatorDT);
      }

      if (positionAlphaDirty)
      {
         positionAlphaDirty = false;
         positionAlpha = ButterworthFilteredYoVariable.computeAlphaGivenBreakFrequency(positionBreakFrequency.getValue(), estimatorDT);
      }
   }

   public OneDoFJointStateReadOnly getJointEstimatedState(OneDoFJointBasics joint)
   {
      if (enableOutput.getValue())
         return jointOutputStates.get(joint);
      else
         return null;
   }
}
