/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Creates an alpha filter defined by:
 *
 * qdFiltered = alpha*qd_from_IMU_estimate + (1-alpha)*qdFromEncoders
 *
 * In which qdFiltered is defined as the filtered version of:
 *
 * qdFromEncoders = {qd_WaistRotator, qd_WaistExtensor, qd_WaistLateralExtensor}
 *
 * call getEncoderVelocityEstimates for output of the filter.
 *
 */
public class IMUBasedJointStateEstimator
{
   private final IMUBasedJointVelocityEstimator velocityEstimator;
   private final YoDouble alphaVelocity;
   private final YoDouble alphaPosition;
   private final GeometricJacobian jacobian;
   private final SensorOutputMapReadOnly sensorMap;
   private final YoDouble slopTime;
   private final BacklashProcessingYoVariable[] jointVelocities;
   private final YoDouble[] jointPositions;
   private final YoDouble[] jointPositionsFromIMUOnly;
   private final OneDoFJoint[] joints;

   private final double estimatorDT;

   public IMUBasedJointStateEstimator(IMUSensorReadOnly pelvisIMU, IMUSensorReadOnly chestIMU, SensorOutputMapReadOnly sensorMap, double estimatorDT,
                                      double slopTime, YoVariableRegistry registry)
   {
      this.sensorMap = sensorMap;
      jacobian = new GeometricJacobian(pelvisIMU.getMeasurementLink(), chestIMU.getMeasurementLink(), chestIMU.getMeasurementLink().getBodyFixedFrame());
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      this.velocityEstimator = new IMUBasedJointVelocityEstimator(jacobian, pelvisIMU, chestIMU, registry);

      String namePrefix = "imuBasedJointVelocityEstimator";
      alphaVelocity = new YoDouble(namePrefix + "AlphaFuseVelocity", registry);
      alphaVelocity.set(0.0);
      alphaPosition = new YoDouble(namePrefix + "AlphaFusePosition", registry);
      alphaPosition.set(0.0);

      this.estimatorDT = estimatorDT;
      this.slopTime = new YoDouble(namePrefix + "SlopTime", registry);
      this.slopTime.set(slopTime);

      jointVelocities = new BacklashProcessingYoVariable[joints.length];
      jointPositions = new YoDouble[joints.length];
      jointPositionsFromIMUOnly = new YoDouble[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         jointVelocities[i] = new BacklashProcessingYoVariable("qd_" + joint.getName() + "_FusedWithIMU", "", estimatorDT, this.slopTime, registry);
         jointPositionsFromIMUOnly[i] = new YoDouble("q_" + joint.getName() + "_IMUBased", registry);
         jointPositions[i] = new YoDouble("q_" + joint.getName() + "_FusedWithIMU", registry);
      }
   }

   public void setAlphaFuse(double alphaVelocity, double alphaPosition)
   {
      this.alphaVelocity.set(alphaVelocity);
      this.alphaPosition.set(alphaPosition);
   }

   public void compute()
   {
      velocityEstimator.compute();

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         double qd_sensorMap = sensorMap.getJointVelocityProcessedOutput(joint);
         double qd_IMU = velocityEstimator.getEstimatedJointVelocity(joint);
         double qd_fused = (1.0 - alphaVelocity.getDoubleValue()) * qd_sensorMap + alphaVelocity.getDoubleValue() * qd_IMU;

         jointVelocities[i].update(qd_fused);

         double q_sensorMap = sensorMap.getJointPositionProcessedOutput(joint);
         double q_IMU = jointPositions[i].getDoubleValue() + estimatorDT * qd_IMU; // is qd_IMU or qd_fused better here?
         double q_fused = (1.0 - alphaPosition.getDoubleValue()) * q_sensorMap + alphaPosition.getDoubleValue() * q_IMU;

         jointPositionsFromIMUOnly[i].set(q_IMU);
         jointPositions[i].set(q_fused);
      }
   }

   public double getEstimatedJointVelocitiy(OneDoFJoint joint)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] == joint)
         {
            return jointVelocities[i].getDoubleValue();
         }
      }
      return Double.NaN;
   }

   public double getEstimatedJointPosition(OneDoFJoint joint)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] == joint)
         {
            return jointPositions[i].getDoubleValue();
         }
      }
      return Double.NaN;
   }
}
