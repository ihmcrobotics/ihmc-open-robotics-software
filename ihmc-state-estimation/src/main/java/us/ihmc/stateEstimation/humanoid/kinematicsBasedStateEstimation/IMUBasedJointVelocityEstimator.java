/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

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
public class IMUBasedJointVelocityEstimator
{
   private final YoDouble alphaVelocity;
   private final YoDouble alphaPosition;
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly pelvisIMU;
   private final IMUSensorReadOnly chestIMU;
   private final SensorOutputMapReadOnly sensorMap;
   private final YoDouble slopTime;
   private final Map<OneDoFJoint, BacklashProcessingYoVariable> jointVelocities = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointVelocitiesFromIMUOnly = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositions = new LinkedHashMap<>();
   private final Map<OneDoFJoint, YoDouble> jointPositionsFromIMUOnly = new LinkedHashMap<>();
   private final OneDoFJoint[] joints;
   private final FrameVector3D chestAngularVelocity = new FrameVector3D();
   private final FrameVector3D pelvisAngularVelocity = new FrameVector3D();

   private final DenseMatrix64F jacobianAngularPart64F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F inverseAngularJacobian64F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F omega = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F qd_estimated = new DenseMatrix64F(3, 1);
   
   private final double estimatorDT;

   public IMUBasedJointVelocityEstimator(IMUSensorReadOnly pelvisIMU, IMUSensorReadOnly chestIMU, SensorOutputMapReadOnly sensorMap,
         double estimatorDT, double slopTime, YoVariableRegistry registry)
   {
      this.sensorMap = sensorMap;
      this.pelvisIMU = pelvisIMU;
      this.chestIMU = chestIMU;
      jacobian = new GeometricJacobian(pelvisIMU.getMeasurementLink(), chestIMU.getMeasurementLink(), chestIMU.getMeasurementLink().getBodyFixedFrame());
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);

      String namePrefix = "imuBasedJointVelocityEstimator";
      alphaVelocity = new YoDouble(namePrefix + "AlphaFuseVelocity", registry);
      alphaVelocity.set(0.0);
      alphaPosition = new YoDouble(namePrefix + "AlphaFusePosition", registry);
      alphaPosition.set(0.0);

      this.estimatorDT = estimatorDT;
      this.slopTime = new YoDouble(namePrefix + "SlopTime", registry);
      this.slopTime.set(slopTime);

      for (OneDoFJoint joint : joints)
      {
         jointVelocitiesFromIMUOnly.put(joint, new YoDouble("qd_" + joint.getName() + "_IMUBased", registry));
         jointVelocities.put(joint, new BacklashProcessingYoVariable("qd_" + joint.getName() + "_FusedWithIMU", "", estimatorDT, this.slopTime, registry));

         jointPositionsFromIMUOnly.put(joint, new YoDouble("q_" + joint.getName() + "_IMUBased", registry));
         jointPositions.put(joint, new YoDouble("q_" + joint.getName() + "_FusedWithIMU", registry));
      }
   }

   public void setAlphaFuse(double alphaVelocity, double alphaPosition)
   {
      this.alphaVelocity.set(alphaVelocity);
      this.alphaPosition.set(alphaPosition);
   }

   public void compute()
   {
      jacobian.compute();
      CommonOps.extract(jacobian.getJacobianMatrix(), 0, 3, 0, 3, jacobianAngularPart64F, 0, 0);
      if (Math.abs(CommonOps.det(jacobianAngularPart64F)) < 1e-5)
         return;
      CommonOps.invert(jacobianAngularPart64F, inverseAngularJacobian64F);

      chestAngularVelocity.setToZero(chestIMU.getMeasurementFrame());
      chestIMU.getAngularVelocityMeasurement(chestAngularVelocity);
      chestAngularVelocity.changeFrame(jacobian.getJacobianFrame());

      pelvisAngularVelocity.setToZero(pelvisIMU.getMeasurementFrame());
      pelvisIMU.getAngularVelocityMeasurement(pelvisAngularVelocity);
      pelvisAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      chestAngularVelocity.sub(pelvisAngularVelocity);

      chestAngularVelocity.get(omega);
      CommonOps.mult(inverseAngularJacobian64F, omega, qd_estimated);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         double qd_sensorMap = sensorMap.getJointVelocityProcessedOutput(joint);
         double qd_IMU = qd_estimated.get(i, 0);
         double qd_fused = (1.0 - alphaVelocity.getDoubleValue()) * qd_sensorMap + alphaVelocity.getDoubleValue() * qd_IMU;

         jointVelocitiesFromIMUOnly.get(joint).set(qd_IMU);
         jointVelocities.get(joint).update(qd_fused);

         double q_sensorMap = sensorMap.getJointPositionProcessedOutput(joint);
         double q_IMU = jointPositions.get(joint).getDoubleValue() + estimatorDT * qd_IMU; // is qd_IMU or qd_fused better here?
         double q_fused = (1.0 - alphaPosition.getDoubleValue()) * q_sensorMap + alphaPosition.getDoubleValue() * q_IMU;

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
