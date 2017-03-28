/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
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
   private final DoubleYoVariable alpha;
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly pelvisIMU;
   private final IMUSensorReadOnly chestIMU;
   private final SensorOutputMapReadOnly sensorMap;
   private final DoubleYoVariable slopTime;
   private final Map<OneDoFJoint, BacklashProcessingYoVariable> jointVelocities = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointVelocitiesFromIMUOnly = new LinkedHashMap<>();
   private final OneDoFJoint[] joints;
   private final FrameVector chestAngularVelocity = new FrameVector();
   private final FrameVector pelvisAngularVelocity = new FrameVector();

   private final DenseMatrix64F jacobianAngularPart64F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F inverseAngularJacobian64F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F omega = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F qd_estimated = new DenseMatrix64F(3, 1);

   public IMUBasedJointVelocityEstimator(IMUSensorReadOnly pelvisIMU, IMUSensorReadOnly chestIMU, SensorOutputMapReadOnly sensorMap,
         double estimatorDT, double slopTime, YoVariableRegistry registry)
   {
      this.sensorMap = sensorMap;
      this.pelvisIMU = pelvisIMU;
      this.chestIMU = chestIMU;
      jacobian = new GeometricJacobian(pelvisIMU.getMeasurementLink(), chestIMU.getMeasurementLink(), chestIMU.getMeasurementLink().getBodyFixedFrame());
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);

      String namePrefix = "imuBasedJointVelocityEstimator";
      alpha = new DoubleYoVariable(namePrefix + "AlphaFuse", registry);
      alpha.set(0.0);

      double dt = estimatorDT;
      this.slopTime = new DoubleYoVariable(namePrefix + "SlopTime", registry);
      this.slopTime.set(slopTime);

      for (OneDoFJoint joint : joints)
      {
         jointVelocitiesFromIMUOnly.put(joint, new DoubleYoVariable("qd_" + joint.getName() + "_IMUBased", registry));
         jointVelocities.put(joint, new BacklashProcessingYoVariable("qd_" + joint.getName() + "_FusedWithIMU", "", dt, this.slopTime, registry));
      }
   }

   public void setAlphaFuse(double alpha)
   {
      this.alpha.set(alpha);
   }

   public void compute()
   {

      jacobian.compute();
      CommonOps.extract(jacobian.getJacobianMatrix(), 0, 3, 0, 3, jacobianAngularPart64F, 0, 0);
      if (Math.abs(CommonOps.det(jacobianAngularPart64F)) < 1e-5)
         return;
      CommonOps.invert(jacobianAngularPart64F, inverseAngularJacobian64F);

      chestAngularVelocity.setToZero(chestIMU.getMeasurementFrame());
      chestIMU.getAngularVelocityMeasurement(chestAngularVelocity.getVector());
      chestAngularVelocity.changeFrame(jacobian.getJacobianFrame());

      pelvisAngularVelocity.setToZero(pelvisIMU.getMeasurementFrame());
      pelvisIMU.getAngularVelocityMeasurement(pelvisAngularVelocity.getVector());
      pelvisAngularVelocity.changeFrame(jacobian.getJacobianFrame());
      chestAngularVelocity.sub(pelvisAngularVelocity);

      chestAngularVelocity.getVector().get(omega);
      CommonOps.mult(inverseAngularJacobian64F, omega, qd_estimated);

      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         double qd_sensorMap = sensorMap.getJointVelocityProcessedOutput(joint);
         double qd_IMU = qd_estimated.get(i, 0);
         double qd_fused = (1.0 - alpha.getDoubleValue()) * qd_sensorMap + alpha.getDoubleValue() * qd_IMU;

         jointVelocitiesFromIMUOnly.get(joint).set(qd_IMU);
         jointVelocities.get(joint).update(qd_fused);
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
}
