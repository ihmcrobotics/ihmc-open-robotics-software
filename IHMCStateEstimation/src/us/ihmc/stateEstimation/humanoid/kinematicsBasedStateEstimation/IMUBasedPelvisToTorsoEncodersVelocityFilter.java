/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class IMUBasedPelvisToTorsoEncodersVelocityFilter
{
   private final DoubleYoVariable alpha;
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly pelvisIMU;
   private final IMUSensorReadOnly chestIMU;
   private final SensorOutputMapReadOnly sensorMap;
   private final Map<OneDoFJoint, DoubleYoVariable> jointVelocities = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> jointVelocitiesFromIMUOnly = new LinkedHashMap<>();
   private final OneDoFJoint[] joints;
   private final FrameVector chestAngularVelocity = new FrameVector();
   private final FrameVector pelvisAngularVelocity = new FrameVector();

   private final DenseMatrix64F jacobianAngularPart64F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F inverseAngularJacobian64F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F omega = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F qd_estimated = new DenseMatrix64F(3, 1);
   private double qd_aps;

   public IMUBasedPelvisToTorsoEncodersVelocityFilter(YoVariableRegistry registry, IMUSensorReadOnly pelvisIMU, IMUSensorReadOnly chestIMU,
         SensorOutputMapReadOnly sensorMap)
   {
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

      this.sensorMap = sensorMap;
      this.pelvisIMU = pelvisIMU;
      this.chestIMU = chestIMU;
      jacobian = new GeometricJacobian(pelvisIMU.getMeasurementLink(), chestIMU.getMeasurementLink(), chestIMU.getMeasurementLink().getBodyFixedFrame());
      alpha = new DoubleYoVariable("alpha_IMUBasedJointVelFilter", registry);
      alpha.set(0.0);
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      for (OneDoFJoint joint : joints)
      {
         jointVelocitiesFromIMUOnly.put(joint, new DoubleYoVariable("qd_" + joint.getName() + "_IMUBased", registry));
         jointVelocities.put(joint, new DoubleYoVariable("qd_" + joint.getName() + "_FusedWithIMU", registry));
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

      omega.setData(chestAngularVelocity.toArray());
      CommonOps.mult(inverseAngularJacobian64F, omega, qd_estimated);

      int i = 0;

      for (OneDoFJoint joint : joints)
      {
         qd_aps = sensorMap.getJointVelocityProcessedOutput(joint);
         jointVelocitiesFromIMUOnly.get(joint).set(qd_estimated.getData()[i]);
         jointVelocities.get(joint).set((1.0 - alpha.getDoubleValue()) * qd_aps + qd_estimated.getData()[i] * alpha.getDoubleValue());
         i++;
      }
   }
   
   public Map<OneDoFJoint, DoubleYoVariable> getJointVelocities(){
      return jointVelocities;
   }

}
