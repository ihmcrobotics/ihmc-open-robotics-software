/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.D1Matrix64F;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class IMUBasedPelvisToTorsoEncodersVelocityFilter
{
   private final DoubleYoVariable alpha;
   //   private double q1, q2;
   //   private final DenseMatrix64F qdFromIMU64F = new DenseMatrix64F(1, 1);
   //   private final DenseMatrix64F omegaVector = new DenseMatrix64F(1, 1);
   //   private final DenseMatrix64F qdFiltered64F = new DenseMatrix64F(1, 1);
   //   private final DenseMatrix64F JwInv64F = new DenseMatrix64F(1, 1);
   //   private final DenseMatrix64F RO3_64F = new DenseMatrix64F(1, 1);
   //   private final DenseMatrix64F pelvisIMUOmegaVector64F = new DenseMatrix64F(1, 1);
   //   private final double qdFromIMU[] = new double[3];
   //   private final D1Matrix64F a = new DenseMatrix64F(0, 0);
   //   private final D1Matrix64F b = new DenseMatrix64F(0, 0);
   //   private double sinq1;
   //   private double cosq1;
   //   private double sinq2;
   //   private double cosq2;
   //   private double sinq1pow2;
   //   private double cosq1pow2;
   //   private final double[][] R03 = new double[4][4];
   //   private final double[][] JwInv = new double[3][3];
   //   private final RowD1Matrix64F chestIMUOmegaVectorInWillsPelvisFrame;
   private final GeometricJacobian jacobian;
   private final IMUSensorReadOnly pelvisIMU;
   private final IMUSensorReadOnly chestIMU;
   private final SensorOutputMapReadOnly sensorMap;
   private final Map<OneDoFJoint, DoubleYoVariable> jointVelocities = new LinkedHashMap<>();
//   private final Vector3d pelvisAngularVelocityTuple = new Vector3d();
//   private final double[] chestAngularVelocityRelativeToPelvisFrameInPelvisFrame = new double[3];
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
         jointVelocities.put(joint, new DoubleYoVariable("qd_" + joint.getName() + "_IMUBased", registry));
   }

   //TODO: This function uses a Valkyrie specific manipulator jacobian. This code should be moved to a Valkyrie project eventually.
   //   public double[] getEncoderVelocityEsimates(double q_WaistRotator, double q_WaistExtensor, double q_WaistLateralExtensor, DenseMatrix64F chestIMUOmegaVector,
   //         double[] pelvisIMUOmegaVector, DenseMatrix64F qdFromEncoders)
   //   {
   //      q1 = q_WaistRotator;
   //      q2 = -q_WaistExtensor; // for some reason they decided the limbo direction of extension is positive.
   //      //      q3 = q_WaistLateralExtensor;
   //
   //      sinq1 = Math.sin(q1);
   //      cosq1 = Math.cos(q1);
   //      sinq2 = Math.sin(q2);
   //      cosq2 = Math.cos(q2);
   //      sinq1pow2 = Math.pow(sinq1, 2.0);
   //      cosq1pow2 = Math.pow(cosq1, 2.0);
   //
   //      //Calculate the inverse of the angular velocity manipulator jacobian, J omega (JwInv).
   //      JwInv[0][0] = (cosq1 * sinq2 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
   //      JwInv[0][1] = (sinq1 * sinq2 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
   //      JwInv[0][2] = 1.0;
   //      JwInv[1][0] = (sinq1 * -1.0) / (cosq1pow2 + sinq1pow2);
   //      JwInv[1][1] = (cosq1 * 1.0) / (cosq1pow2 + sinq1pow2);
   //      JwInv[2][0] = (cosq1 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
   //      JwInv[2][1] = (sinq1 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
   //
   //      JwInv64F.reshape(JwInv.length, JwInv[0].length);
   //      JwInv64F.set(JwInv);
   //
   //      //w = R_03 * w_chest - w_pelvis
   //
   //      //TODO: Get R03 from MATLAB
   //      RO3_64F = new DenseMatrix64F(R03);
   ////      pelvisIMUOmegaVector64F = new DenseMat
   //      CommonOps.mult(RO3_64F, chestIMUOmegaVector, chestIMUOmegaVectorInWillsPelvisFrame); // Will's Pelvis frame has the same orienation as typical root frame but located at the intersection of the rotator axis and the x-axis of the frame attached to the waist extensor axis during zero pose. 
   //      CommonOps.subtract(chestIMUOmegaVectorInWillsPelvisFrame, pelvisIMUOmegaVector64F, omegaVector);
   //
   //      //qd = J^-1 * w
   //      CommonOps.mult(JwInv64F, omegaVector, qdFromIMU64F);
   //
   //      //sign flip waist extensor encoder value
   //      qdFromIMU = qdFromIMU64F.getData();
   //      qdFromIMU[1] = -qdFromIMU[1];
   //      qdFromIMU64F.setData(qdFromIMU);
   //
   //      //apply alpha blending
   //      CommonOps.scale(alpha.getDoubleValue(), qdFromIMU64F, a);
   //      CommonOps.scale((1.0 - alpha.getDoubleValue()), qdFromEncoders, b);
   //      CommonOps.add(a, b, qdFiltered64F);
   //
   //      return qdFiltered64F.getData();
   //   }

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

      //      pelvisAngularVelocity.getVector().get(pelvisAngularVelocityTuple);
      //      pelvisAngularVelocity.getVector().get(chestAngularVelocityRelativeToPelvisFrameInPelvisFrame);
      chestAngularVelocity.sub(pelvisAngularVelocity);

      omega.setData(chestAngularVelocity.toArray());
      CommonOps.mult(inverseAngularJacobian64F, omega, qd_estimated);

      int i = 0;

      for (OneDoFJoint joint : joints)
      {
         qd_aps = sensorMap.getJointVelocityProcessedOutput(joint);
         jointVelocities.get(joint).set((1.0 - alpha.getDoubleValue()) * qd_aps + qd_estimated.getData()[i] * alpha.getDoubleValue());
         i++;
      }
   }
   
   public Map<OneDoFJoint, DoubleYoVariable> getJointVelocities(){
      return jointVelocities;
   }

}
