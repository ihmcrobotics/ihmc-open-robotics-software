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

   private DoubleYoVariable alpha;
   private double q1, q2;
   private DenseMatrix64F qdFromIMU64F;
   private DenseMatrix64F omegaVector;
   private DenseMatrix64F qdFiltered64F;
   private DenseMatrix64F JwInv64F;
   private DenseMatrix64F RO3_64F;
   private DenseMatrix64F pelvisIMUOmegaVector64F;
   private double qdFromIMU[] = new double[3];
   private D1Matrix64F a;
   private D1Matrix64F b;
   private double sinq1;
   private double cosq1;
   private double sinq2;
   private double cosq2;
   private double sinq1pow2;
   private double cosq1pow2;
   private double[][] R03 = new double[4][4];
   private double[][] JwInv = new double[3][3];
   private RowD1Matrix64F chestIMUOmegaVectorInWillsPelvisFrame;
   private GeometricJacobian jacobian;
   private DenseMatrix64F jacobianAngularPart64F;
   private IMUSensorReadOnly pelvisIMU;
   private IMUSensorReadOnly chestIMU;
   private final SensorOutputMapReadOnly sensorMap;
   private final Map<OneDoFJoint, DoubleYoVariable> jointVelocities = new LinkedHashMap<>();
   private DenseMatrix64F inverseAngularJacobian64F;
   private Vector3d pelvisAngularVelocityTuple;
   private double[] chestAngularVelocityRelativeToPelvisFrameInPelvisFrame;
   private RowD1Matrix64F omega;
   private RowD1Matrix64F qd_estimated;
   private OneDoFJoint[] joints;

   public IMUBasedPelvisToTorsoEncodersVelocityFilter(YoVariableRegistry registry, IMUSensorReadOnly pelvisIMU, IMUSensorReadOnly chestIMU, SensorOutputMapReadOnly sensorMap)
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
      joints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      for (OneDoFJoint joint : joints)
         jointVelocities.put(joint, new DoubleYoVariable("qd_" + joint.getName() + "_IMUBased", registry));
   }

   //TODO: This function uses a Valkyrie specific manipulator jacobian. This code should be moved to a Valkyrie project eventually.
   public double[] getEncoderVelocityEsimates(double q_WaistRotator, double q_WaistExtensor, double q_WaistLateralExtensor, DenseMatrix64F chestIMUOmegaVector,
         double[] pelvisIMUOmegaVector, DenseMatrix64F qdFromEncoders)
   {
      q1 = q_WaistRotator;
      q2 = -q_WaistExtensor; // for some reason they decided the limbo direction of extension is positive.
      //      q3 = q_WaistLateralExtensor;

      sinq1 = Math.sin(q1);
      cosq1 = Math.cos(q1);
      sinq2 = Math.sin(q2);
      cosq2 = Math.cos(q2);
      sinq1pow2 = Math.pow(sinq1, 2.0);
      cosq1pow2 = Math.pow(cosq1, 2.0);

      //Calculate the inverse of the angular velocity manipulator jacobian, J omega (JwInv).
      JwInv[0][0] = (cosq1 * sinq2 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
      JwInv[0][1] = (sinq1 * sinq2 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
      JwInv[0][2] = 1.0;
      JwInv[1][0] = (sinq1 * -1.0) / (cosq1pow2 + sinq1pow2);
      JwInv[1][1] = (cosq1 * 1.0) / (cosq1pow2 + sinq1pow2);
      JwInv[2][0] = (cosq1 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);
      JwInv[2][1] = (sinq1 * 1.0) / (cosq1pow2 * cosq2 + cosq2 * sinq1pow2);

      JwInv64F = new DenseMatrix64F(JwInv);

      //w = R_03 * w_chest - w_pelvis

      //TODO: Get R03 from MATLAB
      RO3_64F = new DenseMatrix64F(R03);
//      pelvisIMUOmegaVector64F = new DenseMat
      CommonOps.mult(RO3_64F, chestIMUOmegaVector, chestIMUOmegaVectorInWillsPelvisFrame); // Will's Pelvis frame has the same orienation as typical root frame but located at the intersection of the rotator axis and the x-axis of the frame attached to the waist extensor axis during zero pose. 
      CommonOps.subtract(chestIMUOmegaVectorInWillsPelvisFrame, pelvisIMUOmegaVector64F, omegaVector);

      //qd = J^-1 * w
      CommonOps.mult(JwInv64F, omegaVector, qdFromIMU64F);

      //sign flip waist extensor encoder value
      qdFromIMU = qdFromIMU64F.getData();
      qdFromIMU[1] = -qdFromIMU[1];
      qdFromIMU64F.setData(qdFromIMU);

      //apply alpha blending
      CommonOps.scale(alpha.getDoubleValue(), qdFromIMU64F, a);
      CommonOps.scale((1.0 - alpha.getDoubleValue()), qdFromEncoders, b);
      CommonOps.add(a, b, qdFiltered64F);

      return qdFiltered64F.getData();
   }
   
   public void compute(){
      
      jacobian.compute();
      jacobianAngularPart64F = CommonOps.extract(jacobian.getJacobianMatrix(), 3, 5, 0, 2);
      CommonOps.invert(jacobianAngularPart64F, inverseAngularJacobian64F);
      
      FrameVector chestAngularVelocity = new FrameVector();
      chestAngularVelocity.setToZero(chestIMU.getMeasurementFrame());
      chestIMU.getAngularVelocityMeasurement(chestAngularVelocity.getVector());
      chestAngularVelocity.changeFrame(pelvisIMU.getMeasurementFrame());
      
      FrameVector pelvisAngularVelocity = new FrameVector();
      pelvisAngularVelocity.setToZero(pelvisIMU.getMeasurementFrame());
      pelvisIMU.getAngularVelocityMeasurement(pelvisAngularVelocity.getVector());
      
      pelvisAngularVelocity.getVector().get(pelvisAngularVelocityTuple);
      chestAngularVelocity.getVector().sub(pelvisAngularVelocityTuple);
      chestAngularVelocity.getVector().get(chestAngularVelocityRelativeToPelvisFrameInPelvisFrame);
      
      omega.setData(chestAngularVelocityRelativeToPelvisFrameInPelvisFrame);
      CommonOps.mult(inverseAngularJacobian64F, omega, qd_estimated);
      
      int i = 0;
      
      for (OneDoFJoint joint : joints){
         jointVelocities.get(joint).set(qd_estimated.getData()[i]);
         i++;
      }
   }

   public DoubleYoVariable getResult(OneDoFJoint joint)
   {
      return jointVelocities.get(joint);
   }

}
