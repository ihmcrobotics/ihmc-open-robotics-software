/**
 * Author: Will Rifenburgh 4:30:29 PM Nov 18, 2014
 */
package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import org.ejml.data.D1Matrix64F;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class IMUBasedPelvisToTorsoEncodersVelocityFilter
{
   private DoubleYoVariable alpha;
   private double q1, q2;
   private DenseMatrix64F qdFromIMU64F;
   private DenseMatrix64F omegaVector;
   private DenseMatrix64F qdFiltered64F;
   private double qdFromIMU[] = new double[3];
   private D1Matrix64F a;
   private D1Matrix64F b;

   public IMUBasedPelvisToTorsoEncodersVelocityFilter(DoubleYoVariable alpha)
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

      this.alpha = alpha;

   }

   public double[] getEncoderVelocityEsimates(double q_WaistRotator, double q_WaistExtensor, double q_WaistLateralExtensor, DenseMatrix64F chestIMUOmegaVector,
         DenseMatrix64F pelvisIMUOmegaVector, DenseMatrix64F qdFromEncoders)
   {

      q1 = q_WaistRotator;
      q2 = -q_WaistExtensor;
      //      q3 = q_WaistLateralExtensor;

      double[][] JwInv = new double[3][3];
      JwInv[0][0] = (Math.cos(q1) * Math.sin(q2) * 1.0) / (Math.pow(Math.cos(q1), 2.0) * Math.cos(q2) + Math.cos(q2) * Math.pow(Math.sin(q1), 2.0));
      JwInv[0][1] = (Math.sin(q1) * Math.sin(q2) * 1.0) / (Math.pow(Math.cos(q1), 2.0) * Math.cos(q2) + Math.cos(q2) * Math.pow(Math.sin(q1), 2.0));
      JwInv[0][2] = 1.0;
      JwInv[1][0] = (Math.sin(q1) * -1.0) / (Math.pow(Math.cos(q1), 2.0) + Math.pow(Math.sin(q1), 2.0));
      JwInv[1][1] = (Math.cos(q1) * 1.0) / (Math.pow(Math.cos(q1), 2.0) + Math.pow(Math.sin(q1), 2.0));
      JwInv[2][0] = (Math.cos(q1) * 1.0) / (Math.pow(Math.cos(q1), 2.0) * Math.cos(q2) + Math.cos(q2) * Math.pow(Math.sin(q1), 2.0));
      JwInv[2][1] = (Math.sin(q1) * 1.0) / (Math.pow(Math.cos(q1), 2.0) * Math.cos(q2) + Math.cos(q2) * Math.pow(Math.sin(q1), 2.0));

      DenseMatrix64F JwInv64 = new DenseMatrix64F(JwInv);

      //w = w_chest - w_pelvis
      CommonOps.subtract(chestIMUOmegaVector, pelvisIMUOmegaVector, omegaVector);

      //qd = J^-1 * w
      CommonOps.mult(JwInv64, omegaVector, qdFromIMU64F);

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

}
