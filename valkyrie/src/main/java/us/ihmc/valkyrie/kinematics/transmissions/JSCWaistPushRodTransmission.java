package us.ihmc.valkyrie.kinematics.transmissions;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.valkyrie.kinematics.LinearActuator;
import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;

public class JSCWaistPushRodTransmission implements PushRodTransmissionInterface
{
   /** Maps actuator force to joint torque */
   private final DenseMatrix64F jacobianTranspose = new DenseMatrix64F(2, 2);
   /** Maps joint velocity to actuator velocity */
   private final DenseMatrix64F jacobian = new DenseMatrix64F(2, 2);

   /** Joint position, jointPositions = [q1 q2] = [spine roll, spine pitch]' */
//   private final DenseMatrix64F jointPositions = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F jointVelocites = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F jointTorques = new DenseMatrix64F(2, 1);
   /** Actuator position, pushrodPositions = [left actuator , right actuator]' */
   private final DenseMatrix64F pushrodPositions = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F pushrodVelocities = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F pushrodForces = new DenseMatrix64F(2, 1);

   public JSCWaistPushRodTransmission(PushRodTransmissionJoint joint)
   {
      if (joint != PushRodTransmissionJoint.WAIST)
         throw new RuntimeException("This implementation is only for the waist.");
   }

   @Override
   public void actuatorToJointEffort(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      compute(jnt_data);
      pushrodForces.set(0, 0, act_data[0].getEffort());
      pushrodForces.set(1, 0, act_data[1].getEffort());
      
//      CommonOps.multTransA(jacobian, pushrodForces, jointTorques);
      CommonOps.mult(jacobianTranspose, pushrodForces, jointTorques);


      jnt_data[1].setEffort(jointTorques.get(0, 0));
      jnt_data[0].setEffort(jointTorques.get(1, 0));
   }

   @Override
   public void actuatorToJointVelocity(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      compute(jnt_data);
      pushrodVelocities.set(0, 0, act_data[0].getVelocity());
      pushrodVelocities.set(1, 0, act_data[1].getVelocity());
      
      DenseMatrix64F jacobianInverse = new DenseMatrix64F(2, 2);
      CommonOps.invert(jacobian, jacobianInverse);
      CommonOps.mult(jacobianInverse, pushrodVelocities, jointVelocites);

      jnt_data[1].setVelocity(-jointVelocites.get(0, 0));
      jnt_data[0].setVelocity(-jointVelocites.get(1, 0));
   }

   @Override
   public void actuatorToJointPosition(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data)
   {
   }

   // ValkyrieJointInterface[] jnt_data = [ pitch, roll ]
   @Override
   public void jointToActuatorEffort(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      compute(jnt_data);
      jointTorques.set(0, 0, jnt_data[1].getDesiredEffort());
      jointTorques.set(1, 0, jnt_data[0].getDesiredEffort());
      
      DenseMatrix64F jacobianTransposeInverse = new DenseMatrix64F(2, 2);
      CommonOps.invert(jacobianTranspose, jacobianTransposeInverse);
      
      CommonOps.mult(jacobianTransposeInverse, jointTorques, pushrodForces);

      act_data[0].setEffortCommand(pushrodForces.get(0, 0));
      act_data[1].setEffortCommand(pushrodForces.get(1, 0));
   }

   @Override
   public void jointToActuatorVelocity(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data)
   {
   }

   @Override
   public void jointToActuatorPosition(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data)
   {
   }

   private void compute(ValkyrieJointInterface[] jnt_data)
   {
      double q1 = jnt_data[1].getPosition();
      double q2 = jnt_data[0].getPosition();

      double cq1 = Math.cos(q1);
      double sq1 = Math.sin(q1);

      double cq2 = Math.cos(q2);
      double sq2 = Math.sin(q2);

      double c2q1 = Math.cos(2.0 * q1);
      double c2q2 = Math.cos(2.0 * q2);
      double s2q1 = Math.sin(2.0 * q1);

      double a1 = computeALeft();
      double b1 = computeBLeft(cq1, sq1, cq2, sq2);
      double c1 = computeCLeft(cq1, sq1, cq2, sq2);

      double a2 = computeARight();
      double b2 = computeBRight(cq1, sq1, cq2, sq2);
      double c2 = computeCRight(cq1, sq1, cq2, sq2);

      double Db1Dq1 = computeDb1Dq1(cq1, sq1, sq2);
      double Db1Dq2 = computeDb1Dq2(cq1, cq2, sq2);
      double Dc1Dq1 = computeDc1Dq1(cq1, sq1, sq2, c2q1, c2q2, s2q1);
      double Dc1Dq2 = computeDc1Dq2(cq1, sq1, cq2, sq2, c2q1, s2q1);

      double Db2Dq1 = computeDb2Dq1(cq1, sq1, sq2);
      double Db2Dq2 = computeDb1Dq2(cq1, cq2, sq2);
      double Dc2Dq1 = computeDc2Dq1(cq1, sq1, sq2, c2q1, c2q2, s2q1);
      double Dc2Dq2 = computeDc2Dq2(cq1, sq1, cq2, sq2, c2q1);

      double x1 = computePushrodPosition(a1, b1, c1);
      double x2 = computePushrodPosition(a2, b2, c2);
      pushrodPositions.set(0, 0, x1);
      pushrodPositions.set(1, 0, x2);

      double velocityMap00 = computeVelocityMapCoefficient(Db1Dq1, a1, b1, Dc1Dq1, c1);
      double velocityMap01 = computeVelocityMapCoefficient(Db1Dq2, a1, b1, Dc1Dq2, c1);
      double velocityMap10 = computeVelocityMapCoefficient(Db2Dq1, a2, b2, Dc2Dq1, c2);
      double velocityMap11 = computeVelocityMapCoefficient(Db2Dq2, a2, b2, Dc2Dq2, c2);
      jacobian.set(0, 0, velocityMap00);
      jacobian.set(0, 1, velocityMap01);
      jacobian.set(1, 0, velocityMap10);
      jacobian.set(1, 1, velocityMap11);

      double pushrodForceToJointTorqueMap00 = computePushrodForceToJointTorqueMap00(cq1, sq1, sq2, x1);
      double pushrodForceToJointTorqueMap01 = computePushrodForceToJointTorqueMap01(cq1, sq1, sq2, x2);
      double pushrodForceToJointTorqueMap10 = computePushrodForceToJointTorqueMap10(cq1, sq1, cq2, sq2, x1);
      double pushrodForceToJointTorqueMap11 = computePushrodForceToJointTorqueMap11(cq1, sq1, cq2, sq2, x2);
      jacobianTranspose.set(0, 0, pushrodForceToJointTorqueMap00);
      jacobianTranspose.set(0, 1, pushrodForceToJointTorqueMap01);
      jacobianTranspose.set(1, 0, pushrodForceToJointTorqueMap10);
      jacobianTranspose.set(1, 1, pushrodForceToJointTorqueMap11);
   }

   private double computePushrodForceToJointTorqueMap11(double cq1, double sq1, double cq2, double sq2, double x2)
   {
      return 0.011819574532464065 * cq2 + 0.05154973694809101 * cq1 * cq2 - 0.04063161457507572 * cq2 * sq1
            + 0.003480579913609463 * sq2 - 0.5314015505061906 * cq1 * cq2 * x2 + 0.23659613758478296 * sq2 * x2;
   }

   private double computePushrodForceToJointTorqueMap10(double cq1, double sq1, double cq2, double sq2, double x1)
   {
      return 0.011819574532464065 * cq2 + 0.05154973694809101 * cq1 * cq2 + 0.0406313670492332 * cq2 * sq1
            + 0.003480579913609463 * sq2 - 0.5314015505061906 * cq1 * cq2 * x1 + 0.23659613758478296 * sq2 * x1;
   }

   private double computePushrodForceToJointTorqueMap01(double cq1, double sq1, double sq2, double x2)
   {
      return -0.04520169051768313 * cq1 + 0.013342471028366743 * sq1 - 0.04063161457507572 * cq1 * sq2
            - 0.051549736948091 * sq1 * sq2 + 0.3542749867977717 * cq1 * x2 + 0.14169916972524121 * sq1 * x2 + 0.5314015505061905 * sq1 * sq2 * x2;
   }

   private double computePushrodForceToJointTorqueMap00(double cq1, double sq1, double sq2, double x1)
   {
      return 0.04520162451447644 * cq1 + 0.01334230600774438 * sq1 + 0.0406313670492332 * cq1 * sq2
            - 0.05154973694809101 * sq1 * sq2 - 0.3542749867977717 * cq1 * x1 + 0.14169916972524121 * sq1 * x1 + 0.5314015505061905 * sq1 * sq2 * x1;
   }

   private double computeVelocityMapCoefficient(double Db1Dq1, double a1, double b1, double Dc1Dq1, double c1)
   {
      return -0.5 * Db1Dq1 * Math.pow(a1, -1.)
            - 250. * b1 * Db1Dq1 * Math.pow(a1, -1.) * Math.pow(17161. * a1 - 1.e6 * a1 * c1 + 250000. * Math.pow(b1, 2.), -0.5)
            + 500. * a1 * Dc1Dq1 * Math.pow(a1, -1.) * Math.pow(17161. * a1 - 1.e6 * a1 * c1 + 250000. * Math.pow(b1, 2.), -0.5);
   }

   private double computePushrodPosition(double a, double b, double c)
   {
      return -0.5 * b * Math.pow(a, -1.) - 0.001 * Math.pow(a, -1.) * Math.pow(17161. * a - 1.e6 * a * c + 250000. * Math.pow(b, 2.), 0.5);
   }

   private double computeDc2Dq2(double cq1, double sq1, double cq2, double sq2, double c2q1)
   {
      return 0.0030967292017271613 * cq2 + 6.742215758985032e-10 * c2q1 * cq2 + 0.013506036961482077 * cq1 * cq2 - 0.010645483018669839 * cq2 * sq1
            - 3.3713654125619965e-9 * cq1 * cq2 * sq1 + 0.0009119123344496975 * sq2 - 5.056944109297348e-9 * cq2 * Math.pow(sq1, 2) * sq2;
   }

   private double computeDc2Dq1(double cq1, double sq1, double sq2, double c2q1, double c2q2, double s2q1)
   {
      return -8.989805907517104e-10 * c2q1 - 0.011842846836435108 * cq1 + 1.264236027324337e-9 * c2q2 * s2q1 + 0.0034957258412310374 * sq1
            - 6.404136531652295e-10 * cq1 * sq1 - 3.3713654125619952e-9 * c2q1 * sq2 - 0.010645483018669839 * cq1 * sq2 - 1.3484431517970062e-9 * s2q1 * sq2
            - 0.013506036961482074 * sq1 * sq2;
   }

   private double computeDb2Dq1(double cq1, double sq1, double sq2)
   {
      return 0.0928200869586876 * cq1 + 0.037125198633846786 * sq1 + 0.13922726685788211 * sq1 * sq2;
   }

   private double computeDc1Dq2(double cq1, double sq1, double cq2, double sq2, double c2q1, double s2q1)
   {
      return 0.003096729201727161 * cq2 + 6.742215758985032e-10 * c2q1 * cq2 + 0.013506036961482075 * cq1 * cq2 + 1.6856827062809978e-9 * cq2 * s2q1
            + 0.010645418166899099 * cq2 * sq1 + 0.0009119123344496975 * sq2 - 5.056944109297348e-9 * cq2 * sq2 * Math.pow(sq1, 2.);
   }

   private double computeDc1Dq1(double cq1, double sq1, double sq2, double c2q1, double c2q2, double s2q1)
   {
      return 8.989805907517106e-10 * c2q1 + 0.011842829543594956 * cq1 + 1.264236027324337e-9 * c2q2 * s2q1 + 0.003495682605827978 * sq1
            - 6.404136531652295e-10 * cq1 * sq1 + 3.3713654125619952e-9 * c2q1 * sq2 + 0.0106454181668991 * cq1 * sq2 - 1.3484431517970064e-9 * s2q1 * sq2
            - 0.013506036961482077 * sq1 * sq2;
   }

   private double computeDb1Dq2(double cq1, double cq2, double sq2)
   {
      return -0.13922726685788214 * cq1 * cq2 + 0.06198821503942343 * sq2;
   }

   private double computeDb1Dq1(double cq1, double sq1, double sq2)
   {
      return -0.0928200869586876 * cq1 + 0.037125198633846786 * sq1 + 0.13922726685788211 * sq1 * sq2;
   }

   private double computeCRight(double cq1, double sq1, double cq2, double sq2)
   {
      return 0.012768498089931577 - 0.003495725841231038 * cq1 - 0.0009119123344496975 * cq2 + 0.005806692660122433 * Math.pow(cq2, 2)
            - 0.011842846836435108 * sq1 - 8.989805907517105e-10 * cq1 * sq1 + 0.002993732542738892 * Math.pow(sq1, 2) + 0.013506036961482079 * cq1 * sq2
            - 0.010645483018669839 * sq1 * sq2 - 3.371365412561996e-9 * cq1 * sq1 * sq2 + 0.003096728527505585 * Math.pow(sq1, 2) * sq2
            + 0.005806690131650378 * Math.pow(sq1, 2) * Math.pow(sq2, 2) + 0.002993731598709691 * Math.pow(cq1, 2.)
            + 0.0030967298759487366 * sq2 * Math.pow(cq1, 2.) + 0.005806692660122432 * Math.pow(sq2, 2) * Math.pow(cq1, 2.);
   }

   private double computeBRight(double cq1, double sq1, double cq2, double sq2)
   {
      return -0.15704988201538964 - 0.03712519863384679 * cq1 - 0.061988215039423436 * cq2 + 0.09282008695868763 * sq1
            - 0.13922726685788214 * cq1 * sq2;
   }

   private double computeARight()
   {
      return 1.0;
   }

   private double computeCLeft(double cq1, double sq1, double cq2, double sq2)
   {
      return 0.012768438643301433 - 0.003495682605827978 * cq1 - 0.0009119123344496975 * cq2 + 0.005806692660122433 * Math.pow(cq2, 2)
            + 0.011842829543594958 * sq1 + 8.989805907517105e-10 * cq1 * sq1 + 0.002993732542738892 * Math.pow(sq1, 2) + 0.013506036961482077 * cq1 * sq2
            + 0.0106454181668991 * sq1 * sq2 + 3.371365412561996e-9 * cq1 * sq1 * sq2 + 0.003096728527505585 * Math.pow(sq1, 2) * sq2
            + 0.005806690131650378 * Math.pow(sq1, 2) * Math.pow(sq2, 2) + 0.002993731598709691 * Math.pow(cq1, 2.)
            + 0.0030967298759487366 * sq2 * Math.pow(cq1, 2.) + 0.005806692660122432 * Math.pow(sq2, 2) * Math.pow(cq1, 2.);
   }

   private double computeBLeft(double cq1, double sq1, double cq2, double sq2)
   {
      return -0.15704988201538964 - 0.03712519863384679 * cq1 - 0.061988215039423436 * cq2 - 0.09282008695868763 * sq1
            - 0.13922726685788214 * cq1 * sq2;
   }

   private double computeALeft()
   {
      return 1.0;
   }
}
