package us.ihmc.exampleSimulations.simpleArm;

import javax.vecmath.SingularMatrixException;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.robotics.linearDynamicSystems.SplitUpMatrixExponentialStateSpaceSystemDiscretizer;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleArmEstimator extends SimpleRobotController
{
   private final int nStates = 9;
   private final int nMeasurements = 3;
   private final int nInputs = 0;

   // the robot to read data from
   private final SimpleRobotInputOutputMap robot;

   // the state to display in SCS
   private final YoDouble qYaw = new YoDouble("qYaw", registry);
   private final YoDouble qPitch1 = new YoDouble("qPitch1", registry);
   private final YoDouble qPitch2 = new YoDouble("qPitch2", registry);
   private final YoDouble qdYaw = new YoDouble("qdYaw", registry);
   private final YoDouble qdPitch1 = new YoDouble("qdPitch1", registry);
   private final YoDouble qdPitch2 = new YoDouble("qdPitch2", registry);
   private final YoDouble qddYaw = new YoDouble("qddYaw", registry);
   private final YoDouble qddPitch1 = new YoDouble("qddPitch1", registry);
   private final YoDouble qddPitch2 = new YoDouble("qddPitch2", registry);

   // the state vectors
   // the state is defines as [q1, q2, q3, qd1, qd2, qd3, qdd1, qdd2, qdd3]
   private final DMatrixRMaj xPriori = new DMatrixRMaj(nStates, 1);
   private final DMatrixRMaj xPosteriori = new DMatrixRMaj(nStates, 1);

   // the process
   // x_{k} = A x_{k-1} + B u_{k-1} + w_{k-1}
   // with w being the process noise
   private final DMatrixRMaj A = new DMatrixRMaj(nStates, nStates);
   private final DMatrixRMaj B = new DMatrixRMaj(nStates, nInputs);
   private final DMatrixRMaj u = new DMatrixRMaj(nInputs, 1);

   // the measurement
   // the measurement is [q1_m, q2_m, q3_m]
   // z_{k} = H x_{k} + v_{k}
   // with v being the measurement noise
   private final DMatrixRMaj z = new DMatrixRMaj(nMeasurements, 1);
   private final DMatrixRMaj H = new DMatrixRMaj(nMeasurements, nStates);

   // the noise matrices
   // Q is the process and R is the measurement noise covariance
   // p(w) ~ N(0, Q)
   // p(v) ~ N(0, R)
   // white noise with normal probability
   private final DMatrixRMaj Q = new DMatrixRMaj(nStates, nStates);
   private final DMatrixRMaj R = new DMatrixRMaj(nMeasurements, nMeasurements);

   // the kalman filter matrices
   private final DMatrixRMaj K = new DMatrixRMaj(nStates, nMeasurements);
   private final DMatrixRMaj PPriori = new DMatrixRMaj(nStates, nStates);
   private final DMatrixRMaj PPosteriori = new DMatrixRMaj(nStates, nStates);

   private final LinearSolverDense<DMatrixRMaj> solver;

   public SimpleArmEstimator(SimpleRobotInputOutputMap robot, double dt)
   {
      this.robot = robot;

      // set up the process as constant accelerations for now
      CommonOps_DDRM.fill(A, 0.0);
      A.set(0, 3, 1.0);
      A.set(1, 4, 1.0);
      A.set(2, 5, 1.0);
      A.set(3, 6, 1.0);
      A.set(4, 7, 1.0);
      A.set(5, 8, 1.0);
      CommonOps_DDRM.fill(B, 0.0);
      CommonOps_DDRM.fill(u, 0.0);

      // set up the noise matrices
      double r_cov = 1.0;
      double q_cov = 1000.0;
      CommonOps_DDRM.setIdentity(R);
      CommonOps_DDRM.fill(Q, 0.0);
      Q.set(6, 6, 1.0);
      Q.set(7, 7, 1.0);
      Q.set(8, 8, 1.0);

      SplitUpMatrixExponentialStateSpaceSystemDiscretizer discretizer = new SplitUpMatrixExponentialStateSpaceSystemDiscretizer(nStates, nInputs);
      discretizer.discretize(A, B, Q, dt);

      CommonOps_DDRM.scale(r_cov * r_cov, R);
      CommonOps_DDRM.scale(q_cov * q_cov, Q);

      // set up the measurement matrix
      CommonOps_DDRM.fill(H, 0.0);
      H.set(0, 0, 1.0);
      H.set(1, 1, 1.0);
      H.set(2, 2, 1.0);

      // initialize the error covariance
      CommonOps_DDRM.setIdentity(PPosteriori);

      solver = LinearSolverFactory_DDRM.linear(nMeasurements);
   }

   @Override
   public void doControl()
   {
      updateValues();
      doKalmanUpdate();
      saveState();
   }

   private final DMatrixRMaj APA = new DMatrixRMaj(nStates, nStates);
   private final DMatrixRMaj HPH = new DMatrixRMaj(nMeasurements, nMeasurements);
   private final DMatrixRMaj HPHplusR = new DMatrixRMaj(nMeasurements, nMeasurements);
   private final DMatrixRMaj HPHplusRinverse = new DMatrixRMaj(nMeasurements, nMeasurements);
   private final DMatrixRMaj PH = new DMatrixRMaj(nStates, nMeasurements);
   private final DMatrixRMaj Hx = new DMatrixRMaj(nMeasurements, 1);
   private final DMatrixRMaj residual = new DMatrixRMaj(nMeasurements, 1);
   private final DMatrixRMaj Kresidual = new DMatrixRMaj(nStates, 1);
   private final DMatrixRMaj identity = new DMatrixRMaj(nStates, nStates);
   private final DMatrixRMaj KH = new DMatrixRMaj(nStates, nStates);
   private final DMatrixRMaj IminusKH = new DMatrixRMaj(nStates, nStates);

   private final void doKalmanUpdate()
   {
      // time update
      CommonOps_DDRM.mult(A, xPosteriori, xPriori);
      CommonOps_DDRM.multAdd(B, u, xPriori);

      multBothSides(PPosteriori, A, APA);
      CommonOps_DDRM.add(APA, Q, PPriori);

      // measurement update
      multBothSides(PPriori, H, HPH);
      CommonOps_DDRM.add(HPH, R, HPHplusR);
      if (!solver.setA(HPHplusR))
         throw new SingularMatrixException();
      solver.invert(HPHplusRinverse);
      CommonOps_DDRM.multTransB(PPriori, H, PH);
      CommonOps_DDRM.mult(PH, HPHplusRinverse, K);

      CommonOps_DDRM.mult(H, xPriori, Hx);
      CommonOps_DDRM.subtract(z, Hx, residual);
      CommonOps_DDRM.mult(K, residual, Kresidual);
      CommonOps_DDRM.add(xPriori, Kresidual, xPosteriori);

      CommonOps_DDRM.setIdentity(identity);
      CommonOps_DDRM.mult(K, H, KH);
      CommonOps_DDRM.subtract(identity, KH, IminusKH);
      CommonOps_DDRM.mult(IminusKH, PPriori, PPosteriori);
   }

   private final void updateValues()
   {
      robot.readFromSimulation();
      z.set(0, robot.getYaw());
      z.set(1, robot.getPitch1());
      z.set(2, robot.getPitch2());
   }

   private final void saveState()
   {
      qYaw.set(xPosteriori.get(0));
      qPitch1.set(xPosteriori.get(1));
      qPitch2.set(xPosteriori.get(2));
      qdYaw.set(xPosteriori.get(3));
      qdPitch1.set(xPosteriori.get(4));
      qdPitch2.set(xPosteriori.get(5));
      qddYaw.set(xPosteriori.get(6));
      qddPitch1.set(xPosteriori.get(7));
      qddPitch2.set(xPosteriori.get(8));
   }

   private final DMatrixRMaj tempForMultBothSides = new DMatrixRMaj(nStates, nStates);
   /** c = b * a * b' */
   private void multBothSides(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c)
   {
      tempForMultBothSides.reshape(a.getNumRows(), b.getNumRows());
      CommonOps_DDRM.fill(tempForMultBothSides, 0.0);
      CommonOps_DDRM.multAddTransB(a, b, tempForMultBothSides);
      CommonOps_DDRM.mult(b, tempForMultBothSides, c);
   }
}
