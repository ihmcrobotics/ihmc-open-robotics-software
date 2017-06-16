package us.ihmc.exampleSimulations.simpleArm;

import javax.vecmath.SingularMatrixException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.linearDynamicSystems.SplitUpMatrixExponentialStateSpaceSystemDiscretizer;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

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
   private final DenseMatrix64F xPriori = new DenseMatrix64F(nStates, 1);
   private final DenseMatrix64F xPosteriori = new DenseMatrix64F(nStates, 1);

   // the process
   // x_{k} = A x_{k-1} + B u_{k-1} + w_{k-1}
   // with w being the process noise
   private final DenseMatrix64F A = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F B = new DenseMatrix64F(nStates, nInputs);
   private final DenseMatrix64F u = new DenseMatrix64F(nInputs, 1);

   // the measurement
   // the measurement is [q1_m, q2_m, q3_m]
   // z_{k} = H x_{k} + v_{k}
   // with v being the measurement noise
   private final DenseMatrix64F z = new DenseMatrix64F(nMeasurements, 1);
   private final DenseMatrix64F H = new DenseMatrix64F(nMeasurements, nStates);

   // the noise matrices
   // Q is the process and R is the measurement noise covariance
   // p(w) ~ N(0, Q)
   // p(v) ~ N(0, R)
   // white noise with normal probability
   private final DenseMatrix64F Q = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F R = new DenseMatrix64F(nMeasurements, nMeasurements);

   // the kalman filter matrices
   private final DenseMatrix64F K = new DenseMatrix64F(nStates, nMeasurements);
   private final DenseMatrix64F PPriori = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F PPosteriori = new DenseMatrix64F(nStates, nStates);

   private final LinearSolver<DenseMatrix64F> solver;

   public SimpleArmEstimator(SimpleRobotInputOutputMap robot, double dt)
   {
      this.robot = robot;

      // set up the process as constant accelerations for now
      CommonOps.fill(A, 0.0);
      A.set(0, 3, 1.0);
      A.set(1, 4, 1.0);
      A.set(2, 5, 1.0);
      A.set(3, 6, 1.0);
      A.set(4, 7, 1.0);
      A.set(5, 8, 1.0);
      CommonOps.fill(B, 0.0);
      CommonOps.fill(u, 0.0);

      // set up the noise matrices
      double r_cov = 1.0;
      double q_cov = 1000.0;
      CommonOps.setIdentity(R);
      CommonOps.fill(Q, 0.0);
      Q.set(6, 6, 1.0);
      Q.set(7, 7, 1.0);
      Q.set(8, 8, 1.0);

      SplitUpMatrixExponentialStateSpaceSystemDiscretizer discretizer = new SplitUpMatrixExponentialStateSpaceSystemDiscretizer(nStates, nInputs);
      discretizer.discretize(A, B, Q, dt);

      CommonOps.scale(r_cov * r_cov, R);
      CommonOps.scale(q_cov * q_cov, Q);

      // set up the measurement matrix
      CommonOps.fill(H, 0.0);
      H.set(0, 0, 1.0);
      H.set(1, 1, 1.0);
      H.set(2, 2, 1.0);

      // initialize the error covariance
      CommonOps.setIdentity(PPosteriori);

      solver = LinearSolverFactory.linear(nMeasurements);
   }

   @Override
   public void doControl()
   {
      updateValues();
      doKalmanUpdate();
      saveState();
   }

   private final DenseMatrix64F APA = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F HPH = new DenseMatrix64F(nMeasurements, nMeasurements);
   private final DenseMatrix64F HPHplusR = new DenseMatrix64F(nMeasurements, nMeasurements);
   private final DenseMatrix64F HPHplusRinverse = new DenseMatrix64F(nMeasurements, nMeasurements);
   private final DenseMatrix64F PH = new DenseMatrix64F(nStates, nMeasurements);
   private final DenseMatrix64F Hx = new DenseMatrix64F(nMeasurements, 1);
   private final DenseMatrix64F residual = new DenseMatrix64F(nMeasurements, 1);
   private final DenseMatrix64F Kresidual = new DenseMatrix64F(nStates, 1);
   private final DenseMatrix64F identity = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F KH = new DenseMatrix64F(nStates, nStates);
   private final DenseMatrix64F IminusKH = new DenseMatrix64F(nStates, nStates);

   private final void doKalmanUpdate()
   {
      // time update
      CommonOps.mult(A, xPosteriori, xPriori);
      CommonOps.multAdd(B, u, xPriori);

      multBothSides(PPosteriori, A, APA);
      CommonOps.add(APA, Q, PPriori);

      // measurement update
      multBothSides(PPriori, H, HPH);
      CommonOps.add(HPH, R, HPHplusR);
      if (!solver.setA(HPHplusR))
         throw new SingularMatrixException();
      solver.invert(HPHplusRinverse);
      CommonOps.multTransB(PPriori, H, PH);
      CommonOps.mult(PH, HPHplusRinverse, K);

      CommonOps.mult(H, xPriori, Hx);
      CommonOps.subtract(z, Hx, residual);
      CommonOps.mult(K, residual, Kresidual);
      CommonOps.add(xPriori, Kresidual, xPosteriori);

      CommonOps.setIdentity(identity);
      CommonOps.mult(K, H, KH);
      CommonOps.subtract(identity, KH, IminusKH);
      CommonOps.mult(IminusKH, PPriori, PPosteriori);
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

   private final DenseMatrix64F tempForMultBothSides = new DenseMatrix64F(nStates, nStates);
   /** c = b * a * b' */
   private void multBothSides(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      tempForMultBothSides.reshape(a.getNumRows(), b.getNumRows());
      CommonOps.fill(tempForMultBothSides, 0.0);
      CommonOps.multAddTransB(a, b, tempForMultBothSides);
      CommonOps.mult(b, tempForMultBothSides, c);
   }
}
