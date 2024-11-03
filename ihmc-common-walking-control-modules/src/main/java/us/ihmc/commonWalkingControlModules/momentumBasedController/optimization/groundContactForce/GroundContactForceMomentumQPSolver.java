package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GroundContactForceMomentumQPSolver
{
   private static final boolean SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoFrameVector3D wrenchEquilibriumForceError;
   private final YoFrameVector3D wrenchEquilibriumTorqueError;

   private final YoBoolean firstCall = new YoBoolean("firstCall", registry);
   private final ActiveSetQPSolverWithInactiveVariablesInterface qpSolver;

   private final DMatrixRMaj solverInput_H;
   private final DMatrixRMaj solverInput_f;

   private final DMatrixRMaj solverInput_H_previous;
   private final DMatrixRMaj solverInput_f_previous;

   private final DMatrixRMaj solverInput_Aeq;
   private final DMatrixRMaj solverInput_beq;
   private final DMatrixRMaj solverInput_Ain;
   private final DMatrixRMaj solverInput_bin;

   private final DMatrixRMaj solverInput_lb;
   private final DMatrixRMaj solverInput_ub;

   private final DMatrixRMaj solverInput_lb_previous;
   private final DMatrixRMaj solverInput_ub_previous;

   private final DMatrixRMaj solverInput_activeIndices;

   private final DMatrixRMaj solverOutput;
   private final DMatrixRMaj solverOutput_momentumRate;
   private final DMatrixRMaj solverOutput_rhos;

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble momentumRateRegularization = new YoDouble("momentumRateRegularization", registry);
   private final YoDouble momentumAccelerationRegularization = new YoDouble("momentumAccelerationRegularization", registry);
   private final DMatrixRMaj regularizationMatrix;

   private final DMatrixRMaj tempJtW;
   private final DMatrixRMaj tempMomentumTask_H;
   private final DMatrixRMaj tempMomentumTask_f;
   private final DMatrixRMaj tempRhoTask_H;
   private final DMatrixRMaj tempRhoTask_f;

   private final int momentumSize = SpatialForce.SIZE;
   private final int rhoSize;
   private final int problemSize;

   private final boolean hasFloatingBase;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   public GroundContactForceMomentumQPSolver(ActiveSetQPSolverWithInactiveVariablesInterface qpSolver, int rhoSize, boolean hasFloatingBase,
                                             YoRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.rhoSize = rhoSize;
      this.problemSize = momentumSize + rhoSize;
      this.hasFloatingBase = hasFloatingBase;

      solverInput_H = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f = new DMatrixRMaj(problemSize, 1);

      solverInput_H_previous = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f_previous = new DMatrixRMaj(problemSize, 1);

      solverInput_Aeq = new DMatrixRMaj(0, problemSize);
      solverInput_beq = new DMatrixRMaj(0, 1);
      solverInput_Ain = new DMatrixRMaj(0, problemSize);
      solverInput_bin = new DMatrixRMaj(0, 1);

      solverInput_lb = new DMatrixRMaj(problemSize, 1);
      solverInput_ub = new DMatrixRMaj(problemSize, 1);

      solverInput_lb_previous = new DMatrixRMaj(problemSize, 1);
      solverInput_ub_previous = new DMatrixRMaj(problemSize, 1);

      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverInput_activeIndices = new DMatrixRMaj(problemSize, 1);
      CommonOps_DDRM.fill(solverInput_activeIndices, 1.0);

      solverOutput = new DMatrixRMaj(problemSize, 1);
      solverOutput_momentumRate = new DMatrixRMaj(momentumSize, 1);
      solverOutput_rhos = new DMatrixRMaj(rhoSize, 1);

      tempJtW = new DMatrixRMaj(problemSize, problemSize);
      tempMomentumTask_H = new DMatrixRMaj(momentumSize, momentumSize);
      tempMomentumTask_f = new DMatrixRMaj(momentumSize, 1);
      tempRhoTask_H = new DMatrixRMaj(rhoSize, rhoSize);
      tempRhoTask_f = new DMatrixRMaj(rhoSize, 1);

      regularizationMatrix = new DMatrixRMaj(problemSize, problemSize);

      momentumRateRegularization.set(0.00001);
      momentumAccelerationRegularization.set(0.000001);
      double defaultRhoRegularization = 0.00001;
      for (int i = 0; i < momentumSize; i++)
         regularizationMatrix.set(i, i, momentumRateRegularization.getDoubleValue());
      for (int i = rhoSize; i < problemSize; i++)
         regularizationMatrix.set(i, i, defaultRhoRegularization);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         wrenchEquilibriumForceError = new YoFrameVector3D("wrenchEquilibriumForceError", null, registry);
         wrenchEquilibriumTorqueError = new YoFrameVector3D("wrenchEquilibriumTorqueError", null, registry);
      }
      else
      {
         wrenchEquilibriumForceError = null;
         wrenchEquilibriumTorqueError = null;
      }

      parentRegistry.addChild(registry);
   }

   public void setMomentumRateRegularization(double weight)
   {
      momentumRateRegularization.set(weight);
   }

   public void setMomentumAccelerationRegularization(double weight)
   {
      momentumAccelerationRegularization.set(weight);
   }

   public void setRhoRegularizationWeight(DMatrixRMaj weight)
   {
      CommonOps_DDRM.insert(weight, regularizationMatrix, momentumSize, momentumSize);
   }

   public void setUseWarmStart(boolean useWarmStart)
   {
      this.useWarmStart = useWarmStart;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void notifyResetActiveSet()
   {
      this.resetActiveSet = true;
   }

   private boolean pollResetActiveSet()
   {
      boolean ret = resetActiveSet;
      resetActiveSet = false;
      return ret;
   }

   public void reset()
   {
      for (int i = 0; i < momentumSize; i++)
         regularizationMatrix.set(i, i, momentumRateRegularization.getDoubleValue());

      solverInput_H.zero();

      solverInput_f.zero();

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);

      solverInput_Ain.reshape(0, problemSize);
      solverInput_bin.reshape(0, 1);

      if (!firstCall.getBooleanValue())
         addMomentumAccelerationRegularization();
   }

   public void addRegularization()
   {
      CommonOps_DDRM.addEquals(solverInput_H, regularizationMatrix);
   }

   public void addMomentumAccelerationRegularization()
   {
      for (int i = 0; i < momentumSize; i++)
      {
         solverInput_H.add(i, i, momentumAccelerationRegularization.getDoubleValue());
         solverInput_f.add(i, 0, -momentumAccelerationRegularization.getDoubleValue() * solverOutput_momentumRate.get(i, 0));
      }
   }
   public void addMomentumInput(QPInputTypeA input)
   {
      switch (input.getConstraintType())
      {
      case OBJECTIVE:
         if (input.useWeightScalar())
            addMomentumTask(input.taskJacobian, input.taskObjective, input.getWeightScalar());
         else
            addMomentumTask(input.taskJacobian, input.taskObjective, input.taskWeightMatrix);
         break;
      case EQUALITY:
         addMomentumEqualityConstraint(input.taskJacobian, input.taskObjective);
         break;
      case LEQ_INEQUALITY:
         addMomentumLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
         break;
      case GEQ_INEQUALITY:
         addMomentumGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
         break;
      default:
         throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addMomentumTask(DMatrixRMaj taskJ, DMatrixRMaj taskObjective, double taskWeight)
   {
      int taskSize = taskJ.getNumRows();

      // J^T W
      tempJtW.reshape(momentumSize, taskSize);
      CommonOps_DDRM.transpose(taskJ, tempJtW);

      addMomentumTaskInternal(taskWeight, tempJtW, taskJ, taskObjective);
   }

   /**
    * Sets up a motion objective for the joint accelerations (qddot).
    * <p>
    *    min (hdot - b)^T * W * (hdot - b)
    * </p>
    * @param taskJacobian jacobian to map qddot to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be diagonal.
    */
   public void addMomentumTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(momentumSize, taskSize);
      DiagonalMatrixTools.postMultTransA(taskJacobian, taskWeight, tempJtW);

      addMomentumTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   private void addMomentumTaskInternal(double weight, DMatrixRMaj taskJt, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps_DDRM.multInner(taskJacobian, tempMomentumTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMomentumTask_H, 0, 0, momentumSize, momentumSize, weight);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.mult(taskJt, taskObjective, tempMomentumTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMomentumTask_f, 0, 0, momentumSize, 1, -weight);
   }

   private void addMomentumTaskInternal(DMatrixRMaj taskJtW, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps_DDRM.mult(taskJtW, taskJacobian, tempMomentumTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMomentumTask_H, 0, 0, momentumSize, momentumSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.mult(taskJtW, taskObjective, tempMomentumTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMomentumTask_f, 0, 0, momentumSize, 1, -1.0);
   }

   public void addMomentumEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps_DDRM.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps_DDRM.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void addMomentumLesserOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMomentumInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   public void addMomentumGreaterOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMomentumInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addMomentumInequalityConstraintInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double sign)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_bin.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Ain.reshape(previousSize + taskSize, problemSize, true);
      solverInput_bin.reshape(previousSize + taskSize, 1, true);

      MatrixTools.setMatrixBlock(solverInput_Ain, previousSize, 0, taskJacobian, 0, 0, taskSize, problemSize, sign);
      MatrixTools.setMatrixBlock(solverInput_bin, previousSize, 0, taskObjective, 0, 0, taskSize, 1, sign);
   }

   /**
    * Sets up a motion objective for the generalized contact forces (rhos).
    * <p>
    *    min (rho - b)^T * W * (rho - b)
    * </p>
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be diagonal.
    */
   public void addRhoTask(DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      MatrixTools.addMatrixBlock(solverInput_H, momentumSize, momentumSize, taskWeight, 0, 0, rhoSize, rhoSize, 1.0);

      DiagonalMatrixTools.preMult(taskWeight, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, momentumSize, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   /**
    * Sets up a motion objective for the generalized contact forces (rhos).
    * <p>
    *    min (J rho - b)^T * W * (J rho - b)
    * </p>
    * @param taskJacobian jacobian to map rho to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be diagonal.
    */
   public void addRhoTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();
      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      DiagonalMatrixTools.postMultTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      CommonOps_DDRM.mult(tempJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, momentumSize, momentumSize, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.mult(tempJtW, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, momentumSize, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void solve() throws NoConvergenceException
   {
      if (!hasWrenchesEquilibriumConstraintBeenSetup)
         throw new RuntimeException("The wrench equilibrium constraint has to be setup before calling solve().");

      addRegularization();

      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      numberOfActiveVariables.set((int) CommonOps_DDRM.elementSum(solverInput_activeIndices));

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setVariableBounds(solverInput_lb, solverInput_ub);
      qpSolver.setActiveVariables(solverInput_activeIndices);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      numberOfIterations.set(qpSolver.solve(solverOutput));

      qpSolverTimer.stopMeasurement();

      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (MatrixTools.containsNaN(solverOutput))
      {
         throw new NoConvergenceException(numberOfIterations.getIntegerValue());
      }

      CommonOps_DDRM.extract(solverOutput, 0, momentumSize, 0, 1, solverOutput_momentumRate, 0, 0);
      CommonOps_DDRM.extract(solverOutput, momentumSize, problemSize, 0, 1, solverOutput_rhos, 0, 0);

      firstCall.set(false);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         if (hasFloatingBase)
         {
            CommonOps_DDRM.mult(tempWrenchConstraint_J, solverOutput, tempWrenchConstraint_LHS);
            int index = 0;
            wrenchEquilibriumTorqueError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         }
      }

      solverInput_H_previous.set(solverInput_H);
      solverInput_f_previous.set(solverInput_f);

      solverInput_lb_previous.set(solverInput_lb);
      solverInput_ub_previous.set(solverInput_ub);
   }

   private final DMatrixRMaj tempWrenchConstraint_H = new DMatrixRMaj(200, 200);
   private final DMatrixRMaj tempWrenchConstraint_J = new DMatrixRMaj(Wrench.SIZE, 200);
   private final DMatrixRMaj tempWrenchConstraint_f = new DMatrixRMaj(Wrench.SIZE, 200);
   private final DMatrixRMaj tempWrenchConstraint_LHS = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj tempWrenchConstraint_RHS = new DMatrixRMaj(Wrench.SIZE, 1);

   /**
    * Need to be called before {@link #solve()}. It sets up the constraint that ensures that the
    * solution is dynamically feasible:
    * <p>
    * <li>hDot = Q * &rho; + &sum;W<sub>user</sub> + W<sub>gravity</sub>
    * <li>-hDot = - Q * &rho; - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li>[-I Q] * [hDot<sup>T</sup> &rho;<sup>T</sup>]<sup>T</sup> = -&sum;W<sub>user</sub> - W<sub>gravity</sub>
    * </p>
    *
    * @param momentumJacobian refers to I in the equation.
    * @param rhoJacobian refers to Q in the equation. Q&rho; represents external wrench to be
    *           optimized for.
    * @param additionalExternalWrench refers to &sum;W<sub>user</sub> in the equation. These are
    *           constant wrenches usually used for compensating for the weight of an object that the
    *           robot is holding.
    * @param gravityWrench refers to W<sub>gravity</sub> in the equation. It the wrench induced by
    *           the wieght of the robot.
    */
   public void setupWrenchesEquilibriumConstraint(DMatrixRMaj momentumJacobian, DMatrixRMaj rhoJacobian, DMatrixRMaj additionalExternalWrench,
                                                  DMatrixRMaj gravityWrench)
   {
      if (!hasFloatingBase)
      {
         hasWrenchesEquilibriumConstraintBeenSetup = true;
         return;
      }

      tempWrenchConstraint_RHS.zero();
      CommonOps_DDRM.subtractEquals(tempWrenchConstraint_RHS, additionalExternalWrench);
      CommonOps_DDRM.subtractEquals(tempWrenchConstraint_RHS, gravityWrench);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         tempWrenchConstraint_J.reshape(Wrench.SIZE, problemSize);
         MatrixTools.setMatrixBlock(tempWrenchConstraint_J, 0, 0, momentumJacobian, 0, 0, Wrench.SIZE, momentumSize, -1.0);
         CommonOps_DDRM.insert(rhoJacobian, tempWrenchConstraint_J, 0, momentumSize);

         double weight = 1500.0;
         tempWrenchConstraint_H.reshape(problemSize, problemSize);
         CommonOps_DDRM.multInner(tempWrenchConstraint_J, tempWrenchConstraint_H);
         CommonOps_DDRM.scale(weight, tempWrenchConstraint_H);
         CommonOps_DDRM.addEquals(solverInput_H, tempWrenchConstraint_H);

         tempWrenchConstraint_f.reshape(problemSize, 1);
         CommonOps_DDRM.multTransA(weight, tempWrenchConstraint_J, tempWrenchConstraint_RHS, tempWrenchConstraint_f);
         CommonOps_DDRM.subtractEquals(solverInput_f, tempWrenchConstraint_f);
      }
      else
      {
         int constraintSize = Wrench.SIZE;
         int previousSize = solverInput_beq.getNumRows();

         // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
         solverInput_Aeq.reshape(previousSize + constraintSize, problemSize, true);
         solverInput_beq.reshape(previousSize + constraintSize, 1, true);

         MatrixTools.setMatrixBlock(solverInput_Aeq, previousSize, 0, momentumJacobian, 0, 0, constraintSize, momentumSize, -1.0);
         CommonOps_DDRM.insert(rhoJacobian, solverInput_Aeq, previousSize, momentumSize);

         CommonOps_DDRM.insert(tempWrenchConstraint_RHS, solverInput_beq, previousSize, 0);
      }

      hasWrenchesEquilibriumConstraintBeenSetup = true;
   }

   public DMatrixRMaj getMomentumRate()
   {
      return solverOutput_momentumRate;
   }

   public DMatrixRMaj getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinRho(double rhoMin)
   {
      for (int i = momentumSize; i < problemSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DMatrixRMaj rhoMin)
   {
      CommonOps_DDRM.insert(rhoMin, solverInput_lb, momentumSize, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      for (int i = momentumSize; i < problemSize; i++)
         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DMatrixRMaj rhoMax)
   {
      CommonOps_DDRM.insert(rhoMax, solverInput_ub, momentumSize, 0);
   }


   public void setActiveRhos(DMatrixRMaj activeRhoMatrix)
   {
      CommonOps_DDRM.insert(activeRhoMatrix, solverInput_activeIndices, momentumSize, 0);
   }
}
