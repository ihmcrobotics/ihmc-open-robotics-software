package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GroundContactForceMomentumQPSolver
{
   private static final boolean SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoFrameVector wrenchEquilibriumForceError;
   private final YoFrameVector wrenchEquilibriumTorqueError;

   private final YoBoolean firstCall = new YoBoolean("firstCall", registry);
   private final ActiveSetQPSolverWithInactiveVariablesInterface qpSolver;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;

   private final DenseMatrix64F solverInput_H_previous;
   private final DenseMatrix64F solverInput_f_previous;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;

   private final DenseMatrix64F solverInput_lb_previous;
   private final DenseMatrix64F solverInput_ub_previous;

   private final DenseMatrix64F solverInput_activeIndices;

   private final DenseMatrix64F solverOutput;
   private final DenseMatrix64F solverOutput_momentumRate;
   private final DenseMatrix64F solverOutput_rhos;

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble momentumRateRegularization = new YoDouble("momentumRateRegularization", registry);
   private final YoDouble momentumAccelerationRegularization = new YoDouble("momentumAccelerationRegularization", registry);
   private final DenseMatrix64F regularizationMatrix;

   private final DenseMatrix64F tempJtW;
   private final DenseMatrix64F tempMomentumTask_H;
   private final DenseMatrix64F tempMomentumTask_f;
   private final DenseMatrix64F tempRhoTask_H;
   private final DenseMatrix64F tempRhoTask_f;

   private final int momentumSize = SpatialForceVector.SIZE;
   private final int rhoSize;
   private final int problemSize;

   private final boolean hasFloatingBase;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   public GroundContactForceMomentumQPSolver(ActiveSetQPSolverWithInactiveVariablesInterface qpSolver, int rhoSize, boolean hasFloatingBase,
                                             YoVariableRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.rhoSize = rhoSize;
      this.problemSize = momentumSize + rhoSize;
      this.hasFloatingBase = hasFloatingBase;

      solverInput_H = new DenseMatrix64F(problemSize, problemSize);
      solverInput_f = new DenseMatrix64F(problemSize, 1);

      solverInput_H_previous = new DenseMatrix64F(problemSize, problemSize);
      solverInput_f_previous = new DenseMatrix64F(problemSize, 1);

      solverInput_Aeq = new DenseMatrix64F(0, problemSize);
      solverInput_beq = new DenseMatrix64F(0, 1);
      solverInput_Ain = new DenseMatrix64F(0, problemSize);
      solverInput_bin = new DenseMatrix64F(0, 1);

      solverInput_lb = new DenseMatrix64F(problemSize, 1);
      solverInput_ub = new DenseMatrix64F(problemSize, 1);

      solverInput_lb_previous = new DenseMatrix64F(problemSize, 1);
      solverInput_ub_previous = new DenseMatrix64F(problemSize, 1);

      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverInput_activeIndices = new DenseMatrix64F(problemSize, 1);
      CommonOps.fill(solverInput_activeIndices, 1.0);

      solverOutput = new DenseMatrix64F(problemSize, 1);
      solverOutput_momentumRate = new DenseMatrix64F(momentumSize, 1);
      solverOutput_rhos = new DenseMatrix64F(rhoSize, 1);

      tempJtW = new DenseMatrix64F(problemSize, problemSize);
      tempMomentumTask_H = new DenseMatrix64F(momentumSize, momentumSize);
      tempMomentumTask_f = new DenseMatrix64F(momentumSize, 1);
      tempRhoTask_H = new DenseMatrix64F(rhoSize, rhoSize);
      tempRhoTask_f = new DenseMatrix64F(rhoSize, 1);

      regularizationMatrix = new DenseMatrix64F(problemSize, problemSize);

      momentumRateRegularization.set(0.00001);
      momentumAccelerationRegularization.set(0.0001);
      double defaultRhoRegularization = 0.00001;
      for (int i = 0; i < momentumSize; i++)
         regularizationMatrix.set(i, i, momentumRateRegularization.getDoubleValue());
      for (int i = rhoSize; i < problemSize; i++)
         regularizationMatrix.set(i, i, defaultRhoRegularization);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         wrenchEquilibriumForceError = new YoFrameVector("wrenchEquilibriumForceError", null, registry);
         wrenchEquilibriumTorqueError = new YoFrameVector("wrenchEquilibriumTorqueError", null, registry);
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

   public void setRhoRegularizationWeight(DenseMatrix64F weight)
   {
      CommonOps.insert(weight, regularizationMatrix, 0, 0);
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
      CommonOps.addEquals(solverInput_H, regularizationMatrix);
   }

   public void addMomentumAccelerationRegularization()
   {
      for (int i = 0; i < momentumSize; i++)
      {
         solverInput_H.add(i, i, momentumAccelerationRegularization.getDoubleValue());
         solverInput_f.add(i, 0, -momentumAccelerationRegularization.getDoubleValue() * solverOutput_momentumRate.get(i, 0));
      }
   }
   public void addMomentumInput(MotionQPInput input)
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

   public void addMomentumTask(DenseMatrix64F taskJ, DenseMatrix64F taskObjective, double taskWeight)
   {
      int taskSize = taskJ.getNumRows();

      // J^T W
      tempJtW.reshape(momentumSize, taskSize);
      CommonOps.transpose(taskJ, tempJtW);

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
   public void addMomentumTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(momentumSize, taskSize);
      DiagonalMatrixTools.postMultTransA(taskJacobian, taskWeight, tempJtW);

      addMomentumTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   private void addMomentumTaskInternal(double weight, DenseMatrix64F taskJt, DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps.multInner(taskJacobian, tempMomentumTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMomentumTask_H, 0, 0, momentumSize, momentumSize, weight);

      // Compute: f += - J^T W Objective
      CommonOps.mult(taskJt, taskObjective, tempMomentumTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMomentumTask_f, 0, 0, momentumSize, 1, -weight);
   }

   private void addMomentumTaskInternal(DenseMatrix64F taskJtW, DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps.mult(taskJtW, taskJacobian, tempMomentumTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempMomentumTask_H, 0, 0, momentumSize, momentumSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps.mult(taskJtW, taskObjective, tempMomentumTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempMomentumTask_f, 0, 0, momentumSize, 1, -1.0);
   }

   public void addMomentumEqualityConstraint(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void addMomentumLesserOrEqualInequalityConstraint(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      addMomentumInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   public void addMomentumGreaterOrEqualInequalityConstraint(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      addMomentumInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addMomentumInequalityConstraintInternal(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, double sign)
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
   public void addRhoTask(DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
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
   public void addRhoTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();
      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      DiagonalMatrixTools.postMultTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      CommonOps.mult(tempJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, momentumSize, momentumSize, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps.mult(tempJtW, taskObjective, tempRhoTask_f);
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
         qpSolver.resetActiveConstraints();

      numberOfActiveVariables.set((int) CommonOps.elementSum(solverInput_activeIndices));

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

      CommonOps.extract(solverOutput, 0, momentumSize, 0, 1, solverOutput_momentumRate, 0, 0);
      CommonOps.extract(solverOutput, momentumSize, problemSize, 0, 1, solverOutput_rhos, 0, 0);

      firstCall.set(false);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         if (hasFloatingBase)
         {
            CommonOps.mult(tempWrenchConstraint_J, solverOutput, tempWrenchConstraint_LHS);
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

   private final DenseMatrix64F tempWrenchConstraint_H = new DenseMatrix64F(200, 200);
   private final DenseMatrix64F tempWrenchConstraint_J = new DenseMatrix64F(Wrench.SIZE, 200);
   private final DenseMatrix64F tempWrenchConstraint_f = new DenseMatrix64F(Wrench.SIZE, 200);
   private final DenseMatrix64F tempWrenchConstraint_LHS = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tempWrenchConstraint_RHS = new DenseMatrix64F(Wrench.SIZE, 1);

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
   public void setupWrenchesEquilibriumConstraint(DenseMatrix64F momentumJacobian, DenseMatrix64F rhoJacobian, DenseMatrix64F additionalExternalWrench,
                                                  DenseMatrix64F gravityWrench)
   {
      if (!hasFloatingBase)
      {
         hasWrenchesEquilibriumConstraintBeenSetup = true;
         return;
      }

      tempWrenchConstraint_RHS.zero();
      CommonOps.subtractEquals(tempWrenchConstraint_RHS, additionalExternalWrench);
      CommonOps.subtractEquals(tempWrenchConstraint_RHS, gravityWrench);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         tempWrenchConstraint_J.reshape(Wrench.SIZE, problemSize);
         MatrixTools.setMatrixBlock(tempWrenchConstraint_J, 0, 0, momentumJacobian, 0, 0, Wrench.SIZE, momentumSize, -1.0);
         CommonOps.insert(rhoJacobian, tempWrenchConstraint_J, 0, momentumSize);

         double weight = 150.0;
         tempWrenchConstraint_H.reshape(problemSize, problemSize);
         CommonOps.multInner(tempWrenchConstraint_J, tempWrenchConstraint_H);
         CommonOps.scale(weight, tempWrenchConstraint_H);
         CommonOps.addEquals(solverInput_H, tempWrenchConstraint_H);

         tempWrenchConstraint_f.reshape(problemSize, 1);
         CommonOps.multTransA(weight, tempWrenchConstraint_J, tempWrenchConstraint_RHS, tempWrenchConstraint_f);
         CommonOps.subtractEquals(solverInput_f, tempWrenchConstraint_f);
      }
      else
      {
         int constraintSize = Wrench.SIZE;
         int previousSize = solverInput_beq.getNumRows();

         // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
         solverInput_Aeq.reshape(previousSize + constraintSize, problemSize, true);
         solverInput_beq.reshape(previousSize + constraintSize, 1, true);

         MatrixTools.setMatrixBlock(solverInput_Aeq, previousSize, 0, momentumJacobian, 0, 0, constraintSize, momentumSize, -1.0);
         CommonOps.insert(rhoJacobian, solverInput_Aeq, previousSize, momentumSize);

         CommonOps.insert(tempWrenchConstraint_RHS, solverInput_beq, previousSize, 0);
      }

      hasWrenchesEquilibriumConstraintBeenSetup = true;
   }

   public DenseMatrix64F getMomentumRate()
   {
      return solverOutput_momentumRate;
   }

   public DenseMatrix64F getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinRho(double rhoMin)
   {
      for (int i = momentumSize; i < problemSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DenseMatrix64F rhoMin)
   {
      CommonOps.insert(rhoMin, solverInput_lb, momentumSize, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      for (int i = momentumSize; i < problemSize; i++)
         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DenseMatrix64F rhoMax)
   {
      CommonOps.insert(rhoMax, solverInput_ub, momentumSize, 0);
   }


   public void setActiveRhos(DenseMatrix64F activeRhoMatrix)
   {
      CommonOps.insert(activeRhoMatrix, solverInput_activeIndices, momentumSize, 0);
   }
}
