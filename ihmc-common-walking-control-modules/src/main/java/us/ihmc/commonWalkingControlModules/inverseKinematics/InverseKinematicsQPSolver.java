package us.ihmc.commonWalkingControlModules.inverseKinematics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPVariableSubstitution;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class InverseKinematicsQPSolver
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoBoolean firstCall = new YoBoolean("firstCall", registry);
   private final ActiveSetQPSolverWithInactiveVariablesInterface qpSolver;

   private final QPVariableSubstitution variableSubstitution = new QPVariableSubstitution();

   private final DMatrixRMaj solverInput_H;
   private final DMatrixRMaj solverInput_f;

   private final DMatrixRMaj solverInput_Aeq;
   private final DMatrixRMaj solverInput_beq;
   private final DMatrixRMaj solverInput_Ain;
   private final DMatrixRMaj solverInput_bin;

   private final DMatrixRMaj solverInput_lb;
   private final DMatrixRMaj solverInput_ub;

   private final DMatrixRMaj solverOutput;
   private final DMatrixRMaj desiredJointVelocities;

   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble jointVelocityRegularization = new YoDouble("jointVelocityRegularization", registry);
   private final YoDouble jointAccelerationRegularization = new YoDouble("jointAccelerationRegularization", registry);

   private final int numberOfDoFs;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   private final double dt;

   public InverseKinematicsQPSolver(ActiveSetQPSolverWithInactiveVariablesInterface qpSolver, int numberOfDoFs, double dt, YoRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.numberOfDoFs = numberOfDoFs;
      this.dt = dt;

      firstCall.set(true);

      solverInput_H = new DMatrixRMaj(numberOfDoFs, numberOfDoFs);
      solverInput_f = new DMatrixRMaj(numberOfDoFs, 1);
      solverInput_lb = new DMatrixRMaj(numberOfDoFs, 1);
      solverInput_ub = new DMatrixRMaj(numberOfDoFs, 1);

      solverInput_Aeq = new DMatrixRMaj(0, numberOfDoFs);
      solverInput_beq = new DMatrixRMaj(0, 1);
      solverInput_Ain = new DMatrixRMaj(0, numberOfDoFs);
      solverInput_bin = new DMatrixRMaj(0, 1);

      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverOutput = new DMatrixRMaj(numberOfDoFs, 1);
      jointVelocityRegularization.set(0.5);
      jointAccelerationRegularization.set(10.0);

      desiredJointVelocities = new DMatrixRMaj(numberOfDoFs, 1);

      variableSubstitution.setIgnoreBias(true);

      parentRegistry.addChild(registry);
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
      variableSubstitution.reset();

      solverInput_H.zero();
      for (int i = 0; i < numberOfDoFs; i++)
         solverInput_H.set(i, i, jointVelocityRegularization.getDoubleValue());

      solverInput_f.reshape(numberOfDoFs, 1);
      solverInput_f.zero();

      solverInput_Aeq.reshape(0, numberOfDoFs);
      solverInput_beq.reshape(0, 1);

      solverInput_Ain.reshape(0, numberOfDoFs);
      solverInput_bin.reshape(0, 1);

      if (!firstCall.getBooleanValue())
         addJointAccelerationRegularization();
   }

   private void addJointAccelerationRegularization()
   {
      double factor = dt * dt / jointAccelerationRegularization.getDoubleValue();
      for (int i = 0; i < numberOfDoFs; i++)
      {
         solverInput_H.add(i, i, 1.0 / factor);
         solverInput_f.add(i, 0, -desiredJointVelocities.get(i, 0) / factor);
      }
   }

   private final DMatrixRMaj tempJtW = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj tempTask_H = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj tempTask_f = new DMatrixRMaj(1, 1);

   public void addMotionInput(QPInputTypeA input)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
               addMotionTask(input.taskJacobian, input.taskObjective, input.getWeightScalar());
            else
               addMotionTask(input.taskJacobian, input.taskObjective, input.taskWeightMatrix);
            break;
         case EQUALITY:
            addMotionEqualityConstraint(input.taskJacobian, input.taskObjective);
            break;
         case LEQ_INEQUALITY:
            addMotionLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
            break;
         case GEQ_INEQUALITY:
            addMotionGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addMotionTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(numberOfDoFs, taskSize);
      MatrixTools.scaleTranspose(taskWeight, taskJacobian, tempJtW);

      addMotionTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   public void addMotionTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(numberOfDoFs, taskSize);
      CommonOps_DDRM.multTransA(taskJacobian, taskWeight, tempJtW);

      addMotionTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   private void addMotionTaskInternal(DMatrixRMaj taskJtW, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      // Compute: H += J^T W J
      tempTask_H.reshape(numberOfDoFs, numberOfDoFs);
      CommonOps_DDRM.mult(taskJtW, taskJacobian, tempTask_H);
      CommonOps_DDRM.addEquals(solverInput_H, tempTask_H);

      // Compute: f += - J^T W xDot
      tempTask_f.reshape(numberOfDoFs, 1);
      CommonOps_DDRM.mult(taskJtW, taskObjective, tempTask_f);
      CommonOps_DDRM.subtractEquals(solverInput_f, tempTask_f);
   }

   public void addMotionEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, numberOfDoFs, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps_DDRM.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps_DDRM.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void addMotionLesserOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMotionInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMotionInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addMotionInequalityConstraintInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double sign)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_bin.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Ain.reshape(previousSize + taskSize, numberOfDoFs, true);
      solverInput_bin.reshape(previousSize + taskSize, 1, true);

      MatrixTools.setMatrixBlock(solverInput_Ain, previousSize, 0, taskJacobian, 0, 0, taskSize, numberOfDoFs, sign);
      MatrixTools.setMatrixBlock(solverInput_bin, previousSize, 0, taskObjective, 0, 0, taskSize, 1, sign);
   }

   public void addVariableSubstitution(QPVariableSubstitution substitution)
   {
      this.variableSubstitution.concatenate(substitution);
   }

   public void solve() throws NoConvergenceException
   {
      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      TIntArrayList inactiveIndices = applySubstitution(); // This needs to be done right before configuring the QP and solving.
      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setVariableBounds(solverInput_lb, solverInput_ub);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      if (inactiveIndices != null)
      {
         for (int i = 0; i < inactiveIndices.size(); i++)
         {
            qpSolver.setVariableInactive(inactiveIndices.get(i));
         }
      }

      numberOfIterations.set(qpSolver.solve(solverOutput));
      removeSubstitution(); // This needs to be done right after solving.

      qpSolverTimer.stopMeasurement();

      if (MatrixTools.containsNaN(solverOutput))
         throw new NoConvergenceException(numberOfIterations.getIntegerValue());

      desiredJointVelocities.set(solverOutput);
      firstCall.set(false);
   }

   private TIntArrayList applySubstitution()
   {
      if (variableSubstitution.isEmpty())
         return null;

      variableSubstitution.applySubstitutionToObjectiveFunction(solverInput_H, solverInput_f);
      variableSubstitution.applySubstitutionToLinearConstraint(solverInput_Aeq, solverInput_beq);
      variableSubstitution.applySubstitutionToLinearConstraint(solverInput_Ain, solverInput_bin);
      variableSubstitution.applySubstitutionToBounds(solverInput_lb, solverInput_ub, solverInput_Ain, solverInput_bin);
      return variableSubstitution.getInactiveIndices();
   }

   private void removeSubstitution()
   {
      if (variableSubstitution.isEmpty())
         return;

      variableSubstitution.removeSubstitutionToSolution(solverOutput);
   }

   public DMatrixRMaj getJointVelocities()
   {
      return desiredJointVelocities;
   }

   public void setVelocityRegularizationWeight(double weight)
   {
      jointVelocityRegularization.set(weight);
   }

   public void setAccelerationRegularizationWeight(double weight)
   {
      jointAccelerationRegularization.set(weight);
   }

   public void setMinJointVelocities(DMatrixRMaj qDotMin)
   {
      solverInput_lb.set(qDotMin);
   }

   public void setMaxJointVelocities(DMatrixRMaj qDotMax)
   {
      solverInput_ub.set(qDotMax);
   }
}
