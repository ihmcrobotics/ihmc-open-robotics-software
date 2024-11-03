package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class GroundContactForceQPSolver
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoBoolean firstCall = new YoBoolean("firstCall", registry);
   private final ActiveSetQPSolver qpSolver;

   private final DMatrixRMaj solverInput_H;
   private final DMatrixRMaj solverInput_f;

   private final DMatrixRMaj solverInput_Aeq;
   private final DMatrixRMaj solverInput_beq;
   private final DMatrixRMaj solverInput_Ain;
   private final DMatrixRMaj solverInput_bin;

   private final DMatrixRMaj solverInput_lb;
   private final DMatrixRMaj solverInput_ub;

   private final DMatrixRMaj solverOutput_rhos;

   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final DMatrixRMaj regularizationMatrix;

   private final DMatrixRMaj tempJtW;
   private final DMatrixRMaj tempRhoTask_H;
   private final DMatrixRMaj tempRhoTask_f;

   private final int rhoSize;

   public GroundContactForceQPSolver(ActiveSetQPSolver qpSolver, int rhoSize, YoRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.rhoSize = rhoSize;

      solverInput_H = new DMatrixRMaj(rhoSize, rhoSize);
      solverInput_f = new DMatrixRMaj(rhoSize, 1);

      solverInput_Aeq = new DMatrixRMaj(0, rhoSize);
      solverInput_beq = new DMatrixRMaj(0, 1);
      solverInput_Ain = new DMatrixRMaj(0, rhoSize);
      solverInput_bin = new DMatrixRMaj(0, 1);

      solverInput_lb = new DMatrixRMaj(rhoSize, 1);
      solverInput_ub = new DMatrixRMaj(rhoSize, 1);

      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverOutput_rhos = new DMatrixRMaj(rhoSize, 1);

      tempJtW = new DMatrixRMaj(rhoSize, rhoSize);
      tempRhoTask_H = new DMatrixRMaj(rhoSize, rhoSize);
      tempRhoTask_f = new DMatrixRMaj(rhoSize, 1);

      regularizationMatrix = new DMatrixRMaj(rhoSize, rhoSize);

      double defaultRhoRegularization = 0.00001;
      for (int i = 0; i < rhoSize; i++)
         regularizationMatrix.set(i, i, defaultRhoRegularization);

      parentRegistry.addChild(registry);
   }

   public void setRhoRegularizationWeight(DMatrixRMaj weight)
   {
      CommonOps_DDRM.insert(weight, regularizationMatrix, 0, 0);
   }

   public void reset()
   {
      solverInput_H.zero();

      solverInput_f.zero();

      solverInput_Aeq.reshape(0, rhoSize);
      solverInput_beq.reshape(0, 1);
   }

   public void addRegularization()
   {
      CommonOps_DDRM.addEquals(solverInput_H, regularizationMatrix);
   }

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

   public void addMomentumTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      CommonOps_DDRM.multTransA(taskJacobian, taskWeight, tempJtW);

      addMomentumTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   public void addMotionTask(DMatrixRMaj taskJ, DMatrixRMaj taskObjective, double taskWeight)
   {
      int taskSize = taskJ.getNumRows();

      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      CommonOps_DDRM.transpose(taskJ, tempJtW);

      addMotionTaskInternal(taskWeight, tempJtW, taskJ, taskObjective);
   }

   /**
    * Sets up a motion objective for the joint accelerations (qddot).
    * <p>
    *    min (J qddot - b)^T * W * (J qddot - b)
    * </p>
    * @param taskJacobian jacobian to map qddot to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be diagonal.
    */
   public void addMotionTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      DiagonalMatrixTools.postMultTransA(taskJacobian, taskWeight, tempJtW);

      addMotionTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   // new, and hopefully faster
   private void addMotionTaskInternal(double weight, DMatrixRMaj taskJt, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps_DDRM.multInner(taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempRhoTask_H, 0, 0, rhoSize, rhoSize, weight);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.mult(taskJt, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -weight);
   }

   private void addMotionTaskInternal(DMatrixRMaj taskJtW, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps_DDRM.mult(taskJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.mult(taskJtW, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void addMomentumTaskInternal(DMatrixRMaj taskJtW, DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps_DDRM.mult(taskJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute f += -J^T W Objective
      CommonOps_DDRM.mult(taskJtW, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }


   public void addMotionEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      int taskSize = taskJacobian.getNumRows();
      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, rhoSize, true);
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
      solverInput_Ain.reshape(previousSize + taskSize, rhoSize, true);
      solverInput_bin.reshape(previousSize + taskSize, 1, true);

      MatrixTools.setMatrixBlock(solverInput_Ain, previousSize, 0, taskJacobian, 0, 0, taskSize, rhoSize, sign);
      MatrixTools.setMatrixBlock(solverInput_bin, previousSize, 0, taskObjective, 0, 0, taskSize, 1, sign);
   }

   public void addRhoTask(DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, taskWeight, 0, 0, rhoSize, rhoSize, 1.0);

      CommonOps_DDRM.mult(taskWeight, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void addRhoTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();
      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      CommonOps_DDRM.multTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      CommonOps_DDRM.mult(tempJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.mult(tempJtW, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void solve() throws NoConvergenceException
   {
      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setVariableBounds(solverInput_lb, solverInput_ub);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      numberOfIterations.set(qpSolver.solve(solverOutput_rhos));

      qpSolverTimer.stopMeasurement();

      if (MatrixTools.containsNaN(solverOutput_rhos))
         throw new NoConvergenceException(numberOfIterations.getIntegerValue());

      firstCall.set(false);
   }

   public DMatrixRMaj getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinRho(double rhoMin)
   {
      for (int i = 0; i < rhoSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DMatrixRMaj rhoMin)
   {
      CommonOps_DDRM.insert(rhoMin, solverInput_lb, 0, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      for (int i = 0; i < rhoSize; i++)
         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DMatrixRMaj rhoMax)
   {
      CommonOps_DDRM.insert(rhoMax, solverInput_ub, 0, 0);
   }
}
