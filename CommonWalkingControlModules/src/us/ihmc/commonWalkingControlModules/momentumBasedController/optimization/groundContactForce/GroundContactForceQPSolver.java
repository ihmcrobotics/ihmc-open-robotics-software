package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class GroundContactForceQPSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final BooleanYoVariable firstCall = new BooleanYoVariable("firstCall", registry);
   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;

   private final DenseMatrix64F solverOutput_rhos;

   private final IntegerYoVariable numberOfIterations = new IntegerYoVariable("numberOfIterations", registry);
   private final IntegerYoVariable numberOfEqualityConstraints = new IntegerYoVariable("numberOfEqualityConstraints", registry);
   private final IntegerYoVariable numberOfInequalityConstraints = new IntegerYoVariable("numberOfInequalityConstraints", registry);
   private final IntegerYoVariable numberOfConstraints = new IntegerYoVariable("numberOfConstraints", registry);
   private final DenseMatrix64F regularizationMatrix;

   private final DenseMatrix64F tempJtW;
   private final DenseMatrix64F tempRhoTask_H;
   private final DenseMatrix64F tempRhoTask_f;

   private final int rhoSize;

   public GroundContactForceQPSolver(int rhoSize, YoVariableRegistry parentRegistry)
   {
      this.rhoSize = rhoSize;

      solverInput_H = new DenseMatrix64F(rhoSize, rhoSize);
      solverInput_f = new DenseMatrix64F(rhoSize, 1);

      solverInput_Aeq = new DenseMatrix64F(0, rhoSize);
      solverInput_beq = new DenseMatrix64F(0, 1);
      solverInput_Ain = new DenseMatrix64F(0, rhoSize);
      solverInput_bin = new DenseMatrix64F(0, 1);

      solverInput_lb = new DenseMatrix64F(rhoSize, 1);
      solverInput_ub = new DenseMatrix64F(rhoSize, 1);

      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverOutput_rhos = new DenseMatrix64F(rhoSize, 1);

      tempJtW = new DenseMatrix64F(rhoSize, rhoSize);
      tempRhoTask_H = new DenseMatrix64F(rhoSize, rhoSize);
      tempRhoTask_f = new DenseMatrix64F(rhoSize, 1);

      regularizationMatrix = new DenseMatrix64F(rhoSize, rhoSize);

      double defaultRhoRegularization = 0.00001;
      for (int i = 0; i < rhoSize; i++)
         regularizationMatrix.set(i, i, defaultRhoRegularization);

      parentRegistry.addChild(registry);
   }

   public void setRhoRegularizationWeight(DenseMatrix64F weight)
   {
      CommonOps.insert(weight, regularizationMatrix, 0, 0);
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
      CommonOps.addEquals(solverInput_H, regularizationMatrix);
   }

   public void addMomentumTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();

      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      CommonOps.multTransA(taskJacobian, taskWeight, tempJtW);

      addMomentumTaskInternal(tempJtW, taskJacobian, taskObjective);
   }

   public void addMomentumTaskInternal(DenseMatrix64F taskJtW, DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective)
   {
      // Compute: H += J^T W J
      CommonOps.mult(taskJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute f += -J^T W Objective
      CommonOps.mult(taskJtW, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void addRhoTask(DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, taskWeight, 0, 0, rhoSize, rhoSize, 1.0);

      CommonOps.mult(taskWeight, taskObjective, tempRhoTask_f);
      MatrixTools.addMatrixBlock(solverInput_f, 0, 0, tempRhoTask_f, 0, 0, rhoSize, 1, -1.0);
   }

   public void addRhoTask(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      int taskSize = taskJacobian.getNumRows();
      // J^T W
      tempJtW.reshape(rhoSize, taskSize);
      CommonOps.multTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      CommonOps.mult(tempJtW, taskJacobian, tempRhoTask_H);
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, tempRhoTask_H, 0, 0, rhoSize, rhoSize, 1.0);

      // Compute: f += - J^T W Objective
      CommonOps.mult(tempJtW, taskObjective, tempRhoTask_f);
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

   public DenseMatrix64F getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinRho(double rhoMin)
   {
      for (int i = 0; i < rhoSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DenseMatrix64F rhoMin)
   {
      CommonOps.insert(rhoMin, solverInput_lb, 0, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      for (int i = 0; i < rhoSize; i++)
         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DenseMatrix64F rhoMax)
   {
      CommonOps.insert(rhoMax, solverInput_ub, 0, 0);
   }
}
