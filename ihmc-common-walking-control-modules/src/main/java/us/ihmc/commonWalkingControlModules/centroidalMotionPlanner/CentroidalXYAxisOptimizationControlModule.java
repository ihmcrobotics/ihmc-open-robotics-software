package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;

public class CentroidalXYAxisOptimizationControlModule
{
   private final LinearControlModuleHelper linearHelper;
   private final AngularControlModuleHelper angularHelper;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;
   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;
   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;

   // Variables to store results for runtime
   private final JavaQuadProgSolver qpSolver;
   private final DenseMatrix64F qpSolution;
   private final DenseMatrix64F axisQPSolution;

   private int numberOfXAxisDecisionVariables;
   private int numberOfYAxisDecisionVariables;

   private final DenseMatrix64F tempMatrix1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempMatrix2 = new DenseMatrix64F(0, 1);

   public CentroidalXYAxisOptimizationControlModule(LinearControlModuleHelper helper, AngularControlModuleHelper angularHelper,
                                                    CentroidalMotionPlannerParameters parameters)
   {
      this.linearHelper = helper;
      this.angularHelper = angularHelper;

      // Initialize the QP matrices
      solverInput_H = new DenseMatrix64F(0, 1);
      solverInput_f = new DenseMatrix64F(0, 1);
      solverInput_lb = new DenseMatrix64F(0, 1);
      solverInput_ub = new DenseMatrix64F(0, 1);
      solverInput_Aeq = new DenseMatrix64F(0, 1);
      solverInput_beq = new DenseMatrix64F(0, 1);
      solverInput_Ain = new DenseMatrix64F(0, 1);
      solverInput_bin = new DenseMatrix64F(0, 1);

      qpSolver = new JavaQuadProgSolver();
      qpSolver.setConvergenceThreshold(parameters.getOptimizationConvergenceThreshold());
      qpSolution = new DenseMatrix64F(0, 1);
      axisQPSolution = new DenseMatrix64F(0, 1);
      reset();
   }

   public void reset()
   {
      qpSolver.clear();
   }

   public void compute()
   {
      setQPInputMatrices();
      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLowerBounds(solverInput_lb);
      qpSolver.setUpperBounds(solverInput_ub);
      try
      {
         qpSolver.solve(qpSolution);
      }
      catch (RuntimeException exception)
      {
         PrintTools.debug("Got runtime exception from QP solver");
      }
      submitQPSolution();
   }

   private void submitQPSolution()
   {
      axisQPSolution.reshape(numberOfXAxisDecisionVariables, 1);
      CommonOps.extract(qpSolution, 0, numberOfXAxisDecisionVariables, 0, 1, axisQPSolution, 0, 0);
      linearHelper.setDecisionVariableValues(Axis.X, axisQPSolution);

      axisQPSolution.reshape(numberOfYAxisDecisionVariables, 1);
      CommonOps.extract(qpSolution, numberOfXAxisDecisionVariables, numberOfXAxisDecisionVariables + numberOfYAxisDecisionVariables, 0, 1, axisQPSolution, 0,
                        0);
      linearHelper.setDecisionVariableValues(Axis.Y, axisQPSolution);

   }

   private void setQPInputMatrices()
   {
      Axis xAxis = Axis.X;
      Axis yAxis = Axis.Y;
      DenseMatrix64F Hx = linearHelper.getObjectiveHMatrix(xAxis);
      DenseMatrix64F fx = linearHelper.getObjectivefMatrix(xAxis);
      DenseMatrix64F Hy = linearHelper.getObjectiveHMatrix(yAxis);
      DenseMatrix64F fy = linearHelper.getObjectivefMatrix(yAxis);
      angularHelper.processLinearObjectives(Hx, fx, Hy, fy, solverInput_H, solverInput_f);

      DenseMatrix64F xAeq = linearHelper.getConstraintAeqMatrix(xAxis);
      DenseMatrix64F xbeq = linearHelper.getConstraintbeqMatrix(xAxis);
      DenseMatrix64F yAeq = linearHelper.getConstraintAeqMatrix(yAxis);
      DenseMatrix64F ybeq = linearHelper.getConstraintbeqMatrix(yAxis);
      angularHelper.processLinearConstraints(xAeq, xbeq, yAeq, ybeq, solverInput_Aeq, solverInput_beq);
      angularHelper.getConsolidatedTorqueConstraints(tempMatrix1, tempMatrix2);
      CommonOps.scale(-1.0, tempMatrix2);
      int indexToInsertTorqueConstrains = solverInput_Aeq.getNumRows();
      solverInput_Aeq.reshape(indexToInsertTorqueConstrains + tempMatrix1.getNumRows(), solverInput_Aeq.getNumCols());
      CommonOps.insert(tempMatrix1, solverInput_Aeq, indexToInsertTorqueConstrains, 0);
      CommonOps.insert(tempMatrix2, solverInput_beq, indexToInsertTorqueConstrains, 0);

      DenseMatrix64F xAin = linearHelper.getConstraintAinMatrix(xAxis);
      DenseMatrix64F xbin = linearHelper.getConstraintbinMatrix(xAxis);
      DenseMatrix64F yAin = linearHelper.getConstraintAinMatrix(yAxis);
      DenseMatrix64F ybin = linearHelper.getConstraintbinMatrix(yAxis);
      angularHelper.processLinearConstraints(xAin, xbin, yAin, ybin, solverInput_Ain, solverInput_bin);
      angularHelper.getConsolidatedCoPSupportPolygonConstraints(tempMatrix1, tempMatrix2);
      int indexToInsertCoPConstraints = solverInput_Ain.getNumRows();
      solverInput_Ain.reshape(indexToInsertCoPConstraints + tempMatrix1.getNumRows(), solverInput_Ain.getNumCols(), true);
      solverInput_bin.reshape(indexToInsertCoPConstraints + tempMatrix1.getNumRows(), 1, true);
      CommonOps.insert(tempMatrix1, solverInput_Ain, indexToInsertCoPConstraints, 0);
      CommonOps.insert(tempMatrix2, solverInput_bin, indexToInsertCoPConstraints, 0);
      
      DenseMatrix64F xUb = linearHelper.getDecisionVariableUpperBoundMatrix(xAxis);
      DenseMatrix64F yUb = linearHelper.getDecisionVariableUpperBoundMatrix(yAxis);
      DenseMatrix64F xLb = linearHelper.getDecisionVariableLowerBoundMatrix(xAxis);
      DenseMatrix64F yLb = linearHelper.getDecisionVariableLowerBoundMatrix(yAxis);
      angularHelper.processLinearBounds(xLb, xUb, yLb, yUb, tempMatrix1, tempMatrix2);
      int indexToInsertBoundConstraints = solverInput_Ain.getNumRows();
      solverInput_Ain.reshape(indexToInsertBoundConstraints + tempMatrix1.getNumRows(), solverInput_Ain.getNumCols(), true);
      solverInput_bin.reshape(indexToInsertBoundConstraints + tempMatrix1.getNumRows(), 1, true);
      CommonOps.insert(tempMatrix1, solverInput_Ain, indexToInsertBoundConstraints, 0);
      CommonOps.insert(tempMatrix2, solverInput_bin, indexToInsertBoundConstraints, 0);
      

   }
}