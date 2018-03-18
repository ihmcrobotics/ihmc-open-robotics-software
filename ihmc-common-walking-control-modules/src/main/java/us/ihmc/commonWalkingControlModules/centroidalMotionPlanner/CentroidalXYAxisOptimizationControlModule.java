package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class CentroidalXYAxisOptimizationControlModule
{
   private final OptimizationControlModuleHelper helper;

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

   public CentroidalXYAxisOptimizationControlModule(OptimizationControlModuleHelper helper, CentroidalMotionPlannerParameters parameters)
   {
      this.helper = helper;

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
      CommonOps.extract(qpSolution, 0, 0, numberOfXAxisDecisionVariables, 1, axisQPSolution, 0, 0);
      helper.setDecisionVariableValues(Axis.X, axisQPSolution);

      axisQPSolution.reshape(numberOfYAxisDecisionVariables, 1);
      CommonOps.extract(qpSolution, numberOfXAxisDecisionVariables, 0, numberOfXAxisDecisionVariables + numberOfYAxisDecisionVariables, 1, axisQPSolution, 0, 0);
      helper.setDecisionVariableValues(Axis.Y, axisQPSolution);

   }
   
   private void setQPInputMatrices()
   {
      Axis xAxis = Axis.X;
      Axis yAxis = Axis.Y;
      numberOfXAxisDecisionVariables = helper.getNumberOfDecisionVariables(xAxis);
      numberOfYAxisDecisionVariables = helper.getNumberOfDecisionVariables(yAxis);
      int numberOFDecisionVariables = numberOfXAxisDecisionVariables + numberOfYAxisDecisionVariables;
      DenseMatrix64F xAxisHMatrix = helper.getObjectiveHMatrix(xAxis);
      DenseMatrix64F yAxisHMatrix = helper.getObjectiveHMatrix(yAxis);
      solverInput_H.zero();
      solverInput_H.reshape(numberOFDecisionVariables, numberOFDecisionVariables);
      CommonOps.insert(xAxisHMatrix, solverInput_H, 0, 0);
      CommonOps.insert(yAxisHMatrix, solverInput_H, numberOfXAxisDecisionVariables, numberOfXAxisDecisionVariables);

      DenseMatrix64F xAxisFMatrix = helper.getObjectivefMatrix(xAxis);
      DenseMatrix64F yAxisFMatrix = helper.getObjectivefMatrix(yAxis);
      solverInput_f.reshape(numberOFDecisionVariables, 1);
      CommonOps.insert(xAxisFMatrix, solverInput_f, 0, 0);
      CommonOps.insert(yAxisFMatrix, solverInput_f, numberOfXAxisDecisionVariables, 0);
      
      DenseMatrix64F xAxisAeqMatrix = helper.getConstraintAeqMatrix(xAxis);
      DenseMatrix64F yAxisAeqMatrix = helper.getConstraintAeqMatrix(yAxis);
      int numberOfEqualityConstraints = xAxisAeqMatrix.getNumRows() + yAxisAeqMatrix.getNumRows();
      solverInput_Aeq.reshape(numberOFDecisionVariables, numberOfEqualityConstraints);
      solverInput_Aeq.zero();
      CommonOps.insert(xAxisAeqMatrix, solverInput_Aeq, 0, 0);
      CommonOps.insert(yAxisAeqMatrix, solverInput_Aeq, xAxisAeqMatrix.getNumRows(), numberOfXAxisDecisionVariables);
      
      DenseMatrix64F xAxisbeqMatrix = helper.getConstraintbeqMatrix(xAxis);
      DenseMatrix64F yAxisbeqMatrix = helper.getConstraintbeqMatrix(yAxis);
      solverInput_beq.reshape(numberOfEqualityConstraints, 1);
      CommonOps.insert(xAxisbeqMatrix, solverInput_beq, 0, 0);
      CommonOps.insert(yAxisbeqMatrix, solverInput_beq, xAxisAeqMatrix.getNumRows(), 0);
      
      solverInput_Ain.reshape(0, numberOFDecisionVariables);
      solverInput_bin.reshape(0, 0);
      //setFrictionConeConstraints();
      //setTorqueConstraints();
  }
}