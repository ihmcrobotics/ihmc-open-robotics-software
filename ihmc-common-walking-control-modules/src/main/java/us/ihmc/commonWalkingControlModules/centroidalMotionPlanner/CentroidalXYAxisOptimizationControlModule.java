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
   private final LinearControlModuleHelper linearHelper;

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

   public CentroidalXYAxisOptimizationControlModule(LinearControlModuleHelper helper, AngularControlModuleHelper angularHelper,
                                                    CentroidalMotionPlannerParameters parameters)
   {
      this.linearHelper = helper;

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
      numberOfXAxisDecisionVariables = linearHelper.getNumberOfDecisionVariables(xAxis);
      numberOfYAxisDecisionVariables = linearHelper.getNumberOfDecisionVariables(yAxis);

      int numberOfDecisionVariables = numberOfXAxisDecisionVariables + numberOfYAxisDecisionVariables;
      DenseMatrix64F xAxisHMatrix = linearHelper.getObjectiveHMatrix(xAxis);
      DenseMatrix64F yAxisHMatrix = linearHelper.getObjectiveHMatrix(yAxis);
      solverInput_H.zero();
      solverInput_H.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      CommonOps.insert(xAxisHMatrix, solverInput_H, 0, 0);
      CommonOps.insert(yAxisHMatrix, solverInput_H, numberOfXAxisDecisionVariables, numberOfXAxisDecisionVariables);

      DenseMatrix64F xAxisFMatrix = linearHelper.getObjectivefMatrix(xAxis);
      DenseMatrix64F yAxisFMatrix = linearHelper.getObjectivefMatrix(yAxis);
      solverInput_f.reshape(numberOfDecisionVariables, 1);
      CommonOps.insert(xAxisFMatrix, solverInput_f, 0, 0);
      CommonOps.insert(yAxisFMatrix, solverInput_f, numberOfXAxisDecisionVariables, 0);

      DenseMatrix64F xAxisAeqMatrix = linearHelper.getConstraintAeqMatrix(xAxis);
      DenseMatrix64F yAxisAeqMatrix = linearHelper.getConstraintAeqMatrix(yAxis);
      int numberOfEqualityConstraints = xAxisAeqMatrix.getNumRows() + yAxisAeqMatrix.getNumRows();
      solverInput_Aeq.reshape(numberOfEqualityConstraints, numberOfDecisionVariables);
      solverInput_Aeq.zero();
      CommonOps.insert(xAxisAeqMatrix, solverInput_Aeq, 0, 0);
      CommonOps.insert(yAxisAeqMatrix, solverInput_Aeq, xAxisAeqMatrix.getNumRows(), numberOfXAxisDecisionVariables);

      DenseMatrix64F xAxisbeqMatrix = linearHelper.getConstraintbeqMatrix(xAxis);
      DenseMatrix64F yAxisbeqMatrix = linearHelper.getConstraintbeqMatrix(yAxis);
      solverInput_beq.reshape(numberOfEqualityConstraints, 1);
      CommonOps.insert(xAxisbeqMatrix, solverInput_beq, 0, 0);
      CommonOps.insert(yAxisbeqMatrix, solverInput_beq, xAxisAeqMatrix.getNumRows(), 0);

      DenseMatrix64F xAxisAin = linearHelper.getConstraintAinMatrix(xAxis);
      DenseMatrix64F xAxisbin = linearHelper.getConstraintbinMatrix(xAxis);
      DenseMatrix64F yAxisAin = linearHelper.getConstraintAinMatrix(yAxis);
      DenseMatrix64F yAxisbin = linearHelper.getConstraintbinMatrix(yAxis);
      int numberOfXAxisInequalities = xAxisAin.getNumRows();
      int numberOfYAxisInequalities = yAxisAin.getNumRows();
      int numberOfInequalityConstraints = numberOfXAxisInequalities + numberOfYAxisInequalities;
      solverInput_Ain.reshape(numberOfInequalityConstraints, numberOfDecisionVariables);
      solverInput_Ain.zero();
      solverInput_bin.reshape(numberOfInequalityConstraints, 1);
      CommonOps.insert(xAxisAin, solverInput_Ain, 0, 0);
      CommonOps.insert(yAxisAin, solverInput_Ain, numberOfXAxisInequalities, numberOfXAxisDecisionVariables);
      CommonOps.insert(xAxisbin, solverInput_bin, 0, 0);
      CommonOps.insert(yAxisbin, solverInput_bin, numberOfXAxisInequalities, 0);

      solverInput_lb.reshape(numberOfDecisionVariables, 1);
      CommonOps.insert(linearHelper.getDecisionVariableLowerBoundMatrix(xAxis), solverInput_lb, 0, 0);
      CommonOps.insert(linearHelper.getDecisionVariableLowerBoundMatrix(yAxis), solverInput_lb, numberOfXAxisDecisionVariables, 0);
      solverInput_ub.reshape(numberOfDecisionVariables, 1);
      CommonOps.insert(linearHelper.getDecisionVariableUpperBoundMatrix(xAxis), solverInput_ub, 0, 0);
      CommonOps.insert(linearHelper.getDecisionVariableUpperBoundMatrix(yAxis), solverInput_ub, numberOfXAxisDecisionVariables, 0);
      //setFrictionConeConstraints();
      //setTorqueConstraints();
   }
}