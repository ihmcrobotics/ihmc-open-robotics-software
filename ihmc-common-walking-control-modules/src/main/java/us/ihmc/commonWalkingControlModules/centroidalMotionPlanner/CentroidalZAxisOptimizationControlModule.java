package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;

/**
 * Optimized linear motion along the Z axis to obtain a feasible CoM height trajectory
 * This basically involves trading off between force, velocity and position objectives to obtain a desired motion
 * @author Apoorv S
 *
 */
public class CentroidalZAxisOptimizationControlModule
{
   private final Axis axis = Axis.Z;

   // Planner runtime variables
   private final OptimizationControlModuleHelper helper;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;
   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;
   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;

   private final JavaQuadProgSolver qpSolver;
   private final DenseMatrix64F qpSolution;

   public CentroidalZAxisOptimizationControlModule(OptimizationControlModuleHelper helper, CentroidalMotionPlannerParameters parameters)
   {
      this.helper = helper;

      // Initialize the QP matrices
      solverInput_H = helper.getObjectiveHMatrix(this.axis);
      solverInput_f = helper.getObjectivefMatrix(this.axis);
      solverInput_lb = null;
      solverInput_ub = null;
      solverInput_Aeq = helper.getConstraintAeqMatrix(this.axis);
      solverInput_beq = helper.getConstraintbeqMatrix(this.axis);
      solverInput_Ain = null;
      solverInput_bin = null;

      qpSolver = new JavaQuadProgSolver();
      //qpSolver.setConvergenceThreshold(parameters.getOptimizationConvergenceThreshold());
      qpSolution = new DenseMatrix64F(0, 1);
      reset();
   }

   public void reset()
   {
      qpSolver.clear();
   }

   public void compute()
   {
      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);
      qpSolution.reshape(helper.getNumberOfDecisionVariables(axis), 1);
      try
      {
         qpSolver.solve(qpSolution);
      }
      catch (RuntimeException exception)
      {
         PrintTools.debug("Got runtime exception from QP solver");
      }
      helper.setDecisionVariableValues(axis, qpSolution);
   }
}