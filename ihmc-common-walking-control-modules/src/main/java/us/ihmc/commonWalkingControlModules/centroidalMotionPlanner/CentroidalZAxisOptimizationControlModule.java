package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.Axis;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.Trajectory;

/**
 * Optimized linear motion along the Z axis to obtain a feasible CoM height trajectory
 * This basically involves trading off between force, velocity and position objectives to obtain a desired motion
 * @author Apoorv S
 *
 */
public class CentroidalZAxisOptimizationControlModule
{
   private static final Axis axis = Axis.Z;

   // Planner parameters
   private final double mass;

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

   // Variables to store results for runtime
   public final List<Trajectory> heightTrajectory;
   public final List<Trajectory> linearVelocityProfile;
   public final List<Trajectory> forceProfile;

   private final JavaQuadProgSolver qpSolver;
   private final DenseMatrix64F qpSolution;
   private final DenseMatrix64F nodeForceValues;

   private boolean noConvergenceException;

   public CentroidalZAxisOptimizationControlModule(double robotMass, OptimizationControlModuleHelper helper, CentroidalMotionPlannerParameters parameters)
   {
      this.mass = robotMass;
      this.helper = helper;

      GenericTypeBuilder<Trajectory> positionTrajectoryBuilder = CentroidalMotionPlannerTools.getTrajectoryBuilder(OptimizationControlModuleHelper.positionCoefficients);
      GenericTypeBuilder<Trajectory> velocityTrajectoryBuilder = CentroidalMotionPlannerTools.getTrajectoryBuilder(OptimizationControlModuleHelper.velocityCoefficients);
      GenericTypeBuilder<Trajectory> forceTrajectoryBuilder = CentroidalMotionPlannerTools.getTrajectoryBuilder(OptimizationControlModuleHelper.forceCoefficients);
      
      // Initialize the variables to store the optimization results
      heightTrajectory = new RecyclingArrayList<>(positionTrajectoryBuilder);
      linearVelocityProfile = new RecyclingArrayList<>(velocityTrajectoryBuilder);
      forceProfile = new RecyclingArrayList<>(forceTrajectoryBuilder);

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
      qpSolver.setConvergenceThreshold(parameters.getOptimizationConvergenceThreshold());
      qpSolution = new DenseMatrix64F(0, 1);
      nodeForceValues = new DenseMatrix64F(0, 1);
      reset();
   }

   public void reset()
   {
      qpSolver.clear();
      resetTrajectories();
   }

   private void resetTrajectories()
   {
      heightTrajectory.clear();;
      linearVelocityProfile.clear();;
      forceProfile.clear();
   }

   public void compute()
   {
      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_bin);
      try
      {
         qpSolver.solve(qpSolution);
      }
      catch (RuntimeException exception)
      {
         PrintTools.debug("Got runtime exception from QP solver");
      }
      helper.setDecisionVariableValues(axis, qpSolution);
      helper.processDecisionVariables(axis);
   }

   public List<Trajectory> getForceProfile()
   {
      return forceProfile;
   }

   public List<Trajectory> getHeightTrajectory()
   {
      return heightTrajectory;
   }

   public List<Trajectory> getLinearVelocityProfile()
   {
      return linearVelocityProfile;
   }

}