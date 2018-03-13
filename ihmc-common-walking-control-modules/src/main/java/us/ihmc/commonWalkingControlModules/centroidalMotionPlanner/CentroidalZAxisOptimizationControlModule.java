package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.robotics.math.trajectories.Trajectory;

/**
 * Optimized linear motion along the Z axis to obtain a feasible CoM height trajectory
 * This basically involves trading off between force, velocity and position objectives to obtain a desired motion
 * @author Apoorv S
 *
 */
public class CentroidalZAxisOptimizationControlModule
{
   // Planner constants
   private static final int forceCoefficients = 4;
   private static final int velocityCoefficients = forceCoefficients + 1;
   private static final int positionCoefficients = velocityCoefficients + 1;

   // Planner parameters
   private final double mass;
   private final double Izz;
   private final double deltaTMin;

   // Runtime variables for optimization
   private int numberOfNodes;

   // Variables to store results for runtime
   public Trajectory heightTrajectory;
   public Trajectory linearVelocityProfile;
   public Trajectory forceProfile;

   public CentroidalZAxisOptimizationControlModule(double robotMass, double Izz, double deltaTMin)
   {
      this.mass = robotMass;
      this.Izz = Izz;
      this.deltaTMin = deltaTMin;

      intializeTrajectoriesForStoringOptimizationResults();
      reset();
   }

   public void reset()
   {
      resetTrajectories();
   }

   private void resetTrajectories()
   {
      heightTrajectory.reset();
      linearVelocityProfile.reset();
      forceProfile.reset();
   }

   private void intializeTrajectoriesForStoringOptimizationResults()
   {
      heightTrajectory = new Trajectory(positionCoefficients);
      linearVelocityProfile = new Trajectory(velocityCoefficients);
      forceProfile = new Trajectory(forceCoefficients);
   }

   public Trajectory getForceProfile()
   {
      return forceProfile;
   }

   public Trajectory getHeightTrajectory()
   {
      return heightTrajectory;
   }

   public Trajectory getLinearVelocityProfile()
   {
      return linearVelocityProfile;
   }
}