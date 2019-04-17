package us.ihmc.manipulation.planning.gradientDescent;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class WayPointVelocityOptimizerTest
{
   private final TDoubleArrayList positions = new TDoubleArrayList();
   private final TDoubleArrayList times = new TDoubleArrayList();
   private static final double velocitOptimizerDT = 0.001;

   private enum OptimizationType
   {
      Velocity, Acceleration, Jerk, KineticEnergy, AcccelerationSquare
   }

   public WayPointVelocityOptimizerTest()
   {
      defineTrajectoryPoints();
      TDoubleArrayList velocities = calculateInitialVelocitiesExceptFirstAndFinal(times, positions);

      TDoubleArrayList initial = new TDoubleArrayList();
      initial.addAll(velocities);

      for (int i = 0; i < initial.size(); i++)
         System.out.println("# initial input is " + initial.get(i));

      TDoubleArrayList initialVelocities = new TDoubleArrayList();
      initialVelocities.add(0.0);
      initialVelocities.addAll(initial);
      initialVelocities.add(0.0);
      List<Trajectory> originalTrajectories = calculateTrajectories(times, positions, initialVelocities);
      saveJointPositionAndVelocity("initialTrajectory", originalTrajectories, times, velocitOptimizerDT);

      runOptimizer(OptimizationType.Velocity, initial);
      runOptimizer(OptimizationType.Acceleration, initial);
      runOptimizer(OptimizationType.Jerk, initial);
      runOptimizer(OptimizationType.KineticEnergy, initial);
      runOptimizer(OptimizationType.AcccelerationSquare, initial);

      runTrajectoryPointOptimizer();
   }

   private void runTrajectoryPointOptimizer()
   {
      TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(1);

      ArrayList<TDoubleArrayList> wayPointSets = new ArrayList<>();
      for (int i = 1; i < positions.size() - 1; i++)
      {
         TDoubleArrayList waypoint = new TDoubleArrayList();
         waypoint.add(positions.get(i));
         wayPointSets.add(waypoint);
      }

      TDoubleArrayList times = new TDoubleArrayList();
      double totalTime = this.times.get(this.times.size() - 1);
      for (int i = 1; i < this.times.size() - 1; i++)
      {
         times.add(this.times.get(i) / totalTime);
      }

      TDoubleArrayList startPosition = new TDoubleArrayList();
      startPosition.add(positions.get(0));
      TDoubleArrayList startVelocity = new TDoubleArrayList();
      startVelocity.add(0.0);
      TDoubleArrayList targetPosition = new TDoubleArrayList();
      targetPosition.add(positions.get(positions.size() - 1));
      TDoubleArrayList targetVelocity = new TDoubleArrayList();
      targetVelocity.add(0.0);

      trajectoryPointOptimizer.setEndPoints(startPosition, startVelocity, targetPosition, targetVelocity);
      trajectoryPointOptimizer.setWaypoints(wayPointSets);

      boolean optimizeTimes = false;
      if (optimizeTimes)
      {
         trajectoryPointOptimizer.compute(2000);
      }
      else
      {
         trajectoryPointOptimizer.computeForFixedTime(times);
      }

      TDoubleArrayList velocities = new TDoubleArrayList();
      TDoubleArrayList velocityToPack = new TDoubleArrayList();
      velocities.add(0.0);
      for (int i = 1; i < positions.size() - 1; i++)
      {
         trajectoryPointOptimizer.getWaypointVelocity(velocityToPack, i - 1);
         velocities.add(velocityToPack.get(0) / totalTime);
      }
      velocities.add(0.0);

      List<Trajectory> optimalTrajectories;
      if (optimizeTimes)
      {
         TDoubleArrayList optimizedTimes = new TDoubleArrayList();
         optimizedTimes.add(0.0);
         for (int i = 1; i < positions.size() - 1; i++)
         {
            optimizedTimes.add(trajectoryPointOptimizer.getWaypointTime(i - 1));
         }
         optimizedTimes.add(1.0);
         for (int i = 0; i < optimizedTimes.size(); i++)
            optimizedTimes.set(i, optimizedTimes.get(i) * totalTime);

         optimalTrajectories = calculateTrajectories(optimizedTimes, positions, velocities);
         saveJointPositionAndVelocity("trajectoryPointOptimizer_timeAdjusted", optimalTrajectories, optimizedTimes, velocitOptimizerDT);
      }
      else
      {
         optimalTrajectories = calculateTrajectories(this.times, positions, velocities);
         saveJointPositionAndVelocity("trajectoryPointOptimizer_timeFixed", optimalTrajectories, this.times, velocitOptimizerDT);
      }
   }

   private void runOptimizer(OptimizationType optimizationType, TDoubleArrayList initial)
   {
      SingleQueryFunction function = new EvaluationFunction(optimizationType);
      GradientDescentModule optimizer = new GradientDescentModule(function, initial);
      optimizer.setMaximumIterations(100);

      LogTools.info("initial " + optimizationType.toString() + function.getQuery(initial));

      System.out.println("iteration is " + optimizer.run());
      TDoubleArrayList optimalSolution = optimizer.getOptimalInput();
      for (int i = 0; i < optimalSolution.size(); i++)
         System.out.println("solution is " + optimalSolution.get(i));

      TDoubleArrayList finalVelocities = new TDoubleArrayList();
      finalVelocities.add(0.0);
      finalVelocities.addAll(optimalSolution);
      finalVelocities.add(0.0);
      List<Trajectory> optimalTrajectories = calculateTrajectories(times, positions, finalVelocities);
      saveJointPositionAndVelocity("optimalTrajectory_" + optimizationType.toString(), optimalTrajectories, times, velocitOptimizerDT);

      System.out.println("optimal " + optimizationType.toString() + optimizer.getOptimalQuery());
   }

   private class EvaluationFunction implements SingleQueryFunction
   {
      private OptimizationType optimizationType;

      private EvaluationFunction(OptimizationType optimizationType)
      {
         this.optimizationType = optimizationType;
      }

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         int numberOfTicks = (int) (times.get(times.size() - 1) / velocitOptimizerDT);
         double time = 0.0;
         double kineticEnergy = 0.0;
         for (int i = 1; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(times, time);
            TDoubleArrayList velocities = new TDoubleArrayList();
            velocities.add(0.0);
            velocities.addAll(values);
            velocities.add(0.0);
            List<Trajectory> trajectories = calculateTrajectories(times, positions, velocities);
            Trajectory trajectory = trajectories.get(indexOfTrajectory);
            trajectory.compute(time);

            double previousTime = time - velocitOptimizerDT;
            int indexOfTrajectoryForPreviousTick = findTrajectoryIndex(times, previousTime);
            Trajectory trajectoryForPreviousTick = trajectories.get(indexOfTrajectoryForPreviousTick);
            trajectoryForPreviousTick.compute(previousTime);

            switch (optimizationType)
            {
            case Velocity:
               kineticEnergy += Math.abs(trajectory.getVelocity()) * velocitOptimizerDT;
               break;
            case Acceleration:
               kineticEnergy += Math.abs(trajectory.getAcceleration()) * velocitOptimizerDT;
               break;
            case Jerk:
               kineticEnergy += Math.abs(trajectory.getAcceleration() - trajectoryForPreviousTick.getAcceleration()) / velocitOptimizerDT;
               break;
            case KineticEnergy:
               kineticEnergy += trajectory.getVelocity() * trajectory.getVelocity() * velocitOptimizerDT;
               break;
            case AcccelerationSquare:
               kineticEnergy += trajectory.getAcceleration() * trajectory.getAcceleration() * velocitOptimizerDT;
               break;
            }
            time += velocitOptimizerDT;
         }
         return kineticEnergy;
      }
   }

   private void defineTrajectoryPoints()
   {
      positions.add(1.0);
      positions.add(2.0);
      positions.add(4.0);
      positions.add(3.0);
      positions.add(1.0);

      times.add(0.0);
      times.add(0.2);
      times.add(0.4);
      times.add(0.95);
      times.add(2.0);
   }

   private static int findTrajectoryIndex(TDoubleArrayList times, double time)
   {
      int indexOfTrajectory = 0;
      for (int j = times.size() - 1; j > 0; j--)
      {
         if (times.get(j) < time)
         {
            indexOfTrajectory = j;
            break;
         }
      }
      return indexOfTrajectory;
   }

   private static List<Trajectory> calculateTrajectories(TDoubleArrayList times, TDoubleArrayList positions, TDoubleArrayList velocities)
   {
      List<Trajectory> trajectories = new ArrayList<Trajectory>();

      for (int j = 0; j < times.size() - 1; j++)
      {
         Trajectory cubic = new Trajectory(4);
         cubic.setCubic(times.get(j), times.get(j + 1), positions.get(j), velocities.get(j), positions.get(j + 1), velocities.get(j + 1));
         trajectories.add(cubic);
      }

      return trajectories;
   }

   private TDoubleArrayList calculateInitialVelocitiesExceptFirstAndFinal(TDoubleArrayList times, TDoubleArrayList positions)
   {
      TDoubleArrayList velocities = new TDoubleArrayList();
      for (int i = 1; i < times.size() - 1; i++)
         velocities.add((positions.get(i + 1) - positions.get(i)) / (times.get(i + 1) - times.get(i)));

      return velocities;
   }

   private static void saveJointPositionAndVelocity(String namePrefix, List<Trajectory> trajectories, TDoubleArrayList times, double timeTick)
   {
      int numberOfTicks = (int) (times.get(times.size() - 1) / timeTick);
      FileWriter positionFW;
      try
      {
         positionFW = new FileWriter(new File(namePrefix + "_Pos_Vel_Acc.csv"));

         positionFW.write(String.format("time\t_position\t_velocity\t_acceleration"));
         positionFW.write(System.lineSeparator());

         double time = 0.0;
         for (int i = 0; i < numberOfTicks; i++)
         {
            int indexOfTrajectory = findTrajectoryIndex(times, time);

            positionFW.write(String.format("%.4f (%d)", time, indexOfTrajectory));
            Trajectory trajectory = trajectories.get(indexOfTrajectory);
            trajectory.compute(time);

            positionFW.write(String.format("\t%.4f\t%.4f\t%.4f", trajectory.getPosition(), trajectory.getVelocity(), trajectory.getAcceleration()));

            positionFW.write(System.lineSeparator());
            time += timeTick;
         }

         positionFW.close();
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
      }

      LogTools.info("done");
   }

   public static void main(String[] args)
   {
      WayPointVelocityOptimizerTest test = new WayPointVelocityOptimizerTest();

      LogTools.info("done");
   }
}