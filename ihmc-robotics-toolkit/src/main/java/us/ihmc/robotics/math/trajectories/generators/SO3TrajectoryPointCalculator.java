package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.trajectories.SimpleHermiteCurvedBasedOrientationTrajectoryCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class SO3TrajectoryPointCalculator
{
   private static final boolean debug = false;
   private static final int maxIterations = 100;
   private static final double convergenceThreshold = 10E-5;
   private static final double velocitOptimizerDT = 0.01;
   private static final double optimizerStepSize = -0.1;
   private static final double optimizerPerturbationSize = 0.0001;

   private final FrameSO3TrajectoryPointList trajectoryPoints = new FrameSO3TrajectoryPointList();

   private final TDoubleArrayList times = new TDoubleArrayList();
   private final List<QuaternionBasics> orientations = new ArrayList<>();
   private final List<Vector3DBasics> angularVelocities = new ArrayList<>();

   private final SimpleHermiteCurvedBasedOrientationTrajectoryCalculator trajectoryCalculator = new SimpleHermiteCurvedBasedOrientationTrajectoryCalculator();

   public void clear()
   {
      times.clear();
      orientations.clear();
      angularVelocities.clear();
   }

   public void appendTrajectoryPoint(double time, QuaternionBasics orientation)
   {
      times.add(time);
      orientations.add(new Quaternion(orientation));
      angularVelocities.add(new Vector3D());
   }

   public void appendTrajectoryPoint(double time, QuaternionBasics orientation, Vector3DBasics angularVelocity)
   {
      times.add(time);
      orientations.add(new Quaternion(orientation));
      angularVelocities.add(new Vector3D(angularVelocity));
   }

   public void useFirstOrderInitialGuess()
   {
      for (int i = 1; i < times.size() - 1; i++)
      {
         // case 1: first order
         double timeDiff = times.get(i + 1) - times.get(i);
         Quaternion orientationDiff = new Quaternion();
         orientationDiff.set(orientations.get(i));
         orientationDiff.conjugate();
         orientationDiff.multiply(orientations.get(i + 1));

         AxisAngle orientationDiffAxisAngle = new AxisAngle(orientationDiff);
         AxisAngleConversion.convertQuaternionToAxisAngle(orientationDiff, orientationDiffAxisAngle);
         orientationDiffAxisAngle.setAngle(orientationDiffAxisAngle.getAngle() / timeDiff);

         Vector3D angularVelocity = new Vector3D();
         angularVelocity.setX(orientationDiffAxisAngle.getX() * orientationDiffAxisAngle.getAngle());
         angularVelocity.setY(orientationDiffAxisAngle.getY() * orientationDiffAxisAngle.getAngle());
         angularVelocity.setZ(orientationDiffAxisAngle.getZ() * orientationDiffAxisAngle.getAngle());
         angularVelocities.get(i).set(angularVelocity);
      }
   }

   public void useSecondOrderInitialGuess()
   {
      TDoubleArrayList defaultInitialAngularVelocitiesInDoubleArray = new TDoubleArrayList();
      for (int i = 1; i < times.size() - 1; i++)
         for (int j = 0; j < 3; j++)
            defaultInitialAngularVelocitiesInDoubleArray.add(angularVelocities.get(i).getElement(j));

      SingleQueryFunction function = new SO3TrajectoryPointOptimizerCostFunction();
      double defaultQuery = function.getQuery(defaultInitialAngularVelocitiesInDoubleArray);

      for (int i = 1; i < times.size() - 1; i++)
      {
         // case 2: second order
         double timeDiff = times.get(i + 1) - times.get(i - 1);
         Quaternion orientationDiff = new Quaternion();
         orientationDiff.set(orientations.get(i - 1));
         orientationDiff.conjugate();
         orientationDiff.multiply(orientations.get(i + 1));

         AxisAngle orientationDiffAxisAngle = new AxisAngle(orientationDiff);
         AxisAngleConversion.convertQuaternionToAxisAngle(orientationDiff, orientationDiffAxisAngle);
         orientationDiffAxisAngle.setAngle(orientationDiffAxisAngle.getAngle() / timeDiff);

         Vector3D angularVelocity = new Vector3D();
         angularVelocity.setX(orientationDiffAxisAngle.getX() * orientationDiffAxisAngle.getAngle());
         angularVelocity.setY(orientationDiffAxisAngle.getY() * orientationDiffAxisAngle.getAngle());
         angularVelocity.setZ(orientationDiffAxisAngle.getZ() * orientationDiffAxisAngle.getAngle());
         angularVelocities.get(i).set(angularVelocity);
      }

      TDoubleArrayList initialAngularVelocitiesInDoubleArray = new TDoubleArrayList();
      for (int i = 1; i < times.size() - 1; i++)
         for (int j = 0; j < 3; j++)
            initialAngularVelocitiesInDoubleArray.add(angularVelocities.get(i).getElement(j));

      double modifiedQuery = function.getQuery(initialAngularVelocitiesInDoubleArray);

      if (debug)
      {
         System.out.println("defaultQuery = " + defaultQuery);
         System.out.println("modifiedQuery = " + modifiedQuery);
      }

      /**
       * If the second order initial guess is worse than original guess, use original guess.
       */
      if (modifiedQuery > defaultQuery)
      {
         int arrayIndex = 0;
         for (int i = 1; i < times.size() - 1; i++)
         {
            for (int j = 0; j < 3; j++)
            {
               angularVelocities.get(i).setElement(j, defaultInitialAngularVelocitiesInDoubleArray.get(arrayIndex));
               arrayIndex++;
            }
         }
      }
   }

   /**
    * This computation is using @link SimpleHermiteCurvedBasedOrientationTrajectoryGenerator to get query.
    */
   public void compute()
   {
      if (times.size() != orientations.size())
         throw new RuntimeException("size are not matched. (times) " + times.size() + ", (orientations) " + orientations.size());

      if (MathTools.epsilonEquals(times.get(0), 0.0, 10E-5))
         times.replace(0, 0.0);

      int numberOfPoints = times.size();

      TDoubleArrayList initialAngularVelocitiesInDoubleArray = new TDoubleArrayList();
      for (int i = 1; i < numberOfPoints - 1; i++)
         for (int j = 0; j < 3; j++)
            initialAngularVelocitiesInDoubleArray.add(angularVelocities.get(i).getElement(j));

      SingleQueryFunction function = new SO3TrajectoryPointOptimizerCostFunction();
      GradientDescentModule optimizer = new GradientDescentModule(function, initialAngularVelocitiesInDoubleArray);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);

      List<Vector3DBasics> initialAngularVelocities = new ArrayList<>();
      for (int i = 0; i < numberOfPoints; i++)
         initialAngularVelocities.add(new Vector3D(angularVelocities.get(i)));

      int numberOfIterationToSolve = optimizer.run();

      if (debug)
      {
         System.out.println("# initial query " + function.getQuery(initialAngularVelocitiesInDoubleArray));
         System.out.println("# iteration is " + numberOfIterationToSolve);
         System.out.println("# final query is " + optimizer.getOptimalQuery());
         System.out.println("# computation time is " + optimizer.getComputationTime());
      }
      updateTrajectoryPoints();
   }

   private void updateTrajectoryPoints()
   {
      trajectoryPoints.clear();
      for (int i = 0; i < times.size(); i++)
         trajectoryPoints.addTrajectoryPoint(times.get(i), orientations.get(i), angularVelocities.get(i));
   }

   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.getNumberOfTrajectoryPoints();
   }

   public FrameSO3TrajectoryPoint getTrajectoryPoint(int i)
   {
      return trajectoryPoints.getTrajectoryPoint(i);
   }

   private class SO3TrajectoryPointOptimizerCostFunction implements SingleQueryFunction
   {
      @Override
      public double getQuery(TDoubleArrayList values)
      {
         int numberOfPoints = times.size();
         int optimalSolutionIndex = 0;
         for (int i = 1; i < numberOfPoints - 1; i++)
         {
            angularVelocities.get(i).setX(values.get(optimalSolutionIndex));
            optimalSolutionIndex++;
            angularVelocities.get(i).setY(values.get(optimalSolutionIndex));
            optimalSolutionIndex++;
            angularVelocities.get(i).setZ(values.get(optimalSolutionIndex));
            optimalSolutionIndex++;
         }

         int numberOfTicks = (int) (times.get(times.size() - 1) / velocitOptimizerDT);
         double time = 0.0;
         double cost = 0.0;
         int previousSegmentIndex = -1;

         for (int i = 0; i < numberOfTicks; i++)
         {
            int segmentIndex = getSegmentIndex(time);

            if (previousSegmentIndex != segmentIndex)
            {
               trajectoryCalculator.setInitialConditions(orientations.get(segmentIndex), angularVelocities.get(segmentIndex));
               trajectoryCalculator.setFinalConditions(orientations.get(segmentIndex + 1), angularVelocities.get(segmentIndex + 1));
               trajectoryCalculator.setTrajectoryTime(times.get(segmentIndex + 1) - times.get(segmentIndex));
               trajectoryCalculator.setNumberOfRevolutions(0);
               trajectoryCalculator.initialize();
            }

            double localTime = time - times.get(segmentIndex);
            trajectoryCalculator.compute(localTime);

            Vector3D angularAcceleration = new Vector3D();
            trajectoryCalculator.getAngularAcceleration(angularAcceleration);
            double squareOfAcceleration = 0;
            for (int j = 0; j < 3; j++)
            {
               squareOfAcceleration += angularAcceleration.getElement(j) * angularAcceleration.getElement(j);
            }
            cost += squareOfAcceleration * velocitOptimizerDT;
            time += velocitOptimizerDT;

            previousSegmentIndex = segmentIndex;
         }

         return cost;
      }
   }

   private int getSegmentIndex(double time)
   {
      int segmentIndex = 0;
      for (int i = 0; i < times.size() - 1; i++)
      {
         if (time < times.get(i + 1))
         {
            segmentIndex = i;
            break;
         }
      }

      return segmentIndex;
   }
}
