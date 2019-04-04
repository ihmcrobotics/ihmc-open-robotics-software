package us.ihmc.manipulation.planning.gradientDescent;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.trajectories.SimpleHermiteCurvedBasedOrientationTrajectoryCalculator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SO3TrajectoryPointCalculator
{
   private static final boolean debug = true;
   private static final int maxIterations = 100;
   private static final double convergenceThreshold = 10E-6;
   private static final double velocitOptimizerDT = 0.001;
   private static final double optimizerStepSize = -0.1;
   private static final double optimizerPerturbationSize = 0.001;

   // TODO : wrap up like `OneDofTrajectoryPointCalculator.
   private final FrameSO3TrajectoryPointList trajectoryPoints = new FrameSO3TrajectoryPointList();

   private final TDoubleArrayList times = new TDoubleArrayList();
   private final List<QuaternionBasics> orientations = new ArrayList<>();
   private final List<Vector3DBasics> angularVelocities = new ArrayList<>();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final int maximumNumberOfWayPoint = 300;

   private final MultipleWaypointsOrientationTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("trajectoryGenerator",
                                                                                                                                           maximumNumberOfWayPoint,
                                                                                                                                           worldFrame,
                                                                                                                                           registry);

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
   }

   public void computeFast()
   {
      if (times.size() != orientations.size())
         throw new RuntimeException("size are not matched. (times) " + times.size() + ", (orientations) " + orientations.size());

      if (MathTools.epsilonEquals(times.get(0), 0.0, 10E-5))
         times.replace(0, 0.0);

      int numberOfPoints = times.size();

      // set input of module
      TDoubleArrayList initialAngularVelocitiesInDoubleArray = new TDoubleArrayList();
      for (int i = 1; i < numberOfPoints - 1; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            initialAngularVelocitiesInDoubleArray.add(angularVelocities.get(i).getElement(j));
         }
      }

      SingleQueryFunction function = new FastSO3TrajectoryPointOptimizerCostFunction();
      System.out.println("initial query " + function.getQuery(initialAngularVelocitiesInDoubleArray));

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
         System.out.println("iteration is " + numberOfIterationToSolve);
         System.out.println("final query is " + optimizer.getOptimalQuery());
         System.out.println("computation time is " + optimizer.getComputationTime());
         for (int i = 0; i < numberOfPoints; i++)
         {
            System.out.println(i);
            System.out.println(initialAngularVelocities.get(i));
            System.out.println(angularVelocities.get(i));
         }
      }
   }

   public void compute()
   {
      if (times.size() != orientations.size())
         throw new RuntimeException("size are not matched. (times) " + times.size() + ", (orientations) " + orientations.size());

      if (MathTools.epsilonEquals(times.get(0), 0.0, 10E-5))
         times.replace(0, 0.0);

      int numberOfPoints = times.size();

      // set input of module
      TDoubleArrayList initialAngularVelocitiesInDoubleArray = new TDoubleArrayList();
      for (int i = 1; i < numberOfPoints - 1; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            initialAngularVelocitiesInDoubleArray.add(angularVelocities.get(i).getElement(j));
         }
      }

      SingleQueryFunction function = new SO3TrajectoryPointOptimizerCostFunction();
      System.out.println("initial query " + function.getQuery(initialAngularVelocitiesInDoubleArray));

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
         System.out.println("iteration is " + numberOfIterationToSolve);
         System.out.println("final query is " + optimizer.getOptimalQuery());
         System.out.println("computation time is " + optimizer.getComputationTime());
         for (int i = 0; i < numberOfPoints; i++)
         {
            System.out.println(i);
            System.out.println(initialAngularVelocities.get(i));
            System.out.println(angularVelocities.get(i));
         }
      }
   }

   public void calculateInitialQuery()
   {
      if (times.size() != orientations.size())
         throw new RuntimeException("size are not matched. (times) " + times.size() + ", (orientations) " + orientations.size());

      if (MathTools.epsilonEquals(times.get(0), 0.0, 10E-5))
         times.replace(0, 0.0);

      int numberOfPoints = times.size();

      // set input of module
      TDoubleArrayList initialAngularVelocitiesInDoubleArray = new TDoubleArrayList();
      for (int i = 1; i < numberOfPoints - 1; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            initialAngularVelocitiesInDoubleArray.add(angularVelocities.get(i).getElement(j));
         }
      }

      SingleQueryFunction function = new SO3TrajectoryPointOptimizerCostFunction();
      SingleQueryFunction fastFunction = new FastSO3TrajectoryPointOptimizerCostFunction();

      System.out.println("function query      = " + function.getQuery(initialAngularVelocitiesInDoubleArray));
      System.out.println("fast function query = " + fastFunction.getQuery(initialAngularVelocitiesInDoubleArray));
   }

   public FrameSO3TrajectoryPointList getTrajectoryData()
   {
      return trajectoryPoints;
   }

   private void updateTrajectoryGenerator()
   {
      trajectoryGenerator.clear();
      for (int i = 0; i < times.size(); i++)
      {
         trajectoryGenerator.appendWaypoint(times.get(i), orientations.get(i), angularVelocities.get(i));
      }
      trajectoryGenerator.initialize();
   }

   public void computeTrajectory(double time)
   {
      updateTrajectoryGenerator();
      trajectoryGenerator.compute(time);
   }

   public QuaternionBasics getOrientation()
   {
      FrameQuaternion orientation = new FrameQuaternion();
      trajectoryGenerator.getOrientation(orientation);
      return orientation;
   }

   public Vector3DBasics getAngularVelocity()
   {
      FrameVector3D angularVelocity = new FrameVector3D();
      trajectoryGenerator.getAngularVelocity(angularVelocity);
      return angularVelocity;
   }

   public Vector3DBasics getAngularAcceleration()
   {
      FrameVector3D angularAcceleration = new FrameVector3D();
      trajectoryGenerator.getAngularAcceleration(angularAcceleration);
      return angularAcceleration;
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
            for (int j = 0; j < 3; j++)
            {
               angularVelocities.get(i).setElement(j, values.get(optimalSolutionIndex));
               optimalSolutionIndex++;
            }
         }

         updateTrajectoryGenerator();

         int numberOfTicks = (int) (times.get(times.size() - 1) / velocitOptimizerDT);
         double time = 0.0;
         double cost = 0.0;

         for (int i = 0; i < numberOfTicks; i++)
         {
            long start = System.nanoTime();
            trajectoryGenerator.compute(time);
            slowComputing += System.nanoTime() - start;

            FrameVector3D angularAcceleration = new FrameVector3D();
            trajectoryGenerator.getAngularAcceleration(angularAcceleration);
            double squareOfAcceleration = 0;
            for (int j = 0; j < 3; j++)
            {
               squareOfAcceleration += angularAcceleration.getElement(j) * angularAcceleration.getElement(j);
            }
            cost += squareOfAcceleration * velocitOptimizerDT;
            time += velocitOptimizerDT;
         }

         return cost;
      }
   }

   public long slowComputing = 0;
   public long fastComputing = 0;

   private class FastSO3TrajectoryPointOptimizerCostFunction implements SingleQueryFunction
   {
      @Override
      public double getQuery(TDoubleArrayList values)
      {
         int numberOfPoints = times.size();
         int optimalSolutionIndex = 0;
         for (int i = 1; i < numberOfPoints - 1; i++)
         {
            for (int j = 0; j < 3; j++)
            {
               angularVelocities.get(i).setElement(j, values.get(optimalSolutionIndex));
               optimalSolutionIndex++;
            }
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
               //System.out.println(time+" "+segmentIndex+" "+previousSegmentIndex);
               trajectoryCalculator.setInitialConditions(orientations.get(segmentIndex), angularVelocities.get(segmentIndex));
               trajectoryCalculator.setFinalConditions(orientations.get(segmentIndex + 1), angularVelocities.get(segmentIndex + 1));
               trajectoryCalculator.setTrajectoryTime(times.get(segmentIndex + 1) - times.get(segmentIndex));
               trajectoryCalculator.setNumberOfRevolutions(0);
               trajectoryCalculator.initialize();
            }

            long start = System.nanoTime();
            double localTime = time - times.get(segmentIndex);
            trajectoryCalculator.compute(localTime);
            fastComputing += System.nanoTime() - start;

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
