package us.ihmc.manipulation.planning.gradientDescent;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SO3TrajectoryPointCalculator
{
   private static final int maxIterations = 300;
   private static final double convergenceThreshold = 10E-5;

   private static final double velocitOptimizerDT = 0.001;

   // TODO : wrap up like `OneDofTrajectoryPointCalculator.
   private final FrameSO3TrajectoryPointList trajectoryPoints = new FrameSO3TrajectoryPointList();

   private final TDoubleArrayList times = new TDoubleArrayList();
   private final List<QuaternionBasics> orientations = new ArrayList<>();
   private final List<Vector3DBasics> angularVelocities = new ArrayList<>();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final int maximumNumberOfWayPoint = 200;

   private MultipleWaypointsOrientationTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("trajectoryGenerator",
                                                                                                                                     maximumNumberOfWayPoint,
                                                                                                                                     worldFrame, registry);

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
   
   public void overrideInitialAngularVelocity()
   {
      // TODO : do with first order.
   }

   public void compute()
   {
      if (times.size() != orientations.size())
         throw new RuntimeException("size are not matched. (times) " + times.size() + ", (orientations) " + orientations.size());

      if (MathTools.epsilonEquals(times.get(0), 0.0, 10E-5))
         times.replace(0, 0.0);

      int numberOfPoints = times.size();

      // set input of module
      TDoubleArrayList initialAngularVelocities = new TDoubleArrayList();
      for (int i = 1; i < numberOfPoints - 1; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            initialAngularVelocities.add(angularVelocities.get(i).getElement(j));
         }
      }

      SingleQueryFunction function = new SO3TrajectoryPointOptimizerCostFunction();
      GradientDescentModule optimizer = new GradientDescentModule(function, initialAngularVelocities);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);

      int numberOfIterationToSolve = optimizer.run();
      System.out.println("iteration is " + numberOfIterationToSolve);
      System.out.println("computation time is " + optimizer.getComputationTime());
   }

   public FrameSO3TrajectoryPointList getTrajectoryData()
   {
      return trajectoryPoints;
   }

   public void computeTrajectory(double time)
   {
      trajectoryGenerator.clear();
      for (int i = 0; i < times.size(); i++)
      {
         trajectoryGenerator.appendWaypoint(times.get(i), orientations.get(i), angularVelocities.get(i));
      }
      trajectoryGenerator.initialize();
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

         trajectoryGenerator.clear();
         for (int i = 0; i < times.size(); i++)
         {
            trajectoryGenerator.appendWaypoint(times.get(i), orientations.get(i), angularVelocities.get(i));
         }
         trajectoryGenerator.initialize();

         int numberOfTicks = (int) (times.get(times.size() - 1) / velocitOptimizerDT);
         double time = 0.0;
         double cost = 0.0;

         for (int i = 1; i < numberOfTicks; i++)
         {
            trajectoryGenerator.compute(time);

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
}
