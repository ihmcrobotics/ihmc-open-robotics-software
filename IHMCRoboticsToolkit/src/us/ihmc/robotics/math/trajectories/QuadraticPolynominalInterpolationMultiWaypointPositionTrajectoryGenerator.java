package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * This multiwaypoint trajectory generator will generate velocities at intermediate way points of a trajectory which will result in a smooth path trough those way points.
 * It is based on the paper "Curvature-continuous 3D Path-planning Using QPMI Method" which can be found at http://cdn.intechopen.com/pdfs-wm/48591.pdf
 * It will assume the first and the last way point of a path have a velocity of 0.
 *
 * @author Olger
 */
public class QuadraticPolynominalInterpolationMultiWaypointPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;

   private final ArrayList<double[]> parametersX;
   private final ArrayList<double[]> parametersY;
   private final ArrayList<double[]> parametersZ;

   private final ArrayList<Vector3d> waypoints;
   private final ArrayList<Double> time;

   private final Matrix3d timeMatrix;
   private int numberOfWayPoints;

   public QuadraticPolynominalInterpolationMultiWaypointPositionTrajectoryGenerator(String name, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry registry)
   {
      super(allowMultipleFrames, referenceFrame);
      this.registry = registry;

      String currentPositionName = name + "CurrentPosition";
      String currentVelocityName = name + "CurrentVelocity";
      String currentAccelerationName = name + "CurrentAcceleration";

      currentPosition = new YoFramePoint(currentPositionName, referenceFrame, registry);
      currentVelocity = new YoFrameVector(currentVelocityName, referenceFrame, registry);
      currentAcceleration = new YoFrameVector(currentAccelerationName, referenceFrame, registry);

      currentTime = new DoubleYoVariable(name + "CurrentTime", registry);
      trajectoryTime = new DoubleYoVariable(name + "TrajectoryTime", registry);

      numberOfWayPoints = 0;

      parametersX = new ArrayList<>();
      parametersY = new ArrayList<>();
      parametersZ = new ArrayList<>();
      timeMatrix = new Matrix3d();
      this.time = new ArrayList<>();
      this.waypoints = new ArrayList<>();

      trajectoryTime.set(0);
   }

   public void initialize()
   {
      ArrayList<Double> subTime = new ArrayList<>(time.subList(0, 3));
      ArrayList<Vector3d> subWayPoints = new ArrayList<>(waypoints.subList(0, 3));
      getParameters(subTime, subWayPoints, parametersX.get(0), parametersY.get(0), parametersZ.get(0));

      for (int i = 1; i < numberOfWayPoints - 1; i++)
      {
         subTime = new ArrayList<>(time.subList(i - 1, i + 2));
         subWayPoints = new ArrayList<>(waypoints.subList(i - 1, i + 2));
         getParameters(subTime, subWayPoints, parametersX.get(i), parametersY.get(i), parametersZ.get(i));
      }

      subTime = new ArrayList<>(time.subList(numberOfWayPoints - 3, numberOfWayPoints));
      subWayPoints = new ArrayList<>(waypoints.subList(numberOfWayPoints - 3, numberOfWayPoints));
      getParameters(subTime, subWayPoints, parametersX.get(numberOfWayPoints - 1), parametersY.get(numberOfWayPoints - 1),
            parametersZ.get(numberOfWayPoints - 1));
   }

   public void appendWaypoint(double timeAtWaypoint, Vector3d waypointPosition)
   {
      parametersX.add(new double[3]);
      parametersY.add(new double[3]);
      parametersZ.add(new double[3]);

      waypoints.add(waypointPosition);
      time.add(timeAtWaypoint);

      numberOfWayPoints++;

      trajectoryTime.set(timeAtWaypoint);
   }

   private void getParameters(ArrayList<Double> time, ArrayList<Vector3d> waypoints, double[] parametersXToPack, double[] parametersYToPack,
         double[] parametersZToPack)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            timeMatrix.setElement(i, j, MathTools.powWithInteger(time.get(i), 2 - j));
         }
      }

      System.out.println(timeMatrix);
      timeMatrix.invert();
      Vector3d tempVector = new Vector3d();

      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVector);
         parametersXToPack[i] = (tempVector.dot(new Vector3d(waypoints.get(0).getX(), waypoints.get(1).getX(), waypoints.get(2).getX())));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVector);
         parametersYToPack[i] = (tempVector.dot(new Vector3d(waypoints.get(0).getY(), waypoints.get(1).getY(), waypoints.get(2).getY())));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVector);
         parametersZToPack[i] = (tempVector.dot(new Vector3d(waypoints.get(0).getZ(), waypoints.get(1).getZ(), waypoints.get(2).getZ())));
      }
   }

   public void compute(double currentTime)
   {
      this.currentTime.set(currentTime);
      int waypointIndex = 0;
      for (double currentWaypoint : time)
      {
         if (currentTime > currentWaypoint)
         {
            waypointIndex = time.indexOf(currentWaypoint);
         }
      }

      double muNext = (currentTime - time.get(waypointIndex)) / (time.get(waypointIndex + 1) - time.get(waypointIndex));
      double muPrevious = (time.get(waypointIndex + 1) - currentTime) / (time.get(waypointIndex + 1) - time.get(waypointIndex));

      double xPreviousWayPoint =
            parametersX.get(waypointIndex)[0] * MathTools.square(currentTime) + parametersX.get(waypointIndex)[1] * currentTime + parametersX
                  .get(waypointIndex)[2];
      double xNextWayPoint =
            parametersX.get(waypointIndex + 1)[0] * MathTools.square(currentTime) + parametersX.get(waypointIndex + 1)[1] * currentTime + parametersX
                  .get(waypointIndex + 1)[2];
      double xCurrentTime = muNext * xNextWayPoint + muPrevious * xPreviousWayPoint;

      double yPreviousWayPoint =
            parametersY.get(waypointIndex)[0] * MathTools.square(currentTime) + parametersY.get(waypointIndex)[1] * currentTime + parametersY
                  .get(waypointIndex)[2];
      double yNextWayPoint =
            parametersY.get(waypointIndex + 1)[0] * MathTools.square(currentTime) + parametersY.get(waypointIndex + 1)[1] * currentTime + parametersY
                  .get(waypointIndex + 1)[2];
      double yCurrentTime = muNext * yNextWayPoint + muPrevious * yPreviousWayPoint;

      double zPreviousWayPoint =
            parametersZ.get(waypointIndex)[0] * MathTools.square(currentTime) + parametersZ.get(waypointIndex)[1] * currentTime + parametersZ
                  .get(waypointIndex)[2];
      double zNextWayPoint =
            parametersZ.get(waypointIndex + 1)[0] * MathTools.square(currentTime) + parametersZ.get(waypointIndex + 1)[1] * currentTime + parametersZ
                  .get(waypointIndex + 1)[2];
      double zCurrentTime = muNext * zNextWayPoint + muPrevious * zPreviousWayPoint;

      currentPosition.set(xCurrentTime, yCurrentTime, zCurrentTime);

      double xDotPreviousWayPoint = 2 * parametersX.get(waypointIndex)[0] * currentTime + parametersX.get(waypointIndex)[1];
      double xDotNextWayPoint = 2 * parametersX.get(waypointIndex + 1)[0] * currentTime + parametersX.get(waypointIndex + 1)[1];
      double xDotCurrentTime = muNext * xDotNextWayPoint + muPrevious * xDotPreviousWayPoint;

      double yDotPreviousWayPoint = 2 * parametersY.get(waypointIndex)[0] * currentTime + parametersY.get(waypointIndex)[1];
      double yDotNextWayPoint = 2 * parametersY.get(waypointIndex + 1)[0] * currentTime + parametersY.get(waypointIndex + 1)[1];
      double yDotCurrentTime = muNext * yDotNextWayPoint + muPrevious * yDotPreviousWayPoint;

      double zDotPreviousWayPoint = 2 * parametersZ.get(waypointIndex)[0] * currentTime + parametersZ.get(waypointIndex)[1];
      double zDotNextWayPoint = 2 * parametersZ.get(waypointIndex + 1)[0] * currentTime + parametersZ.get(waypointIndex + 1)[1];
      double zDotCurrentTime = muNext * zDotNextWayPoint + muPrevious * zDotPreviousWayPoint;

      currentVelocity.set(xDotCurrentTime, yDotCurrentTime, zDotCurrentTime);

      double xDDotPreviousWayPoint = 2 * parametersX.get(waypointIndex)[0];
      double xDDotNextWayPoint = 2 * parametersX.get(waypointIndex + 1)[0];
      double xDDotCurrentTime = muNext * xDDotNextWayPoint + muPrevious * xDDotPreviousWayPoint;

      double yDDotPreviousWayPoint = 2 * parametersY.get(waypointIndex)[0];
      double yDDotNextWayPoint = 2 * parametersY.get(waypointIndex + 1)[0];
      double yDDotCurrentTime = muNext * yDDotNextWayPoint + muPrevious * yDDotPreviousWayPoint;

      double zDDotPreviousWayPoint = 2 * parametersZ.get(waypointIndex)[0];
      double zDDotNextWayPoint = 2 * parametersZ.get(waypointIndex + 1)[0];
      double zDDotCurrentTime = muNext * zDDotNextWayPoint + muPrevious * zDDotPreviousWayPoint;

      currentAcceleration.set(xDDotCurrentTime, yDDotCurrentTime, zDDotCurrentTime);
   }

   public void printResultsForTest()
   {
      System.out.println(timeMatrix);
      System.out.println(time.get(0) + " : " + time.get(1) + " : " + time.get(2));
      System.out.println(parametersX.get(1)[0] + " : " + parametersX.get(1)[1] + " : " + parametersX.get(1)[2]);
      System.out.println(parametersY.get(1)[0] + " : " + parametersY.get(1)[1] + " : " + parametersY.get(1)[2]);
      System.out.println(parametersZ.get(1)[0] + " : " + parametersZ.get(1)[1] + " : " + parametersZ.get(1)[2]);

      System.out.println(parametersX.get(1)[0] * MathTools.square(time.get(0)) + parametersX.get(1)[1] * time.get(0) + parametersX.get(1)[2]);
      System.out.println(parametersX.get(1)[0] * MathTools.square(time.get(1)) + parametersX.get(1)[1] * time.get(1) + parametersX.get(1)[2]);
      System.out.println(parametersX.get(1)[0] * MathTools.square(time.get(2)) + parametersX.get(1)[1] * time.get(2) + parametersX.get(1)[2]);

      System.out.println(2 * parametersX.get(1)[0] * time.get(0) + parametersX.get(1)[1]);
      System.out.println(2 * parametersX.get(1)[0] * time.get(1) + parametersX.get(1)[1]);
      System.out.println(2 * parametersX.get(1)[0] * time.get(2) + parametersX.get(1)[1]);

      System.out.println("final check");
   }

   @Override public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public void setToDone()
   {
      currentTime.set(trajectoryTime.getDoubleValue() + 0.01);
   }

   @Override public void get(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      positionToPack.set(currentPosition);
   }

   public void get(Point3d positionToPack)
   {
      currentPosition.get(positionToPack);
   }

   @Override public void packVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void packVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(currentVelocity);
   }

   public void packVelocity(Vector3d velocityToPack)
   {
      currentVelocity.get(velocityToPack);
   }

   @Override public void packAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void packAcceleration(Vector3d accelerationToPack)
   {
      currentAcceleration.get(accelerationToPack);
   }

   @Override public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }

   public void packLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      positionToPack.set(currentPosition);
      velocityToPack.set(currentVelocity);
      accelerationToPack.set(currentAcceleration);
   }

   public void getFinalPosition(FramePoint finalPosition)
   {
      finalPosition.set(this.waypoints.get(numberOfWayPoints));
   }

   @Override public void showVisualization()
   {
   }

   @Override public void hideVisualization()
   {
   }
}
