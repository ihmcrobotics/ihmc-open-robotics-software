package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanWaypoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * This multi waypoint trajectory generator will generate velocities at intermediate way points of a trajectory which will result in a smooth path trough
 * those way points.It is based on the paper "Curvature-continuous 3D Path-planning Using QPMI Method" which can be found at
 * http://cdn.intechopen.com/pdfs-wm/48591.pdf
 *
 * This trajectory generator will assume the initial and final velocity to be 0.The initial and final velocity can be set to a minor
 * velocity however with setInitialVelocity() and setFinalVelocity().But these limit conditions will only influence the path between the first two and last two
 * way points. So if a great initial or final velocity is needed it is recommended to use the other multiple way point generator.
 *
 * This trajectory generator can auto-generate waypoint times but this behavior is turned off by default.
 * To activate it call activateWaypointTimeGeneration(double initialTime, double finalTime)
 *
 * The minimal number of way points is 3.
 *
 * @author Olger
 */
public class PositionPathPlanningTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;
   private final int maximumNumberOfWaypoints;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable initialTime;

   private final YoFrameVector initialVelocity;
   private final YoFrameVector finalVelocity;

   private final RecyclingArrayList<double[]> parametersX;
   private final RecyclingArrayList<double[]> parametersY;
   private final RecyclingArrayList<double[]> parametersZ;

   private final ArrayList<YoFrameEuclideanWaypoint> waypoints;

   private final Matrix3d timeMatrix;
   private int numberOfWayPoints;
   private boolean WaypointTimeGeneration;

   public PositionPathPlanningTrajectoryGenerator(String name, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(name, maximumNumberOfWaypoints, false, referenceFrame, registry);

   }

   public PositionPathPlanningTrajectoryGenerator(String name, int maximumNumberOfWaypoints, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry registry)
   {
      super(allowMultipleFrames, referenceFrame);
      this.registry = registry;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;
      WaypointTimeGeneration = false;

      String currentPositionName = name + "CurrentPosition";
      String currentVelocityName = name + "CurrentVelocity";
      String currentAccelerationName = name + "CurrentAcceleration";
      String initialVelocityName = name + "InitialVelocity";
      String finalVelocityName = name + "FinalVelocity";

      if (allowMultipleFrames)
      {
         YoFrameVectorInMultipleFrames initialVelocity = new YoFrameVectorInMultipleFrames(initialVelocityName, this.registry, referenceFrame);
         YoFrameVectorInMultipleFrames finalVelocity = new YoFrameVectorInMultipleFrames(finalVelocityName, this.registry, referenceFrame);
         YoFramePointInMultipleFrames currentPosition = new YoFramePointInMultipleFrames(currentPositionName, this.registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentVelocity = new YoFrameVectorInMultipleFrames(currentVelocityName, this.registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAcceleration = new YoFrameVectorInMultipleFrames(currentAccelerationName, this.registry, referenceFrame);

         registerMultipleFramesHolders(initialVelocity, finalVelocity, currentPosition, currentVelocity, currentAcceleration);

         this.initialVelocity = initialVelocity;
         this.finalVelocity = finalVelocity;
         this.currentPosition = currentPosition;
         this.currentVelocity = currentVelocity;
         this.currentAcceleration = currentAcceleration;
      }
      else
      {
         initialVelocity = new YoFrameVector(initialVelocityName, referenceFrame, this.registry);
         finalVelocity = new YoFrameVector(finalVelocityName, referenceFrame, this.registry);
         currentPosition = new YoFramePoint(currentPositionName, referenceFrame, this.registry);
         currentVelocity = new YoFrameVector(currentVelocityName, referenceFrame, this.registry);
         currentAcceleration = new YoFrameVector(currentAccelerationName, referenceFrame, this.registry);

      }

      initialVelocity.set(0, 0, 0);
      finalVelocity.set(0, 0, 0);

      currentTime = new DoubleYoVariable(name + "CurrentTime", this.registry);
      trajectoryTime = new DoubleYoVariable(name + "TrajectoryTime", this.registry);
      initialTime = new DoubleYoVariable(name + "InitalTime", this.registry);

      numberOfWayPoints = 0;

      GenericTypeBuilder<double[]> builder = new GenericTypeBuilder<double[]>()
      {
         @Override public double[] newInstance()
         {
            return new double[3];
         }
      };

      parametersX = new RecyclingArrayList<>(maximumNumberOfWaypoints, builder);
      parametersY = new RecyclingArrayList<>(maximumNumberOfWaypoints, builder);
      parametersZ = new RecyclingArrayList<>(maximumNumberOfWaypoints, builder);

      timeMatrix = new Matrix3d();
      this.waypoints = new ArrayList<>();

      trajectoryTime.set(0);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoFrameEuclideanWaypoint waypoint = new YoFrameEuclideanWaypoint(name, "AtWaypoint" + i, registry, referenceFrame);
         waypoints.add(waypoint);
         if (allowMultipleFrames)
            registerMultipleFramesHolders(waypoint);
      }

   }

   private final ArrayList<YoFrameEuclideanWaypoint> subWayPoints = new ArrayList<>();

   public void initialize()
   {
      if (numberOfWayPoints < 3)
      {
         throw new RuntimeException("Minimal three way points are needed.");
      }
      for (int i = 1; i < numberOfWayPoints; i++)
      {
         if (waypoints.get(i).getTime() < waypoints.get(i - 1).getTime())
         {
            throw new RuntimeException("Way points should be sorted on time.");
         }
      }

      if (WaypointTimeGeneration)
      {
         updateWaypointTimes();
      }

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         parametersX.add();
         parametersY.add();
         parametersZ.add();
      }

      initialTime.set(waypoints.get(0).getTime());

      updateFirstParameters(parametersX.get(0), parametersY.get(0), parametersZ.get(0));

      for (int i = 1; i < numberOfWayPoints - 1; i++)
      {
         subWayPoints.clear();
         subWayPoints.addAll(waypoints.subList(i - 1, i + 2));

         updateParameters(subWayPoints, parametersX.get(i), parametersY.get(i), parametersZ.get(i));
      }

      updateLastParameters(parametersX.get(numberOfWayPoints - 1), parametersY.get(numberOfWayPoints - 1), parametersZ.get(numberOfWayPoints - 1));

      for (int i = 0; i < numberOfWayPoints - 1; i++)
      {
         waypoints.get(i).getLinearVelocity()
               .set(calculateVelocity(i, parametersX, 0), calculateVelocity(i, parametersY, 0), calculateVelocity(i, parametersZ, 0));
      }
      waypoints.get(numberOfWayPoints - 1).getLinearVelocity()
            .set(calculateVelocity(numberOfWayPoints - 2, parametersX, 1), calculateVelocity(numberOfWayPoints - 2, parametersY, 1),
                  calculateVelocity(numberOfWayPoints - 2, parametersZ, 1));

   }

   private final Vector3d tempVectorForParameter = new Vector3d();
   private final Vector3d tempPositionVectorForParameter = new Vector3d();

   private void updateParameters(ArrayList<YoFrameEuclideanWaypoint> waypoints, double[] parametersXToPack, double[] parametersYToPack,
         double[] parametersZToPack)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            timeMatrix.setElement(i, j, MathTools.powWithInteger(waypoints.get(i).getTime(), 2 - j));
         }
      }

      timeMatrix.invert();

      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter
               .set(waypoints.get(0).getPosition().getX(), waypoints.get(1).getPosition().getX(), waypoints.get(2).getPosition().getX());
         parametersXToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter
               .set(waypoints.get(0).getPosition().getY(), waypoints.get(1).getPosition().getY(), waypoints.get(2).getPosition().getY());
         parametersYToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter
               .set(waypoints.get(0).getPosition().getZ(), waypoints.get(1).getPosition().getZ(), waypoints.get(2).getPosition().getZ());
         parametersZToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
   }

   private void updateFirstParameters(double[] parametersXToPack, double[] parametersYToPack, double[] parametersZToPack)
   {

      timeMatrix.setElement(0, 0, 2 * waypoints.get(0).getTime());
      timeMatrix.setElement(0, 1, 1);
      timeMatrix.setElement(0, 2, 0);

      for (int i = 1; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            timeMatrix.setElement(i, j, MathTools.powWithInteger(waypoints.get(i).getTime(), 2 - j));
         }
      }

      timeMatrix.invert();

      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter.set(initialVelocity.getX(), waypoints.get(0).getPosition().getX(), waypoints.get(1).getPosition().getX());
         parametersXToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter.set(initialVelocity.getY(), waypoints.get(0).getPosition().getY(), waypoints.get(1).getPosition().getY());
         parametersYToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter.set(initialVelocity.getZ(), waypoints.get(0).getPosition().getZ(), waypoints.get(1).getPosition().getZ());
         parametersZToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
   }

   private void updateLastParameters(double[] parametersXToPack, double[] parametersYToPack, double[] parametersZToPack)
   {
      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            timeMatrix.setElement(i, j, MathTools.powWithInteger(waypoints.get(numberOfWayPoints - 2 + i).getTime(), 2 - j));
         }
      }
      timeMatrix.setElement(2, 0, 2 * waypoints.get(numberOfWayPoints - 1).getTime());
      timeMatrix.setElement(2, 1, 1);
      timeMatrix.setElement(2, 2, 0);

      timeMatrix.invert();

      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter
               .set(waypoints.get(numberOfWayPoints - 2).getPosition().getX(), waypoints.get(numberOfWayPoints - 1).getPosition().getX(), finalVelocity.getX());
         parametersXToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter
               .set(waypoints.get(numberOfWayPoints - 2).getPosition().getY(), waypoints.get(numberOfWayPoints - 1).getPosition().getY(), finalVelocity.getY());
         parametersYToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
      for (int i = 0; i < 3; i++)
      {
         timeMatrix.getRow(i, tempVectorForParameter);
         tempPositionVectorForParameter
               .set(waypoints.get(numberOfWayPoints - 2).getPosition().getZ(), waypoints.get(numberOfWayPoints - 1).getPosition().getZ(), finalVelocity.getZ());
         parametersZToPack[i] = (tempVectorForParameter.dot(tempPositionVectorForParameter));
      }
   }

   private final ArrayList<Double> trajectoryLength = new ArrayList<>();

   private void updateWaypointTimes()
   {
      trajectoryLength.add(0.0);
      for (int i = 1; i < numberOfWayPoints; i++)
      {
         trajectoryLength.add(trajectoryLength.get(i - 1) + Math
               .sqrt(MathTools.square(waypoints.get(i).getPosition().getX() - waypoints.get(i - 1).getPosition().getX()) +
                     MathTools.square(waypoints.get(i).getPosition().getY() - waypoints.get(i - 1).getPosition().getY()) +
                     MathTools.square(waypoints.get(i).getPosition().getZ() - waypoints.get(i - 1).getPosition().getZ())));
      }

      for (int i = 0; i < numberOfWayPoints; i++)
      {
         waypoints.get(i).subtractTimeOffset(
               -1 * (((trajectoryTime.getDoubleValue() - initialTime.getDoubleValue()) / trajectoryLength.get(numberOfWayPoints - 1)) * trajectoryLength.get(i)
                     + initialTime.getDoubleValue()));
      }
   }

   private int waypointIndex = 0;

   public void compute(double currentTime)
   {
      this.currentTime.set(currentTime);
      if (currentTime > waypoints.get(waypointIndex + 1).getTime())
      {
         waypointIndex++;
      }

      double alpha =
            (currentTime - waypoints.get(waypointIndex).getTime()) / (waypoints.get(waypointIndex + 1).getTime() - waypoints.get(waypointIndex).getTime());
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);

      currentPosition.set(calculatePosition(waypointIndex, parametersX, alpha), calculatePosition(waypointIndex, parametersY, alpha),
            calculatePosition(waypointIndex, parametersZ, alpha));

      currentVelocity.set(calculateVelocity(waypointIndex, parametersX, alpha), calculateVelocity(waypointIndex, parametersY, alpha),
            calculateVelocity(waypointIndex, parametersZ, alpha));

      currentAcceleration.set(calculateAcceleration(waypointIndex, parametersX, alpha), calculateAcceleration(waypointIndex, parametersY, alpha),
            calculateAcceleration(waypointIndex, parametersZ, alpha));
   }

   private double previousState = 0;
   private double nextState = 0;

   private double calculatePosition(int waypointIndex, RecyclingArrayList<double[]> parameters, Double alpha)
   {
      previousState =
            parameters.get(waypointIndex)[0] * MathTools.square(currentTime.getDoubleValue()) + parameters.get(waypointIndex)[1] * currentTime.getDoubleValue()
                  + parameters.get(waypointIndex)[2];
      nextState = parameters.get(waypointIndex + 1)[0] * MathTools.square(currentTime.getDoubleValue()) + parameters.get(waypointIndex + 1)[1] * currentTime
            .getDoubleValue() + parameters.get(waypointIndex + 1)[2];

      return alpha * nextState + (1 - alpha) * previousState;
   }

   private double calculateVelocity(int waypointIndex, RecyclingArrayList<double[]> parameters, double alpha)
   {
      previousState = 2 * parameters.get(waypointIndex)[0] * currentTime.getDoubleValue() + parameters.get(waypointIndex)[1];
      nextState = 2 * parameters.get(waypointIndex + 1)[0] * currentTime.getDoubleValue() + parameters.get(waypointIndex + 1)[1];

      return alpha * nextState + (1 - alpha) * previousState;
   }

   private double calculateAcceleration(int waypointIndex, RecyclingArrayList<double[]> parameters, double alpha)
   {
      previousState = 2 * parameters.get(waypointIndex)[0];
      nextState = 2 * parameters.get(waypointIndex + 1)[0];

      return alpha * nextState + (1 - alpha) * previousState;
   }

   public void setInitialVelocity(Vector3d initialVelocity)
   {
      this.initialVelocity.set(initialVelocity);
   }

   public void setFinalVelocity(Vector3d finalVelocity)
   {
      this.finalVelocity.set(finalVelocity);
   }

   public void clear()
   {
      numberOfWayPoints = 0;

      for (YoFrameEuclideanWaypoint waypoint : waypoints)
      {
         waypoint.setToNaN();
      }
   }

   private final FrameVector zeroVelocity = new FrameVector();

   public void appendWaypoint(double timeAtWaypoint, Point3d position)
   {
      if (WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation active, append way points without way point time");
      }
      checkNumberOfWaypoints(numberOfWayPoints + 1);
      trajectoryTime.set(timeAtWaypoint);
      appendWaypointUnsafe(timeAtWaypoint, position);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, Point3d position)
   {
      waypoints.get(numberOfWayPoints).set(timeAtWaypoint, position, zeroVelocity.getVectorCopy());
      numberOfWayPoints++;
   }

   public void appendWaypoint(double timeAtWaypoint, FramePoint position)
   {
      if (WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation active, append way points without way point time");
      }
      checkNumberOfWaypoints(numberOfWayPoints + 1);
      trajectoryTime.set(timeAtWaypoint);
      appendWaypointUnsafe(timeAtWaypoint, position);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, FramePoint position)
   {
      waypoints.get(numberOfWayPoints).set(timeAtWaypoint, position, zeroVelocity);
      numberOfWayPoints++;
   }

   public void appendWaypoints(double[] timeAtWaypoints, FramePoint[] positions)
   {
      if (WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation active, append way points without way point time");
      }
      if (timeAtWaypoints.length != positions.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWayPoints + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i]);
         trajectoryTime.set(timeAtWaypoints[i]);
      }
   }

   public void appendWaypoints(double[] timeAtWaypoints, Point3d[] positions)
   {
      if (WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation active, append way points without way point time");
      }
      if (timeAtWaypoints.length != positions.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWayPoints + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i]);
         trajectoryTime.set(timeAtWaypoints[i]);
      }
   }

   public void appendWaypoint(Point3d position)
   {
      if (!WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation inactive, call activateWaypointTimeGeneration(double initialTime, double finalTime)");
      }
      checkNumberOfWaypoints(numberOfWayPoints + 1);
      appendWaypointUnsafe(0, position);
   }

   public void appendWaypoint(FramePoint position)
   {
      if (!WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation inactive, call activateWaypointTimeGeneration(double initialTime, double finalTime)");
      }
      checkNumberOfWaypoints(numberOfWayPoints + 1);
      appendWaypointUnsafe(0, position);
   }

   public void appendWaypoints(FramePoint[] positions)
   {
      if (!WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation inactive, call activateWaypointTimeGeneration(double initialTime, double finalTime)");
      }

      checkNumberOfWaypoints(numberOfWayPoints + positions.length);

      for (int i = 0; i < positions.length; i++)
      {
         appendWaypointUnsafe(0, positions[i]);
      }
   }

   public void appendWaypoints(Point3d[] positions)
   {
      if (!WaypointTimeGeneration)
      {
         throw new RuntimeException("Automatic way point time generation inactive, call activateWaypointTimeGeneration(double initialTime, double finalTime)");
      }

      checkNumberOfWaypoints(numberOfWayPoints + positions.length);

      for (int i = 0; i < positions.length; i++)
      {
         appendWaypointUnsafe(0, positions[i]);
      }
   }

   private void checkNumberOfWaypoints(int length)
   {
      if (length > maximumNumberOfWaypoints)
         throw new RuntimeException("Cannot exceed the maximum number of waypoints. Number of waypoints provided: " + length);
   }

   /**
    * This method will activate automatic waypoint time generation. It will only work when called before any way points are appended.
    * After this way points can be appended without specifying way point time, this time will be automatically generated based on distance between the way points.
    */
   public void activateWaypointTimeGeneration(double initialTime, double finalTime)
   {
      if (this.numberOfWayPoints > 0)
      {
         throw new RuntimeException("Cannot activate waypoint time generation when there are waypoints appended. Call clear() first.");
      }
      else
      {
         WaypointTimeGeneration = true;
         this.initialTime.set(initialTime);
         this.trajectoryTime.set(finalTime);
      }
   }

   @Override public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public void setToDone()
   {
      currentTime.set(trajectoryTime.getDoubleValue() + 0.01);
   }

   @Override public void getPosition(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      positionToPack.set(currentPosition);
   }

   public void getPosition(Point3d positionToPack)
   {
      currentPosition.get(positionToPack);
   }

   @Override public void getVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(currentVelocity);
   }

   public void getVelocity(Vector3d velocityToPack)
   {
      currentVelocity.get(velocityToPack);
   }

   @Override public void getAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void getAcceleration(Vector3d accelerationToPack)
   {
      currentAcceleration.get(accelerationToPack);
   }

   @Override public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void packLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      positionToPack.set(currentPosition);
      velocityToPack.set(currentVelocity);
      accelerationToPack.set(currentAcceleration);
   }

   public void getFinalPosition(FramePoint finalPosition)
   {
      finalPosition.set(this.waypoints.get(numberOfWayPoints).getPosition().getVector3dCopy());
   }

   @Override public void showVisualization()
   {
   }

   @Override public void hideVisualization()
   {
   }
}
