package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.Sphere3d;

import cern.colt.Arrays;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoConcatenatedSplines;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static double[] DESIRED_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {1.0 / 3.0, 2.0 / 3.0};
   private final static double SPHERE_EDGE_TO_WAYPOINT_DISTANCE = 0.1;
   private final static double MINIMUM_DESIRED_PROPORTION_OF_ARC_LENGTH_TAKEN_AT_CONSTANT_SPEED = 0.3;
   private final static double EPSILON = 1e-3;

   private final DoubleYoVariable groundClearance;
   private final DoubleYoVariable linearSplineLengthFactor;

   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private boolean VISUALIZE = true;
   private final int numberOfVisualizationMarkers = 50;

   private final BagOfBalls trajectoryBagOfBalls;
   private final BagOfBalls waypointBagOfBalls;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSources = new PositionProvider[2];
   private final VectorProvider[] velocitySources = new VectorProvider[2];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   private final ReferenceFrame referenceFrame;

   private DoubleYoVariable[] allTimes = new DoubleYoVariable[6];
   private YoFramePoint[] allPositions = new YoFramePoint[6];
   private YoFrameVector[] allVelocities = new YoFrameVector[6];

   private static final int[] endpointIndices = new int[] {0, 5};
   private static final int[] waypointIndices = new int[] {2, 3};
   private static final int[] oppositeWaypointIndices = new int[] {3, 2};
   private static final int[] accelerationEndpointIndices = new int[] {1, 4};
   private static final int[] nonAccelerationEndpointIndices = new int[] {0, 2, 3, 5};
   private static final int[] allIndices = new int[]
   {
      0, 1, 2, 3, 4, 5
   };

   private final YoConcatenatedSplines concatenatedSplinesWithArcLengthApproximatedByDistance;
   private final YoConcatenatedSplines concatenatedSplinesWithArcLengthCalculatedIteratively;

   public TwoWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableDoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, YoPositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, double groundClearance,
           int arcLengthCalculatorDivisionsPerPolynomial, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);
      trajectoryBagOfBalls = new BagOfBalls(numberOfVisualizationMarkers, 0.01, namePrefix + "TrajectoryBagOfBalls", registry,
              dynamicGraphicObjectsListRegistry);
      waypointBagOfBalls = new BagOfBalls(6, 0.02, namePrefix + "WaypointBagOfBalls", registry, dynamicGraphicObjectsListRegistry);

      this.stepTimeProvider = stepTimeProvider;

      this.referenceFrame = referenceFrame;

      positionSources[0] = initialPositionProvider;
      positionSources[1] = finalPositionProvider;

      velocitySources[0] = initalVelocityProvider;
      velocitySources[1] = finalDesiredVelocityProvider;

      stepTime = new DoubleYoVariable("stepTime", registry);
      timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      this.linearSplineLengthFactor = new DoubleYoVariable(namePrefix + "LinearSplineLengthFactor", registry);

      this.groundClearance = new DoubleYoVariable(namePrefix + "GroundClearance", registry);
      this.groundClearance.set(groundClearance);

      for (int i = 0; i < 6; i++)
      {
         allTimes[i] = new DoubleYoVariable(namePrefix + "FixedPointTime" + i, registry);
         allPositions[i] = new YoFramePoint(namePrefix + "FixedPointPosition" + i, referenceFrame, registry);
         allVelocities[i] = new YoFrameVector(namePrefix + "FixedPointVelocity" + i, referenceFrame, registry);
      }

      concatenatedSplinesWithArcLengthApproximatedByDistance = new YoConcatenatedSplines(new int[] {4, 2, 6, 2, 4}, referenceFrame,
              arcLengthCalculatorDivisionsPerPolynomial, registry, namePrefix + "ConcatenatedSplinesWithArcLengthApproximatedByDistance");
      concatenatedSplinesWithArcLengthCalculatedIteratively = new YoConcatenatedSplines(new int[] {4, 2, 6, 2, 4}, referenceFrame, 2, registry,
              namePrefix + "ConcatenatedSplinesWithArcLengthCalculatedIteratively");
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);

      double totalTime = stepTime.getDoubleValue();
      if (time > totalTime)
         time = totalTime;

      concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time);
      desiredPosition.set(concatenatedSplinesWithArcLengthCalculatedIteratively.getPosition());
      desiredVelocity.set(concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity());
      desiredAcceleration.set(concatenatedSplinesWithArcLengthCalculatedIteratively.getAcceleration());

//    concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time - stepTime.getDoubleValue() / (double) numberOfTimeIntervals);
//    FrameVector previousVelocity = concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity();
//
//    concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time + stepTime.getDoubleValue() / (double) numberOfTimeIntervals);
//    FrameVector nextVelocity = concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity();
//    
//    desiredAcceleration.set(nextVelocity);
//    desiredAcceleration.sub(previousVelocity);
//    desiredAcceleration.scale(0.5 * (double) numberOfTimeIntervals / stepTime.getDoubleValue());

   }

   public void get(FramePoint positionToPack)
   {
      desiredPosition.getFramePointAndChangeFrameOfPackedPoint(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameVectorAndChangeFrameOfPackedVector(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameVectorAndChangeFrameOfPackedVector(accelerationToPack);
   }

   private void setStepTime()
   {
      double stepTime = stepTimeProvider.getValue();
      MathTools.checkIfInRange(stepTime, 0.0, Double.POSITIVE_INFINITY);
      this.stepTime.set(stepTime);
      timeIntoStep.set(0.0);
   }

   public void initialize()
   {
      initialize(null);
   }


   public void initialize(FramePoint[] waypoints)
   {
      setStepTime();

      setInitialAndFinalTimesPositionsAndVelocities();
      setWaypointPositions(waypoints);

      double[] arcLengthsIgnoringAccelerationEndpointPositions = getArcLengthsApproximatedByDistance(nonAccelerationEndpointIndices);
      setLinearSplineLengthFactor(arcLengthsIgnoringAccelerationEndpointPositions);
      setAccelerationEndpointPositions();

      double[] arcLengths;

      arcLengths = getArcLengthsApproximatedByDistance(allIndices);
      setWaypointAndAccelerationEndpointTimesAndVelocities(arcLengths);
      setConcatenatedSplines(concatenatedSplinesWithArcLengthApproximatedByDistance);

      arcLengths = getArcLengthsCalculatedIteratively();
      setWaypointAndAccelerationEndpointTimesAndVelocities(arcLengths);
      setConcatenatedSplines(concatenatedSplinesWithArcLengthCalculatedIteratively);

      if (VISUALIZE)
      {
         visualizeSpline();
      }
   }
   
   private void setLinearSplineLengthFactor(double[] arcLengths)
   {
      double portionOfArcLengthInMiddleSpline = arcLengths[1] / (arcLengths[0] + arcLengths[1] + arcLengths[2]);
      System.out.println(Arrays.toString(arcLengths));
      linearSplineLengthFactor.set(Math.max(MINIMUM_DESIRED_PROPORTION_OF_ARC_LENGTH_TAKEN_AT_CONSTANT_SPEED - portionOfArcLengthInMiddleSpline, 0.0));
   }

   private void setWaypointAndAccelerationEndpointTimesAndVelocities(double[] arcLengths)
   {
      double waypointSpeed = getWaypointSpeed(arcLengths);
      for (int i = 0; i < 2; i++)
      {
         FrameVector waypointVelocity = getOppositeWaypointToEndpoint(i);
         waypointVelocity.normalize();
         waypointVelocity.scale(waypointSpeed * ((i == 0) ? -1 : 1));
         allVelocities[waypointIndices[i]].set(waypointVelocity);
         allVelocities[accelerationEndpointIndices[i]].set(waypointVelocity);
      }

      allTimes[1].set(2 * arcLengths[0] / (allVelocities[0].length() + waypointSpeed));
      allTimes[2].set(allTimes[1].getDoubleValue() + arcLengths[1] / waypointSpeed);
      allTimes[3].set(allTimes[2].getDoubleValue() + arcLengths[2] / waypointSpeed);
      allTimes[4].set(allTimes[3].getDoubleValue() + arcLengths[3] / waypointSpeed);
   }

   private double getWaypointSpeed(double[] arcLengths)
   {
      double lowerBound = 0.0;
      double upperBound = 10.0;
      while (getStepTimeGivenWaypointSpeed(upperBound, arcLengths) > stepTime.getDoubleValue() + EPSILON)
      {
         upperBound *= 2.0;
      }

      double waypointSpeed;
      double resultingStepTime;

      while (true)
      {
         waypointSpeed = (upperBound + lowerBound) / 2.0;
         resultingStepTime = getStepTimeGivenWaypointSpeed(waypointSpeed, arcLengths);

         if (MathTools.epsilonEquals(resultingStepTime, stepTime.getDoubleValue(), EPSILON))
         {
            return waypointSpeed;
         }
         else if (resultingStepTime > stepTime.getDoubleValue())
         {
            lowerBound = waypointSpeed;
         }
         else if (resultingStepTime < stepTime.getDoubleValue())
         {
            upperBound = waypointSpeed;
         }
      }

   }

   private double getStepTimeGivenWaypointSpeed(double waypointSpeed, double[] arcLengths)
   {
      double initialSpeed = allVelocities[0].length();
      double finalSpeed = allVelocities[5].length();

      return 2 * arcLengths[0] / (initialSpeed + waypointSpeed) + (arcLengths[1] + arcLengths[2] + arcLengths[3]) / waypointSpeed
             + 2 * arcLengths[4] / (waypointSpeed + finalSpeed);
   }

   private void setAccelerationEndpointPositions()
   {
      for (int i : new int[] {0, 1})
      {
         FrameVector waypointToEndpoint = getWaypointToEndpoint(i);

         FrameVector oppositeWaypointToEndpoint = getOppositeWaypointToEndpoint(i);

         double scaleFactor = waypointToEndpoint.dot(oppositeWaypointToEndpoint) / oppositeWaypointToEndpoint.length()
                              * linearSplineLengthFactor.getDoubleValue();

         oppositeWaypointToEndpoint.normalize();
         oppositeWaypointToEndpoint.scale(scaleFactor);

         allPositions[accelerationEndpointIndices[i]].set(allPositions[waypointIndices[i]].getFramePointCopy());
         allPositions[accelerationEndpointIndices[i]].add(oppositeWaypointToEndpoint);
      }
   }

   private FrameVector getOppositeWaypointToEndpoint(int i)
   {
      FrameVector oppositeWaypointToEndpoint = allPositions[endpointIndices[i]].getFrameVectorCopy();
      oppositeWaypointToEndpoint.sub(allPositions[oppositeWaypointIndices[i]].getFrameVectorCopy());

      return oppositeWaypointToEndpoint;
   }

   private FrameVector getWaypointToEndpoint(int i)
   {
      FrameVector waypointToEndpoint = allPositions[endpointIndices[i]].getFrameVectorCopy();
      waypointToEndpoint.sub(allPositions[waypointIndices[i]].getFrameVectorCopy());

      return waypointToEndpoint;
   }

   private double[] getArcLengthsApproximatedByDistance(int[] indices)
   {
      double[] arcLengths = new double[indices.length];

      for (int i = 0; i < indices.length - 1; i++)
      {
         arcLengths[i] = allPositions[indices[i]].distance(allPositions[indices[i + 1]]);
      }

      return arcLengths;
   }

   private double[] getArcLengthsCalculatedIteratively()
   {
      double[] arcLengths = new double[5];

      for (int i = 0; i < arcLengths.length; i++)
      {
         arcLengths[i] = concatenatedSplinesWithArcLengthApproximatedByDistance.getSplineByIndex(i).getArcLength();
      }

      return arcLengths;
   }

   private void setInitialAndFinalTimesPositionsAndVelocities()
   {
      FramePoint tempPositions = new FramePoint(referenceFrame);
      FrameVector tempVelocities = new FrameVector(referenceFrame);
      for (int i = 0; i < 2; i++)
      {
         positionSources[i].get(tempPositions);
         velocitySources[i].get(tempVelocities);
         tempPositions.changeFrame(referenceFrame);
         tempVelocities.changeFrame(referenceFrame);
         allPositions[endpointIndices[i]].set(tempPositions);
         allVelocities[endpointIndices[i]].set(tempVelocities);
      }

      allTimes[endpointIndices[0]].set(0.0);
      allTimes[endpointIndices[1]].set(stepTime.getDoubleValue());
   }

   private void setWaypointPositions(FramePoint[] waypoints)
   {
      if (waypoints == null)
      {
         waypoints = getDefaultWaypoints();
      }

      else if (waypoints.length != 2)
      {
         throw new RuntimeException("TwoWaypointPositionTrajectoryGenerator only supports trajectory generation for two waypoints.");
      }

      for (int i = 0; i < 2; i++)
      {
         waypoints[i].changeFrame(referenceFrame);
         allPositions[waypointIndices[i]].set(waypoints[i]);
      }
   }

   private FramePoint[] getDefaultWaypoints()
   {
      FramePoint initialPosition = allPositions[0].getFramePointCopy();
      FramePoint finalPosition = allPositions[3].getFramePointCopy();
      FramePoint[] waypoints = new FramePoint[2];
      waypoints[0] = new FramePoint(referenceFrame);
      waypoints[1] = new FramePoint(referenceFrame);
      positionSources[0].get(initialPosition);
      positionSources[1].get(finalPosition);
      initialPosition.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);

      for (int i = 0; i < 2; i++)
      {
         waypoints[i].set(finalPosition);
         waypoints[i].sub(initialPosition);
         waypoints[i].scale(DESIRED_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE[i]);
         waypoints[i].add(initialPosition);
         waypoints[i].setZ(waypoints[i].getZ() + groundClearance.getDoubleValue());
      }

//    waypoints[0].setX(waypoints[0].getX() + .03);
//    waypoints[1].setX(waypoints[1].getX() + .03);
//    waypoints[0].setY(waypoints[0].getY() + .05);
//    waypoints[1].setY(waypoints[1].getY() - .05);

      return waypoints;
   }

   private void setConcatenatedSplines(YoConcatenatedSplines concatenatedSplines)
   {
      double[] nonAccelerationEndpointTimes = new double[4];
      double[] accelerationEndpointTimes = new double[2];
      FramePoint[] nonAccelerationEndpointPositions = new FramePoint[4];
      FrameVector[] nonAccelerationEndpointVelocities = new FrameVector[4];

      for (int i = 0; i < 4; i++)
      {
         nonAccelerationEndpointTimes[i] = allTimes[nonAccelerationEndpointIndices[i]].getDoubleValue();
         nonAccelerationEndpointPositions[i] = allPositions[nonAccelerationEndpointIndices[i]].getFramePointCopy();
         nonAccelerationEndpointVelocities[i] = allVelocities[nonAccelerationEndpointIndices[i]].getFrameVectorCopy();
      }

      for (int i = 0; i < 2; i++)
      {
         accelerationEndpointTimes[i] = allTimes[accelerationEndpointIndices[i]].getDoubleValue();
      }

      concatenatedSplines.setCubicLinearQuinticLinearCubic(nonAccelerationEndpointTimes, nonAccelerationEndpointPositions, nonAccelerationEndpointVelocities,
              accelerationEndpointTimes);
   }

   private void visualizeSpline()
   {
      for (int i = 0; i < numberOfVisualizationMarkers; i++)
      {
         double t0 = concatenatedSplinesWithArcLengthCalculatedIteratively.getT0();
         double tf = concatenatedSplinesWithArcLengthCalculatedIteratively.getTf();
         double t = t0 + (double) i / (double) (numberOfVisualizationMarkers) * (tf - t0);
         compute(t);
         trajectoryBagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
      }

      for (int i = 0; i < allPositions.length; i++)
      {
         waypointBagOfBalls.setBall(allPositions[i].getFramePointCopy(), YoAppearance.AliceBlue(), i);
      }
   }

   public void initializeUsingSphere(Sphere3d sphere3d)
   {
      FramePoint[] waypoints = getWaypointsForSphere(sphere3d);
      initialize(waypoints);
   }

   private FramePoint[] getWaypointsForSphere(Sphere3d sphere3d)
   {
      FramePoint[] wayPoints = new FramePoint[2];
      Point3d sphereCenterPoint3d = new Point3d();
      sphere3d.getCenter(sphereCenterPoint3d);
      FramePoint sphereCenterFramePoint = new FramePoint(referenceFrame, sphereCenterPoint3d.getX(), sphereCenterPoint3d.getY(), sphereCenterPoint3d.getZ());
      double radius = sphere3d.getRadius();
      boolean areDefaultWaypointsOutsideSphere = true;

      FramePoint tempPositions = new FramePoint(referenceFrame);
      for (int i = 0; i < 2; i++)
      {
         positionSources[i].get(tempPositions);
         tempPositions.changeFrame(referenceFrame);
         allPositions[3 * i].set(tempPositions);
      }

      // setWaypointsToDefault();
      double[] sphereWaypointDistances = new double[2];
      for (int i = 0; i < 2; i++)
      {
         sphereWaypointDistances[i] = wayPoints[i].distance(sphereCenterFramePoint);
         wayPoints[i].set(allPositions[i].getX(), allPositions[i].getY(), allPositions[i].getZ());
         areDefaultWaypointsOutsideSphere = areDefaultWaypointsOutsideSphere && (sphereWaypointDistances[i] < radius);
      }

      if (!areDefaultWaypointsOutsideSphere)
      {
         FrameVector waypointToSphereSurface = new FrameVector(referenceFrame);
         for (int i = 0; i < 2; i++)
         {
            waypointToSphereSurface.sub(wayPoints[i], sphereCenterFramePoint);
            waypointToSphereSurface.normalize();
            double distanceToMoveWaypoint = radius - sphereWaypointDistances[i] + SPHERE_EDGE_TO_WAYPOINT_DISTANCE;
            waypointToSphereSurface.scale(distanceToMoveWaypoint);
            wayPoints[i].add(waypointToSphereSurface);
         }
      }

      return wayPoints;
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }
}
