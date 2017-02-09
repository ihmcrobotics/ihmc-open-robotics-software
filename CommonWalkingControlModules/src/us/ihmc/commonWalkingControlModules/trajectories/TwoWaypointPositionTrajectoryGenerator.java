package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoConcatenatedSplines;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.TwoWaypointTrajectoryGeneratorParameters;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.robotics.trajectories.providers.TrajectoryParametersProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   // JPratt. February 27, 2015: Added this since new Atlas was having trouble with network stuff.
   // It was sending 14,000 variables. This and others reduces it a bit when set to false.
   protected static final boolean REGISTER_YOVARIABLES = true;

   private final static int arcLengthCalculatorDivisionsPerPolynomial = 20;
   private final static double EPSILON = 1e-3;
   private final static double WAYPOINT_CLOSENESS_FACTOR = .15; // waypoints are considered close together if the distance between them is less than the total

   // distance times this fraction; waypoints that are close together are both set to their midpoint and passed through at a velocity of zero

   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final int numberOfVisualizationMarkers = 50;
   private final BooleanYoVariable visualize;

   private final BagOfBalls trajectoryBagOfBalls;
   private final BagOfBalls fixedPointBagOfBalls;

   private final DoubleYoVariable linearSplineLengthFactor;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSources = new PositionProvider[2];
   private final PositionProvider stancePositionSource;
   private final VectorProvider[] velocitySources = new VectorProvider[2];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final BooleanYoVariable setInitialSwingVelocityToZero;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   private final ReferenceFrame referenceFrame;

   private final DoubleYoVariable[] allTimes = new DoubleYoVariable[6];
   protected final YoFramePoint[] allPositions = new YoFramePoint[6];
   protected final YoFramePoint stancePosition;
   private final YoFrameVector[] allVelocities = new YoFrameVector[6];

   private final TrajectoryParametersProvider trajectoryParametersProvider;
   protected TrajectoryParameters trajectoryParameters;

   private static final int[] endpointIndices = new int[] {0, 5};
   private static final int[] waypointIndices = new int[] {2, 3};
   private static final int[] oppositeWaypointIndices = new int[] {3, 2};
   private static final int[] accelerationEndpointIndices = new int[] {1, 4};
   private static final int[] nonAccelerationEndpointIndices = new int[] {0, 2, 3, 5};
   private static final int[] allIndices = new int[] {0, 1, 2, 3, 4, 5};

   private final YoConcatenatedSplines concatenatedSplinesWithArcLengthApproximatedByDistance;
   private final YoConcatenatedSplines concatenatedSplinesWithArcLengthCalculatedIteratively;

   private boolean waypointsAreTheSamePoint = false;
   private final double maxSwingHeightFromStanceFoot;

   public TwoWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
         PositionProvider initialPositionProvider, VectorProvider initialVelocityProvider, PositionProvider stancePositionProvider,
         PositionProvider finalPositionProvider, VectorProvider finalDesiredVelocityProvider, TrajectoryParametersProvider trajectoryParametersProvider,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double maxSwingHeightFromStanceFoot,
         boolean visualize)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      if (REGISTER_YOVARIABLES)
      {
         parentRegistry.addChild(registry);
      }
      else
      {
         visualize = false;
         yoGraphicsListRegistry = null;
      }

      setInitialSwingVelocityToZero = new BooleanYoVariable(namePrefix + "SetInitialSwingVelocityToZero", registry);
      setInitialSwingVelocityToZero.set(false);

      if (visualize)
      {
         trajectoryBagOfBalls = new BagOfBalls(numberOfVisualizationMarkers, 0.01, namePrefix + "TrajectoryBagOfBalls", registry, yoGraphicsListRegistry);
         fixedPointBagOfBalls = new BagOfBalls(6, 0.02, namePrefix + "WaypointBagOfBalls", registry, yoGraphicsListRegistry);
      }

      else
      {
         trajectoryBagOfBalls = null;
         fixedPointBagOfBalls = null;
      }

      this.stepTimeProvider = stepTimeProvider;

      this.referenceFrame = referenceFrame;

      positionSources[0] = initialPositionProvider;
      positionSources[1] = finalPositionProvider;
      stancePositionSource = stancePositionProvider;

      velocitySources[0] = initialVelocityProvider;
      velocitySources[1] = finalDesiredVelocityProvider;

      stepTime = new DoubleYoVariable(namePrefix + "StepTime", registry);
      timeIntoStep = new DoubleYoVariable(namePrefix + "TimeIntoStep", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      linearSplineLengthFactor = new DoubleYoVariable(namePrefix + "LinearSplineLengthFactor", registry);

      this.trajectoryParametersProvider = trajectoryParametersProvider;

      for (int i = 0; i < 6; i++)
      {
         allTimes[i] = new DoubleYoVariable(namePrefix + "FixedPointTime" + i, registry);
         allPositions[i] = new YoFramePoint(namePrefix + "FixedPointPosition" + i, referenceFrame, registry);
         allVelocities[i] = new YoFrameVector(namePrefix + "FixedPointVelocity" + i, referenceFrame, registry);
      }
      stancePosition = new YoFramePoint(namePrefix + "StancePosition", referenceFrame, registry);

      concatenatedSplinesWithArcLengthApproximatedByDistance = new YoConcatenatedSplines(new int[] {4, 2, 6, 2, 4}, referenceFrame,
            arcLengthCalculatorDivisionsPerPolynomial, registry, namePrefix + "ConcatenatedSplinesWithArcLengthApproximatedByDistance");
      concatenatedSplinesWithArcLengthCalculatedIteratively = new YoConcatenatedSplines(new int[] {4, 2, 6, 2, 4}, referenceFrame, 2, registry,
            namePrefix + "ConcatenatedSplinesWithArcLengthCalculatedIteratively");

      this.visualize = new BooleanYoVariable(namePrefix + "Visualize", registry);
      this.visualize.set(visualize);
      this.maxSwingHeightFromStanceFoot = maxSwingHeightFromStanceFoot;
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);

      double totalTime = stepTime.getDoubleValue();
      if (time > totalTime)
         time = totalTime;

      concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time);
      concatenatedSplinesWithArcLengthCalculatedIteratively.getPosition(desiredPosition);
      concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity(desiredVelocity);
      concatenatedSplinesWithArcLengthCalculatedIteratively.getAcceleration(desiredAcceleration);

      // concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time - stepTime.getDoubleValue() / (double) numberOfTimeIntervals);
      // FrameVector previousVelocity = concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity();
      //
      // concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time + stepTime.getDoubleValue() / (double) numberOfTimeIntervals);
      // FrameVector nextVelocity = concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity();
      //
      // desiredAcceleration.set(nextVelocity);
      // desiredAcceleration.sub(previousVelocity);
      // desiredAcceleration.scale(0.5 * (double) numberOfTimeIntervals / stepTime.getDoubleValue());

   }

   public void getPosition(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   private void setStepTime()
   {
      double stepTime = stepTimeProvider.getValue();
      if (stepTime <= 0)
      {
         throw new RuntimeException("stepTimeProvider must provide a positive time value.");
      }

      this.stepTime.set(stepTime);
      timeIntoStep.set(0.0);
   }

   public void initialize()
   {
      setTrajectoryParameters();

      setStepTime();

      setInitialAndFinalTimesPositionsAndVelocities();
      setWaypointPositions();

      double[] arcLengthsIgnoringAccelerationEndpointPositions = getArcLengthsApproximatedByDistance(nonAccelerationEndpointIndices);
      setLinearSplineLengthFactor(arcLengthsIgnoringAccelerationEndpointPositions);
      setAccelerationEndpointPositions();

      double[] arcLengths;

      arcLengths = getArcLengthsApproximatedByDistance(allIndices);
      setWaypointAndAccelerationEndpointTimesAndVelocities(arcLengths);

      if (!waypointsAreTheSamePoint)
      {
         setConcatenatedSplines(concatenatedSplinesWithArcLengthApproximatedByDistance);
         arcLengths = getArcLengthsCalculatedIteratively();
         setWaypointAndAccelerationEndpointTimesAndVelocities(arcLengths);
      }

      setConcatenatedSplines(concatenatedSplinesWithArcLengthCalculatedIteratively);

      if (visualize.getBooleanValue())
      {
         visualizeSpline();
      }
   }

   private void setTrajectoryParameters()
   {
      this.trajectoryParameters = trajectoryParametersProvider.getTrajectoryParameters();
   }

   private void setLinearSplineLengthFactor(double[] arcLengths)
   {
      double portionOfArcLengthInMiddleSpline = arcLengths[1] / (arcLengths[0] + arcLengths[1] + arcLengths[2]);
      linearSplineLengthFactor.set(2.0 * Math
            .max(TwoWaypointTrajectoryGeneratorParameters.getMinimumDesiredProportionOfArcLengthTakenAtConstantSpeed() - portionOfArcLengthInMiddleSpline, 0.0)
            * (1 / (1 - portionOfArcLengthInMiddleSpline)));
   }

   private void setWaypointAndAccelerationEndpointTimesAndVelocities(double[] arcLengths)
   {
      double waypointSpeed = waypointsAreTheSamePoint ? 0.0 : getWaypointSpeed(arcLengths);

      for (int i = 0; i < 2; i++)
      {
         FrameVector waypointVelocity = getOppositeWaypointToEndpoint(i);
         waypointVelocity.normalize();
         waypointVelocity.scale(waypointSpeed * ((i == 0) ? -1 : 1));
         allVelocities[waypointIndices[i]].set(waypointVelocity);
         allVelocities[accelerationEndpointIndices[i]].set(waypointVelocity);
      }

      if (waypointsAreTheSamePoint)
      {
         double totalArcLength = getTotalArcLength(arcLengths);
         allTimes[1].set((arcLengths[0] + arcLengths[1]) / totalArcLength * stepTime.getDoubleValue());

         for (int i = 2; i < 5; i++)
         {
            allTimes[i].set(allTimes[1].getDoubleValue());
         }
      }

      else
      {
         allTimes[1].set(2 * arcLengths[0] / (allVelocities[0].length() + waypointSpeed));
         allTimes[2].set(allTimes[1].getDoubleValue() + arcLengths[1] / waypointSpeed);
         allTimes[3].set(allTimes[2].getDoubleValue() + arcLengths[2] / waypointSpeed);
         allTimes[4].set(allTimes[3].getDoubleValue() + arcLengths[3] / waypointSpeed);
      }
   }

   private boolean waypointsAreCloseTogether()
   {
      double[] arcLengths = getArcLengthsApproximatedByDistance(nonAccelerationEndpointIndices);
      double totalArcLength = getTotalArcLength(arcLengths);
      double arcLengthOfMiddleSpline = arcLengths[1];

      return arcLengthOfMiddleSpline < WAYPOINT_CLOSENESS_FACTOR * totalArcLength;
   }

   private double getTotalArcLength(double[] arcLengths)
   {
      double totalArcLength = 0.0;
      for (int i = 0; i < arcLengths.length; i++)
      {
         totalArcLength += arcLengths[i];
      }

      return totalArcLength;
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
      double[] arcLengths = new double[indices.length - 1];

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
      FramePoint tempPosition = new FramePoint(referenceFrame);
      FrameVector tempVelocity = new FrameVector(referenceFrame);
      for (int i = 0; i < 2; i++)
      {
         positionSources[i].getPosition(tempPosition);
         velocitySources[i].get(tempVelocity);

         boolean setInitialVelocityToZero = setInitialSwingVelocityToZero.getBooleanValue();
         if ((i == 0) && setInitialVelocityToZero)
         {
            tempVelocity.set(0.0, 0.0, 0.0);
         }

         tempPosition.changeFrame(referenceFrame);
         tempVelocity.changeFrame(referenceFrame);
         allPositions[endpointIndices[i]].set(tempPosition);
         allVelocities[endpointIndices[i]].set(tempVelocity);
      }
      if (stancePositionSource != null)
      {
         stancePositionSource.getPosition(tempPosition);
         tempPosition.changeFrame(referenceFrame);
         stancePosition.set(tempPosition);
      }
      allTimes[endpointIndices[0]].set(0.0);
      allTimes[endpointIndices[1]].set(stepTime.getDoubleValue());
   }

   protected void setWaypointPositions()
   {
      List<FramePoint> waypoints = getWaypointsFromTrajectoryParameters(trajectoryParameters);

      for (int i = 0; i < 2; i++)
      {
         waypoints.get(i).changeFrame(referenceFrame);
         allPositions[waypointIndices[i]].set(waypoints.get(i));
      }

      checkForCloseWaypoints();
   }

   protected void checkForCloseWaypoints()
   {
      if (waypointsAreCloseTogether())
      {
         FramePoint midpoint = allPositions[waypointIndices[0]].getFramePointCopy();
         midpoint.add(allPositions[waypointIndices[1]].getFramePointCopy());
         midpoint.scale(0.5);

         for (int i = 0; i < 2; i++)
         {
            allPositions[waypointIndices[i]].set(midpoint);
         }

         waypointsAreTheSamePoint = true;
      }

      else
      {
         waypointsAreTheSamePoint = false;
      }
   }

   private List<FramePoint> getWaypointsFromTrajectoryParameters(TrajectoryParameters trajectoryParameters)
   {
      double swingHeight = trajectoryParameters.getSwingHeight();
      swingHeight = Math.max(swingHeight, TwoWaypointTrajectoryGeneratorParameters.getDefaultGroundClearance());

      double initialHeight = allPositions[0].getZ();
      if (stancePositionSource != null)
      {
         double stanceHeight = stancePosition.getZ();
         double maxWorldHeightForSwing = stanceHeight + maxSwingHeightFromStanceFoot;
         if (initialHeight + swingHeight > maxWorldHeightForSwing)
         {
            swingHeight = maxWorldHeightForSwing - initialHeight;
         }
      }

      switch (trajectoryParameters.getTrajectoryType())
      {
      case OBSTACLE_CLEARANCE:
         return getWaypointsForObstacleClearance(swingHeight);

      case DEFAULT:
      default:
         return getWaypointsAtGroundClearance(TwoWaypointTrajectoryGeneratorParameters.getDefaultGroundClearance());
      }
   }

   private List<FramePoint> getWaypointsForObstacleClearance(double swingHeight)
   {
      List<FramePoint> waypoints = new ArrayList<FramePoint>();
      waypoints.add(allPositions[endpointIndices[0]].getFramePointCopy());
      waypoints.add(allPositions[endpointIndices[1]].getFramePointCopy());

      double zSwingHeight = waypoints.get(0).getZ() + swingHeight;

      // safety, should always clear the ground for the other end foot
      zSwingHeight = Math.max(zSwingHeight, waypoints.get(1).getZ() + TwoWaypointTrajectoryGeneratorParameters.getDefaultGroundClearance());

      for (FramePoint waypoint : waypoints)
      {
         waypoint.setZ(zSwingHeight);
      }

      FrameVector planarEndpointOffset = allPositions[endpointIndices[1]].getFrameVectorCopy();
      planarEndpointOffset.sub(allPositions[endpointIndices[0]].getFrameVectorCopy());
      planarEndpointOffset.setZ(0.0);

      double[] fractionsOfStepDistanceToMoveWaypointForStepOnOrOff = TwoWaypointTrajectoryGeneratorParameters
            .getStepOnOrOffProportionsThroughTrajectoryForGroundClearance();

      for (int i = 0; i < 2; i++)
      {
         FramePoint waypoint = waypoints.get(i);
         FrameVector planarWaypointOffset = new FrameVector(planarEndpointOffset);
         double scaleFactor = fractionsOfStepDistanceToMoveWaypointForStepOnOrOff[i];
         if (i == 1)
            scaleFactor = scaleFactor - 1.0;
         planarWaypointOffset.scale(scaleFactor);
         double offsetLength = planarWaypointOffset.length();
         //         if (offsetLength > TwoWaypointTrajectoryGeneratorParameters.getMaxHorizontalOffsetForWaypoints()){
         //            planarWaypointOffset.scale(TwoWaypointTrajectoryGeneratorParameters.getMaxHorizontalOffsetForWaypoints()/ offsetLength);
         //         }

         waypoint.add(planarWaypointOffset);
      }

      return waypoints;
   }

   private List<FramePoint> getWaypointsAtGroundClearance(double groundClearance)
   {
      return getWaypointsAtGroundClearance(groundClearance,
            TwoWaypointTrajectoryGeneratorParameters.getDefaultProportionsThroughTrajectoryForGroundClearance());
   }

   private List<FramePoint> getWaypointsAtGroundClearance(double groundClearance, double[] proportionsThroughTrajectoryForGroundClearance)
   {
      FramePoint initialPosition = allPositions[0].getFramePointCopy();
      FramePoint finalPosition = allPositions[3].getFramePointCopy();
      positionSources[0].getPosition(initialPosition);
      positionSources[1].getPosition(finalPosition);
      initialPosition.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);

      List<FramePoint> waypoints = getWaypointsAtSpecifiedGroundClearance(initialPosition, finalPosition, groundClearance,
            proportionsThroughTrajectoryForGroundClearance);

      return waypoints;
   }

   public static List<FramePoint> getWaypointsAtSpecifiedGroundClearance(FramePoint initialPosition, FramePoint finalPosition, double groundClearance,
         double[] proportionsThroughTrajectoryForGroundClearance)
   {
      List<FramePoint> waypoints = new ArrayList<FramePoint>();
      waypoints.add(new FramePoint(initialPosition.getReferenceFrame()));
      waypoints.add(new FramePoint(initialPosition.getReferenceFrame()));

      for (int i = 0; i < 2; i++)
      {
         FramePoint waypoint = waypoints.get(i);
         waypoint.set(initialPosition);

         FrameVector offsetFromInitial = new FrameVector(waypoint.getReferenceFrame());
         offsetFromInitial.set(finalPosition);
         offsetFromInitial.sub(initialPosition);
         offsetFromInitial.scale(proportionsThroughTrajectoryForGroundClearance[i]);

         waypoint.add(offsetFromInitial);
         waypoint.setZ(waypoints.get(i).getZ() + groundClearance);
      }

      return waypoints;
   }

   private List<FramePoint> getWaypointsAtGroundClearances(double[] groundClearances, double[] proportionsThroughTrajectoryForGroundClearance)
   {
      FramePoint initialPosition = allPositions[0].getFramePointCopy();
      FramePoint finalPosition = allPositions[3].getFramePointCopy();
      positionSources[0].getPosition(initialPosition);
      positionSources[1].getPosition(finalPosition);
      initialPosition.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);

      List<FramePoint> waypoints = new ArrayList<FramePoint>();
      waypoints.add(new FramePoint(initialPosition.getReferenceFrame()));
      waypoints.add(new FramePoint(initialPosition.getReferenceFrame()));

      for (int i = 0; i < 2; i++)
      {
         FramePoint waypoint = waypoints.get(i);
         waypoint.set(initialPosition);

         FrameVector offsetFromInitial = new FrameVector(waypoint.getReferenceFrame());
         offsetFromInitial.set(finalPosition);
         offsetFromInitial.sub(initialPosition);
         offsetFromInitial.scale(proportionsThroughTrajectoryForGroundClearance[i]);

         waypoint.add(offsetFromInitial);
         waypoint.setZ(waypoints.get(i).getZ() + groundClearances[i]);
      }

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
      double t0;
      double tf;
      double t;

      for (int i = 0; i < numberOfVisualizationMarkers; i++)
      {
         t0 = concatenatedSplinesWithArcLengthCalculatedIteratively.getT0();
         tf = concatenatedSplinesWithArcLengthCalculatedIteratively.getTf();
         t = t0 + (double) i / (double) (numberOfVisualizationMarkers) * (tf - t0);
         compute(t);
         trajectoryBagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
      }

      for (int i = 0; i < nonAccelerationEndpointIndices.length; i++)
      {
         fixedPointBagOfBalls.setBall(allPositions[nonAccelerationEndpointIndices[i]].getFramePointCopy(), YoAppearance.AliceBlue(), i);
      }
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void hideVisualization()
   {
      // TODO Auto-generated method stub
   }

   public void informDone()
   {
      desiredPosition.setToZero(true);
      desiredVelocity.setToZero(true);
      desiredAcceleration.setToZero(true);
   }
}
