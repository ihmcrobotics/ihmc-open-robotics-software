package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TwoWaypointSwingGenerator implements SwingGenerator
{
   private static final int maxTimeIterations = -1; // setting this negative activates continuous updating
   private static final int defaultNumberOfWaypoints = 2;
   private static final double[] defaultWaypointProportions = new double[] {0.15, 0.85};

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final ReferenceFrame trajectoryFrame;

   private final YoDouble stepTime;
   private final YoDouble timeIntoStep;
   private final YoBoolean isDone;
   private final YoBoolean isSteppingDown;
   private final YoDouble swingHeight;
   private final YoDouble minSwingHeight;
   private final YoDouble maxSwingHeight;
   private final YoDouble defaultSwingHeight;
   private final YoDouble customWaypointAngleThreshold;
   private final YoDouble stepUpFirstWaypointHeightFactor;
   private final YoDouble stepDownSecondWaypointHeightFactor;

   private final double[] waypointProportions = new double[2];

   private TrajectoryType trajectoryType;
   private final PositionOptimizedTrajectoryGenerator trajectory;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();
   private final RecyclingArrayList<FramePoint3D> waypointPositions;
   private final FramePoint3D stanceFootPosition = new FramePoint3D();

   private final FrameVector3D interWaypointDisplacement = new FrameVector3D();
   private final FrameVector3D startToWaypointDisplacement = new FrameVector3D();

   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();

   private final Vector3D initialPositionWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final Vector3D initialVelocityWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final Vector3D finalPositionWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final Vector3D finalVelocityWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

   private final FrameVector3D initialVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D finalVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D tempWaypointVelocity;

   private final FramePoint3D tempPoint3D = new FramePoint3D();

   private final BagOfBalls waypointViz;

   private RobotSide swingSide = null;
   private ReferenceFrame stanceZUpFrame = null;
   private final Vector2D swingOffset = new Vector2D();
   private final YoDouble minDistanceToStance;
   private final YoBoolean needToAdjustedSwingForSelfCollision;
   private final YoBoolean crossOverStep;

   private final int maxNumberOfSwingWaypoints;
   private boolean visualize = true;
   private final String namePrefix;

   /**
    * Provider used to obtain the ground height for the initial foot position. Called in
    * {@link #initialize()}.
    */
   private DoubleProvider initialGroundHeightProvider = null;
   /**
    * Provider used to obtain the ground height for the final foot position. Called in
    * {@link #initialize()}.
    */
   private DoubleProvider finalGroundHeightProvider = null;

   public TwoWaypointSwingGenerator(String namePrefix,
                                    double minSwingHeight,
                                    double maxSwingHeight,
                                    double defaultSwingHeight,
                                    double customWaypointAngleThreshold,
                                    YoRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, minSwingHeight, maxSwingHeight, defaultSwingHeight, customWaypointAngleThreshold, worldFrame, parentRegistry, yoGraphicsListRegistry);
   }

   public TwoWaypointSwingGenerator(String namePrefix,
                                    double minSwingHeight,
                                    double maxSwingHeight,
                                    double defaultSwingHeight,
                                    double customWaypointAngleThreshold,
                                    int maxNumberOfSwingWaypoints,
                                    YoRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix,
           minSwingHeight,
           maxSwingHeight,
           defaultSwingHeight,
           customWaypointAngleThreshold,
           maxNumberOfSwingWaypoints,
           worldFrame,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public TwoWaypointSwingGenerator(String namePrefix,
                                    double minSwingHeight,
                                    double maxSwingHeight,
                                    double defaultSwingHeight,
                                    double customWaypointAngleThreshold,
                                    ReferenceFrame trajectoryFrame,
                                    YoRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix,
           minSwingHeight,
           maxSwingHeight,
           defaultSwingHeight,
           customWaypointAngleThreshold,
           Footstep.maxNumberOfSwingWaypoints,
           trajectoryFrame,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public TwoWaypointSwingGenerator(String namePrefix,
                                    double minSwingHeight,
                                    double maxSwingHeight,
                                    double defaultSwingHeight,
                                    double customWaypointAngleThreshold,
                                    int maxNumberOfSwingWaypoints,
                                    ReferenceFrame trajectoryFrame,
                                    YoRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.namePrefix = namePrefix;
      this.maxNumberOfSwingWaypoints = maxNumberOfSwingWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.trajectoryFrame = trajectoryFrame;
      waypointPositions = new RecyclingArrayList<>(maxNumberOfSwingWaypoints, this::createNewWaypoint);
      tempWaypointVelocity = new FrameVector3D(trajectoryFrame);

      stepTime = new YoDouble(namePrefix + "StepTime", registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);
      isDone = new YoBoolean(namePrefix + "IsDone", registry);
      isSteppingDown = new YoBoolean(namePrefix + "IsSteppingDown", registry);
      swingHeight = new YoDouble(namePrefix + "SwingHeight", registry);
      swingHeight.set(minSwingHeight);

      this.maxSwingHeight = new YoDouble(namePrefix + "MaxSwingHeight", registry);
      this.maxSwingHeight.set(maxSwingHeight);

      this.minSwingHeight = new YoDouble(namePrefix + "MinSwingHeight", registry);
      this.minSwingHeight.set(minSwingHeight);

      this.defaultSwingHeight = new YoDouble(namePrefix + "DefaultSwingHeight", registry);
      this.defaultSwingHeight.set(defaultSwingHeight);

      this.minDistanceToStance = new YoDouble(namePrefix + "MinDistanceToStance", registry);
      this.minDistanceToStance.set(Double.NEGATIVE_INFINITY);

      this.customWaypointAngleThreshold = new YoDouble(namePrefix + "CustomWaypointAngleThreshold", registry);
      this.customWaypointAngleThreshold.set(customWaypointAngleThreshold);

      stepUpFirstWaypointHeightFactor = new YoDouble(namePrefix + "StepUpFirstWaypointHeightFactor", registry);
      stepUpFirstWaypointHeightFactor.set(1.0);
      stepDownSecondWaypointHeightFactor = new YoDouble(namePrefix + "StepDownSecondWaypointHeightFactor", registry);
      stepDownSecondWaypointHeightFactor.set(1.0);

      for (int i = 0; i < defaultNumberOfWaypoints; i++)
         this.waypointProportions[i] = defaultWaypointProportions[i];

      trajectory = new PositionOptimizedTrajectoryGenerator(namePrefix,
                                                            registry,
                                                            yoGraphicsListRegistry,
                                                            maxTimeIterations,
                                                            maxNumberOfSwingWaypoints,
                                                            trajectoryFrame);

      if (yoGraphicsListRegistry != null)
         waypointViz = new BagOfBalls(maxNumberOfSwingWaypoints, 0.02, namePrefix + "Waypoints", YoAppearance.White(), registry, yoGraphicsListRegistry);
      else
         waypointViz = null;

      needToAdjustedSwingForSelfCollision = new YoBoolean(namePrefix + "AdjustedSwing", registry);
      crossOverStep = new YoBoolean(namePrefix + "CrossOverStep", registry);
   }

   @Override
   public void setStepTime(double stepTime)
   {
      this.stepTime.set(stepTime);
   }

   @Override
   public void setInitialConditions(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
   }

   public void setInitialConditionWeights(Tuple3DReadOnly initialPositionWeight, Tuple3DReadOnly initialVelocityWeight)
   {
      if (initialPositionWeight == null)
         this.initialPositionWeight.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      else
         this.initialPositionWeight.set(initialPositionWeight);
      if (initialVelocityWeight == null)
         this.initialVelocityWeight.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      else
         this.initialVelocityWeight.set(initialVelocityWeight);
   }

   @Override
   public void setFinalConditions(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
   }

   public void setFinalConditionWeights(Tuple3DReadOnly finalPositionWeight, Tuple3DReadOnly finalVelocityWeight)
   {
      if (finalPositionWeight == null)
         this.finalPositionWeight.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      else
         this.finalPositionWeight.set(finalPositionWeight);
      if (finalVelocityWeight == null)
         this.finalVelocityWeight.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      else
         this.finalVelocityWeight.set(finalVelocityWeight);
   }

   @Override
   public void setTrajectoryType(TrajectoryType trajectoryType, RecyclingArrayList<FramePoint3D> waypoints)
   {
      if (trajectoryType == TrajectoryType.CUSTOM && waypoints == null)
      {
         LogTools.warn("Received no waypoints but trajectory type is custom. Using default trajectory.");
         this.trajectoryType = TrajectoryType.DEFAULT;
      }
      else if (trajectoryType == TrajectoryType.CUSTOM && (waypoints.isEmpty() || waypoints.size() > maxNumberOfSwingWaypoints))
      {
         LogTools.warn("Received unexpected amount ({}) of waypoints. Using default trajectory.", waypoints.size());
         this.trajectoryType = TrajectoryType.DEFAULT;
      }
      else
      {
         this.trajectoryType = trajectoryType;
      }

      if (this.trajectoryType != TrajectoryType.CUSTOM)
         return;

      int initialWaypointIndex = computeStartingWaypointIndex(waypoints);
      waypointPositions.clear();
      for (int i = initialWaypointIndex; i < waypoints.size(); i++)
      {
         FramePoint3D waypoint = waypoints.get(i);
         FramePoint3D waypointToSet = waypointPositions.add();
         waypointToSet.setIncludingFrame(waypoint);
         waypointToSet.changeFrame(trajectoryFrame);
      }
   }

   private int computeStartingWaypointIndex(RecyclingArrayList<FramePoint3D> waypoints)
   {
      // Don't restrict if two or less waypoints
      if (waypoints.size() <= 2)
      {
         return 0;
      }

      initialPosition.changeFrame(ReferenceFrame.getWorldFrame());
      finalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double minimumDotProduct = Math.cos(customWaypointAngleThreshold.getValue());

      for (int i = 0; i < waypoints.size(); i++)
      {
         FramePoint3D waypoint = waypoints.get(i);
         FramePoint3D nextWaypoint = (i == waypoints.size() - 1) ? finalPosition : waypoints.get(i + 1);

         startToWaypointDisplacement.sub(waypoint, initialPosition);
         interWaypointDisplacement.sub(nextWaypoint, waypoint);

         startToWaypointDisplacement.normalize();
         interWaypointDisplacement.normalize();

         double dotProduct = startToWaypointDisplacement.dot(interWaypointDisplacement);
         if (dotProduct > minimumDotProduct)
         {
            return i;
         }
      }

      // If this check fails for all waypoints, just use the whole trajectory. It might have a jerky start but probably shouldn't be aborted.
      // If that happens customWaypointAngleThreshold should be increased.
      return 0;
   }

   @Override
   public void setSwingHeight(double swingHeight)
   {
      if (!isSwingHeightValid(swingHeight))
         this.swingHeight.set(defaultSwingHeight.getDoubleValue());
      else
         this.swingHeight.set(MathTools.clamp(swingHeight, minSwingHeight.getDoubleValue(), maxSwingHeight.getDoubleValue()));
   }

   public boolean isSwingHeightValid(double swingHeight)
   {
      return Double.isFinite(swingHeight) && swingHeight > 0.0;
   }

   @Override
   public void setStanceFootPosition(FramePoint3DReadOnly stanceFootPosition)
   {
      this.stanceFootPosition.setIncludingFrame(stanceFootPosition);
   }

   public void informDone()
   {
      trajectory.informDone();
   }

   @Override
   public void setWaypointProportions(double[] waypointProportions)
   {
      setWaypointProportions(waypointProportions[0], waypointProportions[1]);
   }

   public void setWaypointProportions(double waypointProportions0, double waypointProportions1)
   {
      this.waypointProportions[0] = waypointProportions0;
      this.waypointProportions[1] = waypointProportions1;
   }

   public void setObstacleClearanceWaypointHeightFactors(double stepUpFirstWaypointHeightFactor, double stepDownSecondWaypointHeightFactor)
   {
      this.stepUpFirstWaypointHeightFactor.set(stepUpFirstWaypointHeightFactor);
      this.stepDownSecondWaypointHeightFactor.set(stepDownSecondWaypointHeightFactor);
   }

   public void setInitialGroundHeightProvider(DoubleProvider initialGroundHeightProvider)
   {
      this.initialGroundHeightProvider = initialGroundHeightProvider;
   }

   public void setFinalGroundHeightProvider(DoubleProvider finalGroundHeightProvider)
   {
      this.finalGroundHeightProvider = finalGroundHeightProvider;
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      initialPosition.changeFrame(trajectoryFrame);
      finalPosition.changeFrame(trajectoryFrame);
      stanceFootPosition.changeFrame(trajectoryFrame);

      needToAdjustedSwingForSelfCollision.set(computeSwingAdjustment(initialPosition, finalPosition, stanceFootPosition, swingOffset));

      double initialGroundZ;
      if (initialGroundHeightProvider != null)
         initialGroundZ = initialGroundHeightProvider.getValue();
      else
         initialGroundZ = initialPosition.getZ();

      double finalGroundZ;
      if (finalGroundHeightProvider != null)
         finalGroundZ = finalGroundHeightProvider.getValue();
      else
         finalGroundZ = finalPosition.getZ();

      isSteppingDown.set(false);
      switch (trajectoryType)
      {
         case OBSTACLE_CLEARANCE:
         {
            waypointPositions.clear();

            for (int i = 0; i < defaultNumberOfWaypoints; i++)
            {
               waypointPositions.add();
               waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportions[i]);

               if (needToAdjustedSwingForSelfCollision.getBooleanValue())
               {
                  waypointPositions.get(i).add(swingOffset.getX(), swingOffset.getY(), 0.0);
               }
            }

            if (initialGroundZ < finalGroundZ)
            { // Stepping up
               double alpha = stepUpFirstWaypointHeightFactor.getValue();
               waypointPositions.get(0).setZ(swingHeight.getValue() + EuclidCoreTools.interpolate(initialGroundZ, finalGroundZ, alpha));
               waypointPositions.get(1).setZ(swingHeight.getValue() + finalGroundZ);
            }
            else
            { // Stepping down
               waypointPositions.get(0).setZ(swingHeight.getValue() + initialGroundZ);
               double alpha = stepDownSecondWaypointHeightFactor.getValue();
               waypointPositions.get(1).setZ(swingHeight.getValue() + EuclidCoreTools.interpolate(finalGroundZ, initialGroundZ, alpha));
               isSteppingDown.set(true);
            }

            break;
         }
         case DEFAULT:
         {
            waypointPositions.clear();
            for (int i = 0; i < defaultNumberOfWaypoints; i++)
            {
               waypointPositions.add();
               waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportions[i]);

               if (needToAdjustedSwingForSelfCollision.getBooleanValue())
               {
                  waypointPositions.get(i).add(swingOffset.getX(), swingOffset.getY(), 0.0);
               }
            }

            waypointPositions.get(0).setZ(initialGroundZ + swingHeight.getValue());
            waypointPositions.get(1).setZ(finalGroundZ + swingHeight.getValue());

            break;
         }
         case CUSTOM:
            break;
         default:
            throw new RuntimeException("Trajectory type not implemented");
      }

      if (trajectoryType != TrajectoryType.OBSTACLE_CLEARANCE)
      {
         double maxStepZ = Math.max(initialGroundZ, finalGroundZ);
         double maxWaypointZ;

         if (stanceFootPosition.containsNaN())
            maxWaypointZ = maxStepZ + maxSwingHeight.getDoubleValue();
         else
            maxWaypointZ = Math.max(stanceFootPosition.getZ() + maxSwingHeight.getDoubleValue(), maxStepZ + minSwingHeight.getDoubleValue());

         for (int i = 0; i < waypointPositions.size(); i++)
         {
            waypointPositions.get(i).setZ(Math.min(waypointPositions.get(i).getZ(), maxWaypointZ));
         }
      }

      if (trajectoryType == TrajectoryType.OBSTACLE_CLEARANCE || trajectoryType == TrajectoryType.DEFAULT)
      {
         if (initialGroundHeightProvider != null)
         {
            waypointPositions.get(0).setZ(Math.max(waypointPositions.get(0).getZ(), initialPosition.getZ() + minSwingHeight.getValue()));
         }

         if (finalGroundHeightProvider != null)
         {
            waypointPositions.get(1).setZ(Math.max(waypointPositions.get(1).getZ(), finalPosition.getZ() + minSwingHeight.getValue()));
         }
      }

      initialVelocityNoTimeDimension.setIncludingFrame(initialVelocity);
      finalVelocityNoTimeDimension.setIncludingFrame(finalVelocity);

      initialVelocityNoTimeDimension.scale(stepTime.getDoubleValue());
      finalVelocityNoTimeDimension.scale(stepTime.getDoubleValue());

      trajectory.setEndpointConditions(initialPosition, initialVelocityNoTimeDimension, finalPosition, finalVelocityNoTimeDimension);
      trajectory.setEndpointWeights(initialPositionWeight, initialVelocityWeight, finalPositionWeight, finalVelocityWeight);
      trajectory.setWaypoints(waypointPositions);
      trajectory.initialize();

      if (visualize)
         visualize();
      else
         hide();
   }

   public void setShouldVisualize(boolean visualize)
   {
      this.visualize = visualize;
      trajectory.setShouldVisualize(visualize);
   }

   private final FrameVector2D xyDistanceToStance = new FrameVector2D();
   private final Point2D stance2D = new Point2D();
   private final Point2D pointA2D = new Point2D();
   private final Point2D pointB2D = new Point2D();
   private final FramePoint2D pointAInStance = new FramePoint2D();
   private final FramePoint2D pointBInStance = new FramePoint2D();
   private final Point2D stanceProjection2D = new Point2D();
   private final Point2D swingIntersectionWithStanceY2D = new Point2D();

   /**
    * Given the start and end point of the swing as well as the position of the stance foot this method
    * will compute whether the nominal swing trajectory will be close to the stance foot. This is an
    * indication that self collision between swing and stance leg will occur. In that case a offset
    * vector is computed and packed that will contain a swing trajectory adjustment that will avoid
    * this.
    */
   private boolean computeSwingAdjustment(FramePoint3DReadOnly pointA, FramePoint3DReadOnly pointB, FramePoint3DReadOnly stance, Vector2DBasics offsetToPack)
   {
      if (swingSide == null || stanceZUpFrame == null)
      {
         offsetToPack.setToZero();
         return false;
      }

      pointA2D.set(pointA);
      pointB2D.set(pointB);
      stance2D.set(stance);
      EuclidGeometryTools.orthogonalProjectionOnLine2D(stance2D, pointA2D, pointB2D, stanceProjection2D);
      boolean smallAngleChange = !EuclidGeometryTools.isPoint2DOnLineSegment2D(stanceProjection2D, pointA2D, pointB2D);

      xyDistanceToStance.setToZero(trajectoryFrame);
      xyDistanceToStance.sub(stanceProjection2D, stance2D);
      xyDistanceToStance.changeFrame(stanceZUpFrame);

      // If the nominal trajectory intersects the negative Y axis of the sole frame for a swing with the left side the step is a cross over step.
      pointAInStance.setIncludingFrame(trajectoryFrame, pointA2D);
      pointBInStance.setIncludingFrame(trajectoryFrame, pointB2D);
      pointAInStance.changeFrame(stanceZUpFrame);
      pointBInStance.changeFrame(stanceZUpFrame);
      boolean trajectoryIntersectsY = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(0.0,
                                                                                                    0.0,
                                                                                                    0.0,
                                                                                                    1.0,
                                                                                                    pointAInStance.getX(),
                                                                                                    pointAInStance.getY(),
                                                                                                    pointBInStance.getX(),
                                                                                                    pointBInStance.getY(),
                                                                                                    swingIntersectionWithStanceY2D);

      double intersectionYTowardsSwingSide = swingSide.negateIfRightSide(swingIntersectionWithStanceY2D.getY());
      boolean crossOver = trajectoryIntersectsY && intersectionYTowardsSwingSide < 0.0;
      crossOverStep.set(crossOver);

      // Prevent adjusting on side steps or steps that do not change the angle between the feet much.
      if (!crossOver && smallAngleChange)
      {
         offsetToPack.setToZero();
         return false;
      }

      double distance;
      if (crossOver)
      {
         distance = minDistanceToStance.getDoubleValue() + xyDistanceToStance.norm();
         xyDistanceToStance.negate();
      }
      else
      {
         distance = minDistanceToStance.getDoubleValue() - xyDistanceToStance.norm();
      }

      if (distance < 0.0)
      {
         offsetToPack.setToZero();
         return false;
      }

      xyDistanceToStance.changeFrame(trajectoryFrame);
      xyDistanceToStance.normalize();
      xyDistanceToStance.scale(distance);
      offsetToPack.set(xyDistanceToStance);
      return true;
   }

   /**
    * Calling this method will enable a simple collision avoidance heuristic in the swing generator: if
    * a straight line in the xy plane from the start to the end of the swing is too close to the stance
    * position the trajectory waypoints will be adjusted. To activate this, additional information has
    * to be provided as arguments to this method.
    *
    * @param swingSide           the side of the robot that this swing trajectory will be executed on
    * @param stanceZUpFrame      the zup frame located at the stance foot sole
    * @param minDistanceToStance the minimum clearance that the swing should have from the stance foot
    *                            sole point in the xy plane
    */
   public void enableStanceCollisionAvoidance(RobotSide swingSide, ReferenceFrame stanceZUpFrame, double minDistanceToStance)
   {
      this.swingSide = swingSide;
      this.stanceZUpFrame = stanceZUpFrame;
      this.minDistanceToStance.set(minDistanceToStance);
   }

   private void visualize()
   {
      if (waypointViz == null)
         return;

      tempPoint3D.setToZero(worldFrame);
      waypointViz.reset();
      for (int i = 0; i < waypointPositions.size(); i++)
      {
         tempPoint3D.setMatchingFrame(waypointPositions.get(i));
         waypointViz.setBall(tempPoint3D, i);
      }
   }

   public void hide()
   {
      if (waypointViz == null)
         return;
      waypointViz.reset();
   }

   @Override
   public boolean doOptimizationUpdate()
   {
      return trajectory.doOptimizationUpdate();
   }

   @Override
   public void compute(double time)
   {
      double trajectoryTime = stepTime.getDoubleValue();
      isDone.set(time >= trajectoryTime);

      time = MathTools.clamp(time, 0.0, trajectoryTime);
      timeIntoStep.set(time);

      double percent = time / trajectoryTime;
      trajectory.compute(percent);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return trajectory.getPosition();
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      desiredVelocity.set(trajectory.getVelocity());
      desiredVelocity.scale(1.0 / stepTime.getDoubleValue());

      return desiredVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      desiredAcceleration.set(trajectory.getAcceleration());
      desiredAcceleration.scale(1.0 / stepTime.getDoubleValue());
      desiredAcceleration.scale(1.0 / stepTime.getDoubleValue());

      return desiredAcceleration;
   }

   @Override
   public void showVisualization()
   {
      trajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      waypointViz.hideAll();
      tempPoint3D.setToNaN();
      for (int i = 0; i < waypointPositions.size(); i++)
         waypointViz.setBall(tempPoint3D, i);
      trajectory.hideVisualization();
   }

   public static double[] getDefaultWaypointProportions()
   {
      return defaultWaypointProportions;
   }

   @Override
   public int getNumberOfWaypoints()
   {
      return waypointPositions.size();
   }

   @Override
   public void getWaypointData(int waypointIndex, FrameEuclideanTrajectoryPoint waypointDataToPack)
   {
      double waypointTime = getWaypointTime(waypointIndex);
      trajectory.getWaypointVelocity(waypointIndex, tempWaypointVelocity);
      tempWaypointVelocity.scale(1.0 / stepTime.getDoubleValue());

      waypointDataToPack.setToNaN(trajectoryFrame);
      waypointDataToPack.setTime(waypointTime);
      waypointDataToPack.getPosition().set(waypointPositions.get(waypointIndex));
      waypointDataToPack.getLinearVelocity().set(tempWaypointVelocity);
   }

   /**
    * Computes the initial position from the optimized splines.
    * <p>
    * This is only useful when the endpoint conditions have been set up with actual weights such that
    * the condition can differ from the given input in
    * {@link #setInitialConditions(FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    */
   public void getInitialPosition(FrameVector3DBasics initialPositionToPack)
   {
      trajectory.getInitialPosition(initialPositionToPack);
      initialPositionToPack.scale(1.0 / stepTime.getValue());
   }

   /**
    * Computes the initial velocity from the optimized splines.
    * <p>
    * This is only useful when the endpoint conditions have been set up with actual weights such that
    * the condition can differ from the given input in
    * {@link #setInitialConditions(FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    */
   public void getInitialVelocity(FrameVector3DBasics initialVelocityToPack)
   {
      trajectory.getInitialVelocity(initialVelocityToPack);
      initialVelocityToPack.scale(1.0 / stepTime.getValue());
   }

   /**
    * Computes the final position from the optimized splines.
    * <p>
    * This is only useful when the endpoint conditions have been set up with actual weights such that
    * the condition can differ from the given input in
    * {@link #setFinalConditions(FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    */
   public void getFinalPosition(FrameVector3DBasics finalPositionToPack)
   {
      trajectory.getFinalPosition(finalPositionToPack);
      finalPositionToPack.scale(1.0 / stepTime.getValue());
   }

   /**
    * Computes the final velocity from the optimized splines.
    * <p>
    * This is only useful when the endpoint conditions have been set up with actual weights such that
    * the condition can differ from the given input in
    * {@link #setFinalConditions(FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    */
   public void getFinalVelocity(FrameVector3DBasics finalVelocityToPack)
   {
      trajectory.getFinalVelocity(finalVelocityToPack);
      finalVelocityToPack.scale(1.0 / stepTime.getValue());
   }

   public FramePoint3DReadOnly getWaypoint(int index)
   {
      return waypointPositions.get(index);
   }

   public double computeAndGetMaxSpeed()
   {
      trajectory.computeMaxSpeed();
      return trajectory.getMaxSpeed() / stepTime.getDoubleValue();
   }

   public double getWaypointTime(int waypointIndex)
   {
      return stepTime.getDoubleValue() * trajectory.getWaypointTime(waypointIndex);
   }

   private FramePoint3D createNewWaypoint()
   {
      return new FramePoint3D(trajectoryFrame);
   }

   public EnumMap<Axis3D, ArrayList<YoPolynomial>> getSwingTrajectory()
   {
      return trajectory.getTrajectories();
   }

   public boolean isSteppingDown()
   {
      return isSteppingDown.getBooleanValue();
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(trajectory.getSCS2YoGraphics());
      if (waypointViz != null)
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPointcloud3D(namePrefix + "Waypoint",
                                                                            waypointViz.getPositions(),
                                                                            0.02,
                                                                            ColorDefinitions.White()));
      return group;
   }
}