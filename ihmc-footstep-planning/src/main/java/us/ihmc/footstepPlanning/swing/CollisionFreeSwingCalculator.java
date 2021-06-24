package us.ihmc.footstepPlanning.swing;

import gnu.trove.list.array.TDoubleArrayList;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.AdaptiveSwingTimingTools;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class CollisionFreeSwingCalculator
{
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static final double collisionGradientScale = 0.5;
   private static final double collisionDistanceEpsilon = 1e-4;
   private static final int numberOfKnotPoints = 12;
   private static final double downSamplePercentage = 0.3;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoBoolean collisionFound = new YoBoolean("collisionFound", registry);
   private final YoDouble maxCollisionDistance = new YoDouble("maxCollisionDistance", registry);
   private final boolean visualize;

   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final TickAndUpdatable tickAndUpdatable;

   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();

   private final List<FramePoint3D> defaultWaypoints = new ArrayList<>();
   private final List<FramePoint3D> modifiedWaypoints = new ArrayList<>();

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final List<Vector3D> collisionGradients = new ArrayList<>();
   private final List<Vector3D> convolvedGradients = new ArrayList<>();
   private final TDoubleArrayList convolutionWeights = new TDoubleArrayList();
   private final List<SwingKnotPoint> swingKnotPoints = new ArrayList<>();

   private PlanarRegionsList planarRegionsList;
   private final int footstepGraphicCapacity = 100;
   private final SideDependentList<FootstepVisualizer[]> footstepVisualizers = new SideDependentList<>();
   private final SideDependentList<MutableInt> footstepVisualizerIndices = new SideDependentList<>(side -> new MutableInt());

   private final YoFramePoseUsingYawPitchRoll soleFrameGraphicPose;
   private final YoGraphicPolygon footPolygonGraphic;

   private final FramePose3D tempPose = new FramePose3D();
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator;
   private final YoInteger stepIndex = new YoInteger("stepIndex", registry);
   private final YoEnum<PlanPhase> planPhase = new YoEnum<>("planPhase", registry, PlanPhase.class, true);
   private final BagOfBalls downSampledWaypoints;

   private enum PlanPhase
   {
      PLAN_NOMINAL_TRAJECTORY,
      PERFORM_COLLISION_CHECK,
      RECOMPUTE_FULL_TRAJECTORY,
      COMPUTE_DOWN_SAMPLED_TRAJECTORY
   }

   public CollisionFreeSwingCalculator(FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                       SwingPlannerParametersReadOnly swingPlannerParameters,
                                       WalkingControllerParameters walkingControllerParameters,
                                       SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this(footstepPlannerParameters, swingPlannerParameters, walkingControllerParameters, footPolygons, null, null, null);
   }

   public CollisionFreeSwingCalculator(FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                       SwingPlannerParametersReadOnly swingPlannerParameters,
                                       WalkingControllerParameters walkingControllerParameters,
                                       SideDependentList<ConvexPolygon2D> footPolygons,
                                       TickAndUpdatable tickAndUpdatable,
                                       YoGraphicsListRegistry graphicsListRegistry,
                                       YoRegistry parentRegistry)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.swingPlannerParameters = swingPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.tickAndUpdatable = tickAndUpdatable;
      this.graphicsListRegistry = graphicsListRegistry;
      this.positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator("", registry, graphicsListRegistry, 30, numberOfKnotPoints, ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfKnotPoints; i++)
      {
         double percentage = (i + 1.0) / (numberOfKnotPoints + 1.0);
         swingKnotPoints.add(new SwingKnotPoint(i, percentage, swingPlannerParameters, walkingControllerParameters, graphicsListRegistry, registry));
         collisionGradients.add(new Vector3D());
         convolvedGradients.add(new Vector3D());
      }

      visualize = parentRegistry != null;
      if (visualize)
      {
         YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());
         for (RobotSide robotSide : RobotSide.values())
         {
            FootstepVisualizer[] footstepVisualizerArray = new FootstepVisualizer[footstepGraphicCapacity];
            for (int i = 0; i < footstepVisualizerArray.length; i++)
            {
               footstepVisualizerArray[i] = new FootstepVisualizer(robotSide, footPolygons.get(robotSide), graphicsList);
            }

            footstepVisualizers.put(robotSide, footstepVisualizerArray);
         }

         soleFrameGraphicPose = new YoFramePoseUsingYawPitchRoll("soleGraphicPose", ReferenceFrame.getWorldFrame(), registry);
         YoFrameConvexPolygon2D yoFootPolygon = new YoFrameConvexPolygon2D("footPolygon", "", ReferenceFrame.getWorldFrame(), footPolygons.get(RobotSide.LEFT).getNumberOfVertices(), registry);
         yoFootPolygon.set(footPolygons.get(RobotSide.LEFT));
         footPolygonGraphic = new YoGraphicPolygon("soleGraphicPolygon", yoFootPolygon, soleFrameGraphicPose, 1.0, YoAppearance.RGBColorFromHex(0x386166));
         graphicsList.add(footPolygonGraphic);

         downSampledWaypoints = new BagOfBalls(3, 0.015, YoAppearance.White(), registry, graphicsListRegistry);

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
         parentRegistry.addChild(registry);
      }
      else
      {
         soleFrameGraphicPose = null;
         footPolygonGraphic = null;
         downSampledWaypoints = null;
      }
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void computeSwingTrajectories(SideDependentList<? extends Pose3DReadOnly> initialStanceFootPoses, FootstepPlan footstepPlan)
   {
      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return;
      }

      convolutionWeights.clear();
      for (int i = 0; i < swingKnotPoints.size(); i++)
      {
         swingKnotPoints.get(i).initializeBoxParameters();
         convolutionWeights.add(exp(swingPlannerParameters.getMotionCorrelationAlpha(), i));
      }

      initializeGraphics(initialStanceFootPoses, footstepPlan);
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         stepIndex.set(i);

         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         footstep.setTrajectoryType(TrajectoryType.DEFAULT);
         footstep.getCustomWaypointPositions().clear();

         RobotSide stepSide = footstep.getRobotSide();
         startOfSwingPose.set((i < 2 ? initialStanceFootPoses.get(stepSide) : footstepPlan.getFootstep(i - 2).getFootstepPose()));
         endOfSwingPose.set(footstep.getFootstepPose());

         positionTrajectoryGenerator.reset();
         defaultWaypoints.clear();

         double swingReach = startOfSwingPose.getPosition().distanceXY(endOfSwingPose.getPosition());
         double minXYTranslationToPlanSwing = swingPlannerParameters.getMinXYTranslationToPlanSwing();
         if (swingReach < minXYTranslationToPlanSwing)
         {
            continue;
         }

         /* keep the swing time a simple function of start/end, and set regardless of whether default trajectory is modified */
         double swingDuration = calculateSwingTime(startOfSwingPose.getPosition(), endOfSwingPose.getPosition());
         footstep.setSwingDuration(swingDuration);

         initializeKnotPoints();
         optimizeKnotPoints();

         if (!collisionFound.getValue())
         {
            continue;
         }

         footstep.setTrajectoryType(TrajectoryType.CUSTOM);
         recomputeTrajectory(footstep);
      }
   }

   private void initializeKnotPoints()
   {
      planPhase.set(PlanPhase.PLAN_NOMINAL_TRAJECTORY);

      // see TwoWaypointSwingGenerator.initialize() for trajectoryTypes DEFAULT and OBSTACLE_CLEARANCE
      double[] defaultWaypointProportions = new double[] {0.15, 0.85};
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      for (int i = 0; i < 2; i++)
      {
         FramePoint3D waypoint = new FramePoint3D();
         waypoint.interpolate(startOfSwingPose.getPosition(), endOfSwingPose.getPosition(), defaultWaypointProportions[i]);
         waypoint.addZ(defaultSwingHeightFromStanceFoot);
         defaultWaypoints.add(waypoint);
      }

      double zDifference = Math.abs(startOfSwingPose.getZ() - endOfSwingPose.getZ());
      boolean obstacleClearance = zDifference > walkingControllerParameters.getSwingTrajectoryParameters().getMinHeightDifferenceForStepUpOrDown();
      if (obstacleClearance)
      {
         double maxStepZ = Math.max(startOfSwingPose.getZ(), endOfSwingPose.getZ());
         for (int i = 0; i < 2; i++)
         {
            defaultWaypoints.get(i).setZ(maxStepZ + defaultSwingHeightFromStanceFoot);
         }
      }

      positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
      positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
      positionTrajectoryGenerator.setWaypoints(defaultWaypoints);
      positionTrajectoryGenerator.initialize();

      positionTrajectoryGenerator.setShouldVisualize(visualize);
      for (int i = 0; i < 30; i++)
      {
         boolean isDone = positionTrajectoryGenerator.doOptimizationUpdate();
         if (isDone)
            break;
      }

      for (int i = 0; i < numberOfKnotPoints; i++)
      {
         SwingKnotPoint knotPoint = swingKnotPoints.get(i);
         double percentage = knotPoint.getPercentage();
         positionTrajectoryGenerator.compute(percentage);
         tempPose.getPosition().set(positionTrajectoryGenerator.getPosition());
         tempPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), percentage);

         knotPoint.initialize(tempPose);
         knotPoint.setMaxDisplacement(interpolate(percentage,
                                                  swingPlannerParameters.getPercentageLowMaxDisplacement(),
                                                  swingPlannerParameters.getPercentageHighMaxDisplacement(),
                                                  swingPlannerParameters.getMaxDisplacementLow(),
                                                  swingPlannerParameters.getMaxDisplacementHigh()));
         knotPoint.initializeWaypointAdjustmentFrame(positionTrajectoryGenerator.getVelocity(), startOfSwingPose.getPosition(), endOfSwingPose.getPosition());
      }

      if (visualize)
      {
         downSampledWaypoints.reset();
         soleFrameGraphicPose.setToNaN();
         footPolygonGraphic.update();

         /* Show one collision box at a time */
         for (int boxToShow = 0; boxToShow < numberOfKnotPoints; boxToShow++)
         {
            swingKnotPoints.forEach(kp -> kp.updateGraphics(false));
            swingKnotPoints.get(boxToShow).updateGraphics(true);
            tickAndUpdatable.tickAndUpdate();
         }
      }
   }

   private void optimizeKnotPoints()
   {
      collisionFound.set(false);
      planPhase.set(PlanPhase.PERFORM_COLLISION_CHECK);
      int maxIterations = 30;

      for (int i = 0; i < maxIterations; i++)
      {
         maxCollisionDistance.set(0.0);
         boolean intersectionFound = false;

         for (int j = 0; j < numberOfKnotPoints; j++)
         {
            SwingKnotPoint knotPoint = swingKnotPoints.get(j);

            // collision gradient
            boolean collisionDetected = knotPoint.doCollisionCheck(collisionDetector, planarRegionsList);
            if (collisionDetected)
            {
               collisionFound.set(true);
               EuclidShape3DCollisionResult collisionResult = knotPoint.getCollisionResult();
               collisionGradients.get(j).sub(collisionResult.getPointOnB(), collisionResult.getPointOnA());
               collisionGradients.get(j).scale(collisionGradientScale);
               swingKnotPoints.get(j).project(collisionGradients.get(j));
               maxCollisionDistance.set(Math.max(maxCollisionDistance.getDoubleValue(), collisionResult.getDistance()));
               intersectionFound = true;
            }
            else
            {
               collisionGradients.get(j).setToZero();
               continue;
            }

            if (swingKnotPoints.get(j).getCollisionResult().areShapesColliding())
            {
               double scale = swingKnotPoints.get(j).computeMaximumDisplacementScale(collisionGradients.get(j));
               collisionGradients.get(j).scale(scale);
            }
         }

         for (int j = 0; j < numberOfKnotPoints; j++)
         {
            convolvedGradients.get(j).setToZero();
            for (int k = 0; k < numberOfKnotPoints; k++)
            {
               int indexDifference = Math.abs(j - k);
               double scale = convolutionWeights.get(indexDifference);
               scaleAdd(convolvedGradients.get(j), scale, collisionGradients.get(k));
            }

            swingKnotPoints.get(j).project(convolvedGradients.get(j));
            swingKnotPoints.get(j).shiftWaypoint(convolvedGradients.get(j));

            if (visualize)
            {
               swingKnotPoints.get(j).updateGraphics(swingKnotPoints.get(j).getCollisionResult().areShapesColliding());
            }
         }

         if (visualize)
         {
            tickAndUpdatable.tickAndUpdate();
         }

         if (!intersectionFound || maxCollisionDistance.getDoubleValue() < collisionDistanceEpsilon)
         {
            break;
         }
      }
   }

   /* exponent function assuming non-negative positive exponent */
   static double exp(double base, int exponent)
   {
      double value = 1.0;
      int i = 0;

      while (i < exponent)
      {
         value *= base;
         i++;
      }

      return value;
   }

   /*
    * Different from the Vector3DBasics.scaleAdd, which scales the mutated vector
    * a = a + alpha * b
    */
   static void scaleAdd(Vector3DBasics vectorA, double alpha, Vector3DReadOnly vectorB)
   {
      vectorA.addX(alpha * vectorB.getX());
      vectorA.addY(alpha * vectorB.getY());
      vectorA.addZ(alpha * vectorB.getZ());
   }

   /*
    * Trapezoid-shaped interpolation used for a few parameters to make sure the start/end of swing have smaller collision boxes and don't
    * move too much, while the middle can.
    */
   static double interpolate(double percentage, double percentageLow, double percentageHigh, double valueLow, double valueHigh)
   {
      double effectivePercentage = percentage < 0.5 ? percentage : 1.0 - percentage;
      double alpha = MathTools.clamp((effectivePercentage - percentageLow) / (percentageHigh - percentageLow), 0.0, 1.0);
      return EuclidCoreTools.interpolate(valueLow, valueHigh, alpha);
   }

   private void recomputeTrajectory(PlannedFootstep footstep)
   {
      planPhase.set(PlanPhase.RECOMPUTE_FULL_TRAJECTORY);
      modifiedWaypoints.clear();
      for (int i = 0; i < swingKnotPoints.size(); i++)
      {
         modifiedWaypoints.add(new FramePoint3D(swingKnotPoints.get(i).getOptimizedWaypoint().getPosition()));
      }

      /* Recompute and visualize modified trajectory */
      positionTrajectoryGenerator.reset();
      positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
      positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
      positionTrajectoryGenerator.setWaypoints(modifiedWaypoints);
      positionTrajectoryGenerator.initialize();

      positionTrajectoryGenerator.setShouldVisualize(visualize);
      for (int i = 0; i < 30; i++)
      {
         boolean isDone = positionTrajectoryGenerator.doOptimizationUpdate();
         if (isDone)
            break;
      }

      if (visualize)
      {
         for (int i = 0; i < numberOfKnotPoints; i++)
         {
            swingKnotPoints.get(i).updateGraphics(false);
         }

         soleFrameGraphicPose.setToNaN();
         footPolygonGraphic.update();
         tickAndUpdatable.tickAndUpdate();
      }

      /* Down sample to 3 waypoints */
      planPhase.set(PlanPhase.COMPUTE_DOWN_SAMPLED_TRAJECTORY);

      positionTrajectoryGenerator.compute(downSamplePercentage);
      footstep.getCustomWaypointPositions().add(new Point3D(positionTrajectoryGenerator.getPosition()));

      positionTrajectoryGenerator.compute(0.5);
      footstep.getCustomWaypointPositions().add(new Point3D(positionTrajectoryGenerator.getPosition()));

      positionTrajectoryGenerator.compute(1 - downSamplePercentage);
      footstep.getCustomWaypointPositions().add(new Point3D(positionTrajectoryGenerator.getPosition()));

      /* Recompute and visualize down-sampled trajectory */
      positionTrajectoryGenerator.reset();
      positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
      positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
      positionTrajectoryGenerator.setWaypoints(footstep.getCustomWaypointPositions().stream().map(p -> new FramePoint3D(ReferenceFrame.getWorldFrame(), p)).collect(Collectors.toList()));
      positionTrajectoryGenerator.initialize();
      positionTrajectoryGenerator.setShouldVisualize(visualize);

      for (int i = 0; i < 30; i++)
      {
         boolean isDone = positionTrajectoryGenerator.doOptimizationUpdate();
         if (isDone)
            break;
      }

      if (visualize)
      {
         for (int i = 0; i < numberOfKnotPoints; i++)
         {
            swingKnotPoints.get(i).hide();
         }

         soleFrameGraphicPose.setToNaN();
         footPolygonGraphic.update();
         footstep.getCustomWaypointPositions().forEach(downSampledWaypoints::setBall);
         tickAndUpdatable.tickAndUpdate();
      }
   }

   private void initializeGraphics(SideDependentList<? extends Pose3DReadOnly> initialStanceFootPoses, FootstepPlan footstepPlan)
   {
      if (!visualize)
      {
         return;
      }

      // hide all
      for (RobotSide side : RobotSide.values())
      {
         FootstepVisualizer[] footstepVisualizers = this.footstepVisualizers.get(side);
         for (int i = 0; i < footstepVisualizers.length; i++)
         {
            footstepVisualizers[i].hide();
         }

         footstepVisualizerIndices.get(side).setValue(0);
      }

      // render stance steps
      for (RobotSide side : RobotSide.values())
      {
         FootstepVisualizer footstepVisualizer = getNextFootstepVisualizer(side);
         footstepVisualizer.visualizeFootstep(new FramePose3D(initialStanceFootPoses.get(side)));
      }

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         FootstepVisualizer footstepVisualizer = getNextFootstepVisualizer(footstep.getRobotSide());
         footstepVisualizer.visualizeFootstep(footstep.getFootstepPose());
      }

      tickAndUpdatable.tickAndUpdate();
   }

   private double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double idealStepLength = footstepPlannerParameters.getIdealFootstepLength();
      double maxStepZ = footstepPlannerParameters.getMaxStepZ();

      return AdaptiveSwingTimingTools.calculateSwingTime(idealStepLength,
                                                         footstepPlannerParameters.getMaxSwingReach(),
                                                         maxStepZ,
                                                         swingPlannerParameters.getMinimumSwingTime(),
                                                         swingPlannerParameters.getMaximumSwingTime(),
                                                         startPosition,
                                                         endPosition);
   }

   private FootstepVisualizer getNextFootstepVisualizer(RobotSide robotSide)
   {
      int indexToGet = footstepVisualizerIndices.get(robotSide).getAndIncrement();
      FootstepVisualizer[] footstepVisualizer = this.footstepVisualizers.get(robotSide);

      if (indexToGet >= footstepVisualizer.length)
      {
         throw new RuntimeException("footstepGraphicCapacity is too low");
      }

      return footstepVisualizer[indexToGet];
   }

   private static final SideDependentList<AppearanceDefinition> footPolygonAppearances = new SideDependentList<>(YoAppearance.Purple(), YoAppearance.Green());
   private static SideDependentList<MutableInt> footGraphicIndices = new SideDependentList<>(side -> new MutableInt());

   private class FootstepVisualizer
   {
      private final YoFramePoseUsingYawPitchRoll soleFramePose;
      private final YoGraphicPolygon footPolygonViz;

      FootstepVisualizer(RobotSide robotSide, ConvexPolygon2D footPolygon, YoGraphicsList yoGraphicsList)
      {
         String namePrefix = robotSide.getLowerCaseName() + "Foot" + footGraphicIndices.get(robotSide).getAndIncrement();
         this.soleFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "graphicPolygon", ReferenceFrame.getWorldFrame(), registry);
         YoFrameConvexPolygon2D yoFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "yoPolygon", "", ReferenceFrame.getWorldFrame(), footPolygon.getNumberOfVertices(), registry);
         yoFootPolygon.set(footPolygon);
         footPolygonViz = new YoGraphicPolygon(namePrefix + "graphicPolygon", yoFootPolygon, soleFramePose, 1.0, footPolygonAppearances.get(robotSide));
         yoGraphicsList.add(footPolygonViz);
      }

      void visualizeFootstep(FramePose3DReadOnly footstepPose)
      {
         soleFramePose.set(footstepPose);
         footPolygonViz.update();
      }

      void hide()
      {
         soleFramePose.setToNaN();
         footPolygonViz.update();
      }
   }
}
