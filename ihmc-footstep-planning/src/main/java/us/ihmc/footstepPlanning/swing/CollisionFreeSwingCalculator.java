package us.ihmc.footstepPlanning.swing;

import gnu.trove.list.array.TDoubleArrayList;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class CollisionFreeSwingCalculator
{
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static final int numberOfKnotPoints = 12;
   private static final double motionCorrelationAlpha = 0.5;
   private static final double collisionGradientScale = 0.4;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger iterations = new YoInteger("iterations", registry);
   private final YoDouble intersectionDistance = new YoDouble("intersectionDistance", registry);
   private final YoDouble smoothnessCost = new YoDouble("smoothnessCost", registry);
   private final boolean visualize;

   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final TickAndUpdatable tickAndUpdatable;

   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();

   private final List<FramePoint3D> defaultWaypoints = new ArrayList<>();

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final Vector3D collisionGradient = new Vector3D();
   private final Vector3D scaledGradient = new Vector3D();

   private final List<SwingKnotPoint> swingKnotPoints = new ArrayList<>();
   private final List<FramePoint3DReadOnly> fullTrajectoryPoints = new ArrayList<>();
   private final List<Vector3D> estimatedVelocities = new ArrayList<>();
   private final TDoubleArrayList estimatedDistancesSquared = new TDoubleArrayList();
   private final TDoubleArrayList estimatedAccelerationsSquared = new TDoubleArrayList();

   private PlanarRegionsList planarRegionsList;
   private final int footstepGraphicCapacity = 100;
   private final SideDependentList<FootstepVisualizer[]> footstepVisualizers = new SideDependentList<>();
   private final SideDependentList<MutableInt> footstepVisualizerIndices = new SideDependentList<>(side -> new MutableInt());

   private final YoFramePoseUsingYawPitchRoll soleFrameGraphicPose;
   private final YoGraphicPolygon footPolygonGraphic;

   private final FramePose3D tempPose = new FramePose3D();
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator;

   public CollisionFreeSwingCalculator(SwingPlannerParametersReadOnly swingPlannerParameters,
                                       WalkingControllerParameters walkingControllerParameters,
                                       SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this(swingPlannerParameters, walkingControllerParameters, footPolygons, null, null, null);
   }

   public CollisionFreeSwingCalculator(SwingPlannerParametersReadOnly swingPlannerParameters,
                                       WalkingControllerParameters walkingControllerParameters,
                                       SideDependentList<ConvexPolygon2D> footPolygons,
                                       TickAndUpdatable tickAndUpdatable,
                                       YoGraphicsListRegistry graphicsListRegistry,
                                       YoRegistry parentRegistry)
   {
      this.swingPlannerParameters = swingPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.tickAndUpdatable = tickAndUpdatable;
      this.positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator("", registry, graphicsListRegistry, 200, 100, ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfKnotPoints; i++)
      {
         double pMinMax = swingPlannerParameters.getMinMaxCheckerPercentage();
         double percentage = pMinMax + (1.0 - 2.0 * pMinMax) * i / (numberOfKnotPoints - 1);
         swingKnotPoints.add(new SwingKnotPoint(i, percentage, swingPlannerParameters, walkingControllerParameters, graphicsListRegistry, registry));
      }

      while (estimatedVelocities.size() < numberOfKnotPoints + 2)
      {
         estimatedVelocities.add(new Vector3D());
      }

      estimatedDistancesSquared.ensureCapacity(numberOfKnotPoints + 1);
      estimatedAccelerationsSquared.ensureCapacity(numberOfKnotPoints + 1);

      fullTrajectoryPoints.add(startOfSwingPose.getPosition());
      for (int i = 0; i < numberOfKnotPoints; i++)
      {
         fullTrajectoryPoints.add(swingKnotPoints.get(i).getCurrentWaypoint().getPosition());
      }
      fullTrajectoryPoints.add(endOfSwingPose.getPosition());

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

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
         parentRegistry.addChild(registry);
      }
      else
      {
         soleFrameGraphicPose = null;
         footPolygonGraphic = null;
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

      initializeGraphics(initialStanceFootPoses, footstepPlan);
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         RobotSide stepSide = footstep.getRobotSide();
         startOfSwingPose.set((i < 2 ? initialStanceFootPoses.get(stepSide) : footstepPlan.getFootstep(i - 2).getFootstepPose()));
         endOfSwingPose.set(footstep.getFootstepPose());

         positionTrajectoryGenerator.reset();
         defaultWaypoints.clear();

         initializeKnotPoints();
         optimizeTrajectory();
      }
   }

   private void initializeKnotPoints()
   {
      // see TwoWaypointSwingGenerator.initialize() for trajectoryType DEFAULT
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
         positionTrajectoryGenerator.doOptimizationUpdate();
      }

      for (int i = 0; i < numberOfKnotPoints; i++)
      {
         SwingKnotPoint knotPoint = swingKnotPoints.get(i);
         double percentage = knotPoint.getPercentage();
         positionTrajectoryGenerator.compute(percentage);
         tempPose.getPosition().set(positionTrajectoryGenerator.getPosition());
         tempPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), percentage);

         knotPoint.initialize(tempPose);
         knotPoint.setMaxDisplacement(computeMaxDisplacement(percentage));
      }

      if (visualize)
      {
         for (int i = 0; i < numberOfKnotPoints; i++)
         {
            swingKnotPoints.get(i).updateGraphics();
         }

         soleFrameGraphicPose.setToNaN();
         footPolygonGraphic.update();
         tickAndUpdatable.tickAndUpdate();
      }
   }

   private double computeMaxDisplacement(double percentage)
   {
      double minPercentage = swingPlannerParameters.getMinMaxCheckerPercentage();
      double displacementInterpolationCutoff = swingPlannerParameters.getDisplacementInterpolationCutoff();

      double effectivePercentage = percentage < 0.5 ? percentage : 1.0 - percentage;
      double alpha = MathTools.clamp((effectivePercentage - minPercentage) / (displacementInterpolationCutoff - minPercentage), 0.0, 1.0);
      return EuclidCoreTools.interpolate(swingPlannerParameters.getMaxDisplacementLow(), swingPlannerParameters.getMaxDisplacementHigh(), alpha);
   }

   private void optimizeTrajectory()
   {
      iterations.set(0);

      int maxIterations = 30;
      for (int i = 0; i < maxIterations; i++)
      {
         iterations.increment();
         intersectionDistance.set(0.0);

         for (int j = 0; j < numberOfKnotPoints; j++)
         {
            SwingKnotPoint knotPoint = swingKnotPoints.get(j);
//            totalGradient.setToZero();

            // collision gradient
            boolean collisionDetected = knotPoint.doCollisionCheck(collisionDetector, planarRegionsList);
            if (collisionDetected)
            {
               EuclidShape3DCollisionResult collisionResult = knotPoint.getCollisionResult();
               collisionGradient.sub(collisionResult.getPointOnB(), collisionResult.getPointOnA());
               collisionGradient.scale(collisionGradientScale);
               intersectionDistance.set(Math.max(intersectionDistance.getDoubleValue(), collisionResult.getDistance()));
            }
            else
            {
               continue;
            }

//            // smoothness gradient
//            smoothnessCost.set(computeSmoothnessCost());
//            Point3D currentPosition = new Point3D(knotPoint.getCurrentWaypoint().getPosition());
//
//            for (int k = 0; k < 3; k++)
//            {
//               double gradientEpsilon = 1e-7;
//               knotPoint.getCurrentWaypoint().getPosition().setToZero();
//               knotPoint.getCurrentWaypoint().getPosition().setElement(k, gradientEpsilon);
//               knotPoint.getCurrentWaypoint().getPosition().add(currentPosition);
//               double fxi = computeSmoothnessCost();
//
//               smoothnessGradient.setElement(k, -1e-5 * (fxi - smoothnessCost.getDoubleValue()) / gradientEpsilon);
//            }

            for (int k = 0; k < numberOfKnotPoints; k++)
            {
               int indexDifference = Math.abs(k - j);
               scaledGradient.set(collisionGradient);
               scaledGradient.scale(exp(motionCorrelationAlpha, indexDifference));
               swingKnotPoints.get(k).shiftWaypoint(scaledGradient);
            }

            if (visualize)
            {
               knotPoint.updateGraphics();
            }
         }

         if (visualize)
         {
            tickAndUpdatable.tickAndUpdate();
         }
      }
   }

   /* exponent function assuming non-negative positive exponent */
   private static double exp(double base, int exponent)
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

   private final Vector3D estimatedAcceleration = new Vector3D();

   private double computeSmoothnessCost()
   {
      estimatedVelocities.get(0).sub(fullTrajectoryPoints.get(1), fullTrajectoryPoints.get(0));
      estimatedVelocities.get(numberOfKnotPoints + 1).sub(fullTrajectoryPoints.get(numberOfKnotPoints + 1), fullTrajectoryPoints.get(numberOfKnotPoints));

      for (int i = 1; i < numberOfKnotPoints + 1; i++)
      {
         estimatedVelocities.get(i).sub(fullTrajectoryPoints.get(i + 1), fullTrajectoryPoints.get(i - 1));
      }

      estimatedDistancesSquared.clear();
      for (int i = 0; i < numberOfKnotPoints + 1; i++)
      {
         estimatedDistancesSquared.add(fullTrajectoryPoints.get(i).distanceSquared(fullTrajectoryPoints.get(i + 1)));
      }

      double cost = 0.0;

      for (int i = 0; i < numberOfKnotPoints + 1; i++)
      {
         estimatedAcceleration.sub(estimatedVelocities.get(i + 1), estimatedVelocities.get(i));
         double estimatedAccelerationSquared = estimatedAcceleration.lengthSquared();
         cost += estimatedAccelerationSquared / estimatedDistancesSquared.get(i);
      }

      return cost;
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
