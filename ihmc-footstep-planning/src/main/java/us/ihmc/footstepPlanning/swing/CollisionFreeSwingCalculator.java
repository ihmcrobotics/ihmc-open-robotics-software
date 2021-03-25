package us.ihmc.footstepPlanning.swing;

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
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public class CollisionFreeSwingCalculator
{
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static final int numberOfCollisionChecks = 15;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final boolean visualize;
   private final YoEnum<SolverStep> solverStep = new YoEnum<>("solverStep", registry, SolverStep.class);

   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final TickAndUpdatable tickAndUpdatable;

   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();

   private final List<FramePoint3D> defaultWaypoints = new ArrayList<>();
   private final List<FramePoint3D> modifiedWapoints = new ArrayList<>();
   private final List<Double> modifiedWapointPercentages = new ArrayList<>();

   private final FramePose3D solePose = new FramePose3D();
   private final PoseReferenceFrame solePoseFrame = new PoseReferenceFrame("solePose", ReferenceFrame.getWorldFrame());
   private final Vector3D boxCenterInSoleFrame = new Vector3D();
   private final FramePose3D boxCenterPose = new FramePose3D();
   private final YoFramePoseUsingYawPitchRoll collisionBoxPose = new YoFramePoseUsingYawPitchRoll("collisionBoxPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicShape yoCollisionBoxGraphic;
   private final FrameBox3D collisionBox = new FrameBox3D();
   private final YoFrameVector3D shiftAmount = new YoFrameVector3D("shiftAmount", ReferenceFrame.getWorldFrame(), registry);

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final YoBoolean collisionDetected = new YoBoolean("collisionDetected", registry);

   private PlanarRegionsList planarRegionsList;
   private final int footstepGraphicCapacity = 100;
   private final SideDependentList<FootstepVisualizer[]> footstepVisualizers = new SideDependentList<>();
   private final SideDependentList<MutableInt> footstepVisualizerIndices = new SideDependentList<>(side -> new MutableInt());

   private final YoFramePoseUsingYawPitchRoll soleFrameGraphicPose;
   private final YoGraphicPolygon footPolygonGraphic;

   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator;

   private enum SolverStep
   {
      COMPUTING_DEFUALT_TRAJECTORY,
      FINDING_COLLIDING_POSES,
      RECOMPUTE_TRAJECTORY
   }

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

      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double boxBaseInSoleFrameX = 0.5 * (footForwardOffset - footBackwardOffset);
      boxCenterInSoleFrame.set(boxBaseInSoleFrameX, 0.0, 0.5 * swingPlannerParameters.getCollisionBoxHeight());

      updateBoxSize(0.0);
      Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
      AppearanceDefinition collisionBoxColor = YoAppearance.RGBColorFromHex(0x824e38);
      collisionBoxColor.setTransparency(0.5);
      collisionBoxGraphic.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), true, collisionBoxColor);
      yoCollisionBoxGraphic = new YoGraphicShape("collisionGraphic", collisionBoxGraphic, collisionBoxPose, 1.0);

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

         graphicsList.add(yoCollisionBoxGraphic);
         graphicsListRegistry.registerYoGraphicsList(graphicsList);
         parentRegistry.addChild(registry);
      }
      else
      {
         soleFrameGraphicPose = null;
         footPolygonGraphic = null;
      }
   }

   private void updateBoxSize(double collisionBoxExtraZ)
   {
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double boxSizeX = footForwardOffset + footBackwardOffset + 2.0 * swingPlannerParameters.getCollisionBoxExtraX();
      double boxSizeY = walkingControllerParameters.getSteppingParameters().getFootWidth() + 2.0 * swingPlannerParameters.getCollisionBoxExtraY();
      double boxSizeZ = swingPlannerParameters.getCollisionBoxHeight() + 2.0 * collisionBoxExtraZ;
      collisionBox.getSize().set(boxSizeX, boxSizeY, boxSizeZ);
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

      updateFootstepGraphics(initialStanceFootPoses, footstepPlan);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         RobotSide stepSide = footstep.getRobotSide();
         startOfSwingPose.set((i < 2 ? initialStanceFootPoses.get(stepSide) : footstepPlan.getFootstep(i - 2).getFootstepPose()));
         endOfSwingPose.set(footstep.getFootstepPose());

         solverStep.set(SolverStep.COMPUTING_DEFUALT_TRAJECTORY);
         computeDefaultTrajectory();

         solverStep.set(SolverStep.FINDING_COLLIDING_POSES);
         checkForCollisionsAlongDefaultTrajectory();

         solverStep.set(SolverStep.RECOMPUTE_TRAJECTORY);
         recomputeTrajectory();
      }
   }

   private void computeDefaultTrajectory()
   {
      positionTrajectoryGenerator.reset();
      defaultWaypoints.clear();

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

      if (visualize)
      {
         yoCollisionBoxGraphic.setPoseToNaN();
         soleFrameGraphicPose.setToNaN();
         footPolygonGraphic.update();
         tickAndUpdatable.tickAndUpdate();
      }
   }

   private void checkForCollisionsAlongDefaultTrajectory()
   {
      double minPercentageToCheck = swingPlannerParameters.getMinMaxCheckerPercentage();
      double maxPercentageToCheck = 1.0 - swingPlannerParameters.getMinMaxCheckerPercentage();
      double deltaPercentage = (1.0 - 2.0 * swingPlannerParameters.getMinMaxCheckerPercentage()) / (numberOfCollisionChecks - 1);

      for (int i = 0; i < numberOfCollisionChecks; i++)
      {
         double percentage = swingPlannerParameters.getMinMaxCheckerPercentage() + i * deltaPercentage;
         positionTrajectoryGenerator.compute(percentage);

         double alphaExtraHeight = computeAlphaExtraHeight(percentage, minPercentageToCheck, maxPercentageToCheck);
         updateBoxSize(alphaExtraHeight * swingPlannerParameters.getCollisionBoxExtraZ());

         FramePoint3DReadOnly trajectoryPosition = positionTrajectoryGenerator.getPosition();
         solePose.getPosition().set(trajectoryPosition);
         solePose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), percentage);
         solePoseFrame.setPoseAndUpdate(solePose);

         boxCenterPose.setToZero(solePoseFrame);
         boxCenterPose.getPosition().set(boxCenterInSoleFrame);
         boxCenterPose.changeFrame(ReferenceFrame.getWorldFrame());
         collisionBox.getPose().set(boxCenterPose);

         if (visualize)
         {
            // render pre-shifted collision box
            yoCollisionBoxGraphic.setPose(boxCenterPose);
            tickAndUpdatable.tickAndUpdate();
         }

         collisionDetected.set(checkForCollision());
         if (collisionDetected.getBooleanValue())
         {
            solePose.getPosition().add(shiftAmount);
            modifiedWapoints.add(new FramePoint3D(solePose.getPosition()));
            modifiedWapointPercentages.add(percentage);

            solePoseFrame.setPoseAndUpdate(solePose);

            boxCenterPose.setToZero(solePoseFrame);
            boxCenterPose.getPosition().set(boxCenterInSoleFrame);
            boxCenterPose.changeFrame(ReferenceFrame.getWorldFrame());
            collisionBox.getPose().set(boxCenterPose);
         }
         else
         {
            continue;
         }

         if (visualize)
         {
            // render shifted collision box
            yoCollisionBoxGraphic.setPose(boxCenterPose);
            tickAndUpdatable.tickAndUpdate();
         }
      }
   }

   /**
    * Extruding the collision box vertically up/down is tricky since it can easily collide with the ground.
    * To avoid this the extrusion amount starts at 0.0, ramps up to a max value, then ramps back down.
    */
   private double computeAlphaExtraHeight(double percentage, double minPercentageToCheck, double maxPercentageToCheck)
   {
      double minPForMaxHeight = swingPlannerParameters.getMinMaxHeightInterpolationPercentage();
      double maxPForMaxHeight = 1.0 - swingPlannerParameters.getMinMaxHeightInterpolationPercentage();
      if (MathTools.intervalContains(percentage, minPForMaxHeight, maxPForMaxHeight))
      {
         return 1.0;
      }
      else if (percentage < 0.5)
      {
         return EuclidCoreTools.interpolate(0.0, 1.0, (percentage - minPercentageToCheck) / (minPForMaxHeight - minPercentageToCheck));
      }
      else
      {
         return EuclidCoreTools.interpolate(1.0, 0.0, (percentage - maxPForMaxHeight) / (maxPercentageToCheck - maxPForMaxHeight));
      }
   }

   private void recomputeTrajectory()
   {
      if (modifiedWapointPercentages.get(0) > swingPlannerParameters.getMinMaxPercentageToKeepDefaultWaypoint())
      {
         modifiedWapoints.add(new FramePoint3D(defaultWaypoints.get(0)));
      }
      if (modifiedWapointPercentages.get(modifiedWapoints.size() - 1) < 1.0 - swingPlannerParameters.getMinMaxPercentageToKeepDefaultWaypoint())
      {
         modifiedWapoints.add(new FramePoint3D(defaultWaypoints.get(1)));
      }

      positionTrajectoryGenerator.reset();
      positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
      positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
      positionTrajectoryGenerator.setWaypoints(modifiedWapoints);
      positionTrajectoryGenerator.initialize();

      positionTrajectoryGenerator.setShouldVisualize(visualize);
      for (int i = 0; i < 30; i++)
      {
         positionTrajectoryGenerator.doOptimizationUpdate();
      }

      if (visualize)
      {
         yoCollisionBoxGraphic.setPoseToNaN();
         tickAndUpdatable.tickAndUpdate();
      }

      if (visualize)
      {
         yoCollisionBoxGraphic.setPoseToNaN();
         double deltaPercentage = 1.0 / (numberOfCollisionChecks - 1);

         for (int i = 0; i < numberOfCollisionChecks; i++)
         {
            double percentage = i * deltaPercentage;
            positionTrajectoryGenerator.compute(percentage);

            FramePoint3DReadOnly trajectoryPosition = positionTrajectoryGenerator.getPosition();
            solePose.getPosition().set(trajectoryPosition);
            solePose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), percentage);
            soleFrameGraphicPose.set(solePose);
            footPolygonGraphic.update();

            tickAndUpdatable.tickAndUpdate();
         }
      }
   }

   private boolean checkForCollision()
   {
      double maxDistance = -1.0;
      EuclidShape3DCollisionResult maxPenetrationCollisionResult = null;

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);

         if (planarRegion.getBoundingBox3dInWorld().intersectsExclusive(collisionBox.getBoundingBox()))
         {
            EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
            collisionDetector.evaluateCollision(collisionBox, planarRegion, collisionResult);
            double distance = Math.abs(collisionResult.getDistance());

            if (collisionResult.areShapesColliding() && (maxPenetrationCollisionResult == null || collisionResult.getSignedDistance() > maxDistance))
            {
               maxDistance = distance;
               maxPenetrationCollisionResult = collisionResult;
            }
         }
      }

      if (maxPenetrationCollisionResult == null)
      {
         return false;
      }
      else
      {
         shiftAmount.set(maxPenetrationCollisionResult.getPointOnB());
         shiftAmount.sub(maxPenetrationCollisionResult.getPointOnA());
         return true;
      }
   }

   private void updateFootstepGraphics(SideDependentList<? extends Pose3DReadOnly> initialStanceFootPoses, FootstepPlan footstepPlan)
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

      if (visualize)
      {
         tickAndUpdatable.tickAndUpdate();
      }
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
