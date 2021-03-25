package us.ihmc.footstepPlanning.swing;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
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
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class CollisionFreeSwingCalculator
{
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static final int numberOfCollisionChecks = 15;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final boolean visualize;

   private final int maxWaypoints = 5;
   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final TickAndUpdatable tickAndUpdatable;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();

   private final List<FramePoint3D> waypointPositions = new ArrayList<>();
   private final List<SE3TrajectoryPoint> waypointPoses = new ArrayList<>();

   private final FramePose3D solePose = new FramePose3D();
   private final PoseReferenceFrame solePoseFrame = new PoseReferenceFrame("solePose", ReferenceFrame.getWorldFrame());
   private final Vector3D boxBaseInSoleFrame = new Vector3D();
   private final FramePose3D boxBasePose = new FramePose3D();
   private final YoFramePoseUsingYawPitchRoll collisionBoxPose = new YoFramePoseUsingYawPitchRoll("collisionBoxPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicShape yoCollisionBoxGraphic;

   private PlanarRegionsList planarRegionsList;
   private final int footstepGraphicCapacity = 100;
   private final SideDependentList<FootstepVisualizer[]> footstepVisualizers = new SideDependentList<>();
   private final SideDependentList<MutableInt> footstepVisualizerIndices = new SideDependentList<>(side -> new MutableInt());

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
      this.footPolygons = footPolygons;
      this.tickAndUpdatable = tickAndUpdatable;
      this.positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator("", registry, graphicsListRegistry, maxWaypoints, maxWaypoints, ReferenceFrame.getWorldFrame());

      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double boxBaseInSoleFrameX = 0.5 * (footForwardOffset - footBackwardOffset);
      boxBaseInSoleFrame.set(boxBaseInSoleFrameX, 0.0, 0.0);

      double boxSizeX = footForwardOffset + footBackwardOffset + 2.0 * swingPlannerParameters.getCollisionBoxExtraX();
      double boxSizeY = walkingControllerParameters.getSteppingParameters().getFootWidth() + 2.0 * swingPlannerParameters.getCollisionBoxExtraY();
      double boxSizeZ = swingPlannerParameters.getCollisionBoxHeight() + 2.0 * swingPlannerParameters.getCollisionBoxExtraZ();
      Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
      collisionBoxGraphic.translate(0.0, 0.0, 0.5 * boxSizeZ);
      collisionBoxGraphic.addCube(boxSizeX, boxSizeY, boxSizeZ, true, YoAppearance.RGBColorFromHex(0x824e38));
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

         graphicsList.add(yoCollisionBoxGraphic);
         graphicsListRegistry.registerYoGraphicsList(graphicsList);
         parentRegistry.addChild(registry);
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

      updateFootstepGraphics(initialStanceFootPoses, footstepPlan);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         RobotSide stepSide = footstep.getRobotSide();
         startOfSwingPose.set((i < 2 ? initialStanceFootPoses.get(stepSide) : footstepPlan.getFootstep(i - 2).getFootstepPose()));
         endOfSwingPose.set(footstep.getFootstepPose());

         computeDefaultTrajectory();
         checkForCollisions();
      }
   }

   private void computeDefaultTrajectory()
   {
      waypointPositions.clear();

      // see TwoWaypointSwingGenerator.initialize() for trajectoryType DEFAULT
      double[] defaultWaypointProportions = new double[] {0.15, 0.85};
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      for (int i = 0; i < 2; i++)
      {
         FramePoint3D waypoint = new FramePoint3D();
         waypoint.interpolate(startOfSwingPose.getPosition(), endOfSwingPose.getPosition(), defaultWaypointProportions[i]);
         waypoint.addZ(defaultSwingHeightFromStanceFoot);
         waypointPositions.add(waypoint);
      }

      double zDifference = Math.abs(startOfSwingPose.getZ() - endOfSwingPose.getZ());
      boolean obstacleClearance = zDifference > walkingControllerParameters.getSwingTrajectoryParameters().getMinHeightDifferenceForStepUpOrDown();
      if (obstacleClearance)
      {
         double maxStepZ = Math.max(startOfSwingPose.getZ(), endOfSwingPose.getZ());
         for (int i = 0; i < 2; i++)
         {
            waypointPositions.get(i).setZ(maxStepZ + defaultSwingHeightFromStanceFoot);
         }
      }

      positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
      positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
      positionTrajectoryGenerator.setWaypoints(waypointPositions);
      positionTrajectoryGenerator.initialize();

      positionTrajectoryGenerator.setShouldVisualize(visualize);
      for (int i = 0; i < 30; i++)
      {
         positionTrajectoryGenerator.doOptimizationUpdate();
         if (visualize)
            tickAndUpdatable.tickAndUpdate();
      }
   }

   private void checkForCollisions()
   {
      double deltaPercentage = (1.0 - 2.0 * swingPlannerParameters.getMinMaxCheckerPercentage()) / (numberOfCollisionChecks - 1);
      for (int i = 0; i < numberOfCollisionChecks; i++)
      {
         double percentage = swingPlannerParameters.getMinMaxCheckerPercentage() + i * deltaPercentage;
         positionTrajectoryGenerator.compute(percentage);

         FramePoint3DReadOnly trajectoryPosition = positionTrajectoryGenerator.getPosition();
         solePose.getPosition().set(trajectoryPosition);
         solePose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), percentage);
         solePoseFrame.setPoseAndUpdate(solePose);

         boxBasePose.setToZero(solePoseFrame);
         boxBasePose.getPosition().set(boxBaseInSoleFrame);
         boxBasePose.changeFrame(ReferenceFrame.getWorldFrame());

         if (visualize)
         {
            yoCollisionBoxGraphic.setPose(boxBasePose);
            tickAndUpdatable.tickAndUpdate();
         }
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
      private final YoFrameConvexPolygon2D yoFootPolygon;
      private final YoGraphicPolygon footPolygonViz;

      FootstepVisualizer(RobotSide robotSide, ConvexPolygon2D footPolygon, YoGraphicsList yoGraphicsList)
      {
         String namePrefix = robotSide.getLowerCaseName() + "Foot" + footGraphicIndices.get(robotSide).getAndIncrement();
         this.soleFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "graphicPolygon", ReferenceFrame.getWorldFrame(), registry);
         this.yoFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "yoPolygon", "", ReferenceFrame.getWorldFrame(), footPolygon.getNumberOfVertices(), registry);
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
