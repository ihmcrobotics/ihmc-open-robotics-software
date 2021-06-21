package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.AdaptiveSwingTimingTools;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class CollisionFreeSwingCalculator
{
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static final double[] defaultWaypointProportions = new double[] {0.15, 0.85};

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicCoordinateSystem startOfSwingPoseGraphic;
   private final YoGraphicCoordinateSystem endOfSwingPoseGraphic;
   private final YoGraphicPolygon footstepGraphic;
   private final YoGraphicShape startOfSwingToeCollisionGraphic;
   private final YoGraphicShape endOfSwingHeelCollisionGraphic;
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private PlanarRegionsList planarRegionsList;

   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D midSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();

   private final List<FramePoint3D> nominalTwoWaypoints = new ArrayList<>();
   private final List<Point3D> modifiedWaypoints = new ArrayList<>();

   private final Box3D startOfSwingToeCollisionBox = new Box3D();
   private final Box3D midSwingCollisionBox = new Box3D();
   private final Box3D endOfSwingHeelCollisionBox = new Box3D();

   public CollisionFreeSwingCalculator(SwingPlannerParametersReadOnly swingPlannerParameters,
                                            FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                            WalkingControllerParameters walkingControllerParameters)
   {
      this(swingPlannerParameters, footstepPlannerParameters, walkingControllerParameters, null, null, null);
   }

   public CollisionFreeSwingCalculator(SwingPlannerParametersReadOnly swingPlannerParameters,
                                            FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                            WalkingControllerParameters walkingControllerParameters,
                                            TickAndUpdatable tickAndUpdatable,
                                            YoGraphicsListRegistry graphicsListRegistry,
                                            YoRegistry parentRegistry)
   {
      this.swingPlannerParameters = swingPlannerParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator("", registry, graphicsListRegistry, 30, 2, ReferenceFrame.getWorldFrame());
      this.visualize = parentRegistry != null;

      if (tickAndUpdatable != null)
      {
         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);

         String listName = getClass().getSimpleName();

         startOfSwingPoseGraphic = new YoGraphicCoordinateSystem("startOfSwing", "SwingCalc", registry, false, 0.1);
         endOfSwingPoseGraphic = new YoGraphicCoordinateSystem("endOfSwing", "SwingCalc", registry, false, 0.1);
         footstepGraphic = new YoGraphicPolygon("footstep", 4, registry, false, 1.0, YoAppearance.Red());

         Graphics3DObject toeHeelCollisionBox = new Graphics3DObject();
         toeHeelCollisionBox.addCube(swingPlannerParameters.getFootStubClearance(),
                                     walkingControllerParameters.getSteppingParameters().getFootWidth(),
                                     swingPlannerParameters.getCollisionBoxHeight(),
                                     true,
                                     YoAppearance.Orange());

         startOfSwingToeCollisionGraphic = new YoGraphicShape("startOfSwingToe", toeHeelCollisionBox, "startOfSwingToe", "", registry, 1.0, YoAppearance.Orange());
         endOfSwingHeelCollisionGraphic = new YoGraphicShape("endOfSwingHeel", toeHeelCollisionBox, "endOfSwingHeel", "", registry, 1.0, YoAppearance.Orange());

         graphicsListRegistry.registerYoGraphic(listName, startOfSwingPoseGraphic);
         graphicsListRegistry.registerYoGraphic(listName, endOfSwingPoseGraphic);
         graphicsListRegistry.registerYoGraphic(listName, footstepGraphic);
         graphicsListRegistry.registerYoGraphic(listName, startOfSwingToeCollisionGraphic);
         graphicsListRegistry.registerYoGraphic(listName, endOfSwingHeelCollisionGraphic);
      }
      else
      {
         this.tickAndUpdatable = null;
         startOfSwingPoseGraphic = null;
         endOfSwingPoseGraphic = null;
         footstepGraphic = null;
         startOfSwingToeCollisionGraphic = null;
         endOfSwingHeelCollisionGraphic = null;
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

      // TODO initialize graphics here

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         footstep.getCustomWaypointPositions().clear();

         RobotSide stepSide = footstep.getRobotSide();
         startOfSwingPose.set((i < 2 ? initialStanceFootPoses.get(stepSide) : footstepPlan.getFootstep(i - 2).getFootstepPose()));
         endOfSwingPose.set(footstep.getFootstepPose());

         positionTrajectoryGenerator.reset();

         /* keep the swing time a simple function of start/end, and set regardless of whether default trajectory is modified */
         double swingDuration = calculateSwingTime(startOfSwingPose.getPosition(), endOfSwingPose.getPosition());
         footstep.setSwingDuration(swingDuration);

         checkForFootCollision(startOfSwingPose, footstep);
         for (int j = 0; j < modifiedWaypoints.size(); j++)
         {
            footstep.getCustomWaypointPositions().add(new Point3D(modifiedWaypoints.get(i)));
         }

         if (tickAndUpdatable != null)
         {
            updateGraphics(footstep);
         }
      }
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

   private void checkForFootCollision(Pose3DReadOnly startPose, PlannedFootstep step)
   {
      Pose3D endPose = new Pose3D(step.getFootstepPose());

      /* Check toe collision at start-of-swing and heel collision at end-of-swing */
      setCollisionBoxPose(startOfSwingToeCollisionBox, startPose, FootCollisionCheckType.TOE);
      setCollisionBoxPose(endOfSwingHeelCollisionBox, endPose, FootCollisionCheckType.HEEL);

      boolean startOfSwingToeCollision = collisionDetected(startOfSwingToeCollisionBox);
      boolean endOfSwingHeelCollision = collisionDetected(endOfSwingHeelCollisionBox);

      /* Check mid-swing collision. Need to compute nominal swing trajectory for this */
      computeNominalSwingTrajectory();

      double midSwingPercentage = 0.5;
      positionTrajectoryGenerator.compute(midSwingPercentage);
      midSwingPose.getPosition().set(positionTrajectoryGenerator.getPosition());
      midSwingPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), 0.5);
      midSwingCollisionBox.getPose().set(midSwingPose);

      double swingReach = startOfSwingPose.getPosition().distanceXY(endOfSwingPose.getPosition());
      double minCollisionBoxSizeX = 0.05;
      double midSwingCollisionSizeX = Math.max(swingReach - 2.0 * swingPlannerParameters.getFootStubClearance(), minCollisionBoxSizeX);
      double midSwingCollisionSizeY = 2.0 * walkingControllerParameters.getSteppingParameters().getFootWidth() + swingPlannerParameters.getMidSwingCollisionBoxExtraY();
      midSwingCollisionBox.getSize().set(midSwingCollisionSizeX, midSwingCollisionSizeY, 2.0 * swingPlannerParameters.getMidSwingCollisionBoxExtraZ());
      boolean midSwingCollision = collisionDetected(midSwingCollisionBox);

      /* Compute waypoint adjustments */
      if (!startOfSwingToeCollision && !endOfSwingHeelCollision && !midSwingCollision)
      {
         return;
      }

      step.setTrajectoryType(TrajectoryType.CUSTOM);
      modifiedWaypoints.clear();

      Point3D firstNominalWaypoint = new Point3D(nominalTwoWaypoints.get(0));
      modifiedWaypoints.add(firstNominalWaypoint);
      if (startOfSwingToeCollision)
      {
         FramePose3D waypointPose = new FramePose3D();
         waypointPose.getPosition().set(nominalTwoWaypoints.get(0));
         waypointPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), defaultWaypointProportions[0]);
         waypointPose.appendTranslation(-1.0 * swingPlannerParameters.getXShiftForToeOrHeelCollision(), 0.0, swingPlannerParameters.getZShiftForToeOrHeelCollision());
         firstNominalWaypoint.set(waypointPose.getPosition());
      }

      if (midSwingCollision)
      {
         midSwingPose.getPosition().addZ(swingPlannerParameters.getZShiftForMidSwingCollision());
         modifiedWaypoints.add(new Point3D(midSwingPose.getPosition()));
      }

      Point3D secondNominalWaypoint = new Point3D(nominalTwoWaypoints.get(1));
      modifiedWaypoints.add(secondNominalWaypoint);
      if (endOfSwingHeelCollision)
      {
         FramePose3D waypointPose = new FramePose3D();
         waypointPose.getPosition().set(secondNominalWaypoint);
         waypointPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), defaultWaypointProportions[1]);
         waypointPose.appendTranslation(swingPlannerParameters.getXShiftForToeOrHeelCollision(), 0.0, swingPlannerParameters.getZShiftForToeOrHeelCollision());
         secondNominalWaypoint.set(waypointPose.getPosition());
      }
   }

   private void computeNominalSwingTrajectory()
   {
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();
      nominalTwoWaypoints.clear();

      for (int i = 0; i < 2; i++)
      {
         FramePoint3D waypoint = new FramePoint3D();
         waypoint.interpolate(startOfSwingPose.getPosition(), endOfSwingPose.getPosition(), defaultWaypointProportions[i]);
         waypoint.addZ(defaultSwingHeightFromStanceFoot);
         nominalTwoWaypoints.add(waypoint);
      }

      double zDifference = Math.abs(startOfSwingPose.getZ() - endOfSwingPose.getZ());
      boolean obstacleClearance = zDifference > walkingControllerParameters.getSwingTrajectoryParameters().getMinHeightDifferenceForStepUpOrDown();
      if (obstacleClearance)
      {
         double maxStepZ = Math.max(startOfSwingPose.getZ(), endOfSwingPose.getZ());
         for (int i = 0; i < 2; i++)
         {
            nominalTwoWaypoints.get(i).setZ(maxStepZ + defaultSwingHeightFromStanceFoot);
         }
      }

      positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
      positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
      positionTrajectoryGenerator.setWaypoints(nominalTwoWaypoints);
      positionTrajectoryGenerator.initialize();

      positionTrajectoryGenerator.setShouldVisualize(visualize);
      for (int i = 0; i < 30; i++)
      {
         boolean isDone = positionTrajectoryGenerator.doOptimizationUpdate();
         if (isDone)
         {
            break;
         }
      }
   }

   private void setCollisionBoxPose(Box3D collisionBox, Pose3DReadOnly footstepPose, FootCollisionCheckType footCollisionCheckType)
   {
      double boxHeight = swingPlannerParameters.getCollisionBoxHeight();
      double boxGroundClearance = swingPlannerParameters.getCollisionBoxGroundClearance();

      collisionBox.getSize().set(swingPlannerParameters.getFootStubClearance(), walkingControllerParameters.getSteppingParameters().getFootWidth(), boxHeight);

      double footHalfLength = 0.5 * walkingControllerParameters.getSteppingParameters().getFootLength();
      double xOffset = (footCollisionCheckType.checkToeCollision() ? 1.0 : -1.0) * (swingPlannerParameters.getFootStubClearance() + 0.5 * footHalfLength);
      double zOffset = boxGroundClearance + 0.5 * boxHeight;
      collisionBox.getPose().set(footstepPose);
      collisionBox.getPose().appendTranslation(xOffset, 0.0, zOffset);
   }

   private boolean collisionDetected(Box3DReadOnly collisionBox)
   {
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         if(collisionDetector.evaluateCollision(collisionBox, planarRegionsList.getPlanarRegion(i)).areShapesColliding())
         {
            return true;
         }
      }

      return false;
   }

   private void updateGraphics(PlannedFootstep footstep)
   {
      startOfSwingPoseGraphic.setPose(startOfSwingPose);
      endOfSwingPoseGraphic.setPose(endOfSwingPose);

      footstepGraphic.setPose(endOfSwingPose);
      footstepGraphic.updateConvexPolygon2d(footstep.getFoothold());

      startOfSwingToeCollisionGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), startOfSwingToeCollisionBox.getPosition(), startOfSwingToeCollisionBox.getOrientation()));
      endOfSwingHeelCollisionGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), endOfSwingHeelCollisionBox.getPosition(), endOfSwingHeelCollisionBox.getOrientation()));

      tickAndUpdatable.tickAndUpdate();
   }

   private enum FootCollisionCheckType
   {
      TOE, HEEL;

      boolean checkToeCollision()
      {
         return this == TOE;
      }
   }
}
