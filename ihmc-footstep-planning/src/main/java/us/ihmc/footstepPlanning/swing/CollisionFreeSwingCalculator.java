package us.ihmc.footstepPlanning.swing;

import org.apache.commons.collections.Bag;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.AdaptiveSwingTimingTools;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
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
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class CollisionFreeSwingCalculator
{
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static final double[] defaultWaypointProportions = new double[] {0.15, 0.85};
   private static final FramePose3D nanPose = new FramePose3D();
   private static final AppearanceDefinition boxColor = YoAppearance.Orange();

   static
   {
      nanPose.setToNaN();
      boxColor.setTransparency(0.5);
   }

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger footIndex = new YoInteger("stepIndex", registry);
   private final YoGraphicCoordinateSystem startOfSwingPoseGraphic;
   private final YoGraphicCoordinateSystem endOfSwingPoseGraphic;
   private final YoGraphicPolygon footstepGraphic;
   private final YoGraphicShape startOfSwingToeCollisionGraphic;
   private final YoGraphicShape endOfSwingHeelCollisionGraphic;
   private final YoGraphicPolygon midSwingCollisionGraphic;
   private final BagOfBalls initialWaypointGraphic, modifiedWaypointGraphic;
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private PlanarRegionsList planarRegionsList;

   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();
   private final FramePose3D nominalMidSwingPose = new FramePose3D();
   private final FramePose3D modifiedMidSwingPose = new FramePose3D();

   private final List<FramePoint3D> nominalTwoWaypoints = new ArrayList<>();
   private final List<Point3D> modifiedWaypoints = new ArrayList<>();

   private final Box3D startOfSwingToeCollisionBox = new Box3D();
   private final Box3D midSwingCollisionBox = new Box3D();
   private final Box3D endOfSwingHeelCollisionBox = new Box3D();

   private final YoBoolean toeCollisionDetected = new YoBoolean("toeCollisionDetected", registry);
   private final YoBoolean heelCollisionDetected = new YoBoolean("heelCollisionDetected", registry);
   private final YoBoolean midSwingCollisionDetected = new YoBoolean("midSwingCollisionDetected", registry);

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
      this.positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator("", registry, graphicsListRegistry, 30, 3, ReferenceFrame.getWorldFrame());
      this.visualize = parentRegistry != null;

      if (tickAndUpdatable != null)
      {
         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);

         String listName = getClass().getSimpleName();

         startOfSwingPoseGraphic = new YoGraphicCoordinateSystem("startOfSwing", "SwingCalc", registry, false, 0.1);
         endOfSwingPoseGraphic = new YoGraphicCoordinateSystem("endOfSwing", "SwingCalc", registry, false, 0.1);
         footstepGraphic = new YoGraphicPolygon("footstep", 4, registry, false, 1.0, YoAppearance.Red());

         YoFrameConvexPolygon2D midSwingPolygon = new YoFrameConvexPolygon2D("midSwingGraphicConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
         YoFramePoint3D midSwingPosition = new YoFramePoint3D("midSwingPosition", ReferenceFrame.getWorldFrame(), registry);
         YoFrameQuaternion midSwingOrientation = new YoFrameQuaternion("midSwingOrientation", ReferenceFrame.getWorldFrame(), registry);
         midSwingCollisionGraphic = new YoGraphicPolygon("midSwingGraphic",
                                                         midSwingPolygon,
                                                         midSwingPosition,
                                                         midSwingOrientation,
                                                         1.0,
                                                         2.0 * swingPlannerParameters.getMidSwingCollisionBoxExtraZ(),
                                                         boxColor);

         initialWaypointGraphic = new BagOfBalls(3, 0.02, "initialWaypointGraphic", YoAppearance.DarkGreen(), registry, graphicsListRegistry);
         modifiedWaypointGraphic = new BagOfBalls(3, 0.02, "modifiedWaypointGraphic", YoAppearance.DarkRed(), registry, graphicsListRegistry);

         Graphics3DObject toeHeelCollisionBox = new Graphics3DObject();
         toeHeelCollisionBox.addCube(swingPlannerParameters.getFootStubClearance(),
                                     walkingControllerParameters.getSteppingParameters().getFootWidth(),
                                     swingPlannerParameters.getCollisionBoxHeight(),
                                     true,
                                     YoAppearance.Orange());

         startOfSwingToeCollisionGraphic = new YoGraphicShape("startOfSwingToe", toeHeelCollisionBox, "startOfSwingToe", "", registry, 1.0, boxColor);
         endOfSwingHeelCollisionGraphic = new YoGraphicShape("endOfSwingHeel", toeHeelCollisionBox, "endOfSwingHeel", "", registry, 1.0, boxColor);

         graphicsListRegistry.registerYoGraphic(listName, startOfSwingPoseGraphic);
         graphicsListRegistry.registerYoGraphic(listName, endOfSwingPoseGraphic);
         graphicsListRegistry.registerYoGraphic(listName, midSwingCollisionGraphic);
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
         midSwingCollisionGraphic = null;
         startOfSwingToeCollisionGraphic = null;
         endOfSwingHeelCollisionGraphic = null;
         initialWaypointGraphic = null;
         modifiedWaypointGraphic = null;
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

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         footIndex.set(i);
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
            footstep.getCustomWaypointPositions().add(new Point3D(modifiedWaypoints.get(j)));
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
      modifiedWaypoints.clear();

      Pose3D endPose = new Pose3D(step.getFootstepPose());
      if (endPose.getPosition().distanceXY(startPose.getPosition()) < swingPlannerParameters.getMinTranslationToPlanSwing())
      {
         return;
      }

      /* Check toe collision at start-of-swing and heel collision at end-of-swing */
      setCollisionBoxPose(startOfSwingToeCollisionBox, startPose, FootCollisionCheckType.TOE);
      setCollisionBoxPose(endOfSwingHeelCollisionBox, endPose, FootCollisionCheckType.HEEL);

      toeCollisionDetected.set(collisionDetected(startOfSwingToeCollisionBox));
      heelCollisionDetected.set(collisionDetected(endOfSwingHeelCollisionBox));

      /* Check mid-swing collision. Need to compute nominal swing trajectory for this */
      computeNominalSwingTrajectory();

      double midSwingPercentage = 0.5;
      positionTrajectoryGenerator.compute(midSwingPercentage);
      nominalMidSwingPose.getPosition().set(positionTrajectoryGenerator.getPosition());
      double midSwingYaw = Math.atan2(endOfSwingPose.getPosition().getY() - startPose.getPosition().getY(), endOfSwingPose.getPosition().getX() - startOfSwingPose.getPosition().getX());
      nominalMidSwingPose.getOrientation().setToYawOrientation(midSwingYaw);
      midSwingCollisionBox.getPose().set(nominalMidSwingPose);

      boolean addMidSwingCollisionCheck = computeMidSwingBoxSize();
      if (addMidSwingCollisionCheck)
      {
         midSwingCollisionDetected.set(collisionDetected(midSwingCollisionBox));
      }
      else
      {
         midSwingCollisionDetected.set(false);
      }

      /* Compute waypoint adjustments */
      if (!toeCollisionDetected.getValue() && !heelCollisionDetected.getValue() && !midSwingCollisionDetected.getValue())
      {
         return;
      }

      step.setTrajectoryType(TrajectoryType.CUSTOM);

      /* Adjust first waypoint */
      {
         Point3D firstNominalWaypoint = new Point3D(nominalTwoWaypoints.get(0));
         modifiedWaypoints.add(firstNominalWaypoint);
         double xShift = swingPlannerParameters.getXShiftForToeOrHeelCollision() * (toeCollisionDetected.getValue() ? -1.0 : 0.0);
         double zShift = swingPlannerParameters.getZShiftForToeOrHeelCollision() * (toeCollisionDetected.getValue() ? 1.0 : swingPlannerParameters.getMotionCorrelationAlpha());

         FramePose3D waypointPose = new FramePose3D();
         waypointPose.getPosition().set(firstNominalWaypoint);
         waypointPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), defaultWaypointProportions[0]);
         waypointPose.appendTranslation(xShift, 0.0, zShift);
         firstNominalWaypoint.set(waypointPose.getPosition());
      }

      /* Adjust mid swing waypoint */
      if (midSwingCollisionDetected.getValue())
      {
         modifiedMidSwingPose.set(nominalMidSwingPose);
         modifiedMidSwingPose.getPosition().addZ(swingPlannerParameters.getZShiftForMidSwingCollision());
         modifiedWaypoints.add(new Point3D(modifiedMidSwingPose.getPosition()));
      }

      /* Adjust second waypoint */
      {
         Point3D secondNominalWaypoint = new Point3D(nominalTwoWaypoints.get(1));
         modifiedWaypoints.add(secondNominalWaypoint);
         double xShift = swingPlannerParameters.getXShiftForToeOrHeelCollision() * (heelCollisionDetected.getValue() ? 1.0 : 0.0);
         double zShift = swingPlannerParameters.getZShiftForToeOrHeelCollision() * (heelCollisionDetected.getValue() ? 1.0 : swingPlannerParameters.getMotionCorrelationAlpha());

         FramePose3D waypointPose = new FramePose3D();
         waypointPose.getPosition().set(secondNominalWaypoint);
         waypointPose.getOrientation().interpolate(startOfSwingPose.getOrientation(), endOfSwingPose.getOrientation(), defaultWaypointProportions[1]);
         waypointPose.appendTranslation(xShift, 0.0, zShift);
         secondNominalWaypoint.set(waypointPose.getPosition());
      }
   }

   private boolean computeMidSwingBoxSize()
   {
      double swingReach = startOfSwingPose.getPosition().distanceXY(endOfSwingPose.getPosition());
      double midSwingCollisionSizeX = swingReach - 2.0 * swingPlannerParameters.getFootStubClearance();
      double midSwingCollisionSizeY = 2.0 * (walkingControllerParameters.getSteppingParameters().getFootWidth() + swingPlannerParameters.getMidSwingCollisionBoxExtraY());
      midSwingCollisionBox.getSize().set(midSwingCollisionSizeX, midSwingCollisionSizeY, 2.0 * swingPlannerParameters.getMidSwingCollisionBoxExtraZ());

      double minCollisionBoxSizeX = 0.05;
      return midSwingCollisionBox.getSize().getX() > minCollisionBoxSizeX;
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
      {
         // show initial trajectory and collision boxes
         startOfSwingPoseGraphic.setPose(startOfSwingPose);
         endOfSwingPoseGraphic.setPose(endOfSwingPose);

         FramePose3D midSwingPoseBase = new FramePose3D(nominalMidSwingPose);
         midSwingPoseBase.getPosition().subZ(swingPlannerParameters.getMidSwingCollisionBoxExtraZ());
         midSwingCollisionGraphic.setPose(midSwingPoseBase);

         footstepGraphic.setPose(endOfSwingPose);
         footstepGraphic.updateConvexPolygon2d(footstep.getFoothold());

         startOfSwingToeCollisionGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), startOfSwingToeCollisionBox.getPosition(), startOfSwingToeCollisionBox.getOrientation()));
         endOfSwingHeelCollisionGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), endOfSwingHeelCollisionBox.getPosition(), endOfSwingHeelCollisionBox.getOrientation()));

         ConvexPolygon2D midSwingCollisionPolygon = new ConvexPolygon2D();
         midSwingCollisionPolygon.addVertex(0.5 * midSwingCollisionBox.getSize().getX(), 0.5 * midSwingCollisionBox.getSize().getY());
         midSwingCollisionPolygon.addVertex(0.5 * midSwingCollisionBox.getSize().getX(), -0.5 * midSwingCollisionBox.getSize().getY());
         midSwingCollisionPolygon.addVertex(-0.5 * midSwingCollisionBox.getSize().getX(), 0.5 * midSwingCollisionBox.getSize().getY());
         midSwingCollisionPolygon.addVertex(-0.5 * midSwingCollisionBox.getSize().getX(), -0.5 * midSwingCollisionBox.getSize().getY());
         midSwingCollisionPolygon.update();
         midSwingCollisionGraphic.updateConvexPolygon2d(midSwingCollisionPolygon);

         initialWaypointGraphic.reset();
         modifiedWaypointGraphic.reset();

         tickAndUpdatable.tickAndUpdate();
      }

      if (!modifiedWaypoints.isEmpty())
      {
         // hide collision boxes and show modified trajectory
         startOfSwingToeCollisionGraphic.setPose(nanPose);
         endOfSwingHeelCollisionGraphic.setPose(nanPose);
         midSwingCollisionGraphic.setPose(nanPose);

         positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
         positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
         positionTrajectoryGenerator.setWaypoints(modifiedWaypoints.stream().map(p -> new FramePoint3D(ReferenceFrame.getWorldFrame(), p)).collect(Collectors.toList()));
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

         initialWaypointGraphic.setBall(nominalTwoWaypoints.get(0));
         initialWaypointGraphic.setBall(nominalMidSwingPose.getPosition());
         initialWaypointGraphic.setBall(nominalTwoWaypoints.get(1));
         modifiedWaypoints.forEach(modifiedWaypointGraphic::setBall);

         tickAndUpdatable.tickAndUpdate();
      }
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
