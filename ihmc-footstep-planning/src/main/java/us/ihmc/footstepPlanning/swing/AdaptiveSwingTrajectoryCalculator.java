package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class AdaptiveSwingTrajectoryCalculator
{
   private static final double boxHeight = 0.25;
   private static final double boxGroundClearance = 0.04;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicCoordinateSystem startOfSwingPoseGraphic;
   private final YoGraphicCoordinateSystem endOfSwingPoseGraphic;
   private final YoGraphicPolygon footstepGraphic;
   private final YoGraphicShape startOfSwingToeCollisionGraphic;
   private final YoGraphicShape endOfSwingHeelCollisionGraphic;
   private final TickAndUpdatable tickAndUpdatable;

   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private PlanarRegionsList planarRegionsList;

   private final FramePose3D startOfSwingPose = new FramePose3D();
   private final FramePose3D endOfSwingPose = new FramePose3D();

   private final Box3D startOfSwingToeCollisionBox = new Box3D();
   private final Box3D endOfSwingHeelCollisionBox = new Box3D();

   public AdaptiveSwingTrajectoryCalculator(SwingPlannerParametersReadOnly swingPlannerParameters,
                                            FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                            WalkingControllerParameters walkingControllerParameters)
   {
      this(swingPlannerParameters, footstepPlannerParameters, walkingControllerParameters, null, null, null);
   }

   public AdaptiveSwingTrajectoryCalculator(SwingPlannerParametersReadOnly swingPlannerParameters,
                                            FootstepPlannerParametersReadOnly footstpePlannerParameters,
                                            WalkingControllerParameters walkingControllerParameters,
                                            TickAndUpdatable tickAndUpdatable,
                                            YoGraphicsListRegistry graphicsListRegistry,
                                            YoRegistry parentRegistry)
   {
      this.swingPlannerParameters = swingPlannerParameters;
      this.footstepPlannerParameters = footstpePlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      if (tickAndUpdatable != null)
      {
         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);

         String listName = getClass().getSimpleName();

         startOfSwingPoseGraphic = new YoGraphicCoordinateSystem("startOfSwing", "SwingCalc", registry, false, 0.1);
         endOfSwingPoseGraphic = new YoGraphicCoordinateSystem("endOfSwing", "SwingCalc", registry, false, 0.1);
         footstepGraphic = new YoGraphicPolygon("footstep", 4, registry, false, 1.0, YoAppearance.Red());

         Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
         collisionBoxGraphic.addCube(swingPlannerParameters.getFootStubClearance(), walkingControllerParameters.getSteppingParameters().getFootWidth(), boxHeight, true, YoAppearance.Orange());

         startOfSwingToeCollisionGraphic = new YoGraphicShape("startOfSwingToe", collisionBoxGraphic, "startOfSwingToe", "", registry, 1.0, YoAppearance.Orange());
         endOfSwingHeelCollisionGraphic = new YoGraphicShape("endOfSwingHeel", collisionBoxGraphic, "endOfSwingHeel", "", registry, 1.0, YoAppearance.Orange());

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

   public double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double idealStepLength = footstepPlannerParameters.getIdealFootstepLength();

      double maxStepZ = footstepPlannerParameters.getMaxStepZ();
      double maximumStepDistance = EuclidCoreTools.norm(footstepPlannerParameters.getMaximumStepReach(), maxStepZ);

      double stepDistance = startPosition.distance(endPosition);
      double alpha = MathTools.clamp((stepDistance - idealStepLength) / (maximumStepDistance - idealStepLength), 0.0, 1.0);
      return swingPlannerParameters.getMinimumSwingTime() + alpha * (swingPlannerParameters.getMaximumSwingTime() - swingPlannerParameters.getMinimumSwingTime());
   }

   public void checkForFootCollision(Pose3DReadOnly startPose, PlannedFootstep step)
   {      
      if(planarRegionsList == null)
      {
         return;
      }

      double[] waypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();      
      Pose3D endPose = new Pose3D(step.getFootstepPose());

      setBoxPose(startOfSwingToeCollisionBox, startPose, true);
      setBoxPose(endOfSwingHeelCollisionBox, endPose, false);

      boolean startOfSwingToeCollision = collisionDetected(startOfSwingToeCollisionBox);
      boolean endOfSwingHeelCollision = collisionDetected(endOfSwingHeelCollisionBox);

      if (startOfSwingToeCollision)
      {
         waypointProportions[0] = waypointProportions[0] - swingPlannerParameters.getWaypointProportionShiftForStubAvoidance();
      }

      if (endOfSwingHeelCollision)
      {
         waypointProportions[1] = waypointProportions[1] + swingPlannerParameters.getWaypointProportionShiftForStubAvoidance();
      }

      boolean collisionDetected = startOfSwingToeCollision || endOfSwingHeelCollision;
      if (collisionDetected)
      {
         step.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE);
         step.setSwingHeight(swingPlannerParameters.getSwingHeightIfCollisionDetected());
         step.setCustomWaypointProportions(waypointProportions[0], waypointProportions[1]);
      }
   }

   private void setBoxPose(Box3D collisionBox, Pose3DReadOnly footstepPose, boolean shiftForward)
   {
      collisionBox.getSize().set(swingPlannerParameters.getFootStubClearance(), walkingControllerParameters.getSteppingParameters().getFootWidth(), boxHeight);

      double footHalfLength = 0.5 * walkingControllerParameters.getSteppingParameters().getFootLength();
      double xOffset = (shiftForward ? 1.0 : -1.0) * (swingPlannerParameters.getFootStubClearance() + 0.5 * footHalfLength);
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
   
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setSwingParameters(SideDependentList<? extends Pose3DReadOnly> initialStanceFootPoses, FootstepPlan footstepPlan)
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         endOfSwingPose.set(footstep.getFootstepPose());

         if(i < 2)
         {
            startOfSwingPose.set(initialStanceFootPoses.get(footstep.getRobotSide()));
         }
         else
         {
            PlannedFootstep previousStep = footstepPlan.getFootstep(i - 2);
            startOfSwingPose.set(previousStep.getFootstepPose());
         }

         // Checks collision boxes at toe of start of swing and heel of end of swing.
         // Sets waypoint proportions and swing heights based on the collision results
         checkForFootCollision(startOfSwingPose, footstep);

         // Calculates swing duration based on step translation
         footstep.setSwingDuration(calculateSwingTime(startOfSwingPose.getPosition(), endOfSwingPose.getPosition()));

         if (tickAndUpdatable != null)
         {
            updateGraphics(footstep);
         }
      }
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
}
