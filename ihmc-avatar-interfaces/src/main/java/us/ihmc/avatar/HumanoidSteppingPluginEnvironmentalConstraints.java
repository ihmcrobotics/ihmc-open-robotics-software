package us.ihmc.avatar;

import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionCalculator;
import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.SteppableRegionsProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.GarbageFreePlanarRegionListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class HumanoidSteppingPluginEnvironmentalConstraints
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean shouldSnapToRegions;

   private final SteppingParameters steppingParameters;

   private final PlanarRegionFootstepSnapper stepSnapper;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();
//   private final BipedalSupportPlanarRegionCalculator supportPlanarRegionCalculator;

   private final BoundingBoxCollisionDetector collisionDetector;

   private SteppableRegionsProvider steppableRegionsProvider;

   // temp variables
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();
   private final PlanarRegion tempRegion = new PlanarRegion();

   public HumanoidSteppingPluginEnvironmentalConstraints(RobotContactPointParameters<RobotSide> contactPointParameters,
                                                         SteppingParameters steppingParameters)
   {
      this.steppingParameters = steppingParameters;

      shouldSnapToRegions = new YoBoolean("shouldSnapToRegions", registry);
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(side ->
                                                                                {
                                                                                   ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
                                                                                   return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
                                                                                });
      stepSnapper = new PlanarRegionFootstepSnapper(footPolygons)
      {
         @Override
         public boolean adjustFootstep(FramePose3DReadOnly stanceFootPose,
                                       FramePose2DReadOnly footstepPose,
                                       RobotSide footSide,
                                       FixedFramePose3DBasics adjustedPoseToPack)
         {
            if (!shouldSnapToRegions.getValue())
               return true;

            return super.adjustFootstep(stanceFootPose, footstepPose, footSide, adjustedPoseToPack);
         }
      };

      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight);

      footstepValidityIndicators.add(this::isStepSnappable);
      footstepValidityIndicators.add(this::isSafeStepHeight);
      //      footstepValidityIndicators.add(this::isSafeDistanceFromObstacle);

      registry.addChild(stepSnapper.getRegistry());
   }

   public void setShouldSnapToRegions(boolean shouldSnapToRegions)
   {
      this.shouldSnapToRegions.set(shouldSnapToRegions);
   }

   public void setSteppableRegionsProvider(SteppableRegionsProvider steppableRegionsProvider)
   {
      stepSnapper.setSteppableRegionsProvider(steppableRegionsProvider);
      this.steppableRegionsProvider = steppableRegionsProvider;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public FootstepAdjustment getFootstepAdjustment()
   {
      return stepSnapper;
   }

   public List<FootstepValidityIndicator> getFootstepValidityIndicators()
   {
      return footstepValidityIndicators;
   }


   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (steppableRegionsProvider == null || steppableRegionsProvider.getSteppableRegions().isEmpty() || !shouldSnapToRegions.getValue())
         return true;

      footPolygon.set(stepSnapper.getFootPolygon(swingSide));
      footPolygon.applyTransform(touchdownPose, false);

      return stepSnapper.getSnapper()
                        .snapPolygonToPlanarRegionsList(footPolygon,
                                                        steppableRegionsProvider.getSteppableRegions(),
                                                        Double.POSITIVE_INFINITY,
                                                        tempRegion,
                                                        snapTransform);
   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   // FIXME this generates a LOT of garbage
   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      // FIXME should not use the step snapper regions, as those may filter out collisions
      if (steppableRegionsProvider == null || steppableRegionsProvider.getSteppableRegions().isEmpty() || !shouldSnapToRegions.getValue())
         return true;

      double halfStanceWidth = 0.5 * steppingParameters.getInPlaceWidth();

      /** Shift box vertically by max step up, regions below this could be steppable */
      double heightOffset = steppingParameters.getMaxStepUp();

      double soleYaw = touchdownPose.getYaw();
      double lateralOffset = swingSide.negateIfLeftSide(halfStanceWidth);
      double offsetX = -lateralOffset * Math.sin(soleYaw);
      double offsetY = lateralOffset * Math.cos(soleYaw);
      collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY, touchdownPose.getZ() + heightOffset, soleYaw);

      return !collisionDetector.checkForCollision(null);
   }
}
