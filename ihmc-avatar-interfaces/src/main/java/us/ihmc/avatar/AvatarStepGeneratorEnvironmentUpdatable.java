package us.ihmc.avatar;

import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class AvatarStepGeneratorEnvironmentUpdatable implements Updatable
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final StepGeneratorCommandInputManager commandInputManager;
   private final YoBoolean shouldSnapToRegions;

   private final SteppingParameters steppingParameters;

   private final PlanarRegionFootstepSnapper stepSnapper;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();

   private final BoundingBoxCollisionDetector collisionDetector;

   public AvatarStepGeneratorEnvironmentUpdatable(SteppingParameters steppingParameters, StepGeneratorCommandInputManager commandInputManager)
   {
      this.steppingParameters = steppingParameters;

      shouldSnapToRegions = new YoBoolean("shouldSnapToRegions", registry);

      stepSnapper = new PlanarRegionFootstepSnapper(steppingParameters)
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

//      footstepValidityIndicators.add(this::isStepSnappable);
      footstepValidityIndicators.add(this::isSafeStepHeight);
      footstepValidityIndicators.add(this::isSafeDistanceFromObstacle);

      registry.addChild(stepSnapper.getRegistry());

      this.commandInputManager = commandInputManager;
   }

   public void setShouldSnapToRegions(boolean shouldSnapToRegions)
   {
      this.shouldSnapToRegions.set(shouldSnapToRegions);
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

   @Override
   public void update(double time)
   {
      if (commandInputManager != null)
      {
         if (commandInputManager.getCommandInputManager().isNewCommandAvailable(PlanarRegionsListCommand.class))
         {
            PlanarRegionsListCommand commands = commandInputManager.getCommandInputManager().pollNewestCommand(PlanarRegionsListCommand.class);
            stepSnapper.setPlanarRegions(commands);
         }

         commandInputManager.getCommandInputManager().clearCommands(PlanarRegionsListCommand.class);
      }
   }

//   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
//   {
//      if (stepSnapper.getSteppableRegionsList().isEmpty() || !shouldSnapToRegions.getValue())
//         return true;
//
//      tempTransform.getTranslation().set(touchdownPose.getPosition().getX(), touchdownPose.getPosition().getY(), 0.0);
//      tempTransform.getRotation().setToYawOrientation(touchdownPose.getYaw());
//
//      footPolygon.set(footPolygons.get(swingSide));
//      footPolygon.applyTransform(tempTransform, false);
//
//      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
//
//      return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, Double.POSITIVE_INFINITY, tempRegion) != null;
//   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (stepSnapper.getSteppableRegionsList().isEmpty() || !shouldSnapToRegions.getValue())
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
