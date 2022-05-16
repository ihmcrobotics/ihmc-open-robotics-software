package us.ihmc.avatar;

import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.avatar.stepAdjustment.PlanarRegionStepConstraintCalculator;
import us.ihmc.avatar.stepAdjustment.PlanarRegionsFilterForStepping;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class AvatarStepGeneratorController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ContinuousStepGenerator continuousStepGenerator;
   private final CommandInputManager commandInputManager;
   private final DoubleProvider timeProvider;

   private final SteppingParameters steppingParameters;

   private final PlanarRegionsFilterForStepping planarRegionsFilterForStepping = new PlanarRegionsFilterForStepping();
   private final PlanarRegionFootstepSnapper planarRegionFootstepSnapper;
   private final PlanarRegionStepConstraintCalculator stepConstraintCalculator;
   private final BoundingBoxCollisionDetector collisionDetector;

   public AvatarStepGeneratorController(ContinuousStepGenerator continuousStepGenerator,
                                        CommandInputManager commandInputManager,
                                        SteppingParameters steppingParameters,
                                        DoubleProvider timeProvider)
   {
      this.continuousStepGenerator = continuousStepGenerator;
      this.commandInputManager = commandInputManager;
      this.steppingParameters = steppingParameters;
      this.timeProvider = timeProvider;

      planarRegionFootstepSnapper = new PlanarRegionFootstepSnapper(continuousStepGenerator,
                                                                    steppingParameters,
                                                                    registry);
      stepConstraintCalculator = new PlanarRegionStepConstraintCalculator(steppingParameters);


      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight);

      continuousStepGenerator.setFootstepAdjustment(planarRegionFootstepSnapper);
      continuousStepGenerator.setStepConstraintRegionCalculator(stepConstraintCalculator);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeStepHeight);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);
      continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
   }

   @Override
   public void doControl()
   {
      consumePlanarRegions();

      continuousStepGenerator.update(timeProvider.getValue());
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private void consumePlanarRegions()
   {
      if (commandInputManager != null)
      {
         if (commandInputManager.isNewCommandAvailable(PlanarRegionsListCommand.class))
         {
            PlanarRegionsListCommand commands = commandInputManager.pollNewestCommand(PlanarRegionsListCommand.class);

            planarRegionsFilterForStepping.setPlanarRegions(commands);

            planarRegionFootstepSnapper.setPlanarRegions(planarRegionsFilterForStepping.getFilteredPlanarRegions());
            stepConstraintCalculator.setPlanarRegions(planarRegionsFilterForStepping.getFilteredPlanarRegions());
            collisionDetector.setPlanarRegionsList(planarRegionsFilterForStepping.getBigEnoughPlanarRegions());
         }

         commandInputManager.clearCommands(PlanarRegionsListCommand.class);
      }
   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   /**
    * Determine whether the footstep places the robot too close to an obstacle
    *
    * @param touchdownPose -- expected pose of the swing foot when it touches down
    * @param stancePose    -- pose of the current support foot
    * @param swingSide     -- which foot will be taking the step
    * @return -- True if the step is safe; false otherwise
    */
   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsFilterForStepping.getBigEnoughPlanarRegions().isEmpty())
         return true;

      double halfStanceWidth = 0.5 * steppingParameters.getInPlaceWidth();

      /**
       * Shift box vertically by max step up, regions below this could be steppable
       */
      double heightOffset = steppingParameters.getMaxStepUp();

      double soleYaw = touchdownPose.getYaw();
      double lateralOffset = swingSide.negateIfLeftSide(halfStanceWidth);
      double offsetX = -lateralOffset * Math.sin(soleYaw);
      double offsetY = lateralOffset * Math.cos(soleYaw);
      collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY, touchdownPose.getZ() + heightOffset, soleYaw);

      return !collisionDetector.checkForCollision().isCollisionDetected();
   }

   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      return !touchdownPose.containsNaN();
   }

}
