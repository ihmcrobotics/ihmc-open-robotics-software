package us.ihmc.simpleWholeBodyWalking;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.CenterOfMassHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * this class manages the center of mass height or the pelvis height using the PelvisHeightTrajectoryCommand and PelvisTrajectoryCommand respectively
 * PelvisTrajectoryCommand is considered a special user command which gets forwarded to PelvisHeightControlState,
 * In this user mode, the controller won't change the height of the pelvis to ensure the legs don't reach singularities while swinging. You must
 * take the robot's configuration into account while using this command. The PelvisTrajectoryCommand also allows the user to enable User Pelvis Control During Walking.
 * If this is turned off, the controller will switch back to CenterOfMassHeightControlState during walking
 * Only the Z component of the PelvisTrajectoryCommand is used to control the pelvis height.
 *
 * The PelvisHeightTrajectoryCommand uses a pdController to compute the Linear Momentum Z and sends a momentum command to the controller core
 * If you want to the controller to manage the pelvis height while walking use the PelvisHeightTrajectoryCommand.
 */
public class SimpleCenterOfMassHeightManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final CenterOfMassHeightControlState centerOfMassHeightControlState;

   private final DoubleProvider time;

   public SimpleCenterOfMassHeightManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                                          WalkingControllerParameters walkingControllerParameters,
                                          YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      time = controllerToolbox.getYoTime();

      // Normal control during walking
      centerOfMassHeightControlState = new CenterOfMassHeightControlState(controllerToolbox, walkingControllerParameters, registry);
   }

   public void initialize()
   {
      centerOfMassHeightControlState.initialize();
   }


   public void compute()
   {
      centerOfMassHeightControlState.doAction(time.getValue());
   }

   /**
    * checks that the command is valid and switches to user mode
    * The controller will try to achieve the pelvis height regardless of the robot configuration
    * @param command - only the linear z portion of this command is used
    */
   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      centerOfMassHeightControlState.handlePelvisTrajectoryCommand(command);
   }

   /**
    * switches to center of mass height controller, this is the standard height manager
    * the height in this command will be adjusted based on the legs
    * @param command
    */
   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      centerOfMassHeightControlState.handlePelvisHeightTrajectoryCommand(command);
   }

   /**
    * set the desired height to walkingControllerParameters.nominalHeightAboveAnkle()
    * @param trajectoryTime
    */
   public void goHome(double trajectoryTime)
   {
      centerOfMassHeightControlState.goHome(trajectoryTime);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      centerOfMassHeightControlState.handleStopAllTrajectoryCommand(command);
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      centerOfMassHeightControlState.setSupportLeg(supportLeg);
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      centerOfMassHeightControlState.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }


   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return centerOfMassHeightControlState.getFeedbackControlCommand();
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {

      return centerOfMassHeightControlState.getFeedbackControlCommand();
   }

   public void setComHeightGains(PIDGainsReadOnly walkingControllerComHeightGains,
                                 DoubleProvider walkingControllerMaxComHeightVelocity)
   {
      centerOfMassHeightControlState.setGains(walkingControllerComHeightGains, walkingControllerMaxComHeightVelocity);
   }

   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return centerOfMassHeightControlState.pollStatusToReport();
   }
}
