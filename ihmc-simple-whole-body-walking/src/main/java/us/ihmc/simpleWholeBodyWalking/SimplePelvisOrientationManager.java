package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.controlModules.pelvis.ControllerPelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class SimplePelvisOrientationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean enableUserPelvisControlDuringWalking = new YoBoolean("EnableUserPelvisControlDuringWalking", registry);
   private final YoBoolean doPrepareForLocomotion = new YoBoolean("doPreparePelvisForLocomotion", registry);

   private final ControllerPelvisOrientationManager walkingManager;
   private final DoubleProvider time;

   public SimplePelvisOrientationManager(PID3DGainsReadOnly gains,
                                         HighLevelHumanoidControllerToolbox controllerToolbox,
                                         YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      time = controllerToolbox.getYoTime();

      walkingManager = new ControllerPelvisOrientationManager(gains, null, null, controllerToolbox, registry);

      enableUserPelvisControlDuringWalking.set(false);
   }

   public void setWeights(Vector3DReadOnly weight)
   {
      walkingManager.setWeights(weight);
   }

   public void setPrepareForLocomotion(boolean value)
   {
      doPrepareForLocomotion.set(value);
   }

   public void compute()
   {
      walkingManager.doAction(time.getValue());
   }

   public void initialize()
   {
      walkingManager.resetOrientationOffset();
      walkingManager.setToZeroInMidFeetZUpFrame();
   }

   public void prepareForLocomotion(double trajectoryTime)
   {
      if (enableUserPelvisControlDuringWalking.getBooleanValue())
         return;

      walkingManager.goToHomeFromCurrentDesired(trajectoryTime);
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return walkingManager.getFeedbackControlCommand();
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      walkingManager.goToHomeFromCurrentDesired(trajectoryTime);
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      walkingManager.setTrajectoryTime(trajectoryTime);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      walkingManager.moveToAverageInSupportFoot(supportSide);
   }

   public void resetOrientationOffset()
   {
      walkingManager.resetOrientationOffset();
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      walkingManager.setToHoldCurrentDesiredInMidFeetZUpFrame();
   }

   public void centerInMidFeetZUpFrame(double trajectoryTime)
   {
      walkingManager.centerInMidFeetZUpFrame(trajectoryTime);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      walkingManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
   }

   public void setToHoldCurrentInWorldFrame()
   {
      walkingManager.setToHoldCurrentInWorldFrame();
   }

   public void setToZeroInMidFeetZUpFrame()
   {
      walkingManager.setToZeroInMidFeetZUpFrame();
   }

   public void initializeStanding()
   {
      walkingManager.initializeStanding();
   }

   public void initializeSwing(RobotSide supportSide, double swingDuration, double nextTransferDuration, double nextSwingDuration)
   {
      walkingManager.initializeSwing(supportSide, swingDuration, nextTransferDuration, nextSwingDuration);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      walkingManager.setUpcomingFootstep(upcomingFootstep);
   }

   public void initializeTransfer(RobotSide transferToSide, double transferDuration, double swingDuration)
   {
      walkingManager.initializeTransfer(transferToSide, transferDuration, swingDuration);
   }

   public void initializeTrajectory()
   {
      walkingManager.updateTrajectoryFromFootstep();
   }

   public void updateTrajectoryFromFootstep()
   {
      walkingManager.updateTrajectoryFromFootstep();
   }

   public void setTrajectoryFromFootstep()
   {
      walkingManager.setTrajectoryFromFootstep();
   }

   public boolean handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command)
   {
      if (command.getSO3Trajectory().useCustomControlFrame())
      {
         LogTools.warn("Can not use custom control frame with pelvis orientation.");
         return false;
      }
      enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
      walkingManager.setToHoldCurrentInWorldFrame();
      return false;
   }

   private final PelvisOrientationTrajectoryCommand tempPelvisOrientationTrajectoryCommand = new PelvisOrientationTrajectoryCommand();

   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SelectionMatrix3D angularSelectionMatrix = command.getSE3Trajectory().getSelectionMatrix().getAngularPart();

      if (angularSelectionMatrix.isXSelected() || angularSelectionMatrix.isYSelected() || angularSelectionMatrix.isZSelected())
      { // At least one axis is to be controlled, process the command.
         tempPelvisOrientationTrajectoryCommand.set(command);
         return handlePelvisOrientationTrajectoryCommands(tempPelvisOrientationTrajectoryCommand);
      }
      else
      { // The user does not want to control the pelvis orientation, do nothing.
         // TODO Has to return true otherwise the command won't get to the height and XY managers.
         return true;
      }
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      walkingManager.setSelectionMatrix(selectionMatrix);
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      ret.addCommand(walkingManager.getFeedbackControlCommand());
      return ret;
   }
}
