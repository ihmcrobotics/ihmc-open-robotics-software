package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class UserDesiredPelvisHeightControllerCommandGenerators
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean userDoPelvisHeight = new YoBoolean("userDesiredPelvisHeightExecute", registry);
   private final YoBoolean userDesiredSetPelvisHeightToActual = new YoBoolean("userDesiredPelvisSetHeightToActual", registry);

   private final YoDouble userDesiredPelvisHeightTrajectoryTime = new YoDouble("userDesiredPelvisHeightTrajectoryTime", registry);

   private final YoDouble userDesiredPelvisHeight;

   public UserDesiredPelvisHeightControllerCommandGenerators(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel,
         double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      userDesiredPelvisHeight = new YoDouble("userDesiredPelvisHeight", registry);


      userDesiredSetPelvisHeightToActual.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userDesiredSetPelvisHeightToActual.getBooleanValue())
            {
               ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
               FramePose3D currentPose = new FramePose3D(pelvisFrame);
               currentPose.changeFrame(ReferenceFrame.getWorldFrame());

               userDesiredPelvisHeight.set(currentPose.getZ());

               userDesiredSetPelvisHeightToActual.set(false);
            }
         }
      });
      
      userDoPelvisHeight.addVariableChangedListener(new VariableChangedListener()
      {
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userDoPelvisHeight.getBooleanValue())
            {
               PelvisHeightTrajectoryCommand pelvisHeightTrajectoryControllerCommand = new PelvisHeightTrajectoryCommand();
               pelvisHeightTrajectoryControllerCommand.addTrajectoryPoint(userDesiredPelvisHeightTrajectoryTime.getDoubleValue(), userDesiredPelvisHeight.getDoubleValue(), 0.0);

               System.out.println("Submitting " + pelvisHeightTrajectoryControllerCommand);
               controllerCommandInputManager.submitCommand(pelvisHeightTrajectoryControllerCommand);
               
               userDoPelvisHeight.set(false);
            }
         }
      });

      userDesiredPelvisHeightTrajectoryTime.set(defaultTrajectoryTime);
      parentRegistry.addChild(registry);
   }
}
