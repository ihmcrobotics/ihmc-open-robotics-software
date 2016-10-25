package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class UserDesiredPelvisHeightControllerCommandGenerators
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable userDoPelvisHeight = new BooleanYoVariable("userDesiredPelvisHeightExecute", registry);
   private final BooleanYoVariable userDesiredSetPelvisHeightToActual = new BooleanYoVariable("userDesiredPelvisSetHeightToActual", registry);

   private final DoubleYoVariable userDesiredPelvisHeightTrajectoryTime = new DoubleYoVariable("userDesiredPelvisHeightTrajectoryTime", registry);

   private final DoubleYoVariable userDesiredPelvisHeight;

   public UserDesiredPelvisHeightControllerCommandGenerators(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel,
         double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      userDesiredPelvisHeight = new DoubleYoVariable("userDesiredPelvisHeight", registry);


      userDesiredSetPelvisHeightToActual.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (userDesiredSetPelvisHeightToActual.getBooleanValue())
            {
               ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
               FramePose currentPose = new FramePose(pelvisFrame);
               currentPose.changeFrame(ReferenceFrame.getWorldFrame());

               userDesiredPelvisHeight.set(currentPose.getFramePointCopy().getZ());

               userDesiredSetPelvisHeightToActual.set(false);
            }
         }
      });
      
      userDoPelvisHeight.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
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
