package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class UserDesiredChestOrientationControllerCommandGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable userDesiredChestGoToHomeOrientation = new BooleanYoVariable("userDesiredChestGoToHomeOrientation", registry);
   private final DoubleYoVariable userDesiredChestTrajectoryTime = new DoubleYoVariable("userDesiredChestTrajectoryTime", registry);
   private final BooleanYoVariable userDoChestOrientation = new BooleanYoVariable("userDoChestOrientation", registry);
   private final YoFrameOrientation userDesiredChestOrientation;

   private final FrameOrientation frameOrientation = new FrameOrientation();

   public UserDesiredChestOrientationControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      userDesiredChestOrientation = new YoFrameOrientation("userDesiredChest", ReferenceFrame.getWorldFrame(), registry);

      userDoChestOrientation.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            if (userDoChestOrientation.getBooleanValue())
            {
               userDesiredChestOrientation.getFrameOrientationIncludingFrame(frameOrientation);

               ChestTrajectoryCommand chestTrajectoryControllerCommand = new ChestTrajectoryCommand();
               chestTrajectoryControllerCommand.addTrajectoryPoint(userDesiredChestTrajectoryTime.getDoubleValue(), frameOrientation.getQuaternionCopy(), new Vector3D());
               controllerCommandInputManager.submitCommand(chestTrajectoryControllerCommand);

               userDoChestOrientation.set(false);
            }
         }
      });

      userDesiredChestGoToHomeOrientation.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (userDesiredChestGoToHomeOrientation.getBooleanValue())
            {
               userDesiredChestOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
               userDoChestOrientation.set(true, true);
               userDesiredChestGoToHomeOrientation.set(false);
            }
         }
      });

      userDesiredChestTrajectoryTime.set(defaultTrajectoryTime);

      parentRegistry.addChild(registry);
   }
}
