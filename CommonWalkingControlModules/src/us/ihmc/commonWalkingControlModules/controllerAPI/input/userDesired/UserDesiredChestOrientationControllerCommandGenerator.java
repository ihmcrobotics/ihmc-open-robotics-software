package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerCommandInputManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ChestTrajectoryControllerCommand;
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

   public UserDesiredChestOrientationControllerCommandGenerator(final ControllerCommandInputManager controllerCommandInputManager, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      userDesiredChestOrientation = new YoFrameOrientation("userDesiredChest", ReferenceFrame.getWorldFrame(), registry);

      userDoChestOrientation.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            if (userDoChestOrientation.getBooleanValue())
            {
               userDesiredChestOrientation.getFrameOrientationIncludingFrame(frameOrientation);

               ChestTrajectoryControllerCommand chestTrajectoryControllerCommand = new ChestTrajectoryControllerCommand();
               chestTrajectoryControllerCommand.addTrajectoryPoint(userDesiredChestTrajectoryTime.getDoubleValue(), frameOrientation.getQuaternionCopy(), new Vector3d());
               controllerCommandInputManager.submitControllerCommand(chestTrajectoryControllerCommand);

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
