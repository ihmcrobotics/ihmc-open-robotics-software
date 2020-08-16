package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class UserDesiredChestOrientationControllerCommandGenerator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean userDesiredChestGoToHomeOrientation = new YoBoolean("userDesiredChestGoToHomeOrientation", registry);
   private final YoDouble userDesiredChestTrajectoryTime = new YoDouble("userDesiredChestTrajectoryTime", registry);
   private final YoBoolean userDoChestOrientation = new YoBoolean("userDoChestOrientation", registry);
   private final YoFrameYawPitchRoll userDesiredChestOrientation;

   private final FrameQuaternion frameOrientation = new FrameQuaternion();

   public UserDesiredChestOrientationControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, double defaultTrajectoryTime, YoRegistry parentRegistry)
   {
      userDesiredChestOrientation = new YoFrameYawPitchRoll("userDesiredChest", ReferenceFrame.getWorldFrame(), registry);

      userDoChestOrientation.addListener(new YoVariableChangedListener()
      {
         public void changed(YoVariable v)
         {
            if (userDoChestOrientation.getBooleanValue())
            {
               frameOrientation.setIncludingFrame(userDesiredChestOrientation);

               ChestTrajectoryCommand chestTrajectoryControllerCommand = new ChestTrajectoryCommand();
               chestTrajectoryControllerCommand.getSO3Trajectory().addTrajectoryPoint(userDesiredChestTrajectoryTime.getDoubleValue(), frameOrientation, new Vector3D());
               controllerCommandInputManager.submitCommand(chestTrajectoryControllerCommand);

               userDoChestOrientation.set(false);
            }
         }
      });

      userDesiredChestGoToHomeOrientation.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
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
