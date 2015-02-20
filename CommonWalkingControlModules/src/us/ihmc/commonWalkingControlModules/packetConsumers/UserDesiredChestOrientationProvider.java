package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class UserDesiredChestOrientationProvider extends ChestOrientationProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable goToHomeOrientation = new BooleanYoVariable("userDesiredChestGoToHomeOrientation", registry);
   private final DoubleYoVariable chestTrajectoryTime = new DoubleYoVariable("userDesiredChestTrajectoryTime", registry);
   private final BooleanYoVariable isNewChestOrientationInformationAvailable = new BooleanYoVariable("isNewChestOrientationInformationAvailable", registry);
   private final YoFrameOrientation userChest;

   private final FrameOrientation frameOrientation = new FrameOrientation();

   private final ReferenceFrame chestOrientationFrame;

   public UserDesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame, YoVariableRegistry parentRegistry)
   {
      this.chestOrientationFrame = chestOrientationFrame;
      userChest = new YoFrameOrientation("userDesiredChest", chestOrientationFrame, registry);

      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            isNewChestOrientationInformationAvailable.set(true);
            userChest.getFrameOrientationIncludingFrame(frameOrientation);
         }
      };

      userChest.attachVariableChangedListener(variableChangedListener);

      goToHomeOrientation.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (goToHomeOrientation.getBooleanValue())
               userChest.setYawPitchRoll(0.0, 0.0, 0.0, false);
         }
      });

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean checkForNewChestOrientation()
   {
      return isNewChestOrientationInformationAvailable.getBooleanValue();
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      if (goToHomeOrientation.getBooleanValue())
      {
         goToHomeOrientation.set(false);
         return true;
      }
      return false;
   }

   public FrameOrientationWaypoint[] getDesiredChestOrientations()
   {
      if (!isNewChestOrientationInformationAvailable.getBooleanValue())
         return null;

      isNewChestOrientationInformationAvailable.set(false);
      
      return new FrameOrientationWaypoint[]{ new FrameOrientationWaypoint( chestTrajectoryTime.getDoubleValue(), frameOrientation ) };
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }

}
