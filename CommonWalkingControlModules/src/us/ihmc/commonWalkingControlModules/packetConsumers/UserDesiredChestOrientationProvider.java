package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class UserDesiredChestOrientationProvider implements ChestOrientationProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable chestTrajectoryTime = new DoubleYoVariable("userDesiredChestTrajectoryTime", registry);
   private final BooleanYoVariable isNewChestOrientationInformationAvailable = new BooleanYoVariable("isNewChestOrientationInformationAvailable", registry);
   private final YoFrameOrientation userChest;

   private final FrameOrientation frameOrientation = new FrameOrientation();

   private final ReferenceFrame chestOrientationFrame;

   public UserDesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame, YoVariableRegistry parentRegistry)
   {
      this.chestOrientationFrame = chestOrientationFrame;
      userChest = new YoFrameOrientation("userChest", chestOrientationFrame, registry);

      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            isNewChestOrientationInformationAvailable.set(true);
            userChest.getFrameOrientationIncludingFrame(frameOrientation);
         }
      };

      userChest.attachVariableChangedListener(variableChangedListener);
      
      parentRegistry.addChild(registry);
   }

   @Override
   public boolean isNewChestOrientationInformationAvailable()
   {
      return isNewChestOrientationInformationAvailable.getBooleanValue();
   }

   @Override
   public FrameOrientation getDesiredChestOrientation()
   {
      if (!isNewChestOrientationInformationAvailable.getBooleanValue())
         return null;

      isNewChestOrientationInformationAvailable.set(false);

      return frameOrientation;
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }

   @Override
   public double getTrajectoryTime()
   {
      return chestTrajectoryTime.getDoubleValue();
   }
}
