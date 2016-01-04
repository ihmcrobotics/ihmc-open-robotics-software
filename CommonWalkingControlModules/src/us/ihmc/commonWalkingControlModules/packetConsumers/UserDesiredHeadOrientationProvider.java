package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class UserDesiredHeadOrientationProvider implements HeadOrientationProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable userDesiredHeadYaw, userDesiredHeadPitch, userDesiredHeadRoll;
   private final DoubleYoVariable trajectoryTime = new DoubleYoVariable("userHeadOrientationTrajectoryTime", registry);
   private final ReferenceFrame headOrientationFrame;

   private final AtomicBoolean isNewHeadOrientationInformationAvailable = new AtomicBoolean(true);
   private final FrameOrientation desiredHeadOrientation = new FrameOrientation();

   public UserDesiredHeadOrientationProvider(ReferenceFrame headOrientationFrame, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.headOrientationFrame = headOrientationFrame;

      userDesiredHeadYaw = new DoubleYoVariable("userDesiredHeadYaw", registry);
      userDesiredHeadPitch = new DoubleYoVariable("userDesiredHeadPitch", registry);
      userDesiredHeadRoll = new DoubleYoVariable("userDesiredHeadRoll", registry);

      trajectoryTime.set(defaultTrajectoryTime);

      setupListeners();

      parentRegistry.addChild(registry);
   }

   private void setupListeners()
   {
      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            isNewHeadOrientationInformationAvailable.set(true);
            desiredHeadOrientation.setIncludingFrame(headOrientationFrame, userDesiredHeadYaw.getDoubleValue(), userDesiredHeadPitch.getDoubleValue(),
                  userDesiredHeadRoll.getDoubleValue());
         }
      };

      userDesiredHeadYaw.addVariableChangedListener(variableChangedListener);
      userDesiredHeadPitch.addVariableChangedListener(variableChangedListener);
      userDesiredHeadRoll.addVariableChangedListener(variableChangedListener);

      variableChangedListener.variableChanged(null);
   }

   @Override
   public boolean isNewHeadOrientationInformationAvailable()
   {
      return isNewHeadOrientationInformationAvailable.getAndSet(false);
   }

   @Override
   public FrameOrientation getDesiredHeadOrientation()
   {
      return desiredHeadOrientation;
   }

   @Override
   public boolean isNewLookAtInformationAvailable()
   {
      return false;
   }

   @Override
   public FramePoint getLookAtPoint()
   {
      return null;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime.getDoubleValue();
   }

   @Override
   public ReferenceFrame getHeadOrientationExpressedInFrame()
   {
      return headOrientationFrame;
   }
}
