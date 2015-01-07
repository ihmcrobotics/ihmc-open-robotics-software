package us.ihmc.commonWalkingControlModules.packetConsumers;

import javax.vecmath.Quat4d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class UserDesiredPelvisPoseProvider implements PelvisPoseProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable userPelvisTrajectoryTime = new DoubleYoVariable("userDesiredPelvisTrajectoryTime", registry);
   private final BooleanYoVariable isNewPelvisOrientationInformationAvailable = new BooleanYoVariable("isNewPelvisOrientationInformationAvailable", registry);
   private final YoFrameOrientation userPelvis;

   private final Quat4d desiredQuat = new Quat4d();
   private final FrameOrientation frameOrientation = new FrameOrientation();

   public UserDesiredPelvisPoseProvider(YoVariableRegistry parentRegistry)
   {
      userPelvis = new YoFrameOrientation("userDesiredPelvis", null, registry);

      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            isNewPelvisOrientationInformationAvailable.set(true);
            userPelvis.getQuaternion(desiredQuat);
         }
      };

      userPelvis.attachVariableChangedListener(variableChangedListener);
      
      parentRegistry.addChild(registry);
   }

   @Override
   public boolean checkForNewOrientation()
   {
      return isNewPelvisOrientationInformationAvailable.getBooleanValue();
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame)
   {
      if (!isNewPelvisOrientationInformationAvailable.getBooleanValue())
         return null;

      isNewPelvisOrientationInformationAvailable.set(false);

      frameOrientation.setIncludingFrame(desiredPelvisFrame, desiredQuat);
      
      return frameOrientation;
   }

   @Override
   public boolean checkForNewPosition()
   {
      return false;
   }

   @Override
   public FramePoint getDesiredPelvisPosition()
   {
      return null;
   }

   @Override
   public double getTrajectoryTime()
   {
      return userPelvisTrajectoryTime.getDoubleValue();
   }
}
