package us.ihmc.commonWalkingControlModules.controllerCore.data;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameQuaternion;

public class QuaternionData3D extends YoMutableFrameQuaternion implements FeedbackControllerData
{
   private final List<BooleanProvider> activeFlags = new ArrayList<>();
   private final Type type;

   public QuaternionData3D(String namePrefix, Type type, YoVariableRegistry registry)
   {
      super(FeedbackControllerData.createNamePrefix(namePrefix, type, Space.ORIENTATION), "", registry);

      this.type = type;
   }

   @Override
   public void addActiveFlag(BooleanProvider activeFlag)
   {
      this.activeFlags.add(activeFlag);
   }

   @Override
   public boolean isActive()
   {
      for (int i = 0; i < activeFlags.size(); i++)
      {
         if (activeFlags.get(i).getValue())
            return true;
      }
      return false;
   }

   @Override
   public boolean clearIfInactive()
   {
      if (!isActive())
      {
         setToNaN();
         return true;
      }
      return false;
   }

   @Override
   public Space getSpace()
   {
      return Space.ORIENTATION;
   }

   @Override
   public Type getType()
   {
      return type;
   }
}
