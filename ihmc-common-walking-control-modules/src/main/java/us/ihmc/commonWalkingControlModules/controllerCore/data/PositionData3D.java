package us.ihmc.commonWalkingControlModules.controllerCore.data;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;

public class PositionData3D extends YoMutableFramePoint3D implements FeedbackControllerData
{
   private final List<BooleanProvider> activeFlags = new ArrayList<>();
   private final Type type;

   public PositionData3D(String namePrefix, Type type, YoVariableRegistry registry)
   {
      super(FeedbackControllerData.createNamePrefix(namePrefix, type, Space.POSITION), "", registry);

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
      return Space.POSITION;
   }

   @Override
   public Type getType()
   {
      return type;
   }
}
