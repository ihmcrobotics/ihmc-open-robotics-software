package us.ihmc.commonWalkingControlModules.controllerCore.data;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameVector3D;

public class VectorData3D extends YoMutableFrameVector3D implements FeedbackControllerData
{
   private final List<BooleanProvider> activeFlags = new ArrayList<>();
   private final Type type;
   private final Space space;

   public VectorData3D(String namePrefix, Type type, Space space, YoVariableRegistry registry)
   {
      super(FeedbackControllerData.createNamePrefix(namePrefix, type, space), "", registry);

      this.type = type;
      this.space = space;
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
      return space;
   }

   @Override
   public Type getType()
   {
      return type;
   }
}
