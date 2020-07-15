package us.ihmc.commonWalkingControlModules.controllerCore.data;

import us.ihmc.yoVariables.providers.BooleanProvider;

public interface ClearableData
{
   void addActiveFlag(BooleanProvider activeFlag);

   boolean isActive();

   boolean clearIfInactive();

   Space getSpace();

   Type getType();

   static String createNamePrefix(String namePrefix, Type type, Space space)
   {
      return namePrefix + type.getName() + space.getName();
   }
}
