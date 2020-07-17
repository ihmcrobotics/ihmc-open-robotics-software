package us.ihmc.commonWalkingControlModules.controllerCore.data;

import us.ihmc.yoVariables.providers.BooleanProvider;

public interface FeedbackControllerData
{
   void addActiveFlag(BooleanProvider activeFlag);

   boolean isActive();

   boolean clearIfInactive();

   int getCommandId();

   static String createNamePrefix(String namePrefix, Type type, SpaceData3D space)
   {
      return namePrefix + type.getName() + space.getName();
   }
}
