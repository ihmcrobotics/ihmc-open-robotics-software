package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;


public class UserDesiredHandLoadBearingProvider implements HandLoadBearingProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final EnumYoVariable<RobotSide> userLoadBearingRobotSide = new EnumYoVariable<RobotSide>("userLoadBearingRobotSide", registry, RobotSide.class);
   private final BooleanYoVariable userLoadBearingLoadIt = new BooleanYoVariable("userLoadBearingLoadIt", registry);

   public UserDesiredHandLoadBearingProvider(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public boolean checkForNewInformation(RobotSide robotSide)
   {
      if (!userLoadBearingLoadIt.getBooleanValue())
         return false;
      if (userLoadBearingRobotSide.getEnumValue() != robotSide)
         return false;

      return true;
   }

   public boolean hasLoadBearingBeenRequested(RobotSide robotSide)
   {
      if (!userLoadBearingLoadIt.getBooleanValue())
         return false;
      if (userLoadBearingRobotSide.getEnumValue() != robotSide)
         return false;

      userLoadBearingLoadIt.set(false);
      return true;
   }

}
