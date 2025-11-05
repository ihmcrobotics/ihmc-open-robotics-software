package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RLJointControlBlenderFactory implements CommandBlenderFactory
{
   private final String nameSuffix;

   public RLJointControlBlenderFactory(String nameSuffix)
   {
      this.nameSuffix = nameSuffix;
   }

   public RLJointControlBlenderFactory()
   {
      this("_StandTransition");
   }

   @Override
   public CommandBlender create(OneDoFJointBasics joint, YoRegistry parentRegistry)
   {
      return new RLJointControlBlender(nameSuffix, joint, parentRegistry);
   }
}
