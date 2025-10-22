package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointControlBlenderFactory implements CommandBlenderFactory
{
   private final String nameSuffix;

   public JointControlBlenderFactory(String nameSuffix)
   {
      this.nameSuffix = nameSuffix;
   }

   public JointControlBlenderFactory()
   {
      this("_StandTransition");
   }

   @Override
   public CommandBlender create(OneDoFJointBasics joint, YoRegistry parentRegistry)
   {
      return new JointControlBlender(nameSuffix, joint, parentRegistry);
   }
}
