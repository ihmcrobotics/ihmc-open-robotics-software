package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TorqueSpaceBlenderFactory implements CommandBlenderFactory
{
   @Override
   public CommandBlender create(OneDoFJointBasics joint, YoRegistry parentRegistry)
   {
      return new TorqueSpaceBlender(joint, parentRegistry);
   }
}
