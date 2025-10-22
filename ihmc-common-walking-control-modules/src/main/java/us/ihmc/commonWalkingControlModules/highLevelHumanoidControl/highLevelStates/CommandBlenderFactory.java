package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface CommandBlenderFactory
{
   CommandBlender create(OneDoFJointBasics joint, YoRegistry parentRegistry);
}
