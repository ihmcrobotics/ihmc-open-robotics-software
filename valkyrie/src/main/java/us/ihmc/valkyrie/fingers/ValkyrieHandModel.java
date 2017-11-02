package us.ihmc.valkyrie.fingers;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;

public class ValkyrieHandModel implements HandModel
{
   @Override
   public ValkyrieHandJointName[] getHandJointNames()
   {
      return ValkyrieHandJointName.values;
   }
}
