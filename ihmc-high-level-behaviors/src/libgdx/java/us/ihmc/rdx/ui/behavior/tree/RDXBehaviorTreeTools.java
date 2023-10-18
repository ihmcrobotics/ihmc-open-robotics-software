package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.rdx.ui.behavior.actions.RDXArmJointAnglesAction;

public class RDXBehaviorTreeTools
{
   public static Class<?> getClassFromTypeName(String typeName)
   {
      if (typeName.equals(RDXArmJointAnglesAction.class.getSimpleName()))
      {
         return RDXArmJointAnglesAction.class;
      }

      return null;
   }
}
