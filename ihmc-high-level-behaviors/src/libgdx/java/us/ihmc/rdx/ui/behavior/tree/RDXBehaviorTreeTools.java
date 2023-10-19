package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.rdx.ui.behavior.actions.*;

public class RDXBehaviorTreeTools
{
   public static Class<?> getClassFromTypeName(String typeName)
   {
      if (typeName.equals(ArmJointAnglesActionDefinition.class.getSimpleName()))
      {
         return RDXArmJointAnglesAction.class;
      }
      if (typeName.equals(ChestOrientationActionDefinition.class.getSimpleName()))
      {
         return RDXChestOrientationAction.class;
      }
      if (typeName.equals(FootstepPlanActionDefinition.class.getSimpleName()))
      {
         return RDXFootstepPlanAction.class;
      }
      if (typeName.equals(HandPoseActionDefinition.class.getSimpleName()))
      {
         return RDXHandPoseAction.class;
      }
      if (typeName.equals(HandWrenchActionDefinition.class.getSimpleName()))
      {
         return RDXHandWrenchAction.class;
      }
      if (typeName.equals(PelvisHeightPitchActionDefinition.class.getSimpleName()))
      {
         return RDXPelvisHeightPitchAction.class;
      }
      if (typeName.equals(SakeHandCommandActionDefinition.class.getSimpleName()))
      {
         return RDXSakeHandCommandAction.class;
      }
      if (typeName.equals(WaitDurationActionDefinition.class.getSimpleName()))
      {
         return RDXWaitDurationAction.class;
      }
      if (typeName.equals(WalkActionDefinition.class.getSimpleName()))
      {
         return RDXWalkAction.class;
      }
      else
      {
         return null;
      }
   }
}
