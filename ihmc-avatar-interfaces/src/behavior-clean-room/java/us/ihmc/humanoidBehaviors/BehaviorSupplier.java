package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.messager.Messager;

public interface BehaviorSupplier
{
   BehaviorInterface build(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel);
}
