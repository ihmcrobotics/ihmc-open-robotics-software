package us.ihmc.humanoidBehaviors.ui;

import javafx.scene.Group;
import javafx.scene.SubScene;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.messager.Messager;

public abstract class BehaviorUIInterface extends Group
{
   public abstract void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel);

   public abstract void setEnabled(boolean enabled);
}
