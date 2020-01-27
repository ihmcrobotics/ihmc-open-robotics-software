package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.navigation.NavigationBehavior;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.messager.Messager;

public class NavigationBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(NavigationBehavior.DEFINITION, NavigationBehaviorUI::new);

   @Override
   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {

   }

   @FXML public void step()
   {

   }
}
