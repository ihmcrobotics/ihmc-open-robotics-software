package us.ihmc.atlas.behaviors;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.ui.tools.DirectRobotUI;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;

public class AtlasDirectControlUI
{
   public AtlasDirectControlUI()
   {
      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(this::buildApp);
   }

   private void buildApp()
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
      Ros2Node ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "direct_robot_ui");

      Stage stage = new Stage();
      stage.setTitle(getClass().getSimpleName());
      DirectRobotUI directRobotUI = new DirectRobotUI();
      AnchorPane directRobotUIPane = JavaFXMissingTools.loadFromFXML(directRobotUI);
      directRobotUI.init(ros2Node, drcRobotModel);
      Scene mainScene = new Scene(directRobotUIPane);
      stage.setScene(mainScene);
      stage.show();
   }

   public static void main(String[] args)
   {
      new AtlasDirectControlUI();
   }
}
