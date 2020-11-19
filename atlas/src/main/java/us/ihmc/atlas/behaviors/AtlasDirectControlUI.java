package us.ihmc.atlas.behaviors;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasUIAuxiliaryData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.ui.controllers.RobotIKUI;
import us.ihmc.humanoidBehaviors.ui.tools.DirectRobotUI;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

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
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "direct_robot_ui");

      Stage stage = new Stage();
      stage.setTitle(getClass().getSimpleName());

      TabPane tabPane = new TabPane();

      DirectRobotUI directRobotUI = new DirectRobotUI();
      AnchorPane directRobotUIPane = JavaFXMissingTools.loadFromFXML(directRobotUI);
      directRobotUI.init(ros2Node, drcRobotModel);

      RobotIKUI robotIKUI = new RobotIKUI();
      AtlasUIAuxiliaryData atlasUIAuxiliaryData = new AtlasUIAuxiliaryData();
      robotIKUI.setAuxiliaryRobotData(atlasUIAuxiliaryData);
      robotIKUI.setFullRobotModel(drcRobotModel.createFullRobotModel(), drcRobotModel);
      AnchorPane robotIKUIPane = JavaFXMissingTools.loadFromFXML(robotIKUI);

      tabPane.getTabs().add(new Tab("General", directRobotUIPane));
      tabPane.getTabs().add(new Tab("IK", robotIKUIPane));

      Scene mainScene = new Scene(tabPane);
      stage.setScene(mainScene);
      stage.show();
   }

   public static void main(String[] args)
   {
      new AtlasDirectControlUI();
   }
}
