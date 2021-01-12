package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.scene.Group;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.StackPane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.ui.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.DirectRobotUI;
import us.ihmc.humanoidBehaviors.ui.video.JavaFXROS2VideoView;
import us.ihmc.humanoidBehaviors.ui.video.JavaFXVideoViewOverlay;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.ros2.ROS2Node;

public class BehaviorDirectRobotUI extends Group
{
   private final DirectRobotUI directRobotUI = new DirectRobotUI();
   private final AnchorPane directRobotAnchorPane;

   private JavaFXLivePlanarRegionsGraphic lidarRegionsGraphic;
   private JavaFXLivePlanarRegionsGraphic realsenseRegionsGraphic;
   private JavaFXLivePlanarRegionsGraphic mapRegionsGraphic;
   private JavaFXLivePlanarRegionsGraphic supportRegionsGraphic;
   private JavaFXVideoViewOverlay multisenseVideoOverlay;
   private StackPane multisenseVideoStackPane;
   private JavaFXVideoViewOverlay realsenseVideoOverlay;
   private StackPane realsenseVideoStackPane;

   public BehaviorDirectRobotUI()
   {
      directRobotAnchorPane = JavaFXMissingTools.loadFromFXML(directRobotUI);

      directRobotUI.setShowLidarRegionsConsumer(this::showLidarRegions);
      directRobotUI.setShowRealsenseRegionsConsumer(this::showRealsenseRegions);
      directRobotUI.setShowMapRegionsConsumer(this::showMapRegions);
      directRobotUI.setShowSupportRegionsConsumer(this::showSupportRegions);
      directRobotUI.setShowMultisenseVideoConsumer(this::showMultisenseVideo);
      directRobotUI.setShowRealsenseVideoConsumer(this::showRealsenseVideo);
   }

   public void init(AnchorPane mainAnchorPane, ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      directRobotUI.init(ros2Node,robotModel);

      lidarRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, false);
      lidarRegionsGraphic.setEnabled(false);
      getChildren().add(lidarRegionsGraphic);
      realsenseRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(ros2Node, LookAndStepBehaviorAPI.REGIONS_FOR_FOOTSTEP_PLANNING, false);
      realsenseRegionsGraphic.setEnabled(false);
      getChildren().add(realsenseRegionsGraphic);
      mapRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(ros2Node, ROS2Tools.MAP_REGIONS, false);
      mapRegionsGraphic.setEnabled(false);
      getChildren().add(mapRegionsGraphic);
      supportRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(ros2Node, ROS2Tools.BIPEDAL_SUPPORT_REGIONS, false);
      supportRegionsGraphic.setEnabled(false);
      getChildren().add(supportRegionsGraphic);

      multisenseVideoOverlay = new JavaFXVideoViewOverlay(new JavaFXROS2VideoView(ros2Node, ROS2Tools.VIDEO, 1024, 544, false, false));
      multisenseVideoStackPane = new StackPane(multisenseVideoOverlay.getNode());
      multisenseVideoStackPane.setVisible(false);
      AnchorPane.setTopAnchor(multisenseVideoStackPane, 10.0);
      AnchorPane.setRightAnchor(multisenseVideoStackPane, 10.0);
      multisenseVideoOverlay.getNode().addEventHandler(MouseEvent.MOUSE_PRESSED, event -> multisenseVideoOverlay.toggleMode());
      mainAnchorPane.getChildren().add(multisenseVideoStackPane);

      realsenseVideoOverlay = new JavaFXVideoViewOverlay(new JavaFXROS2VideoView(ros2Node, ROS2Tools.D435_VIDEO, 640, 480, false, false));
      realsenseVideoStackPane = new StackPane(realsenseVideoOverlay.getNode());
      realsenseVideoStackPane.setVisible(false);
      AnchorPane.setBottomAnchor(realsenseVideoStackPane, 10.0);
      AnchorPane.setRightAnchor(realsenseVideoStackPane, 10.0);
      realsenseVideoOverlay.getNode().addEventHandler(MouseEvent.MOUSE_PRESSED, event -> realsenseVideoOverlay.toggleMode());
      mainAnchorPane.getChildren().add(realsenseVideoStackPane);
   }

   private void showLidarRegions(boolean showLidarRegions)
   {
      lidarRegionsGraphic.setEnabled(showLidarRegions);
      lidarRegionsGraphic.clear();
   }

   private void showRealsenseRegions(boolean showRealsenseRegions)
   {
      realsenseRegionsGraphic.setEnabled(showRealsenseRegions);
      realsenseRegionsGraphic.clear();
   }

   private void showMapRegions(boolean showMapRegions)
   {
      mapRegionsGraphic.setEnabled(showMapRegions);
      mapRegionsGraphic.clear();
   }

   private void showSupportRegions(boolean showSupportRegions)
   {
      supportRegionsGraphic.setEnabled(showSupportRegions);
      supportRegionsGraphic.clear();
   }

   private void showMultisenseVideo(boolean showMultisenseVideo)
   {
      multisenseVideoStackPane.setVisible(showMultisenseVideo);
      if (showMultisenseVideo)
      {
         multisenseVideoOverlay.start();
      }
      else
      {
         multisenseVideoOverlay.stop();
      }
   }

   private void showRealsenseVideo(boolean showRealsenseVideo)
   {
      realsenseVideoStackPane.setVisible(showRealsenseVideo);
      if (showRealsenseVideo)
      {
         realsenseVideoOverlay.start();
      }
      else
      {
         realsenseVideoOverlay.stop();
      }
   }

   public AnchorPane getDirectRobotAnchorPane()
   {
      return directRobotAnchorPane;
   }

   public void destroy()
   {
      lidarRegionsGraphic.destroy();
      mapRegionsGraphic.destroy();
      realsenseRegionsGraphic.destroy();
      supportRegionsGraphic.destroy();
   }
}
