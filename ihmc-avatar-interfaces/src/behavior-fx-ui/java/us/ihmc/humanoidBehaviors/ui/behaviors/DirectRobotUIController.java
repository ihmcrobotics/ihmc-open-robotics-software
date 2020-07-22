package us.ihmc.humanoidBehaviors.ui.behaviors;

import com.sun.javafx.collections.ImmutableObservableList;
import controller_msgs.msg.dds.*;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.AtlasDirectRobotInterface;
import us.ihmc.humanoidBehaviors.ui.tools.ValkyrieDirectRobotInterface;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicBoolean;

public class DirectRobotUIController extends Group
{
   @FXML private ComboBox<Integer> pumpPSI;
   @FXML private CheckBox enableSupportRegions;
   @FXML private Spinner<Double> supportRegionScale;
   @FXML private CheckBox showLidarRegions;
   @FXML private CheckBox showRealsenseRegions;
   @FXML private CheckBox showMapRegions;
   @FXML private CheckBox showSupportRegions;
   @FXML private Slider stanceHeightSlider;
   @FXML private Slider leanForwardSlider;
   @FXML private Slider neckSlider;

   private RobotLowLevelMessenger robotLowLevelMessenger;
   private IHMCROS2Publisher<GoHomeMessage> goHomePublisher;
   private IHMCROS2Publisher<BipedalSupportPlanarRegionParametersMessage> supportRegionsParametersPublisher;
   private IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;
   private IHMCROS2Publisher<NeckTrajectoryMessage> neckTrajectoryPublisher;
   private LivePlanarRegionsGraphic lidarRegionsGraphic;
   private LivePlanarRegionsGraphic realsenseRegionsGraphic;
   private LivePlanarRegionsGraphic mapRegionsGraphic;
   private LivePlanarRegionsGraphic supportRegionsGraphic;

   public void init(SubScene subScene, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      if (robotName.toLowerCase().contains("atlas"))
      {
         robotLowLevelMessenger = new AtlasDirectRobotInterface(ros2Node, robotModel);

         neckTrajectoryPublisher = new IHMCROS2Publisher<>(ros2Node, NeckTrajectoryMessage.class,
                                                           ROS2Tools.HUMANOID_CONTROLLER.withRobot(robotName).withInput());
         OneDoFJointBasics neckJoint = fullRobotModel.getNeckJoint(NeckJointName.PROXIMAL_NECK_PITCH);
         setupSlider(neckSlider, () ->
         {
            double percent = neckSlider.getValue() / 100.0;
            percent = 1.0 - percent;
            MathTools.checkIntervalContains(percent, 0.0, 1.0);
            double range = neckJoint.getJointLimitUpper() - neckJoint.getJointLimitLower();
            double jointAngle = neckJoint.getJointLimitLower() + percent * range;
            LogTools.info("Commanding neck trajectory: slider: {} angle: {}", neckSlider.getValue(), jointAngle);
            neckTrajectoryPublisher.publish(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[] {jointAngle}));
         });

      }
      else if (robotName.toLowerCase().contains("valkyrie"))
      {
         robotLowLevelMessenger = new ValkyrieDirectRobotInterface(ros2Node, robotModel);
      }
      else
      {
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
      }

      goHomePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                           ROS2TopicNameTools.newMessageInstance(GoHomeCommand.class).getMessageClass(),
                                                           ROS2Tools.getControllerInputTopic(robotName));

      supportRegionsParametersPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                             BipedalSupportPlanarRegionParametersMessage.class,
                                                                             ROS2Tools.BIPED_SUPPORT_REGION_PUBLISHER.withRobot(robotName)
                                                                                       .withInput());

      pumpPSI.setItems(new ImmutableObservableList<>(1500, 2300, 2500, 2800));
      pumpPSI.getSelectionModel().select(1);
      pumpPSI.valueProperty().addListener((ChangeListener) -> sendPumpPSI());

      lidarRegionsGraphic = new LivePlanarRegionsGraphic(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, false);
      lidarRegionsGraphic.setEnabled(false);
      getChildren().add(lidarRegionsGraphic);
      realsenseRegionsGraphic = new LivePlanarRegionsGraphic(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, false);
      realsenseRegionsGraphic.setEnabled(false);
      getChildren().add(realsenseRegionsGraphic);
      mapRegionsGraphic = new LivePlanarRegionsGraphic(ros2Node, ROS2Tools.MAP_REGIONS, false);
      mapRegionsGraphic.setEnabled(false);
      getChildren().add(mapRegionsGraphic);
      supportRegionsGraphic = new LivePlanarRegionsGraphic(ros2Node, ROS2Tools.BIPEDAL_SUPPORT_REGIONS, false);
      supportRegionsGraphic.setEnabled(false);
      getChildren().add(supportRegionsGraphic);

      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, ROS2Tools.REA.withInput());

      supportRegionScale.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, BipedalSupportPlanarRegionPublisher.defaultScaleFactor, 0.1));
      enableSupportRegions.setSelected(true);

      setupSlider(stanceHeightSlider, () -> LogTools.info("stanceHeightSlider: {}", stanceHeightSlider.getValue()));
      setupSlider(leanForwardSlider, () -> LogTools.info("leanForwardSlider: {}", leanForwardSlider.getValue()));
   }

   private void setupSlider(Slider slider, Runnable onChange)
   {
      AtomicBoolean changing = new AtomicBoolean(false);
      slider.valueProperty().addListener((observable, oldValue, newValue) -> {
         if (!changing.get())
         {
            onChange.run();
         }
      });
      slider.valueChangingProperty().addListener((observable, wasChanging, isChanging) -> {
         changing.set(isChanging);
         if (wasChanging)
         {
            onChange.run();
         }
      });
   }

   @FXML public void homeAll()
   {
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      goHomePublisher.publish(homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
      goHomePublisher.publish(homeRightArm);
   }

   @FXML public void freeze()
   {
      robotLowLevelMessenger.sendFreezeRequest();
   }

   @FXML public void standPrep()
   {
      robotLowLevelMessenger.sendStandRequest();
   }

   @FXML public void shutdown()
   {
      robotLowLevelMessenger.sendShutdownRequest();
   }

   private void sendPumpPSI()
   {
      robotLowLevelMessenger.setHydraulicPumpPSI(pumpPSI.getValue());
   }

   @FXML public void sendSupportRegionParameters()
   {
      BipedalSupportPlanarRegionParametersMessage supportPlanarRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportPlanarRegionParametersMessage.setEnable(enableSupportRegions.isSelected());
      supportPlanarRegionParametersMessage.setSupportRegionScaleFactor(supportRegionScale.getValue());
      supportRegionsParametersPublisher.publish(supportPlanarRegionParametersMessage);
   }

   @FXML public void showLidarRegions()
   {
      lidarRegionsGraphic.setEnabled(showLidarRegions.isSelected());
      lidarRegionsGraphic.clear();
   }

   @FXML public void showRealsenseRegions()
   {
      realsenseRegionsGraphic.setEnabled(showRealsenseRegions.isSelected());
      realsenseRegionsGraphic.clear();
   }

   @FXML public void showMapRegions()
   {
      mapRegionsGraphic.setEnabled(showMapRegions.isSelected());
      mapRegionsGraphic.clear();
   }

   @FXML public void showSupportRegions()
   {
      supportRegionsGraphic.setEnabled(showSupportRegions.isSelected());
      supportRegionsGraphic.clear();
   }

   @FXML public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }

   public void stanceHeightSliderDragged()
   {
      LogTools.info("Stand height slider drag exited: {}", stanceHeightSlider.getValue());
   }
}
