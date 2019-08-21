package us.ihmc.humanoidBehaviors.ui.behaviors;

import com.sun.javafx.collections.ImmutableObservableList;
import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidBehaviors.ui.tools.AtlasDirectRobotInterface;
import us.ihmc.humanoidBehaviors.ui.tools.ValkyrieDirectRobotInterface;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.ros2.Ros2Node;

public class DirectRobotUIController
{
   @FXML private Button homeAll;
   @FXML private Button freeze;
   @FXML private Button standPrep;
   @FXML private Button shutdown;
   @FXML private ComboBox<Integer> pumpPSI;
   @FXML private CheckBox enableSupportRegions;
   @FXML private Spinner<Double> supportRegionScale;
   @FXML private Button clearREA;

   private RobotLowLevelMessenger robotLowLevelMessenger;
   private IHMCROS2Publisher<GoHomeMessage> goHomePublisher;
   private IHMCROS2Publisher<BipedalSupportPlanarRegionParametersMessage> supportRegionsParametersPublisher;
   private IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   public void init(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      if (robotModel.getSimpleRobotName().toLowerCase().contains("atlas"))
      {
         robotLowLevelMessenger = new AtlasDirectRobotInterface(ros2Node, robotModel);
      }
      else if (robotModel.getSimpleRobotName().toLowerCase().contains("valkyrie"))
      {
         robotLowLevelMessenger = new ValkyrieDirectRobotInterface(ros2Node, robotModel);
      }
      else
      {
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotModel.getSimpleRobotName());
      }

      goHomePublisher = ROS2Tools.createPublisher(ros2Node,
                                                  ROS2Tools.newMessageInstance(GoHomeCommand.class).getMessageClass(),
                                                  ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      supportRegionsParametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                    BipedalSupportPlanarRegionParametersMessage.class,
                                                                    ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(),
                                                                                                    ROS2Tools.BIPED_SUPPORT_REGION_PUBLISHER,
                                                                                                    ROS2TopicQualifier.INPUT));

      pumpPSI.setItems(new ImmutableObservableList<>(1500, 2300, 2500, 2800));
      pumpPSI.valueProperty().addListener((ChangeListener) -> sendPumpPSI());
      pumpPSI.getSelectionModel().select(1);

      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, ROS2Tools.REA);

      supportRegionScale.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 2.0, 0.1));
      enableSupportRegions.setSelected(true);
   }

   @FXML
   public void homeAll()
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

   @FXML
   public void freeze()
   {
      robotLowLevelMessenger.sendFreezeRequest();
   }

   @FXML
   public void standPrep()
   {
      robotLowLevelMessenger.sendStandRequest();
   }

   @FXML
   public void shutdown()
   {
      robotLowLevelMessenger.sendShutdownRequest();
   }

   private void sendPumpPSI()
   {
      robotLowLevelMessenger.setHydraulicPumpPSI(pumpPSI.getValue());
   }

   @FXML
   public void sendSupportRegionParameters()
   {
      BipedalSupportPlanarRegionParametersMessage supportPlanarRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportPlanarRegionParametersMessage.setEnable(enableSupportRegions.isSelected());
      supportPlanarRegionParametersMessage.setSupportRegionScaleFactor(supportRegionScale.getValue());
      supportRegionsParametersPublisher.publish(supportPlanarRegionParametersMessage);
   }

   @FXML
   public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }
}
