package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.Color;
import controller_msgs.msg.dds.HandSakeDesiredCommandMessage;
import controller_msgs.msg.dds.HandSakeStatusMessage;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXSakeHandTorqueSlider
{
   private static final double UPDATE_PERIOD = UnitConversions.hertzToSeconds(10.0);
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double ROBOT_DATA_EXPIRATION_DURATION = 1.0;
   private static final float MAX_TORQUE = 1f;
   private static final float MIN_TORQUE = 0f;
   private static final double MAX_ANGLE_LIMIT = 105.0;
   private static final double EPSILON = 1E-6;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final IHMCROS2Input<HandSakeStatusMessage> handStatusMessage;
   private final CommunicationHelper communicationHelper;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double loadValueFromRobot = Double.NaN;
   private double presentGoalTorque = Double.NaN;
   private double presentGoalPosition = Double.NaN;
   private final RobotSide handSide;
   private final Throttler updateThrottler = new Throttler();
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandTorqueSlider(ROS2SyncedRobotModel syncedRobot, CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.syncedRobot = syncedRobot;
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;
      sliderName = handSide.getPascalCaseName() + " torque";

      handStatusMessage = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                 .withTypeName(HandSakeStatusMessage.class),
                                                        message -> message.getRobotSide() == handSide.toByte());
   }

   private void receiveSakeHandData()
   {
      if (updateThrottler.run(UPDATE_PERIOD) && handStatusMessage.hasReceivedFirstMessage())
      {
         loadValueFromRobot = handStatusMessage.getLatest().getPresentTorqueRatio();
         presentGoalTorque = handStatusMessage.getLatest().getGoalTorqueRatio();
         presentGoalPosition = handStatusMessage.getLatest().getGoalPositionRatio();
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD) && syncedRobot.getDataReceptionTimerSnapshot().isRunning(ROBOT_DATA_EXPIRATION_DURATION));
         {
            HandSakeDesiredCommandMessage message = new HandSakeDesiredCommandMessage();

            // This attempts to keep the hand's position identical when sending a new goal torque
            // TODO: ensure resending goal position does not interfere with command sent before
            message.setRobotSide(handSide.toByte());
            message.setDesiredHandConfiguration((byte) 5); // GOTO
            message.setPostionRatio(presentGoalPosition);
            message.setTorqueRatio(sliderValue[0]);

            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
         }
      }
      else
      {
         sliderValue[0] = (float) presentGoalTorque;
      }

      receiveSakeHandData();
   }

   private final ImVec2 textSize = new ImVec2();

   private boolean renderImGuiSliderAndReturnChanged()
   {
      renderPresentTorqueBar();

      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, MIN_TORQUE, MAX_TORQUE);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) & !MathTools.epsilonEquals(currentValue, previousValue, EPSILON);
   }

   private void renderPresentTorqueBar()
   {
      int barColor = getGreenToRedGradiatedColor(loadValueFromRobot, 0.5, 0.7, 0.9);
      float presentTorqueBar = (float) ((ImGui.getItemRectSizeX() - textSize.x - 12.0f) * loadValueFromRobot);
      float windowPositionX = ImGui.getWindowPosX();
      float windowPositionY = ImGui.getWindowPosY();
      ImGui.getWindowDrawList().addRectFilled(windowPositionX + ImGui.getCursorPosX() + 2.0f,
                                              windowPositionY + ImGui.getCursorPosY() + 2.0f,
                                              windowPositionX + ImGui.getCursorPosX() + presentTorqueBar + 12.0f,
                                              windowPositionY + ImGui.getCursorPosY() + 20.0f,
                                              barColor);

      ImGui.pushStyleColor(ImGuiCol.PlotHistogram, barColor);
      ImGui.calcTextSize(textSize, sliderName);
      ImGui.popStyleColor();
   }

   private int getGreenToRedGradiatedColor(double value, double... colorSwitchValues)
   {
      float redValue = 0.0f;
      float greenValue = 1.0f;

      for (double switchValue : colorSwitchValues)
      {
         if (value < switchValue)
            break;

         redValue = 1.0f;
         greenValue -= 1.0 / colorSwitchValues.length;
      }

      return new Color(redValue, greenValue, 0.0f, 0.5f).toIntBits();
   }
}
