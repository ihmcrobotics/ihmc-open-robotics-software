package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sakeHandCommunication.ROS2SakeHandCommander;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MIN_RATIO_VALUE;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_RATIO_VALUE;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_TORQUE_NEWTONS;

public class RDXSakeHandTorqueSlider
{
   private static final double UPDATE_PERIOD = UnitConversions.hertzToSeconds(10.0);
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double ROBOT_DATA_EXPIRATION_DURATION = 1.0;
   private static final double EPSILON = 1E-6;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final IHMCROS2Input<SakeHandStatusMessage> handStatusMessage;
   private final ROS2SakeHandCommander handCommander;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double measuredTorque = Double.NaN;
   private double desiredTorque = Double.NaN;
   private final RobotSide handSide;
   private final Throttler updateThrottler = new Throttler();
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandTorqueSlider(ROS2SyncedRobotModel syncedRobot, CommunicationHelper communicationHelper, RobotSide handSide)
   {
      this.syncedRobot = syncedRobot;
      this.handSide = handSide;
      handCommander = ROS2SakeHandCommander.getSakeHandCommander(communicationHelper);
      sliderName = handSide.getPascalCaseName() + " torque";

      handStatusMessage = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                 .withTypeName(SakeHandStatusMessage.class),
                                                        message -> message.getRobotSide() == handSide.toByte());
   }

   private void receiveSakeHandData()
   {
      if (updateThrottler.run(UPDATE_PERIOD) && handStatusMessage.hasReceivedFirstMessage())
      {
         measuredTorque = handStatusMessage.getLatest().getNormalizedMeasuredTorque();
         desiredTorque = handStatusMessage.getLatest().getNormalizedDesiredTorque();
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD) && syncedRobot.getDataReceptionTimerSnapshot().isRunning(ROBOT_DATA_EXPIRATION_DURATION));
         {
            handCommander.setDesiredTorque(handSide, sliderValue[0]);
         }
      }
      else
      {
         sliderValue[0] = (float) desiredTorque;
      }

      receiveSakeHandData();
   }

   private final ImVec2 textSize = new ImVec2();

   private boolean renderImGuiSliderAndReturnChanged()
   {
      renderPresentTorqueBar();

      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName),
                        sliderValue,
                        (float) MIN_RATIO_VALUE,
                        (float) MAX_RATIO_VALUE,
                        String.format("%.1f N", (sliderValue[0] * MAX_TORQUE_NEWTONS)));
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) & !MathTools.epsilonEquals(currentValue, previousValue, EPSILON);
   }

   private void renderPresentTorqueBar()
   {
      int barColor = ImGuiTools.greenToRedGradiatedColor(measuredTorque, 0.5, 0.7, 0.9);
      float presentTorqueBar = (float) ((ImGui.getItemRectSizeX() - textSize.x - 12.0f) * measuredTorque);

      ImGui.getWindowDrawList().addRectFilled(ImGui.getCursorScreenPosX() + 2.0f,
                                              ImGui.getCursorScreenPosY() + 2.0f,
                                              ImGui.getCursorScreenPosX() + presentTorqueBar + 12.0f,
                                              ImGui.getCursorScreenPosY() + 20.0f,
                                              barColor);

      ImGui.pushStyleColor(ImGuiCol.PlotHistogram, barColor);
      ImGui.calcTextSize(textSize, sliderName);
      ImGui.popStyleColor();
   }
}
