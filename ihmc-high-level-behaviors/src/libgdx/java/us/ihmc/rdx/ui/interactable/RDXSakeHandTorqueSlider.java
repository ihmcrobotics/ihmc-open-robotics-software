package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.HandSakeDesiredCommandMessage;
import controller_msgs.msg.dds.HandSakeStatusMessage;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
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
   private static final float MAX_TORQUE = 1f;
   private static final float MIN_TORQUE = 0f;
   private static final double EPSILON = 1E-6;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final IHMCROS2Input<HandSakeStatusMessage> handStatusMessage;
   private final CommunicationHelper communicationHelper;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double valueFromRobot = Double.NaN;
   private final RobotSide handSide;
   private final Throttler updateThrottler = new Throttler();
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandTorqueSlider(CommunicationHelper communicationhelper, RobotSide handSide)
   {
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;
      sliderName = handSide.getPascalCaseName() + " torque";

      handStatusMessage = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                 .withTypeName(HandSakeStatusMessage.class),
                                                        message -> message.getRobotSide() == handSide.toByte());
   }

   private void receiveHandTorqueData()
   {
      if (updateThrottler.run(UPDATE_PERIOD) && handStatusMessage.hasReceivedFirstMessage())
      {
         valueFromRobot = handStatusMessage.getLatest().getTorqueRatio();
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD))
         {
            HandSakeDesiredCommandMessage message = new HandSakeDesiredCommandMessage();

            message.setRobotSide(handSide.toByte());
            message.setDesiredHandConfiguration((byte) 5); // GOTO
            message.setPostionRatio(0);
            message.setTorqueRatio(sliderValue[0]);

            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
         }
      }

      receiveHandTorqueData();
   }

   private final ImVec2 textSize = new ImVec2();

   private boolean renderImGuiSliderAndReturnChanged()
   {
      float previousValue = sliderValue[0];
      float presentTorqueBar = (float) ((ImGui.getItemRectSizeX() - textSize.x - 12.0f) * valueFromRobot);
      float windowPositionX = ImGui.getWindowPosX();
      float windowPositionY = ImGui.getWindowPosY();
      ImGui.getWindowDrawList().addRectFilled(windowPositionX + ImGui.getCursorPosX() + presentTorqueBar + 2.0f,
                                              windowPositionY + ImGui.getCursorPosY() + 2.0f,
                                              windowPositionX + ImGui.getCursorPosX() + presentTorqueBar + 12.0f,
                                              windowPositionY + ImGui.getCursorPosY() + 20.0f,
                                              ImGuiTools.GREEN);

      ImGui.pushStyleColor(ImGuiCol.PlotHistogram, ImGuiTools.GREEN);
      ImGui.calcTextSize(textSize, sliderName);
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, MIN_TORQUE, MAX_TORQUE);
      ImGui.popStyleColor();

      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) & !MathTools.epsilonEquals(currentValue, previousValue, EPSILON);
   }
}
