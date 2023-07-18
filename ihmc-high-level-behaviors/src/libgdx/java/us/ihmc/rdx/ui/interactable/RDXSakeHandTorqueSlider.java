package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.HandSakeStatusMessage;
import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.SakeHandCommandOption;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

public class RDXSakeHandTorqueSlider
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final float MAX_TORQUE = 1f;
   private static final float MIN_TORQUE = 0f;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final IHMCROS2Input<HandSakeStatusMessage> handStatusMessage;
   private final CommunicationHelper communicationHelper;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private final RobotSide handSide;
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandTorqueSlider(CommunicationHelper communicationhelper,
                                  RobotSide handSide)
   {
      this.communicationHelper = communicationhelper;
      this.handSide = handSide;
      sliderName = handSide.getPascalCaseName() + " goal torque";

      handStatusMessage = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                 .withTypeName(HandSakeStatusMessage.class),
                                                        message -> message.getRobotSide() == handSide.toByte());

   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD))
         {
            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic,
                                        HumanoidMessageTools.createHandSakeDesiredCommandMessage(handSide,
                                                                                                 SakeHandCommandOption.SET_GOAL_TORQUE,
                                                                                                 0.0,
                                                                                                 sliderValue[0]));
         }
      }
      else if (handStatusMessage != null)
      {
         // FIXME: Doesn't show current value very well. Seems to get noise even from simulated data????
         sliderValue[0] = (float) handStatusMessage.getLatest().getTorqueRatio();
      }
   }

   private boolean renderImGuiSliderAndReturnChanged()
   {
      float previousValue = sliderValue[0];
      ImGui.sliderFloat(labels.get(sliderName), sliderValue, MIN_TORQUE, MAX_TORQUE);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) & currentValue != previousValue;
   }
}
