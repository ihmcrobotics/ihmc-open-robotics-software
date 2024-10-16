package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.ROS2SakeHandStatus;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.SakeHandAPI;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXSakeHandWidgets;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.annotation.Nullable;

/**
 * Manages the UI for a humanoid robot's hands. A hand configuration is like "open", "closed", etc.
 */
public class RDXHandConfigurationManager
{
   private final SideDependentList<RDXIconTexture> handIcons = new SideDependentList<>();
   private final SideDependentList<RDXHandQuickAccessButtons> handQuickAccessButtons = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandWidgets> sakeHandWidgets = new SideDependentList<>();
   private final SideDependentList<ROS2SakeHandStatus> sakeHandStatus = new SideDependentList<>();
   private CommunicationHelper communicationHelper;
   private String robotName;

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.communicationHelper = communicationHelper;

      robotName = syncedRobotModel.getRobotModel().getSimpleRobotName();

      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobotModel.getRobotModel().getRobotVersion().hasSakeGripperJoints(side))
         {
            handIcons.put(side, new RDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));

            Runnable openHand = () -> publishHandCommand(side, SakeHandPreset.OPEN, false, false);
            Runnable closeHand = () -> publishHandCommand(side, SakeHandPreset.CLOSE, false, false);
            Runnable calibrateHand = () -> publishHandCommand(side, null, true, false);
            Runnable resetHand = () -> publishHandCommand(side, null, false, true);
            handQuickAccessButtons.put(side, new RDXHandQuickAccessButtons(baseUI, side, openHand, closeHand, calibrateHand, resetHand));

            sakeHandWidgets.put(side, new RDXSakeHandWidgets(communicationHelper, syncedRobotModel, side));
            sakeHandStatus.put(side, syncedRobotModel.getSakeHandStatus().get(side));
         }
      }
   }

   public void update()
   {
      for (RobotSide side : sakeHandWidgets.sides())
      {
         sakeHandWidgets.get(side).update();
         handQuickAccessButtons.get(side).update(sakeHandWidgets.get(side).getCalibrated(), sakeHandWidgets.get(side).getNeedsReset());
      }
   }

   public void renderImGuiWidgets()
   {
      for (RobotSide side : sakeHandWidgets.sides())
      {
         ImGui.image(handIcons.get(side).getTexture().getTextureObjectHandle(), 22.0f, 22.0f);
         ImGui.sameLine();
         sakeHandWidgets.get(side).renderImGuiWidgets();
      }
   }

   public void publishHandCommand(RobotSide side, @Nullable SakeHandPreset handPreset, boolean calibrate, boolean reset)
   {
      SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage = new SakeHandDesiredCommandMessage();
      sakeHandDesiredCommandMessage.setRobotSide(side.toByte());
      SakeHandParameters.resetDesiredCommandMessage(sakeHandDesiredCommandMessage);

      if (calibrate)
      {
         sakeHandDesiredCommandMessage.setRequestCalibration(true);
      }
      else if (reset)
      {
         sakeHandDesiredCommandMessage.setRequestResetErrors(true);
      }
      else if (handPreset != null)
      {
         double handPositionLowerLimit = sakeHandStatus.get(side).getPositionLowerLimit();
         double handPositionUpperLimit = sakeHandStatus.get(side).getPositionUpperLimit();

         sakeHandDesiredCommandMessage.setGripperDesiredPosition(SakeHandParameters.handOpenAngleToPosition(handPreset.getHandOpenAngle(),
                                                                                                            handPositionLowerLimit,
                                                                                                            handPositionUpperLimit));
         sakeHandDesiredCommandMessage.setRawGripperTorqueLimit(SakeHandParameters.gripForceToRawTorque(handPreset.getFingertipGripForceLimit()));
      }

      RDXBaseUI.pushNotification("Commanding hand configuration...");
      communicationHelper.publish(SakeHandAPI.getHandSakeCommandTopic(robotName, side), sakeHandDesiredCommandMessage);
   }
}
