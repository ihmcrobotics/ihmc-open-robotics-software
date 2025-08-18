package us.ihmc.rdx.ui.vr;

import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.rdx.ui.teleoperation.RDXHandConfigurationManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXVRHandControl
{
   private final RDXVRContext vrContext;
   private final RDXHandConfigurationManager handManager;

   private final SideDependentList<RDXHandControlMode> handsControlModes;
   private final SideDependentList<MutableBoolean> handsAreOpen = new SideDependentList<>(new MutableBoolean(false), new MutableBoolean(false));
   private final ImBoolean userIsControllingRobot;

   public RDXVRHandControl(RDXVRContext vrContext,
                           RDXHandConfigurationManager handManager,
                           ImBoolean userIsControllingRobot,
                           SideDependentList<RDXHandControlMode> handControlModes)
   {
      this.vrContext = vrContext;
      this.handManager = handManager;
      this.userIsControllingRobot = userIsControllingRobot;
      this.handsControlModes = handControlModes;
   }

   public void processVRInput()
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
            boolean triggerPressed = clickTriggerButton.bChanged() && !clickTriggerButton.bState();

            if (userIsControllingRobot.get())
            {
               switch (handsControlModes.get(side))
               {
                  case HAND_CONFIGURATION ->
                  {
                     if (triggerPressed)
                     {
                        publishHandCommand(side);
                     }
                  }
                  case  FINGER_STREAMING ->
                  {
                     // TODO need to implement the logic for finger streaming
                  }
               }
            }
         });
      }
   }

   private void publishHandCommand(RobotSide side)
   {
      //TODO update with Psyonic hand API. Try to use a brand-agnostic API class
      boolean close = handsAreOpen.get(side).booleanValue();
      handsAreOpen.get(side).setValue(!close);
      handManager.publishHandCommand(side, close ? SakeHandPreset.GRIP : SakeHandPreset.OPEN, false, false);
   }

   public SideDependentList<RDXHandControlMode> getHandsControlMode()
   {
      return handsControlModes;
   }
}
