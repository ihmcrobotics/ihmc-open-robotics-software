package us.ihmc.rdx.ui.vr;

import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.rdx.ui.hands.RDXHandInterface.HandAction;
import us.ihmc.rdx.ui.hands.RDXHandManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXVRHandControl
{
   private final RDXVRContext vrContext;
   private final RDXHandManager handManager;

   private final SideDependentList<RDXHandControlMode> handsControlModes;
   private final SideDependentList<MutableBoolean> handsAreOpen = new SideDependentList<>(new MutableBoolean(false), new MutableBoolean(false));
   private final ImBoolean userIsControllingRobot;

   public RDXVRHandControl(RDXVRContext vrContext,
                           RDXHandManager handManager,
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
      boolean close = handsAreOpen.get(side).booleanValue();
      handsAreOpen.get(side).setValue(!close);
      handManager.getHand(side).sendCommand(close ? HandAction.GRIP : HandAction.OPEN);
   }

   public SideDependentList<RDXHandControlMode> getHandsControlMode()
   {
      return handsControlModes;
   }
}
