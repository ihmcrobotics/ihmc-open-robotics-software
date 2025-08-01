package us.ihmc.rdx.ui.vr;

import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import org.lwjgl.openvr.VRSkeletalSummaryData;
import us.ihmc.log.LogTools;
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
   private static final float CONTROL_JOYSTICK_THRESHOLD = 0.5f;
   private static final float THUMB_OPPOSITION_JOYSTICK_INCREMENT = 0.01f;
   private final SideDependentList<Float> thumbOpposition = new SideDependentList<>(0.5f, 0.5f);;

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
                     InputDigitalActionData touchJoystickButton = controller.getJoystickTouchedActionData();
                     boolean joystickTouched = touchJoystickButton.bState();

                     VRSkeletalSummaryData skeleton = controller.getSkeletalSummaryData();
                     for (int i = 0; i < 5; i++)
                     {
                        // Do not send thumb curl command when touching the joystick
                        if (!(i == 0 && joystickTouched))
                        {
                           handManager.getHand(side).sendFingerPosition(i, skeleton.flFingerCurl(i));
                        }
                     }

                     float lateralJoystick  = controller.getJoystickActionData().x();
                     if (Math.abs(lateralJoystick) > CONTROL_JOYSTICK_THRESHOLD)
                     {
                        float newThumbOpposition = thumbOpposition.get(side) + side.negateIfRightSide(1.0f) * Math.signum(lateralJoystick) * THUMB_OPPOSITION_JOYSTICK_INCREMENT;
                        newThumbOpposition = Math.max(0.0f, Math.min(newThumbOpposition, 1.0f));
                        thumbOpposition.put(side, newThumbOpposition);
                        if (side==RobotSide.RIGHT)
                           LogTools.info(thumbOpposition.get(side));
                        handManager.getHand(side).sendFingerPosition(5, thumbOpposition.get(side));
                     }

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
