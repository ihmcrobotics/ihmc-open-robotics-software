package us.ihmc.rdx.ui.vr;

import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import org.lwjgl.openvr.VRSkeletalSummaryData;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.handsros2.HandType;
import us.ihmc.rdx.ui.hands.RDXHandInterface.HandAction;
import us.ihmc.rdx.ui.hands.RDXHandManager;
import us.ihmc.rdx.ui.hands.psyonicAbilityHand.RDXAbilityHand;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXVRHandControl
{
   private final RDXVRContext vrContext;
   private final RDXHandManager handManager;
   private final RobotVersion robotVersion;

   private final SideDependentList<RDXHandControlMode> handsControlModes;
   private final SideDependentList<MutableBoolean> handsAreOpen = new SideDependentList<>(new MutableBoolean(false), new MutableBoolean(false));
   private final ImBoolean userIsControllingRobot;
   private static final float CONTROL_JOYSTICK_THRESHOLD = 0.5f;
   private static final float THUMB_OPPOSITION_JOYSTICK_INCREMENT = 0.01f;
   private static final float THUMB_CURL_JOYSTICK_INCREMENT = 0.01f;
   private static final float FINGER_CURL_LIMIT_MARGIN = 5.0f;
   private final SideDependentList<Float> thumbOpposition = new SideDependentList<>(0.5f, 0.5f);
   private final SideDependentList<Float> thumbCurl = new SideDependentList<>(0.5f, 0.5f);

   public RDXVRHandControl(RDXVRContext vrContext,
                           RDXHandManager handManager,
                           ImBoolean userIsControllingRobot,
                           SideDependentList<RDXHandControlMode> handControlModes,
                           RobotVersion robotVersion)
   {
      this.vrContext = vrContext;
      this.handManager = handManager;
      this.userIsControllingRobot = userIsControllingRobot;
      this.handsControlModes = handControlModes;
      this.robotVersion = robotVersion;
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
                  case  FINGER_STREAMING -> // Supported for Pysonic Ability hand and INDEX controller
                  {
                     if (handManager.getHand(side) != null &&
                         robotVersion.getHandType(side) == HandType.ABILITY_HAND)
                     {
                        switch (vrContext.getVRModel())
                        {
                           case INDEX ->
                           {
                              InputDigitalActionData touchJoystickButton = controller.getJoystickTouchedActionData();
                              boolean joystickTouched = touchJoystickButton.bState();

                              VRSkeletalSummaryData skeleton = controller.getSkeletalSummaryData();
                              for (int i = 0; i < 5; i++)
                              {
                                 // Do not send thumb curl command when touching the joystick
                                 if (!(i == 0 && joystickTouched))
                                 {
                                    // 0-3 indices correspond to index-pinky finger curls, 4 is the thumb curl, 5 the thumb opposition
                                    int mappedIndex = switch (i)
                                    {
                                       case 0 -> 4;  // thumb curl
                                       case 5 -> 5;  // thumb opposition
                                       default -> i - 1; // other fingers curl
                                    };

                                    float mappedValue;
                                    float min = switch (mappedIndex)
                                    {
                                       case 0 -> RDXAbilityHand.THUMB_CURL_MIN;
                                       case 5 -> RDXAbilityHand.THUMB_OPPOSITION_MIN;
                                       default -> RDXAbilityHand.FINGER_CURL_MIN;
                                    };
                                    float max = switch (mappedIndex)
                                    {
                                       case 0 -> RDXAbilityHand.THUMB_CURL_MAX;
                                       case 5 -> RDXAbilityHand.THUMB_OPPOSITION_MAX;
                                       default -> RDXAbilityHand.FINGER_CURL_MAX;
                                    };

                                    if (skeleton.flFingerCurl(i) < 0.05f)
                                    {
                                       mappedValue = min;
                                    }
                                    else if (skeleton.flFingerCurl(i) <= 0.85f)
                                    {
                                       // scale linearly from MIN at 0.05 to MAX at 0.85
                                       mappedValue = min + (skeleton.flFingerCurl(i) - 0.05f) / (0.85f - 0.05f) * (max - min);
                                    }
                                    else
                                    {
                                       mappedValue = max;
                                    }

                                    if (mappedIndex == 5) // opposition is negative
                                    {
                                       mappedValue = -1.0f * mappedValue;
                                    }

                                    handManager.getHand(side).sendFingerPosition(mappedIndex, mappedValue);
                                 }
                              }

                              float lateralJoystick  = controller.getJoystickActionData().x();
                              if (Math.abs(lateralJoystick) > CONTROL_JOYSTICK_THRESHOLD)
                              {
                                 float newThumbOpposition = thumbOpposition.get(side) + side.negateIfRightSide(1.0f) * Math.signum(lateralJoystick) * THUMB_OPPOSITION_JOYSTICK_INCREMENT;
                                 newThumbOpposition = Math.max(0.0f, Math.min(newThumbOpposition, 1.0f));
                                 thumbOpposition.put(side, newThumbOpposition);
                                 handManager.getHand(side).sendFingerPosition(5, 120.0f * thumbOpposition.get(side));
                              }
                           }
                           case FOCUS3 ->
                           {
                              float lateralJoystick  = controller.getJoystickActionData().x();
                              if (Math.abs(lateralJoystick) > CONTROL_JOYSTICK_THRESHOLD)
                              {
                                 float newThumbOpposition = thumbOpposition.get(side) + side.negateIfRightSide(1.0f) * Math.signum(lateralJoystick) * THUMB_OPPOSITION_JOYSTICK_INCREMENT;
                                 newThumbOpposition = Math.max(0.0f, Math.min(newThumbOpposition, 1.0f));
                                 thumbOpposition.put(side, newThumbOpposition);
                                 handManager.getHand(side).sendFingerPosition(5, 120.0f * thumbOpposition.get(side));
                              }

                              float forwardJoystick = controller.getJoystickActionData().y();
                              if (Math.abs(forwardJoystick) > CONTROL_JOYSTICK_THRESHOLD)
                              {
                                 float newFingerCurl = thumbCurl.get(side) + Math.signum(forwardJoystick) * THUMB_CURL_JOYSTICK_INCREMENT;
                                 newFingerCurl = Math.max(0.0f, Math.min(newFingerCurl, 1.0f));
                                 thumbCurl.put(side, newFingerCurl);

                                 float fingerCurlAngle = RDXAbilityHand.THUMB_CURL_MAX * thumbCurl.get(side);
                                 handManager.getHand(side).sendFingerPosition(4, fingerCurlAngle);
                              }

                              float gripValue = controller.getGripActionData().x();
                              boolean gripPressed = gripValue > CONTROL_JOYSTICK_THRESHOLD;
                              float fingerMin = RDXAbilityHand.FINGER_CURL_MIN + FINGER_CURL_LIMIT_MARGIN;
                              float fingerMax = RDXAbilityHand.FINGER_CURL_MAX - FINGER_CURL_LIMIT_MARGIN;
                              float middleRingPinkyCurl = gripPressed ? fingerMax : fingerMin;
                              for (int i = 1; i <= 3; i++)
                              {
                                 handManager.getHand(side).sendFingerPosition(i, middleRingPinkyCurl);
                              }

                              float triggerValue = controller.getTriggerActionData().x();
                              float clampedTrigger = Math.max(0.0f, Math.min(triggerValue, 1.0f));
                              float indexCurl = fingerMin + clampedTrigger * (fingerMax - fingerMin);
                              handManager.getHand(side).sendFingerPosition(0, indexCurl);
                           }
                        }
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
}
