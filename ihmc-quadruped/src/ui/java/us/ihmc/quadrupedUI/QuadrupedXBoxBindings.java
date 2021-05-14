package us.ihmc.quadrupedUI;

import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class QuadrupedXBoxBindings
{
   // walking
   static XBoxOneMapping enableWalkingTeleop = XBoxOneMapping.Y;

   static XBoxOneMapping xVelocityMapping = XBoxOneMapping.LEFT_STICK_Y;
   static boolean xVelocityInvert = true;

   static XBoxOneMapping yVelocityMapping = XBoxOneMapping.LEFT_STICK_X;
   static boolean yVelocityInvert = true;

   static XBoxOneMapping leftTurnMapping = XBoxOneMapping.LEFT_TRIGGER;
   static XBoxOneMapping rightTurnMapping = XBoxOneMapping.RIGHT_TRIGGER;

   static XBoxOneMapping endPhaseShiftUp = XBoxOneMapping.RIGHT_BUMPER;
   static XBoxOneMapping endPhaseShiftDown = XBoxOneMapping.LEFT_BUMPER;

   // orientation
   static XBoxOneMapping enableOrientationTeleop = XBoxOneMapping.B;

   static XBoxOneMapping negativeYawMapping = XBoxOneMapping.LEFT_TRIGGER;
   static boolean negativeYawInvert = false;

   static XBoxOneMapping positiveYawMapping = XBoxOneMapping.RIGHT_TRIGGER;
   static boolean positiveYawInvert = false;

   static XBoxOneMapping rollMapping = XBoxOneMapping.RIGHT_STICK_X;
   static boolean rollInvert = true;

   static XBoxOneMapping pitchMapping = XBoxOneMapping.RIGHT_STICK_Y;
   static boolean pitchInvert = true;

   // height
   static XBoxOneMapping enableHeightTeleop = XBoxOneMapping.A;
   static XBoxOneMapping heightMapping = XBoxOneMapping.DPAD;
}
