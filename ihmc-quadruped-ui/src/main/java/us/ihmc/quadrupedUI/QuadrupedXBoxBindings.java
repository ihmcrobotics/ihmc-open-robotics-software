package us.ihmc.quadrupedUI;

import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class QuadrupedXBoxBindings
{
   // velocity
   static XBoxOneMapping xVelocityMapping = XBoxOneMapping.LEFT_STICK_Y;
   static boolean xVelocityInvert = true;

   static XBoxOneMapping yVelocityMapping = XBoxOneMapping.LEFT_STICK_X;
   static boolean yVelocityInvert = true;

   static XBoxOneMapping negativeYawRateMapping = XBoxOneMapping.LEFT_TRIGGER;
   static boolean negativeYawRateInvert = false;

   static XBoxOneMapping positiveYawRateMapping = XBoxOneMapping.RIGHT_TRIGGER;
   static boolean positiveYawRateInvert = false;

   // translation
   static XBoxOneMapping xTranslationMapping = XBoxOneMapping.LEFT_STICK_Y;
   static boolean xTranslationInvert = true;

   static XBoxOneMapping yTranslationMapping = XBoxOneMapping.LEFT_STICK_X;
   static boolean yTranslationInvert = true;

   static XBoxOneMapping negativeYawMapping = XBoxOneMapping.LEFT_TRIGGER;
   static boolean negativeYawInvert = false;

   static XBoxOneMapping positiveYawMapping = XBoxOneMapping.RIGHT_TRIGGER;
   static boolean positiveYawInvert = false;

   // orientation
   static XBoxOneMapping rollMapping = XBoxOneMapping.RIGHT_STICK_X;
   static boolean rollInvert = true;

   static XBoxOneMapping pitchMapping = XBoxOneMapping.RIGHT_STICK_Y;
   static boolean pitchInvert = true;


   static XBoxOneMapping endPhaseShiftUp = XBoxOneMapping.RIGHT_BUMPER;
   static XBoxOneMapping endPhaseShiftDown = XBoxOneMapping.LEFT_BUMPER;
}
