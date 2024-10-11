package us.ihmc.humanoidOperatorInterface.footstep;

import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface FootstepValidityMetric
{
   boolean isValid(Footstep swingStart, Footstep stance, Footstep swingEnd);//Does not assert, returns true or false
   boolean assertValid(Footstep swingStart, Footstep stance, Footstep swingEnd);//will return true or invoke an assertion.
   boolean assertValid(String message, Footstep swingStart, Footstep stance, Footstep swingEnd);//will return true or invoke an assertion with the given message.
}
