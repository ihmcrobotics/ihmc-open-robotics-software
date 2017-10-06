package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum ExecutionTiming
{
   @RosEnumValueDocumentation(documentation = "During the execution of this message the controller will attempt to achieve the given durations for segments of the whole trajectory.")
   CONTROL_DURATIONS,
   @RosEnumValueDocumentation(documentation = "During the execution of this message the controller will attempt to achieve the absolute timings at the knot points relative to the start of execution.")
   CONTROL_ABSOLUTE_TIMINGS;

   public static final ExecutionTiming[] values = values();
}
