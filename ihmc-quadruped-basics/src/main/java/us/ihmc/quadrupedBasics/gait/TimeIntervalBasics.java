package us.ihmc.quadrupedBasics.gait;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TimeIntervalCommand;

public interface TimeIntervalBasics extends TimeIntervalReadOnly
{
   /**
    * Use the interval setter when using this method! Otherwise, an incorrect interval may be set.
    */
   @Deprecated
   void setStartTime(double startTime);

   /**
    * Use the interval setter when using this method! Otherwise, an incorrect interval may be set.
    */
   @Deprecated
   void setEndTime(double endTime);

   void setInterval(double startTime, double endTime);

   TimeIntervalBasics shiftInterval(double shiftTime);

   void set(TimeIntervalReadOnly timeInterval);

   void set(TimeIntervalCommand command);
}
