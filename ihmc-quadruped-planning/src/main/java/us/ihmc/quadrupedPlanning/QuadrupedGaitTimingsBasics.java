package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedGaitTimingsPacket;

public interface QuadrupedGaitTimingsBasics extends QuadrupedGaitTimingsReadOnly
{
   void setMaxSpeed(double maxSpeed);

   void setStepDuration(double stepDuration);

   void setEndDoubleSupportDuration(double endDoubleSupportDuration);

   default void set(QuadrupedGaitTimingsReadOnly other)
   {
      setMaxSpeed(other.getMaxSpeed());
      setStepDuration(other.getStepDuration());
      setEndDoubleSupportDuration(other.getEndDoubleSupportDuration());
   }

   default void set(QuadrupedGaitTimingsPacket packet)
   {
      if (packet.getMaxSpeed() != -1.0)
         setMaxSpeed(packet.getMaxSpeed());
      if (packet.getStepDuration() != -1.0)
         setStepDuration(packet.getStepDuration());
      if (packet.getEndDoubleSupportDuration() != -1.0)
         setEndDoubleSupportDuration(packet.getEndDoubleSupportDuration());
   }
}
