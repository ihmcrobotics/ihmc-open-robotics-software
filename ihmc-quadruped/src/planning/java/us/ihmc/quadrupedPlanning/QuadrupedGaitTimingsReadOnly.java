package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedGaitTimingsPacket;

public interface QuadrupedGaitTimingsReadOnly
{
   double getMaxSpeed();

   /**
    * Time duration of each swing phase (in seconds).
    */
   double getStepDuration();

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    */
   double getEndDoubleSupportDuration();

   /** !! WARNING !! produces garbage */
   default QuadrupedGaitTimingsPacket getAsPacket()
   {
      QuadrupedGaitTimingsPacket packet = new QuadrupedGaitTimingsPacket();

      packet.setMaxSpeed(getMaxSpeed());
      packet.setStepDuration(getStepDuration());
      packet.setEndDoubleSupportDuration(getEndDoubleSupportDuration());

      return packet;
   }
}
