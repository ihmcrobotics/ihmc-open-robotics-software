package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;

public class QuadrupedXGaitSettings implements QuadrupedXGaitSettingsReadOnly
{
   private double stanceLength;
   private double stanceWidth;
   private double stepGroundClearance;
   private double stepDuration;
   private double endDoubleSupportDuration;
   private double endPhaseShift;

   private final QuadrupedXGaitSettingsPacket packet = new QuadrupedXGaitSettingsPacket();

   public QuadrupedXGaitSettings()
   {
   }

   public QuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly other)
   {
      set(other);
   }

   @Override
   public double getStanceLength()
   {
      return stanceLength;
   }

   @Override
   public double getStanceWidth()
   {
      return stanceWidth;
   }

   @Override
   public double getStepGroundClearance()
   {
      return stepGroundClearance;
   }

   @Override
   public double getStepDuration()
   {
      return stepDuration;
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return endDoubleSupportDuration;
   }

   @Override
   public double getEndPhaseShift()
   {
      return endPhaseShift;
   }

   public void setStanceLength(double stanceLength)
   {
      this.stanceLength = stanceLength;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public void setStepGroundClearance(double stepGroundClearance)
   {
      this.stepGroundClearance = stepGroundClearance;
   }

   public void setStepDuration(double stepDuration)
   {
      this.stepDuration = stepDuration;
   }

   public void setEndDoubleSupportDuration(double sndDoubleSupportDuration)
   {
      this.endDoubleSupportDuration = sndDoubleSupportDuration;
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      this.endPhaseShift = endPhaseShift;
   }

   public void set(QuadrupedXGaitSettingsReadOnly other)
   {
      stanceLength = other.getStanceLength();
      stanceWidth = other.getStanceWidth();
      stepGroundClearance = other.getStepGroundClearance();
      stepDuration = other.getStepDuration();
      endDoubleSupportDuration = other.getEndDoubleSupportDuration();
      endPhaseShift = other.getEndPhaseShift();
   }


   public QuadrupedXGaitSettingsPacket getAsPacket()
   {
      packet.setStanceLength(stanceLength);
      packet.setStanceWidth(stanceWidth);
      packet.setStepGroundClearance(stepGroundClearance);
      packet.setStepDuration(stepDuration);
      packet.setEndDoubleSupportDuration(endDoubleSupportDuration);
      packet.setEndPhaseShift(endPhaseShift);

      return packet;
   }
}
