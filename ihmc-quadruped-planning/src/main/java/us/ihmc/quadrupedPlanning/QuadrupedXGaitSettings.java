package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;

public class QuadrupedXGaitSettings implements QuadrupedXGaitSettingsBasics
{
   private double maxSpeed;
   private double stanceLength;
   private double stanceWidth;
   private double stepGroundClearance;
   private double stepDuration;
   private double endDoubleSupportDuration;

   private final QuadrupedXGaitSettingsPacket packet = new QuadrupedXGaitSettingsPacket();

   public QuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly defaultSettings)
   {
      set(defaultSettings);
   }

   @Override
   public double getMaxSpeed()
   {
      return maxSpeed;
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
   public void setMaxSpeed(double maxSpeed)
   {
      this.maxSpeed = maxSpeed;
   }

   @Override
   public void setStanceLength(double stanceLength)
   {
      this.stanceLength = stanceLength;
   }

   @Override
   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   @Override
   public void setStepGroundClearance(double stepGroundClearance)
   {
      this.stepGroundClearance = stepGroundClearance;
   }

   @Override
   public void setStepDuration(double stepDuration)
   {
      this.stepDuration = stepDuration;
   }

   @Override
   public void setEndDoubleSupportDuration(double sndDoubleSupportDuration)
   {
      this.endDoubleSupportDuration = sndDoubleSupportDuration;
   }

   @Override
   public QuadrupedXGaitSettingsPacket getAsPacket()
   {
      packet.setMaxSpeed(getMaxSpeed());
      packet.setStanceLength(getStanceLength());
      packet.setStanceWidth(getStanceWidth());
      packet.setStepGroundClearance(getStepGroundClearance());
      packet.setStepDuration(getStepDuration());
      packet.setEndDoubleSupportDuration(getEndDoubleSupportDuration());

      return packet;
   }
}
