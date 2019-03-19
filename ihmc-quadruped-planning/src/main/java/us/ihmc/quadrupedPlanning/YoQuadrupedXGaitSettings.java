package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoQuadrupedXGaitSettings implements QuadrupedXGaitSettingsBasics
{
   private final YoDouble yoMaxSpeed;
   private final YoDouble yoStanceLength;
   private final YoDouble yoStanceWidth;
   private final YoDouble yoStepGroundClearance;
   private final YoDouble yoStepDuration;
   private final YoDouble yoEndDoubleSupportDuration;

   private final QuadrupedXGaitSettingsPacket packet = new QuadrupedXGaitSettingsPacket();

   public YoQuadrupedXGaitSettings(String prefix, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      yoMaxSpeed = new YoDouble(prefix + "_MaxSpeed", registry);
      yoStanceLength = new YoDouble(prefix + "_StanceLength", registry);
      yoStanceWidth = new YoDouble(prefix + "_StanceWidth", registry);
      yoStepGroundClearance = new YoDouble(prefix + "_StepGroundClearance", registry);
      yoStepDuration = new YoDouble(prefix + "_StepDuration", registry);
      yoEndDoubleSupportDuration = new YoDouble(prefix + "_EndDoubleSupportDuration", registry);

      set(defaultXGaitSettings);

      parentRegistry.addChild(registry);
   }

   public void addVariableChangedListener(VariableChangedListener listener)
   {
      yoMaxSpeed.addVariableChangedListener(listener);
      yoStanceLength.addVariableChangedListener(listener);
      yoStanceWidth.addVariableChangedListener(listener);
      yoStepGroundClearance.addVariableChangedListener(listener);
      yoStepDuration.addVariableChangedListener(listener);
      yoEndDoubleSupportDuration.addVariableChangedListener(listener);
   }

   @Override
   public double getMaxSpeed()
   {
      return yoMaxSpeed.getDoubleValue();
   }

   @Override
   public double getStanceLength()
   {
      return yoStanceLength.getDoubleValue();
   }

   @Override
   public double getStanceWidth()
   {
      return yoStanceWidth.getDoubleValue();
   }

   @Override
   public double getStepGroundClearance()
   {
      return yoStepGroundClearance.getDoubleValue();
   }

   @Override
   public double getStepDuration()
   {
      return yoStepDuration.getDoubleValue();
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return yoEndDoubleSupportDuration.getDoubleValue();
   }

   @Override
   public void setMaxSpeed(double maxSpeed)
   {
      yoMaxSpeed.set(maxSpeed);
   }

   @Override
   public void setStanceLength(double stanceLength)
   {
      yoStanceLength.set(stanceLength);
   }

   @Override
   public void setStanceWidth(double stanceWidth)
   {
      yoStanceWidth.set(stanceWidth);
   }

   @Override
   public void setStepGroundClearance(double stepGroundClearance)
   {
      yoStepGroundClearance.set(stepGroundClearance);
   }

   @Override
   public void setStepDuration(double stepDuration)
   {
      yoStepDuration.set(stepDuration);
   }

   @Override
   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      yoEndDoubleSupportDuration.set(endDoubleSupportDuration);
   }

   public void set(QuadrupedXGaitSettingsPacket packet)
   {
      if (packet.getMaxSpeed() != -1.0)
         setMaxSpeed(packet.getMaxSpeed());
      if (packet.getStanceLength() != -1.0)
         setStanceLength(packet.getStanceLength());
      if (packet.getStanceWidth() != -1.0)
         setStanceWidth(packet.getStanceWidth());
      if (packet.getStepGroundClearance() != -1.0)
         setStepGroundClearance(packet.getStepGroundClearance());
      if (packet.getStepDuration() != -1.0)
         setStepDuration(packet.getStepDuration());
      if (packet.getEndDoubleSupportDuration() != -1.0)
         setEndDoubleSupportDuration(packet.getEndDoubleSupportDuration());
   }

   public QuadrupedXGaitSettingsPacket getAsPacket()
   {
      packet.setMaxSpeed(yoMaxSpeed.getDoubleValue());
      packet.setStanceLength(yoStanceLength.getDoubleValue());
      packet.setStanceWidth(yoStanceWidth.getDoubleValue());
      packet.setStepGroundClearance(yoStepGroundClearance.getDoubleValue());
      packet.setStepDuration(yoStepDuration.getDoubleValue());
      packet.setEndDoubleSupportDuration(yoEndDoubleSupportDuration.getDoubleValue());

      return packet;
   }
}
