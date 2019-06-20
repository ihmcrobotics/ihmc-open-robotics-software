package us.ihmc.quadrupedPlanning;

import controller_msgs.msg.dds.QuadrupedGaitTimingsPacket;
import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoQuadrupedGaitTimings implements QuadrupedGaitTimingsBasics
{
   private final YoDouble maxSpeed;
   private final YoDouble stepDuration;
   private final YoDouble endDoubleSupportDuration;

   private final QuadrupedGaitTimingsPacket packet = new QuadrupedGaitTimingsPacket();

   public YoQuadrupedGaitTimings(String prefix, QuadrupedGaitTimingsReadOnly defaultTimings, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      maxSpeed = new YoDouble(prefix + "_MaxSpeed", registry);
      stepDuration = new YoDouble(prefix + "_StepDuration", registry);
      endDoubleSupportDuration = new YoDouble(prefix + "_EndDoubleSupportDuration", registry);

      set(defaultTimings);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setMaxSpeed(double maxSpeed)
   {
      this.maxSpeed.set(maxSpeed);
   }

   @Override
   public void setStepDuration(double stepDuration)
   {
      this.stepDuration.set(stepDuration);
   }

   @Override
   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      this.endDoubleSupportDuration.set(endDoubleSupportDuration);
   }

   @Override
   public double getMaxSpeed()
   {
      return maxSpeed.getDoubleValue();
   }

   @Override
   public double getStepDuration()
   {
      return stepDuration.getDoubleValue();
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return endDoubleSupportDuration.getDoubleValue();
   }
}
