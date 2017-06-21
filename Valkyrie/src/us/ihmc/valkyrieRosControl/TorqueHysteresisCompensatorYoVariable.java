package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class TorqueHysteresisCompensatorYoVariable extends YoDouble
{
   public enum HysteresisState
   {
      ZERO, RAMP_UP, RAMP_DOWN, MAX_HYSTERESIS
   };

   private final OneDoFJoint joint;
   private final YoDouble torqueHysteresisAmplitude;
   private final YoDouble jointAccelerationMin;
   private final YoDouble jointVelocityMax;

   private final YoDouble yoTime;
   private final YoDouble ramp;
   private final YoDouble rampUpTime;
   private final YoDouble rampDownTime;
   private final YoDouble rampStartTime;

   private final YoEnum<HysteresisState> hysteresisState;

   private final YoBoolean isAccelerationHigh;
   private final YoBoolean isVelocityLow;

   private final YoDouble hysteresisSign;

   private final YoBoolean enabled;

   public TorqueHysteresisCompensatorYoVariable(String prefix, OneDoFJoint joint, YoDouble torqueHysteresisAmplitude,
         YoDouble jointAccelerationMin, YoDouble jointVelocityMax, YoDouble rampTime, YoDouble yoTime,
         YoVariableRegistry registry)
   {
      this(prefix, joint, torqueHysteresisAmplitude, jointAccelerationMin, jointVelocityMax, rampTime, rampTime, yoTime, registry);
   }

   public TorqueHysteresisCompensatorYoVariable(String prefix, OneDoFJoint joint, YoDouble torqueHysteresisAmplitude,
         YoDouble jointAccelerationMin, YoDouble jointVelocityMax, YoDouble rampUpTime, YoDouble rampDownTime,
         YoDouble yoTime, YoVariableRegistry registry)
   {
      super(prefix + joint.getName(), registry);
      this.joint = joint;
      this.torqueHysteresisAmplitude = torqueHysteresisAmplitude;
      this.jointAccelerationMin = jointAccelerationMin;
      this.jointVelocityMax = jointVelocityMax;
      this.rampUpTime = rampUpTime;
      this.rampDownTime = rampDownTime;
      this.yoTime = yoTime;

      ramp = new YoDouble(getName() + "Ramp", registry);
      rampStartTime = new YoDouble(getName() + "RampStartTime", registry);
      hysteresisState = new YoEnum<>(getName() + "State", registry, HysteresisState.class, false);

      isAccelerationHigh = new YoBoolean(getName() + "IsQddHigh", registry);
      isVelocityLow = new YoBoolean(getName() + "IsQdLow", registry);

      hysteresisSign = new YoDouble(getName() + "HysteresisSign", registry);

      enabled = new YoBoolean(getName() + "Enabled", registry);
   }

   public void reset()
   {
      hysteresisState.set(HysteresisState.ZERO);
   }

   public void enable()
   {
      enabled.set(true);
   }

   public void disable()
   {
      enabled.set(false);
   }

   public void update()
   {
      if (!enabled.getBooleanValue())
      {
         set(0.0);
         return;
      }

      checkAcceleration();
      checkVelocity();

      switch (hysteresisState.getEnumValue())
      {
      case ZERO:
         updateZeroState();
         break;
      case RAMP_UP:
         updateRampUpState();
         break;
      case MAX_HYSTERESIS:
         updateMaxHysteresisState();
         break;
      case RAMP_DOWN:
         updateRampDownState();
         break;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   private void updateZeroState()
   {
      boolean startRampUp = isVelocityLow.getBooleanValue() && isAccelerationHigh.getBooleanValue();
      if (startRampUp)
      {
         hysteresisSign.set(Math.signum(joint.getQddDesired()));
         rampStartTime.set(yoTime.getDoubleValue());
         hysteresisState.set(HysteresisState.RAMP_UP);
      }
      else
      {
         hysteresisSign.set(0.0);
         rampStartTime.set(Double.NaN);
         hysteresisState.set(HysteresisState.ZERO);
      }

      set(0.0);
   }

   private void updateRampUpState()
   {
      double timeInRampUp = yoTime.getDoubleValue() - rampStartTime.getDoubleValue();
      ramp.set(MathTools.clamp(timeInRampUp / rampUpTime.getDoubleValue(), 0.0, 1.0));
      
      double tau_off_hyst = ramp.getDoubleValue() * torqueHysteresisAmplitude.getDoubleValue();
      tau_off_hyst *= hysteresisSign.getDoubleValue();

      set(tau_off_hyst);

      boolean qddDesiredChangedSign = hysteresisSign.getDoubleValue() * joint.getQddDesired() < 0.0;
      boolean startRampDown = !isVelocityLow.getBooleanValue() || !isAccelerationHigh.getBooleanValue() || qddDesiredChangedSign;
      if (startRampDown)
      {
         rampStartTime.set(yoTime.getDoubleValue() - rampDownTime.getDoubleValue() * (1.0 - ramp.getDoubleValue()));
         hysteresisState.set(HysteresisState.RAMP_DOWN);
      }
      else if (timeInRampUp >= rampUpTime.getDoubleValue())
      {
         rampStartTime.set(Double.NaN);
         hysteresisState.set(HysteresisState.MAX_HYSTERESIS);
      }
      else
      {
         hysteresisState.set(HysteresisState.RAMP_UP);
      }
   }

   private void updateMaxHysteresisState()
   {
      double tau_off_hyst = torqueHysteresisAmplitude.getDoubleValue();
      tau_off_hyst *= hysteresisSign.getDoubleValue();
      set(tau_off_hyst);

      boolean qddDesiredChangedSign = hysteresisSign.getDoubleValue() * joint.getQddDesired() < 0.0;
      boolean startRampDown = !isVelocityLow.getBooleanValue() || !isAccelerationHigh.getBooleanValue() || qddDesiredChangedSign;
      if (startRampDown)
      {
         rampStartTime.set(yoTime.getDoubleValue());
         hysteresisState.set(HysteresisState.RAMP_DOWN);
      }
      else
      {
         hysteresisState.set(HysteresisState.MAX_HYSTERESIS);
      }
   }

   private void updateRampDownState()
   {
      double timeInRampDown = yoTime.getDoubleValue() - rampStartTime.getDoubleValue();
      ramp.set(MathTools.clamp(1.0 - timeInRampDown / rampDownTime.getDoubleValue(), 0.0, 1.0));

      double tau_off_hyst = ramp.getDoubleValue() * torqueHysteresisAmplitude.getDoubleValue();
      tau_off_hyst *= hysteresisSign.getDoubleValue();
      set(tau_off_hyst);

      boolean startRampUp = isVelocityLow.getBooleanValue() && isAccelerationHigh.getBooleanValue();
      if (startRampUp)
      {
         rampStartTime.set(yoTime.getDoubleValue() - rampUpTime.getDoubleValue() * ramp.getDoubleValue());
         hysteresisState.set(HysteresisState.RAMP_UP);
      }
      else if (timeInRampDown >= rampDownTime.getDoubleValue())
      {
         rampStartTime.set(Double.NaN);
         hysteresisState.set(HysteresisState.ZERO);
      }
      else
      {
         hysteresisState.set(HysteresisState.RAMP_DOWN);
      }
   }

   private void checkAcceleration()
   {
      isAccelerationHigh.set(Math.abs(joint.getQddDesired()) > jointAccelerationMin.getDoubleValue());
   }

   private void checkVelocity()
   {
      isVelocityLow.set(Math.abs(joint.getQd()) < jointVelocityMax.getDoubleValue());
   }
}
