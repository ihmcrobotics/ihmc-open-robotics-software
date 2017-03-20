package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class TorqueHysteresisCompensatorYoVariable extends DoubleYoVariable
{
   public enum HysteresisState
   {
      ZERO, RAMP_UP, RAMP_DOWN, MAX_HYSTERESIS
   };

   private final OneDoFJoint joint;
   private final DoubleYoVariable torqueHysteresisAmplitude;
   private final DoubleYoVariable jointAccelerationMin;
   private final DoubleYoVariable jointVelocityMax;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable ramp;
   private final DoubleYoVariable rampUpTime;
   private final DoubleYoVariable rampDownTime;
   private final DoubleYoVariable rampStartTime;

   private final EnumYoVariable<HysteresisState> hysteresisState;

   private final BooleanYoVariable isAccelerationHigh;
   private final BooleanYoVariable isVelocityLow;

   private final DoubleYoVariable hysteresisSign;

   private final BooleanYoVariable enabled;

   public TorqueHysteresisCompensatorYoVariable(String prefix, OneDoFJoint joint, DoubleYoVariable torqueHysteresisAmplitude,
         DoubleYoVariable jointAccelerationMin, DoubleYoVariable jointVelocityMax, DoubleYoVariable rampTime, DoubleYoVariable yoTime,
         YoVariableRegistry registry)
   {
      this(prefix, joint, torqueHysteresisAmplitude, jointAccelerationMin, jointVelocityMax, rampTime, rampTime, yoTime, registry);
   }

   public TorqueHysteresisCompensatorYoVariable(String prefix, OneDoFJoint joint, DoubleYoVariable torqueHysteresisAmplitude,
         DoubleYoVariable jointAccelerationMin, DoubleYoVariable jointVelocityMax, DoubleYoVariable rampUpTime, DoubleYoVariable rampDownTime,
         DoubleYoVariable yoTime, YoVariableRegistry registry)
   {
      super(prefix + joint.getName(), registry);
      this.joint = joint;
      this.torqueHysteresisAmplitude = torqueHysteresisAmplitude;
      this.jointAccelerationMin = jointAccelerationMin;
      this.jointVelocityMax = jointVelocityMax;
      this.rampUpTime = rampUpTime;
      this.rampDownTime = rampDownTime;
      this.yoTime = yoTime;

      ramp = new DoubleYoVariable(getName() + "Ramp", registry);
      rampStartTime = new DoubleYoVariable(getName() + "RampStartTime", registry);
      hysteresisState = new EnumYoVariable<>(getName() + "State", registry, HysteresisState.class, false);

      isAccelerationHigh = new BooleanYoVariable(getName() + "IsQddHigh", registry);
      isVelocityLow = new BooleanYoVariable(getName() + "IsQdLow", registry);

      hysteresisSign = new DoubleYoVariable(getName() + "HysteresisSign", registry);

      enabled = new BooleanYoVariable(getName() + "Enabled", registry);
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
