package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoJointDesiredOutput implements JointDesiredOutputBasics
{
   private final YoEnum<JointDesiredControlMode> controlMode;

   private final YoDouble desiredTorque;
   private final YoDouble desiredPosition;
   private final YoDouble desiredVelocity;
   private final YoDouble desiredAcceleration;
   private final YoBoolean resetIntegrators;

   private final YoDouble stiffness;
   private final YoDouble damping;
   private final YoDouble masterGain;

   private final YoDouble velocityScaling;
   private final YoDouble velocityIntegrationBreakFrequency;
   private final YoDouble positionIntegrationBreakFrequency;
   private final YoDouble positionIntegrationMaxError;
   private final YoDouble velocityIntegrationMaxError;

   public YoJointDesiredOutput(String namePrefix, YoVariableRegistry registry, String suffixString)
   {
      namePrefix += "LowLevel";

      controlMode = new YoEnum<>(namePrefix + "ControlMode" + suffixString, registry, JointDesiredControlMode.class, true);
      desiredTorque = new YoDouble(namePrefix + "DesiredTorque" + suffixString, registry);
      desiredPosition = new YoDouble(namePrefix + "DesiredPosition" + suffixString, registry);
      desiredVelocity = new YoDouble(namePrefix + "DesiredVelocity" + suffixString, registry);
      desiredAcceleration = new YoDouble(namePrefix + "DesiredAcceleration" + suffixString, registry);
      resetIntegrators = new YoBoolean(namePrefix + "ResetIntegrators" + suffixString, registry);

      stiffness = new YoDouble(namePrefix + "Stiffness" + suffixString, registry);
      damping = new YoDouble(namePrefix + "Damping" + suffixString, registry);
      masterGain = new YoDouble(namePrefix + "MasterGain" + suffixString, registry);

      velocityScaling = new YoDouble(namePrefix + "VelocityScaling" + suffixString, registry);
      velocityIntegrationBreakFrequency = new YoDouble(namePrefix + "VelocityIntegrationBreakFrequency" + suffixString, registry);
      positionIntegrationBreakFrequency = new YoDouble(namePrefix + "PositionIntegrationBreakFrequency" + suffixString, registry);
      positionIntegrationMaxError = new YoDouble(namePrefix + "PositionIntegrationMaxError" + suffixString, registry);
      velocityIntegrationMaxError = new YoDouble(namePrefix + "VelocityIntegrationMaxError" + suffixString, registry);

      clear();
   }

   @Override
   public void clear()
   {
      controlMode.set(null);
      desiredTorque.set(Double.NaN);
      desiredPosition.set(Double.NaN);
      desiredVelocity.set(Double.NaN);
      desiredAcceleration.set(Double.NaN);
      stiffness.set(Double.NaN);
      damping.set(Double.NaN);
      masterGain.set(Double.NaN);
      velocityScaling.set(Double.NaN);
      velocityIntegrationBreakFrequency.set(Double.NaN);
      positionIntegrationBreakFrequency.set(Double.NaN);
      positionIntegrationMaxError.set(Double.NaN);
      velocityIntegrationMaxError.set(Double.NaN);
      resetIntegrators.set(false);
   }

   @Override
   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   @Override
   public void setDesiredTorque(double tau)
   {
      desiredTorque.set(tau);
   }

   @Override
   public void setDesiredPosition(double q)
   {
      desiredPosition.set(q);
   }

   @Override
   public void setDesiredVelocity(double qd)
   {
      desiredVelocity.set(qd);
   }

   @Override
   public void setDesiredAcceleration(double qdd)
   {
      desiredAcceleration.set(qdd);
   }

   @Override
   public void setResetIntegrators(boolean reset)
   {
      resetIntegrators.set(reset);
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      return controlMode.getEnumValue();
   }

   @Override
   public double getDesiredTorque()
   {
      return desiredTorque.getDoubleValue();
   }

   @Override
   public double getDesiredPosition()
   {
      return desiredPosition.getDoubleValue();
   }

   @Override
   public double getDesiredVelocity()
   {
      return desiredVelocity.getDoubleValue();
   }

   @Override
   public double getDesiredAcceleration()
   {
      return desiredAcceleration.getDoubleValue();
   }

   @Override
   public boolean pollResetIntegratorsRequest()
   {
      boolean request = resetIntegrators.getBooleanValue();
      resetIntegrators.set(false);
      return request;
   }

   @Override
   public boolean peekResetIntegratorsRequest()
   {
      return resetIntegrators.getBooleanValue();
   }

   @Override
   public double getStiffness()
   {
      return stiffness.getValue();
   }

   @Override
   public double getDamping()
   {
      return damping.getValue();
   }

   @Override
   public void setStiffness(double stiffness)
   {
      this.stiffness.set(stiffness);
   }

   @Override
   public void setDamping(double damping)
   {
      this.damping.set(damping);
   }

   @Override
   public double getMasterGain()
   {
      return masterGain.getDoubleValue();
   }

   @Override
   public void setMasterGain(double masterGain)
   {
      this.masterGain.set(masterGain);
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling.getDoubleValue();
   }

   @Override
   public void setVelocityScaling(double velocityScaling)
   {
      this.velocityScaling.set(velocityScaling);
   }

   @Override
   public double getVelocityIntegrationBreakFrequency()
   {
      return velocityIntegrationBreakFrequency.getDoubleValue();
   }

   @Override
   public void setVelocityIntegrationBreakFrequency(double velocityIntegrationBreakFrequency)
   {
      this.velocityIntegrationBreakFrequency.set(velocityIntegrationBreakFrequency);
   }

   @Override
   public double getPositionIntegrationBreakFrequency()
   {
      return positionIntegrationBreakFrequency.getDoubleValue();
   }

   @Override
   public void setPositionIntegrationBreakFrequency(double positionIntegrationBreakFrequency)
   {
      this.positionIntegrationBreakFrequency.set(positionIntegrationBreakFrequency);
   }

   @Override
   public double getPositionIntegrationMaxError()
   {
      return positionIntegrationMaxError.getDoubleValue();
   }

   @Override
   public void setPositionIntegrationMaxError(double maxPositionError)
   {
      this.positionIntegrationMaxError.set(maxPositionError);
   }

   @Override
   public double getVelocityIntegrationMaxError()
   {
      return velocityIntegrationMaxError.getDoubleValue();
   }

   @Override
   public void setVelocityIntegrationMaxError(double maxVelocityError)
   {
      this.velocityIntegrationMaxError.set(maxVelocityError);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof JointDesiredOutputReadOnly)
         return JointDesiredOutputBasics.super.equals((JointDesiredOutputReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return getRepresentativeString();
   }
}
