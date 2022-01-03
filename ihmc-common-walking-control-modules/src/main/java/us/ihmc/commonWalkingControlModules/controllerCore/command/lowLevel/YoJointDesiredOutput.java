package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredLoadMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoJointDesiredOutput implements JointDesiredOutputBasics
{
   private final YoEnum<JointDesiredControlMode> controlMode;
   private final YoEnum<JointDesiredLoadMode> loadMode;

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

   private final YoDouble positionFeedbackMaxError;
   private final YoDouble velocityFeedbackMaxError;
   
   private final YoDouble maxTorque;

   public YoJointDesiredOutput(String namePrefix, YoRegistry registry, String suffixString)
   {
      namePrefix += "LowLevel";

      controlMode = new YoEnum<>(namePrefix + "ControlMode" + suffixString, registry, JointDesiredControlMode.class, true);
      loadMode = new YoEnum<>(namePrefix + "LoadMode" + suffixString, registry, JointDesiredLoadMode.class, true);
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

      positionFeedbackMaxError = new YoDouble(namePrefix + "PositionFeedbackMaxError" + suffixString, registry);
      velocityFeedbackMaxError = new YoDouble(namePrefix + "VelocityFeedbackMaxError" + suffixString, registry);

      maxTorque = new YoDouble(namePrefix + "MaxTorque" + suffixString, registry);

      clear();
   }

   @Override
   public void clear()
   {
      controlMode.set(null);
      loadMode.set(null);
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
      positionFeedbackMaxError.set(Double.NaN);
      velocityFeedbackMaxError.set(Double.NaN);
      maxTorque.set(Double.NaN);
      resetIntegrators.set(false);
   }

   @Override
   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   @Override
   public void setLoadMode(JointDesiredLoadMode loadMode)
   {
      this.loadMode.set(loadMode);
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
   public JointDesiredLoadMode getLoadMode()
   {
      return loadMode.getValue();
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
   public double getPositionFeedbackMaxError()
   {
      return positionFeedbackMaxError.getValue();
   }

   @Override
   public void setPositionFeedbackMaxError(double positionFeedbackMaxError)
   {
      this.positionFeedbackMaxError.set(positionFeedbackMaxError);
   }

   @Override
   public double getVelocityFeedbackMaxError()
   {
      return velocityFeedbackMaxError.getValue();
   }

   @Override
   public void setVelocityFeedbackMaxError(double velocityFeedbackMaxError)
   {
      this.velocityFeedbackMaxError.set(velocityFeedbackMaxError);
   }
   

   @Override
   public double getMaxTorque()
   {
      return maxTorque.getValue();
   }

   public void setMaxTorque(double feedbackMaxTorque)
   {
      this.maxTorque.set(feedbackMaxTorque);
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
