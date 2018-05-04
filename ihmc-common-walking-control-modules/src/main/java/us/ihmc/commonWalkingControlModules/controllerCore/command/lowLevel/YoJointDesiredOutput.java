package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoJointDesiredOutput extends JointDesiredOutputReadOnly
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
   private final YoDouble maxPositionError;
   private final YoDouble maxVelocityError;

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
      maxPositionError = new YoDouble(namePrefix + "MaxPositionError" + suffixString, registry);
      maxVelocityError = new YoDouble(namePrefix + "MaxVelocityError" + suffixString, registry);

      clear();
   }

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
      maxPositionError.set(Double.NaN);
      maxVelocityError.set(Double.NaN);
      resetIntegrators.set(false);
   }

   public void set(JointDesiredOutputReadOnly other)
   {
      controlMode.set(other.getControlMode());
      desiredTorque.set(other.getDesiredTorque());
      desiredPosition.set(other.getDesiredPosition());
      desiredVelocity.set(other.getDesiredVelocity());
      desiredAcceleration.set(other.getDesiredAcceleration());
      resetIntegrators.set(other.peekResetIntegratorsRequest());
      stiffness.set(other.getStiffness());
      damping.set(other.getDamping());
      masterGain.set(other.getMasterGain());
      velocityScaling.set(other.getVelocityScaling());
      velocityIntegrationBreakFrequency.set(other.getVelocityIntegrationBreakFrequency());
      positionIntegrationBreakFrequency.set(other.getPositionIntegrationBreakFrequency());
      maxPositionError.set(other.getMaxPositionError());
      maxVelocityError.set(other.getMaxVelocityError());
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(JointDesiredOutputReadOnly other)
   {
      if (!hasControlMode())
         controlMode.set(other.getControlMode());
      if (!hasDesiredTorque())
         desiredTorque.set(other.getDesiredTorque());
      if (!hasDesiredPosition())
         desiredPosition.set(other.getDesiredPosition());
      if (!hasDesiredVelocity())
         desiredVelocity.set(other.getDesiredVelocity());
      if (!hasDesiredAcceleration())
         desiredAcceleration.set(other.getDesiredAcceleration());
      if (!peekResetIntegratorsRequest())
         resetIntegrators.set(other.peekResetIntegratorsRequest());
      if(!hasStiffness())
         stiffness.set(other.getStiffness());
      if(!hasDamping())
         damping.set(other.getDamping());
      if(!hasMasterGain())
         masterGain.set(other.getMasterGain());
      if(!hasVelocityScaling())
         velocityScaling.set(other.getVelocityScaling());
      if(!hasVelocityIntegrationBreakFrequency())
         velocityIntegrationBreakFrequency.set(other.getVelocityIntegrationBreakFrequency());
      if(!hasPositionIntegrationBreakFrequency())
         positionIntegrationBreakFrequency.set(other.getPositionIntegrationBreakFrequency());
      if(!hasMaxPositionError())
         maxPositionError.set(other.getMaxPositionError());
      if(!hasMaxVelocityError())
         maxVelocityError.set(other.getMaxVelocityError());
   }

   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   public void setDesiredTorque(double tau)
   {
      desiredTorque.set(tau);
   }

   public void setDesiredPosition(double q)
   {
      desiredPosition.set(q);
   }

   public void setDesiredVelocity(double qd)
   {
      desiredVelocity.set(qd);
   }

   public void setDesiredAcceleration(double qdd)
   {
      desiredAcceleration.set(qdd);
   }

   public void setResetIntegrators(boolean reset)
   {
      resetIntegrators.set(reset);
   }

   @Override
   public boolean hasControlMode()
   {
      return controlMode.getEnumValue() != null;
   }

   @Override
   public boolean hasDesiredTorque()
   {
      return !desiredTorque.isNaN();
   }

   @Override
   public boolean hasDesiredPosition()
   {
      return !desiredPosition.isNaN();
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return !desiredVelocity.isNaN();
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return !desiredAcceleration.isNaN();
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
   public boolean hasStiffness()
   {
      return !stiffness.isNaN();
   }

   @Override
   public boolean hasDamping()
   {
      return !damping.isNaN();
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

   public void setStiffness(double stiffness)
   {
      this.stiffness.set(stiffness);
   }

   public void setDamping(double damping)
   {
      this.damping.set(damping);
   }

   @Override
   public boolean hasMasterGain()
   {
      return !masterGain.isNaN();
   }

   @Override
   public double getMasterGain()
   {
      return masterGain.getDoubleValue();
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain.set(masterGain);
   }

   @Override
   public boolean hasVelocityScaling()
   {
      return !velocityScaling.isNaN();
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling.getDoubleValue();
   }

   public void setVelocityScaling(double velocityScaling)
   {
      this.velocityScaling.set(velocityScaling);
   }

   @Override
   public boolean hasVelocityIntegrationBreakFrequency()
   {
      return !velocityIntegrationBreakFrequency.isNaN();
   }

   @Override
   public double getVelocityIntegrationBreakFrequency()
   {
      return velocityIntegrationBreakFrequency.getDoubleValue();
   }

   public void setVelocityIntegrationBreakFrequency(double velocityIntegrationBreakFrequency)
   {
      this.velocityIntegrationBreakFrequency.set(velocityIntegrationBreakFrequency);
   }

   @Override
   public boolean hasPositionIntegrationBreakFrequency()
   {
      return !positionIntegrationBreakFrequency.isNaN();
   }

   @Override
   public double getPositionIntegrationBreakFrequency()
   {
      return positionIntegrationBreakFrequency.getDoubleValue();
   }

   public void setPositionIntegrationBreakFrequency(double positionIntegrationBreakFrequency)
   {
      this.positionIntegrationBreakFrequency.set(positionIntegrationBreakFrequency);
   }

   @Override
   public boolean hasMaxPositionError()
   {
      return maxPositionError.isNaN();
   }

   @Override
   public double getMaxPositionError()
   {
      return maxPositionError.getDoubleValue();
   }

   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError.set(maxPositionError);
   }

   @Override
   public boolean hasMaxVelocityError()
   {
      return maxVelocityError.isNaN();
   }

   @Override
   public double getMaxVelocityError()
   {
      return maxVelocityError.getDoubleValue();
   }

   public void setMaxVelocityError(double maxVelocityError)
   {
      this.maxVelocityError.set(maxVelocityError);
   }
}
