package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class YoLowLevelJointData implements LowLevelJointDataReadOnly
{
   private final EnumYoVariable<LowLevelJointControlMode> controlMode;
   private final DoubleYoVariable desiredTorque;
   private final DoubleYoVariable desiredPosition;
   private final DoubleYoVariable desiredVelocity;
   private final DoubleYoVariable desiredAcceleration;
   private final DoubleYoVariable desiredCurrent;
   private final BooleanYoVariable resetIntegrators;

   public YoLowLevelJointData(String namePrefix, YoVariableRegistry registry, String suffixString)
   {
      namePrefix += "LowLevel";

      controlMode = new EnumYoVariable<>(namePrefix + "ControlMode" + suffixString, registry, LowLevelJointControlMode.class, true);
      desiredTorque = new DoubleYoVariable(namePrefix + "DesiredTorque" + suffixString, registry);
      desiredPosition = new DoubleYoVariable(namePrefix + "DesiredPosition" + suffixString, registry);
      desiredVelocity = new DoubleYoVariable(namePrefix + "DesiredVelocity" + suffixString, registry);
      desiredAcceleration = new DoubleYoVariable(namePrefix + "DesiredAcceleration" + suffixString, registry);
      desiredCurrent = new DoubleYoVariable(namePrefix + "DesiredCurrent" + suffixString, registry);
      resetIntegrators = new BooleanYoVariable(namePrefix + "ResetIntegrators" + suffixString, registry);

      clear();
   }

   public void clear()
   {
      controlMode.set(null);
      desiredTorque.set(Double.NaN);
      desiredPosition.set(Double.NaN);
      desiredVelocity.set(Double.NaN);
      desiredAcceleration.set(Double.NaN);
      desiredCurrent.set(Double.NaN);
      resetIntegrators.set(false);
   }

   public void set(LowLevelJointDataReadOnly other)
   {
      controlMode.set(other.getControlMode());
      desiredTorque.set(other.getDesiredTorque());
      desiredPosition.set(other.getDesiredPosition());
      desiredVelocity.set(other.getDesiredVelocity());
      desiredAcceleration.set(other.getDesiredAcceleration());
      desiredCurrent.set(other.getDesiredCurrent());
      resetIntegrators.set(other.peekResetIntegratorsRequest());
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(LowLevelJointDataReadOnly other)
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
      if (!hasDesiredCurrent())
         desiredCurrent.set(other.getDesiredCurrent());
      if (!peekResetIntegratorsRequest())
         resetIntegrators.set(other.peekResetIntegratorsRequest());
   }
   
   public void setDesiredsFromOneDoFJoint(OneDoFJoint jointToExtractDesiredsFrom)
   {
      setDesiredTorque(jointToExtractDesiredsFrom.getTau());
      setDesiredPosition(jointToExtractDesiredsFrom.getqDesired());
      setDesiredVelocity(jointToExtractDesiredsFrom.getQdDesired());
      setDesiredAcceleration(jointToExtractDesiredsFrom.getQddDesired());
      setResetIntegrators(jointToExtractDesiredsFrom.getResetIntegrator());
   }

   public void setControlMode(LowLevelJointControlMode controlMode)
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

   public void setDesiredCurrent(double i)
   {
      desiredCurrent.set(i);
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
   public boolean hasDesiredCurrent()
   {
      return !desiredCurrent.isNaN();
   }

   @Override
   public LowLevelJointControlMode getControlMode()
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
   public double getDesiredCurrent()
   {
      return desiredCurrent.getDoubleValue();
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
   public String toString()
   {
      String ret = "controlMode = " + getControlMode() + "\n";
      ret += "desiredTorque = " + getDesiredTorque() + "\n";
      ret += "desiredPosition = " + getDesiredPosition() + "\n";
      ret += "desiredVelocity = " + getDesiredVelocity() + "\n";
      ret += "desiredAcceleration = " + getDesiredAcceleration() + "\n";
      ret += "desiredCurrent = " + getDesiredCurrent() + "\n";
      return ret;
   }
}
