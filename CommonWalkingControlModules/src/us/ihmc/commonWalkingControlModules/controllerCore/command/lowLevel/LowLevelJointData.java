package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.concurrent.atomic.AtomicBoolean;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class LowLevelJointData implements LowLevelJointDataReadOnly
{
   private LowLevelJointControlMode controlMode;
   private final AtomicDouble desiredTorque = new AtomicDouble(Double.NaN);
   private final AtomicDouble desiredPosition = new AtomicDouble(Double.NaN);
   private final AtomicDouble desiredVelocity = new AtomicDouble(Double.NaN);
   private final AtomicDouble desiredAcceleration = new AtomicDouble(Double.NaN);
   private final AtomicDouble desiredCurrent = new AtomicDouble(Double.NaN);
   private final AtomicBoolean resetIntegrators = new AtomicBoolean(false);

   public LowLevelJointData()
   {
      clear();
   }

   public void clear()
   {
      controlMode = null;
      desiredTorque.set(Double.NaN);
      desiredPosition.set(Double.NaN);
      desiredVelocity.set(Double.NaN);
      desiredAcceleration.set(Double.NaN);
      desiredCurrent.set(Double.NaN);
      resetIntegrators.set(false);
   }

   public void set(LowLevelJointDataReadOnly other)
   {
      controlMode = other.getControlMode();
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
         controlMode = other.getControlMode();
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
      this.controlMode = controlMode;
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
      return controlMode != null;
   }

   @Override
   public boolean hasDesiredTorque()
   {
      return !Double.isNaN(desiredTorque.get());
   }

   @Override
   public boolean hasDesiredPosition()
   {
      return !Double.isNaN(desiredPosition.get());
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return !Double.isNaN(desiredVelocity.get());
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return !Double.isNaN(desiredAcceleration.get());
   }

   @Override
   public boolean hasDesiredCurrent()
   {
      return !Double.isNaN(desiredCurrent.get());
   }

   @Override
   public LowLevelJointControlMode getControlMode()
   {
      return controlMode;
   }

   @Override
   public double getDesiredTorque()
   {
      return desiredTorque.doubleValue();
   }

   @Override
   public double getDesiredPosition()
   {
      return desiredPosition.doubleValue();
   }

   @Override
   public double getDesiredVelocity()
   {
      return desiredVelocity.doubleValue();
   }

   @Override
   public double getDesiredAcceleration()
   {
      return desiredAcceleration.doubleValue();
   }

   @Override
   public double getDesiredCurrent()
   {
      return desiredCurrent.doubleValue();
   }

   @Override
   public boolean pollResetIntegratorsRequest()
   {
      return resetIntegrators.getAndSet(false);
   }

   @Override
   public boolean peekResetIntegratorsRequest()
   {
      return resetIntegrators.get();
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