package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class LowLevelJointData implements LowLevelJointDataReadOnly
{
   private LowLevelJointControlMode controlMode;
   private double desiredTorque = Double.NaN;
   private double desiredPosition = Double.NaN;
   private double desiredVelocity = Double.NaN;
   private double desiredAcceleration = Double.NaN;
   private double desiredCurrent = Double.NaN;
   private boolean resetIntegrators = false;
   
   private double kp = Double.NaN;
   private double kd = Double.NaN;

   public LowLevelJointData()
   {
      clear();
   }

   public void clear()
   {
      controlMode = null;
      desiredTorque = Double.NaN;
      desiredPosition = Double.NaN;
      desiredVelocity = Double.NaN;
      desiredAcceleration = Double.NaN;
      desiredCurrent = Double.NaN;
      resetIntegrators = false;
      
      kp = Double.NaN;
      kd = Double.NaN;
   }

   public void set(LowLevelJointDataReadOnly other)
   {
      controlMode = other.getControlMode();
      desiredTorque = other.getDesiredTorque();
      desiredPosition = other.getDesiredPosition();
      desiredVelocity = other.getDesiredVelocity();
      desiredAcceleration = other.getDesiredAcceleration();
      desiredCurrent = other.getDesiredCurrent();
      resetIntegrators = other.peekResetIntegratorsRequest();
      kp = other.getKp();
      kd = other.getKd();
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
         desiredTorque = other.getDesiredTorque();
      if (!hasDesiredPosition())
         desiredPosition = other.getDesiredPosition();
      if (!hasDesiredVelocity())
         desiredVelocity = other.getDesiredVelocity();
      if (!hasDesiredAcceleration())
         desiredAcceleration = other.getDesiredAcceleration();
      if (!hasDesiredCurrent())
         desiredCurrent = other.getDesiredCurrent();
      if (!peekResetIntegratorsRequest())
         resetIntegrators = other.peekResetIntegratorsRequest();
      if(!hasKp())
         kp = other.getKp();
      if(!hasKd())
         kd = other.getKd();
   }
   
   public void setDesiredsFromOneDoFJoint(OneDoFJoint jointToExtractDesiredsFrom)
   {
      setDesiredTorque(jointToExtractDesiredsFrom.getTau());
      setDesiredPosition(jointToExtractDesiredsFrom.getqDesired());
      setDesiredVelocity(jointToExtractDesiredsFrom.getQdDesired());
      setDesiredAcceleration(jointToExtractDesiredsFrom.getQddDesired());
      setResetIntegrators(jointToExtractDesiredsFrom.getResetIntegrator());
      setKp(jointToExtractDesiredsFrom.getKp());
      setKd(jointToExtractDesiredsFrom.getKd());
   }

   public void setControlMode(LowLevelJointControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   public void setDesiredTorque(double tau)
   {
      desiredTorque = tau;
   }

   public void setDesiredPosition(double q)
   {
      desiredPosition = q;
   }

   public void setDesiredVelocity(double qd)
   {
      desiredVelocity = qd;
   }

   public void setDesiredAcceleration(double qdd)
   {
      desiredAcceleration = qdd;
   }

   public void setDesiredCurrent(double i)
   {
      desiredCurrent = i;
   }

   public void setResetIntegrators(boolean reset)
   {
      resetIntegrators = reset;
   }

   @Override
   public boolean hasControlMode()
   {
      return controlMode != null;
   }

   @Override
   public boolean hasDesiredTorque()
   {
      return !Double.isNaN(desiredTorque);
   }

   @Override
   public boolean hasDesiredPosition()
   {
      return !Double.isNaN(desiredPosition);
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return !Double.isNaN(desiredVelocity);
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return !Double.isNaN(desiredAcceleration);
   }

   @Override
   public boolean hasDesiredCurrent()
   {
      return !Double.isNaN(desiredCurrent);
   }

   @Override
   public LowLevelJointControlMode getControlMode()
   {
      return controlMode;
   }

   @Override
   public double getDesiredTorque()
   {
      return desiredTorque;
   }

   @Override
   public double getDesiredPosition()
   {
      return desiredPosition;
   }

   @Override
   public double getDesiredVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public double getDesiredAcceleration()
   {
      return desiredAcceleration;
   }

   @Override
   public double getDesiredCurrent()
   {
      return desiredCurrent;
   }

   @Override
   public boolean pollResetIntegratorsRequest()
   {
      boolean resetIntegrators = this.resetIntegrators;
      this.resetIntegrators = false;
      return resetIntegrators;
      
   }

   @Override
   public boolean peekResetIntegratorsRequest()
   {
      return resetIntegrators;
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

   @Override
   public boolean hasKp()
   {
      return !Double.isNaN(kp);
   }

   @Override
   public boolean hasKd()
   {
      return !Double.isNaN(kd);
   }

   @Override
   public double getKp()
   {
      return kp;
   }

   @Override
   public double getKd()
   {
      return kd;
   }
   
   public void setKp(double kp)
   {
      this.kp = kp;
   }

   public void setKd(double kd)
   {
      this.kd = kd;
   }
}