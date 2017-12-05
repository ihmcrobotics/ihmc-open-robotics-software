package us.ihmc.sensorProcessing.outputData;

public class JointDesiredOutput extends JointDesiredOutputReadOnly
{
   private JointDesiredControlMode controlMode;

   private double desiredTorque = Double.NaN;
   private double desiredPosition = Double.NaN;
   private double desiredVelocity = Double.NaN;
   private double desiredAcceleration = Double.NaN;
   private boolean resetIntegrators = false;

   private double stiffness = Double.NaN;
   private double damping = Double.NaN;
   private double masterGain = Double.NaN;

   private double velocityScaling = Double.NaN;
   private double velocityIntegrationLeakRate = Double.NaN;
   private double positionIntegrationLeakRate = Double.NaN;

   public JointDesiredOutput()
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
      resetIntegrators = false;

      stiffness = Double.NaN;
      damping = Double.NaN;
      masterGain = Double.NaN;

      velocityScaling = Double.NaN;
      velocityIntegrationLeakRate = Double.NaN;
      positionIntegrationLeakRate = Double.NaN;
   }

   public void set(JointDesiredOutputReadOnly other)
   {
      controlMode = other.getControlMode();
      desiredTorque = other.getDesiredTorque();
      desiredPosition = other.getDesiredPosition();
      desiredVelocity = other.getDesiredVelocity();
      desiredAcceleration = other.getDesiredAcceleration();
      resetIntegrators = other.peekResetIntegratorsRequest();
      stiffness = other.getStiffness();
      damping = other.getDamping();
      masterGain = other.getMasterGain();
      velocityScaling = other.getVelocityScaling();
      velocityIntegrationLeakRate = other.getVelocityIntegrationLeakRate();
      positionIntegrationLeakRate = other.getPositionIntegrationLeakRate();
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(JointDesiredOutputReadOnly other)
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
      if (!peekResetIntegratorsRequest())
         resetIntegrators = other.peekResetIntegratorsRequest();
      if(!hasStiffness())
         stiffness = other.getStiffness();
      if(!hasDamping())
         damping = other.getDamping();
      if (!hasMasterGain())
         masterGain = other.getMasterGain();
      if (!hasVelocityScaling())
         velocityScaling = other.getVelocityScaling();
      if (!hasVelocityIntegrationLeakRate())
         velocityIntegrationLeakRate = other.getVelocityIntegrationLeakRate();
      if (!hasPositionIntegrationLeakRate())
         positionIntegrationLeakRate = other.getPositionIntegrationLeakRate();
   }

   public void setControlMode(JointDesiredControlMode controlMode)
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
   public JointDesiredControlMode getControlMode()
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
   public boolean hasStiffness()
   {
      return !Double.isNaN(stiffness);
   }

   @Override
   public boolean hasDamping()
   {
      return !Double.isNaN(damping);
   }

   @Override
   public double getStiffness()
   {
      return stiffness;
   }

   @Override
   public double getDamping()
   {
      return damping;
   }

   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   @Override
   public boolean hasMasterGain()
   {
      return !Double.isNaN(masterGain);
   }

   @Override
   public double getMasterGain()
   {
      return masterGain;
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain = masterGain;
   }

   @Override
   public boolean hasVelocityScaling()
   {
      return !Double.isNaN(velocityScaling);
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling;
   }

   public void setVelocityScaling(double velocityScaling)
   {
      this.velocityScaling = velocityScaling;
   }

   @Override
   public boolean hasVelocityIntegrationLeakRate()
   {
      return !Double.isNaN(velocityIntegrationLeakRate);
   }

   @Override
   public double getVelocityIntegrationLeakRate()
   {
      return velocityIntegrationLeakRate;
   }

   public void setVelocityIntegrationLeakRate(double velocityIntegrationLeakRate)
   {
      this.velocityIntegrationLeakRate = velocityIntegrationLeakRate;
   }

   @Override
   public boolean hasPositionIntegrationLeakRate()
   {
      return !Double.isNaN(positionIntegrationLeakRate);
   }

   @Override
   public double getPositionIntegrationLeakRate()
   {
      return positionIntegrationLeakRate;
   }

   public void setPositionIntegrationLeakRate(double positionIntegrationLeakRate)
   {
      this.positionIntegrationLeakRate = positionIntegrationLeakRate;
   }
}