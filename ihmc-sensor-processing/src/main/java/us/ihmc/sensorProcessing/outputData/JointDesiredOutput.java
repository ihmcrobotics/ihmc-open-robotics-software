package us.ihmc.sensorProcessing.outputData;

public class JointDesiredOutput implements JointDesiredOutputBasics
{
   private JointDesiredControlMode controlMode;
   private JointDesiredLoadMode loadMode;

   private double desiredTorque = Double.NaN;
   private double desiredPosition = Double.NaN;
   private double desiredVelocity = Double.NaN;
   private double desiredAcceleration = Double.NaN;
   private boolean resetIntegrators = false;

   private double stiffness = Double.NaN;
   private double damping = Double.NaN;
   private double masterGain = Double.NaN;

   private double velocityScaling = Double.NaN;

   private double velocityIntegrationBreakFrequency = Double.NaN;
   private double positionIntegrationBreakFrequency = Double.NaN;
   private double positionIntegrationMaxError = Double.NaN;
   private double velocityIntegrationMaxError = Double.NaN;

   private double positionFeedbackMaxError = Double.NaN;
   private double velocityFeedbackMaxError = Double.NaN;

   public JointDesiredOutput()
   {
      clear();
   }

   @Override
   public void clear()
   {
      controlMode = null;
      loadMode = null;
      desiredTorque = Double.NaN;
      desiredPosition = Double.NaN;
      desiredVelocity = Double.NaN;
      desiredAcceleration = Double.NaN;
      resetIntegrators = false;

      stiffness = Double.NaN;
      damping = Double.NaN;
      masterGain = Double.NaN;

      velocityScaling = Double.NaN;

      velocityIntegrationBreakFrequency = Double.NaN;
      positionIntegrationBreakFrequency = Double.NaN;
      positionIntegrationMaxError = Double.NaN;
      velocityIntegrationMaxError = Double.NaN;

      positionFeedbackMaxError = Double.NaN;
      velocityFeedbackMaxError = Double.NaN;
   }

   @Override
   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   @Override
   public void setLoadMode(JointDesiredLoadMode loadMode)
   {
      this.loadMode = loadMode;
   }

   @Override
   public void setDesiredTorque(double tau)
   {
      desiredTorque = tau;
   }

   @Override
   public void setDesiredPosition(double q)
   {
      desiredPosition = q;
   }

   @Override
   public void setDesiredVelocity(double qd)
   {
      desiredVelocity = qd;
   }

   @Override
   public void setDesiredAcceleration(double qdd)
   {
      desiredAcceleration = qdd;
   }

   @Override
   public void setResetIntegrators(boolean reset)
   {
      resetIntegrators = reset;
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      return controlMode;
   }

   @Override
   public JointDesiredLoadMode getLoadMode()
   {
      return loadMode;
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
   public double getStiffness()
   {
      return stiffness;
   }

   @Override
   public double getDamping()
   {
      return damping;
   }

   @Override
   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   @Override
   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   @Override
   public double getMasterGain()
   {
      return masterGain;
   }

   @Override
   public void setMasterGain(double masterGain)
   {
      this.masterGain = masterGain;
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling;
   }

   @Override
   public void setVelocityScaling(double velocityScaling)
   {
      this.velocityScaling = velocityScaling;
   }

   @Override
   public double getVelocityIntegrationBreakFrequency()
   {
      return velocityIntegrationBreakFrequency;
   }

   @Override
   public void setVelocityIntegrationBreakFrequency(double velocityIntegrationBreakFrequency)
   {
      this.velocityIntegrationBreakFrequency = velocityIntegrationBreakFrequency;
   }

   @Override
   public double getPositionIntegrationBreakFrequency()
   {
      return positionIntegrationBreakFrequency;
   }

   @Override
   public void setPositionIntegrationBreakFrequency(double positionIntegrationBreakFrequency)
   {
      this.positionIntegrationBreakFrequency = positionIntegrationBreakFrequency;
   }

   @Override
   public double getPositionIntegrationMaxError()
   {
      return positionIntegrationMaxError;
   }

   @Override
   public void setPositionIntegrationMaxError(double maxPositionError)
   {
      this.positionIntegrationMaxError = maxPositionError;
   }

   @Override
   public double getVelocityIntegrationMaxError()
   {
      return velocityIntegrationMaxError;
   }

   @Override
   public void setVelocityIntegrationMaxError(double maxVelocityError)
   {
      this.velocityIntegrationMaxError = maxVelocityError;
   }

   @Override
   public double getPositionFeedbackMaxError()
   {
      return positionFeedbackMaxError;
   }

   @Override
   public double getVelocityFeedbackMaxError()
   {
      return velocityFeedbackMaxError;
   }

   @Override
   public void setPositionFeedbackMaxError(double positionFeedbackMaxError)
   {
      this.positionFeedbackMaxError = positionFeedbackMaxError;
   }

   @Override
   public void setVelocityFeedbackMaxError(double velocityFeedbackMaxError)
   {
      this.velocityFeedbackMaxError = velocityFeedbackMaxError;
   }

   public void set(JointDesiredOutput other)
   {
      JointDesiredOutputBasics.super.set(other);
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