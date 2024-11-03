package us.ihmc.sensorProcessing.outputData;

/**
 * Mutable implementation if the {@link JointDesiredBehaviorReadOnly} interface.
 */
public class JointDesiredBehavior implements JointDesiredBehaviorReadOnly
{
   private JointDesiredControlMode controlMode;
   private double stiffness;
   private double damping;
   private double masterGain;
   private double velocityScaling;
   private double maxPositionError;
   private double maxVelocityError;

   public JointDesiredBehavior(JointDesiredControlMode controlMode)
   {
      this(controlMode, 0.0, 0.0, 1.0, 1.0);
   }

   public JointDesiredBehavior(JointDesiredControlMode controlMode, double stiffness, double damping)
   {
      this(controlMode, stiffness, damping, 1.0, 1.0);
   }

   public JointDesiredBehavior(JointDesiredControlMode controlMode, double stiffness, double damping, double masterGain,
                               double velocityScaling)
   {
      this.controlMode = controlMode;
      this.stiffness = stiffness;
      this.damping = damping;
      this.masterGain = masterGain;
      this.velocityScaling = velocityScaling;
      this.maxPositionError = Double.POSITIVE_INFINITY;
      this.maxVelocityError = Double.POSITIVE_INFINITY;
   }

   public void set(JointDesiredBehaviorReadOnly other)
   {
      setControlMode(other.getControlMode());
      setStiffness(other.getStiffness());
      setDamping(other.getDamping());
      setMasterGain(other.getMasterGain());
      setVelocityScaling(other.getVelocityScaling());
      setMaxPositionError(other.getMaxPositionError());
      setMaxVelocityError(other.getMaxVelocityError());
   }

   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain = masterGain;
   }

   public void setVelocityScaling(double velocityScaling)
   {
      this.velocityScaling = velocityScaling;
   }

   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError = maxPositionError;
   }

   public void setMaxVelocityError(double maxVelocityError)
   {
      this.maxVelocityError = maxVelocityError;
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      return controlMode;
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
   public double getMasterGain()
   {
      return masterGain;
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling;
   }

   @Override
   public double getMaxPositionError()
   {
      return maxPositionError;
   }

   @Override
   public double getMaxVelocityError()
   {
      return maxVelocityError;
   }
}
