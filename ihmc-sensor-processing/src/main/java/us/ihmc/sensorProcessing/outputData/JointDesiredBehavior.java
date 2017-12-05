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

   public JointDesiredBehavior(JointDesiredControlMode controlMode, double stiffness, double damping, double masterGain)
   {
      this.controlMode = controlMode;
      this.stiffness = stiffness;
      this.damping = damping;
      this.masterGain = masterGain;
   }

   public void set(JointDesiredBehaviorReadOnly other)
   {
      setControlMode(other.getControlMode());
      setStiffness(other.getStiffness());
      setDamping(other.getDamping());
      setMasterGain(other.getMasterGain());
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

}
