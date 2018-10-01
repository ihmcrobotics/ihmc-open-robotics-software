package us.ihmc.sensorProcessing.outputData;

public interface JointDesiredOutputBasics extends JointDesiredOutputReadOnly
{
   void clear();

   void set(JointDesiredOutputReadOnly other);

   void completeWith(JointDesiredOutputReadOnly other);

   void setControlMode(JointDesiredControlMode controlMode);

   void setDesiredTorque(double tau);

   void setDesiredPosition(double q);

   void setDesiredVelocity(double qd);

   void setDesiredAcceleration(double qdd);

   void setResetIntegrators(boolean reset);

   void setMasterGain(double masterGain);

   void setVelocityScaling(double velocityScaling);

   void setVelocityIntegrationBreakFrequency(double velocityIntegrationBreakFrequency);

   void setPositionIntegrationBreakFrequency(double positionIntegrationBreakFrequency);

   void setMaxPositionError(double maxPositionError);

   void setMaxVelocityError(double maxVelocityError);
}