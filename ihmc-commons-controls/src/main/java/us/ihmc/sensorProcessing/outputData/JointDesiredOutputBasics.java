package us.ihmc.sensorProcessing.outputData;

public interface JointDesiredOutputBasics extends JointDesiredOutputReadOnly
{
   void clear();

   default void set(JointDesiredOutputReadOnly other)
   {
      setControlMode(other.getControlMode());
      setLoadMode(other.getLoadMode());
      setDesiredTorque(other.getDesiredTorque());
      setDesiredPosition(other.getDesiredPosition());
      setDesiredVelocity(other.getDesiredVelocity());
      setDesiredAcceleration(other.getDesiredAcceleration());
      setResetIntegrators(other.peekResetIntegratorsRequest());
      setStiffness(other.getStiffness());
      setDamping(other.getDamping());
      setMasterGain(other.getMasterGain());
      setVelocityScaling(other.getVelocityScaling());
      setVelocityIntegrationBreakFrequency(other.getVelocityIntegrationBreakFrequency());
      setPositionIntegrationBreakFrequency(other.getPositionIntegrationBreakFrequency());
      setPositionIntegrationMaxError(other.getPositionIntegrationMaxError());
      setVelocityIntegrationMaxError(other.getVelocityIntegrationMaxError());
      setPositionFeedbackMaxError(other.getPositionFeedbackMaxError());
      setVelocityFeedbackMaxError(other.getVelocityFeedbackMaxError());
      setMaxTorque(other.getMaxTorque());
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   default void completeWith(JointDesiredOutputReadOnly other)
   {
      if (!hasControlMode())
         setControlMode(other.getControlMode());
      if (!hasLoadMode())
         setLoadMode(other.getLoadMode());
      if (!hasDesiredTorque())
         setDesiredTorque(other.getDesiredTorque());
      if (!hasDesiredPosition())
         setDesiredPosition(other.getDesiredPosition());
      if (!hasDesiredVelocity())
         setDesiredVelocity(other.getDesiredVelocity());
      if (!hasDesiredAcceleration())
         setDesiredAcceleration(other.getDesiredAcceleration());
      if (!peekResetIntegratorsRequest())
         setResetIntegrators(other.peekResetIntegratorsRequest());
      if (!hasStiffness())
         setStiffness(other.getStiffness());
      if (!hasDamping())
         setDamping(other.getDamping());
      if (!hasMasterGain())
         setMasterGain(other.getMasterGain());
      if (!hasVelocityScaling())
         setVelocityScaling(other.getVelocityScaling());
      if (!hasVelocityIntegrationBreakFrequency())
         setVelocityIntegrationBreakFrequency(other.getVelocityIntegrationBreakFrequency());
      if (!hasPositionIntegrationBreakFrequency())
         setPositionIntegrationBreakFrequency(other.getPositionIntegrationBreakFrequency());
      if (!hasPositionIntegrationMaxError())
         setPositionIntegrationMaxError(other.getPositionIntegrationMaxError());
      if (!hasVelocityIntegrationMaxError())
         setVelocityIntegrationMaxError(other.getVelocityIntegrationMaxError());
      if (!hasPositionFeedbackMaxError())
         setPositionFeedbackMaxError(other.getPositionFeedbackMaxError());
      if (!hasVelocityFeedbackMaxError())
         setVelocityFeedbackMaxError(other.getVelocityFeedbackMaxError());
      if(!hasMaxTorque())
         setMaxTorque(other.getMaxTorque());
   }

   void setControlMode(JointDesiredControlMode controlMode);

   void setLoadMode(JointDesiredLoadMode loadMode);

   void setDesiredTorque(double tau);

   void setDesiredPosition(double q);

   void setDesiredVelocity(double qd);

   void setDesiredAcceleration(double qdd);

   void setResetIntegrators(boolean reset);

   void setStiffness(double stiffness);

   void setDamping(double damping);

   void setMasterGain(double masterGain);

   void setVelocityScaling(double velocityScaling);

   void setVelocityIntegrationBreakFrequency(double velocityIntegrationBreakFrequency);

   void setPositionIntegrationBreakFrequency(double positionIntegrationBreakFrequency);

   void setPositionIntegrationMaxError(double maxPositionError);

   void setVelocityIntegrationMaxError(double maxVelocityError);

   void setPositionFeedbackMaxError(double positionFeedbackMaxError);

   void setVelocityFeedbackMaxError(double velocityFeedbackMaxError);
   
   void setMaxTorque(double maxTorque);
}
