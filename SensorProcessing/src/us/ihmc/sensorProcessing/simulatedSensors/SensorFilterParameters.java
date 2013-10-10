package us.ihmc.sensorProcessing.simulatedSensors;

public class SensorFilterParameters
{
   private final double jointPositionFilterFrequencyInHertz;
   private final double jointVelocityFilterFrequencyInHertz;
   private final double orientationFilterFrequencyInHertz;
   private final double angularVelocityFilterFrequencyInHertz;
   private final double linearAccelerationFilterFrequencyInHertz;
   private final double updateDT;

   public SensorFilterParameters(double jointPositionFilterFrequencyInHertz, double jointVelocityFilterFrequencyInHertz,
                                 double orientationFilterFrequencyInHertz, double angularVelocityFilterFrequencyInHertz,
                                 double linearAccelerationFilterFrequencyInHertz, double updateDT)
   {
      this.jointPositionFilterFrequencyInHertz = jointPositionFilterFrequencyInHertz;
      this.jointVelocityFilterFrequencyInHertz = jointVelocityFilterFrequencyInHertz;
      this.orientationFilterFrequencyInHertz = orientationFilterFrequencyInHertz;
      this.angularVelocityFilterFrequencyInHertz = angularVelocityFilterFrequencyInHertz;
      this.linearAccelerationFilterFrequencyInHertz = linearAccelerationFilterFrequencyInHertz;
      this.updateDT = updateDT;
   }

   public double getJointPositionFilterFrequencyInHertz()
   {
      return jointPositionFilterFrequencyInHertz;
   }

   public double getJointVelocityFilterFrequencyInHertz()
   {
      return jointVelocityFilterFrequencyInHertz;
   }

   public double getOrientationFilterFrequencyInHertz()
   {
      return orientationFilterFrequencyInHertz;
   }

   public double getAngularVelocityFilterFrequencyInHertz()
   {
      return angularVelocityFilterFrequencyInHertz;
   }

   public double getLinearAccelerationFilterFrequencyInHertz()
   {
      return linearAccelerationFilterFrequencyInHertz;
   }

   public double getUpdateDT()
   {
      return updateDT;
   }

}
