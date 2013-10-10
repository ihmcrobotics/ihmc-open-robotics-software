package us.ihmc.sensorProcessing.simulatedSensors;

public class SensorFilterParameters
{
   private final double jointPositionFilterFrequencyInHertz;
   private final double jointVelocityFilterFrequencyInHertz;
   private final double updateDT;

   public SensorFilterParameters(double jointPositionFilterFrequencyInHertz, double jointVelocityFilterFrequencyInHertz, double updateDT)
   {
      this.jointPositionFilterFrequencyInHertz = jointPositionFilterFrequencyInHertz;
      this.jointVelocityFilterFrequencyInHertz = jointVelocityFilterFrequencyInHertz;
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

   public double getUpdateDT()
   {
      return updateDT;
   }
}
