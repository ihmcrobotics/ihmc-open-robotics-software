package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.HashMap;

public class SensorFilterParameters
{
   private final double jointPositionFilterFrequencyInHertz;
   private final double jointVelocityFilterFrequencyInHertz;
   private final double orientationFilterFrequencyInHertz;
   private final double angularVelocityFilterFrequencyInHertz;
   private final double linearAccelerationFilterFrequencyInHertz;
   private final double jointVelocitySlopTimeForBacklashCompensation;
   private final double updateDT;

   /** Required for Valkyrie as no filtering is done on-board */
   private final boolean useTwoPolesForIMUFiltering;

   private final boolean doFiniteDifferenceForJointVelocities;
   
   private final boolean doElasticityCompensation;
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness;

   public SensorFilterParameters(double jointPositionFilterFrequencyInHertz, double jointVelocityFilterFrequencyInHertz,
         double orientationFilterFrequencyInHertz, double angularVelocityFilterFrequencyInHertz, double linearAccelerationFilterFrequencyInHertz,
         double jointVelocitySlopTimeForBacklashCompensation, double updateDT, boolean useTwoPolesForIMUFiltering,
         boolean doFiniteDifferenceForJointVelocities, boolean doElasticityCompensation, double defaultJointStiffness,
         HashMap<String, Double> jointSpecificStiffness)
   {
      this.jointPositionFilterFrequencyInHertz = jointPositionFilterFrequencyInHertz;
      this.jointVelocityFilterFrequencyInHertz = jointVelocityFilterFrequencyInHertz;
      this.orientationFilterFrequencyInHertz = orientationFilterFrequencyInHertz;
      this.angularVelocityFilterFrequencyInHertz = angularVelocityFilterFrequencyInHertz;
      this.linearAccelerationFilterFrequencyInHertz = linearAccelerationFilterFrequencyInHertz;
      this.jointVelocitySlopTimeForBacklashCompensation = jointVelocitySlopTimeForBacklashCompensation;
      this.updateDT = updateDT;

      this.useTwoPolesForIMUFiltering = useTwoPolesForIMUFiltering;
      this.doFiniteDifferenceForJointVelocities = doFiniteDifferenceForJointVelocities;
      
      this.doElasticityCompensation = doElasticityCompensation;
      this.defaultJointStiffness = defaultJointStiffness;
      this.jointSpecificStiffness = jointSpecificStiffness;
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

   public double getJointVelocitySlopTimeForBacklashCompensation()
   {
      return jointVelocitySlopTimeForBacklashCompensation;
   }

   public double getEstimatorDT()
   {
      return updateDT;
   }

   public boolean getUseTwoPolesForIMUFiltering()
   {
      return useTwoPolesForIMUFiltering;
   }

   public boolean getDoFiniteDifferenceForJointVelocities()
   {
      return doFiniteDifferenceForJointVelocities;
   }

   public boolean isDoElasticityCompensation()
   {
      return doElasticityCompensation;
   }

   public double getDefaultJointStiffness()
   {
      return defaultJointStiffness;
   }

   public HashMap<String, Double> getJointSpecificStiffness()
   {
      return jointSpecificStiffness;
   }
}
