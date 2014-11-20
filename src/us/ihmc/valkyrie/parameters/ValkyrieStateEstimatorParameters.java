package us.ihmc.valkyrie.parameters;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ValkyrieStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double kinematicsPelvisLinearVelocityFilterFreqInHertz;
   private final double kinematicsPelvisPositionFilterFreqInHertz;

   private final double jointVelocitySlopTimeForBacklashCompensation;

   private final double jointPositionFilterFrequencyHz;
   private final double jointVelocityFilterFrequencyHz;
   private final double orientationFilterFrequencyHz;
   private final double angularVelocityFilterFrequencyHz;
   private final double linearAccelerationFilterFrequencyHz;

//   private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
//   private SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuningSeptember2013();
   private SensorNoiseParameters sensorNoiseParameters = null;

   private final boolean doElasticityCompensation;
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness = new HashMap<String, Double>();

   public ValkyrieStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      jointPositionFilterFrequencyHz = runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;
      jointVelocityFilterFrequencyHz = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;

      orientationFilterFrequencyHz        = 50.0; //runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;
      angularVelocityFilterFrequencyHz    = 50.0; //30.0; //30.0; //runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;
      linearAccelerationFilterFrequencyHz = runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;

      jointVelocitySlopTimeForBacklashCompensation = 0.07;

      doElasticityCompensation = runningOnRealRobot;
      defaultJointStiffness = 10000; //40000.0; //Double.POSITIVE_INFINITY;

      kinematicsPelvisPositionFilterFreqInHertz = Double.POSITIVE_INFINITY;
      kinematicsPelvisLinearVelocityFilterFreqInHertz = 50.0; //16.0;
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();

      DoubleYoVariable jointPositionAlphaFilter = sensorProcessing.createAlphaFilter("jointPositionAlphaFilter", jointPositionFilterFrequencyHz);
      Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);
      DoubleYoVariable jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", jointVelocityFilterFrequencyHz);
      DoubleYoVariable jointVelocitySlopTime = new DoubleYoVariable("jointBacklashSlopTime", registry);
      jointVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);

      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", orientationFilterFrequencyHz);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", angularVelocityFilterFrequencyHz);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", linearAccelerationFilterFrequencyHz);

      sensorProcessing.addJointPositionAlphaFilter(jointPositionAlphaFilter, false);
      if (doElasticityCompensation)
         sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, false);

      sensorProcessing.computeJointVelocityWithBacklashCompensator(jointVelocityAlphaFilter, jointVelocitySlopTime, false);
      sensorProcessing.addJointVelocityAlphaFilter(jointVelocityAlphaFilter, false);

      sensorProcessing.addIMUOrientationAlphaFilter(orientationAlphaFilter, false);
      sensorProcessing.addIMUAngularVelocityAlphaFilter(angularVelocityAlphaFilter, false);
      sensorProcessing.addIMULinearAccelerationAlphaFilter(linearAccelerationAlphaFilter, false);
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return sensorNoiseParameters;
   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return kinematicsPelvisPositionFilterFreqInHertz;
   }

   @Override
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return kinematicsPelvisLinearVelocityFilterFreqInHertz;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return 4.0;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   @Override
   public boolean estimateGravity()
   {
      return false;
   }

   @Override
   public double getGravityFilterFreqInHertz()
   {
      return 5.3052e-4;
   }

   @Override
   public double getPelvisPositionFusingFrequency()
   {
      return 11.7893; // alpha = 0.8 with dt = 0.003
   }

   @Override
   public double getPelvisLinearVelocityFusingFrequency()
   {
      return 0.4261; // alpha = 0.992 with dt = 0.003
   }

   @Override
   public double getPelvisVelocityBacklashSlopTime()
   {
      return jointVelocitySlopTimeForBacklashCompensation;
   }

   @Override
   public double getDelayTimeForTrustingFoot()
   {
      return 0.02;
   }

   @Override
   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      return 0.3;
   }

   @Override
   public boolean estimateIMUDrift()
   {
      return true;
   }

   @Override
   public boolean compensateIMUDrift()
   {
      return true;
   }

   @Override
   public double getIMUDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   @Override
   public double getFootVelocityUsedForImuDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   @Override
   public double getFootVelocityThresholdToEnableIMUDriftCompensation()
   {
      return 0.03;
   }

   @Override
   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return true;
   }

   @Override
   public boolean useTwistForPelvisLinearStateEstimation()
   {
      return true;
   }

   @Override
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0.15;
   }

   @Override
   public boolean createFusedIMUSensor()
   {
      return false;
   }

   @Override
   public double getContactThresholdForce()
   {
      return 120.0;
   }

   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
      return 0.02;
   }
}
