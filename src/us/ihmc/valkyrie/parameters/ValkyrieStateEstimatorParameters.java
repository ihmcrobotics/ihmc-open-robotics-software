package us.ihmc.valkyrie.parameters;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.robotSide.SideDependentList;
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

   private final double jointVelocityFilterFrequencyHz;
   private final double orientationFilterFrequencyHz;
   private final double angularVelocityFilterFrequencyHz;
   private final double linearAccelerationFilterFrequencyHz;
   
   private final ValkyrieSensorInformation sensorInformation;
   private final ValkyrieJointMap jointMap;

//   private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
//   private SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuningSeptember2013();
   private SensorNoiseParameters sensorNoiseParameters = null;

   private final boolean doElasticityCompensation;
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness = new HashMap<String, Double>();

   private final SideDependentList<String> wristForceSensorNames;

   public ValkyrieStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT, ValkyrieSensorInformation sensorInformation, ValkyrieJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;
      
      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;
      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      jointVelocityFilterFrequencyHz = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;

      orientationFilterFrequencyHz        = 50.0; //runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;
      angularVelocityFilterFrequencyHz    = 50.0; //30.0; //30.0; //runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;
      linearAccelerationFilterFrequencyHz = runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;

      jointVelocitySlopTimeForBacklashCompensation = 0.03; //0.07; => Add a shit load of delay!!

      doElasticityCompensation = runningOnRealRobot;
      defaultJointStiffness = 10000; //40000.0; //Double.POSITIVE_INFINITY;

      kinematicsPelvisPositionFilterFreqInHertz = Double.POSITIVE_INFINITY;
      kinematicsPelvisLinearVelocityFilterFreqInHertz = 50.0; //16.0;
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      String[] namesOfJointsUsingOutputEncoder = jointMap.getNamesOfJointsUsingOutputEncoder();
      
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();

      Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);

      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", orientationFilterFrequencyHz);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", angularVelocityFilterFrequencyHz);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", linearAccelerationFilterFrequencyHz);

      if (doElasticityCompensation)
         sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, false);

      //velocity from filtered positions
      // For joints using output encoder
      DoubleYoVariable jointOutputEncoderVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointOutputEncoderVelocityAlphaFilter", jointVelocityFilterFrequencyHz);
      DoubleYoVariable jointOutputEncoderVelocitySlopTime = new DoubleYoVariable("jointOutputEncoderBacklashSlopTime", registry);
      jointOutputEncoderVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);
      sensorProcessing.computeJointVelocityWithBacklashCompensatorOnlyForSpecifiedJoints(jointOutputEncoderVelocityAlphaFilter, jointOutputEncoderVelocitySlopTime, false, namesOfJointsUsingOutputEncoder); 
      sensorProcessing.addJointVelocityAlphaFilterOnlyForSpecifiedJoints(jointOutputEncoderVelocityAlphaFilter, false, namesOfJointsUsingOutputEncoder);

      // For joints using input encoder: only one pass since one is done onboard at 20Hz
      DoubleYoVariable jointInputEncoderVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointInputEncoderVelocityAlphaFilter", jointVelocityFilterFrequencyHz);
      sensorProcessing.addJointVelocityAlphaFilterWithJointsToIgnore(jointInputEncoderVelocityAlphaFilter, false, namesOfJointsUsingOutputEncoder);

      //imu
      sensorProcessing.addIMUOrientationAlphaFilter(orientationAlphaFilter, false);
      sensorProcessing.addIMUAngularVelocityAlphaFilter(angularVelocityAlphaFilter, false);
      sensorProcessing.addIMULinearAccelerationAlphaFilter(linearAccelerationAlphaFilter, false);
      
      
      // Raw finite difference on all joint positions
      DoubleYoVariable dummyAlphaFilter = new DoubleYoVariable("dummyAlphaFilter", registry);
      sensorProcessing.computeJointVelocityFromFiniteDifference(dummyAlphaFilter, true);
      
      DoubleYoVariable tauAlphaFilter = sensorProcessing.createAlphaFilter("tauAlphaFilter", 1.0);
      sensorProcessing.addJointTauAlphaFilter(tauAlphaFilter, true);
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
   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
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

   @Override
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      return true;
   }

   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      return 0.8;
   }

   /**
    * IMUs to use to compute the spine joint velocities.
    * @return {@code Pair<String, String>} the first element is the name of one pelvis IMU, the second is the name of one IMU of the trunk. 
    */
   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return new ImmutablePair<String, String>(sensorInformation.getLeftPelvisIMUSensor(), sensorInformation.getLeftTrunkIMUSensor());
   }
   
   @Override
   public double getContactThresholdHeight()
   {
      return 0.05;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return FootSwitchType.WrenchBased;
//      return runningOnRealRobot ? FootSwitchType.WrenchAndContactSensorFused : FootSwitchType.WrenchBased;
   }

   @Override
   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return wristForceSensorNames;
   }
}
