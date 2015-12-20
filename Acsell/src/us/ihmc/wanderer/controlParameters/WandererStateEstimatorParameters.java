package us.ihmc.wanderer.controlParameters;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.wanderer.hardware.WandererJoint;

public class WandererStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double jointVelocitySlopTimeForBacklashCompensation;

   private final double defaultFilterBreakFrequency;

   // private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
   // private SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuningSeptember2013();
   private SensorNoiseParameters sensorNoiseParameters = null;

   private final boolean doElasticityCompensation;
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness = new HashMap<String, Double>();

   public WandererStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      defaultFilterBreakFrequency = runningOnRealRobot ? 67.0 : Double.POSITIVE_INFINITY;
      jointVelocitySlopTimeForBacklashCompensation = 0.06;
      
      doElasticityCompensation = runningOnRealRobot;
      defaultJointStiffness = Double.POSITIVE_INFINITY;
      
      jointSpecificStiffness.put(WandererJoint.LEFT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
      jointSpecificStiffness.put(WandererJoint.RIGHT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
      
      jointSpecificStiffness.put(WandererJoint.LEFT_HIP_X.getSdfName(), 8000.0); //4500
      jointSpecificStiffness.put(WandererJoint.RIGHT_HIP_X.getSdfName(), 8000.0); //4500
      
      jointSpecificStiffness.put(WandererJoint.LEFT_HIP_Y.getSdfName(), 16000.0);//10000
      jointSpecificStiffness.put(WandererJoint.RIGHT_HIP_Y.getSdfName(), 16000.0);
      
      jointSpecificStiffness.put(WandererJoint.LEFT_KNEE_Y.getSdfName(), 4000.0);//6000
      jointSpecificStiffness.put(WandererJoint.RIGHT_KNEE_Y.getSdfName(), 4000.0);

   }
   

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
//      DoubleYoVariable jointPositionAlphaFilter = sensorProcessing.createAlphaFilter("jointPositionAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", 0.1);
      Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);
      DoubleYoVariable jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", 25.0); //16

//      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", 16.0);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", defaultFilterBreakFrequency);

//      sensorProcessing.addSensorAlphaFilter(jointPositionAlphaFilter, false, SensorType.JOINT_POSITION);
      if (doElasticityCompensation)
         sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, maxDeflection, false);

      sensorProcessing.computeJointVelocityFromFiniteDifference(jointVelocityAlphaFilter, true); //vizonly
      sensorProcessing.addSensorAlphaFilter(jointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY);
      sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

//      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, SensorType.IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, SensorType.IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, SensorType.IMU_LINEAR_ACCELERATION);
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
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return 16.0; //50.0;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      // TODO Try to crank up that one, might be a bit sluggish with 4Hz. If the change is not evident, leave it to 4Hz then.
      return 4.0;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   @Override
   public boolean estimateAccelerationBias()
   {
      return false;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return true;
   }

   @Override
   public double getAccelerationBiasFilterFreqInHertz()
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
      return Double.NaN;
   }

   @Override
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return false;
   }

   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return 0;
   }

   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return null;
   }
   
   @Override
   public double getContactThresholdHeight()
   {
      return 0.01;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return FootSwitchType.WrenchBased;
   }

   @Override
   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }


   @Override
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return null;
   }
}
