package us.ihmc.acsell.controlParameters;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class BonoStateEstimatorParameters implements StateEstimatorParameters
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

   public BonoStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      defaultFilterBreakFrequency = runningOnRealRobot ? 16.0 : Double.POSITIVE_INFINITY;
      jointVelocitySlopTimeForBacklashCompensation = 0.06;
      
      doElasticityCompensation = runningOnRealRobot;
      defaultJointStiffness = Double.POSITIVE_INFINITY;
      
      jointSpecificStiffness.put(StepprJoint.LEFT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
      jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
      
      jointSpecificStiffness.put(StepprJoint.LEFT_HIP_X.getSdfName(), 3600.0);
      jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_X.getSdfName(), 3600.0);
      
      jointSpecificStiffness.put(StepprJoint.LEFT_HIP_Y.getSdfName(), 4500.0);
      jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_Y.getSdfName(), 4500.0);
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      DoubleYoVariable jointPositionAlphaFilter = sensorProcessing.createAlphaFilter("jointPositionAlphaFilter", defaultFilterBreakFrequency);
      Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);
      DoubleYoVariable jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", defaultFilterBreakFrequency);

      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", defaultFilterBreakFrequency);

      sensorProcessing.addJointPositionAlphaFilter(jointPositionAlphaFilter, false);
      if (doElasticityCompensation)
         sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, false);

      sensorProcessing.computeJointVelocityFromFiniteDifference(jointVelocityAlphaFilter, false);
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
   public Pair<String, String> getIMUsForSpineJointVelocityEstimation()
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
}
