package us.ihmc.steppr.controlParameters;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.steppr.hardware.StepprJoint;

public class BonoStateEstimatorParameters extends StateEstimatorParameters
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

      defaultFilterBreakFrequency = runningOnRealRobot ? 67.0 : Double.POSITIVE_INFINITY;
      jointVelocitySlopTimeForBacklashCompensation = 0.06;
      
      doElasticityCompensation = runningOnRealRobot;
      defaultJointStiffness = Double.POSITIVE_INFINITY;
      
      jointSpecificStiffness.put(StepprJoint.LEFT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
      jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_Z.getSdfName(), Double.POSITIVE_INFINITY);
      
      jointSpecificStiffness.put(StepprJoint.LEFT_HIP_X.getSdfName(), 4500.0); //6000
      jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_X.getSdfName(), 4500.0); //6000
      
      jointSpecificStiffness.put(StepprJoint.LEFT_HIP_Y.getSdfName(), 10000.0);
      jointSpecificStiffness.put(StepprJoint.RIGHT_HIP_Y.getSdfName(), 10000.0);
      
      jointSpecificStiffness.put(StepprJoint.LEFT_KNEE_Y.getSdfName(), 6000.0);
      jointSpecificStiffness.put(StepprJoint.RIGHT_KNEE_Y.getSdfName(), 6000.0);

   }
   

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      YoDouble maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", 0.1);
      Map<OneDoFJoint, YoDouble> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);
      YoDouble jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", 25.0); //16

      YoDouble angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", 16.0);
      YoDouble linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", defaultFilterBreakFrequency);

      if (doElasticityCompensation)
         sensorProcessing.addJointPositionElasticyCompensator(jointPositionStiffness, maxDeflection, false);

      sensorProcessing.computeJointVelocityFromFiniteDifference(jointVelocityAlphaFilter, true); //vizonly
      sensorProcessing.addSensorAlphaFilter(jointVelocityAlphaFilter, false, JOINT_VELOCITY);
      sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, IMU_LINEAR_ACCELERATION);
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
   public double getCoPFilterFreqInHertz()
   {
      // TODO Try to crank up that one, might be a bit sluggish with 4Hz. If the change is not evident, leave it to 4Hz then.
      return 4.0;
   }

   @Override
   public boolean enableIMUBiasCompensation()
   {
      return false;
   }

   @Override
   public boolean enableIMUYawDriftCompensation()
   {
      return false;
   }

   @Override
   public double getIMUBiasFilterFreqInHertz()
   {
      return 6.0e-3;
   }

   @Override
   public double getIMUYawDriftFilterFreqInHertz()
   {
      return 1.0e-3;
   }

   @Override
   public double getIMUBiasVelocityThreshold()
   {
      return 0.015;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return true;
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
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0.15;
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
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }


   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return null;
   }


   @Override
   public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
   {
      return false;
   }
   
   @Override
   public double getCenterOfMassVelocityFusingFrequency()
   {
      return 0.4261;
   }

   @Override
   public boolean useGroundReactionForcesToComputeCenterOfMassVelocity()
   {
      return false;
   }
}
