package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.EnumMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.acsell.parameters.StrainGaugeInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprAnkle;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.state.slowSensors.PressureSensor;
import us.ihmc.steppr.hardware.state.slowSensors.StepprSlowSensor;
import us.ihmc.steppr.hardware.state.slowSensors.StrainSensor;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class StepprState
{
   private static final int[] leftFootForceSensorsToUse = { 1, 1, 2, 3 }; //leftMost sensor0 is broken
   private static final int[] rightFootForceSensorsToUse = { 0, 1, 2, 2 }; //rightMost sensor3 is broken
   private static final boolean USE_STRAIN_GAUGES_FOR_Z_FORCE = false;

   private final YoVariableRegistry registry = new YoVariableRegistry("Steppr");

   private final LongYoVariable lastReceivedTime = new LongYoVariable("lastReceivedTime", registry);
   private final LongYoVariable timeSincePreviousPacket = new LongYoVariable("timeSincePreviousPacket", registry);

   private final LongYoVariable stateCollectionStartTime = new LongYoVariable("stateCollectionStartTime", registry);
   private final LongYoVariable stateCollectionFinishTime = new LongYoVariable("stateCollectionFinishTime", registry);

   private final EnumMap<StepprActuator, StepprActuatorState> actuatorStates = new EnumMap<>(StepprActuator.class);
   private final StepprPowerDistributionADCState powerDistributionState = new StepprPowerDistributionADCState("powerDistribution", registry);
   private final DoubleYoVariable totalMotorPower = new DoubleYoVariable("totalMotorPower", registry);
   private final StepprXSensState xsens = new StepprXSensState("xsens", registry);

   private final EnumMap<StepprJoint, StepprJointState> jointStates = new EnumMap<>(StepprJoint.class);

   private final SideDependentList<DenseMatrix64F> footWrenches = new SideDependentList<>();

   private final StepprUpperBodyOffsetCalculator stepprUpperBodyOffsetCalculator;

   private final BooleanYoVariable updateOffsets = new BooleanYoVariable("updateOffsets", registry);
   private final BooleanYoVariable tareSensors = new BooleanYoVariable("tareSensors", registry);
   
   private final StrainSensor leftFootStrainGauge; 
   private final StrainSensor rightFootStrainGauge; 

   public StepprState(double dt, YoVariableRegistry parentRegistry)
   {
      createActuators();
      createJoints();

      stepprUpperBodyOffsetCalculator = new StepprUpperBodyOffsetCalculator(actuatorStates.get(StepprActuator.TORSO_Z),
            actuatorStates.get(StepprActuator.TORSO_X), actuatorStates.get(StepprActuator.TORSO_Y), actuatorStates.get(StepprActuator.TORSO_Z), xsens, dt,
            registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         footWrenches.put(robotSide, new DenseMatrix64F(6, 1));
      }

      leftFootStrainGauge = getCalibratedJointStrainGauge(StepprAnkle.LEFT.getShankSensor());
      rightFootStrainGauge = getCalibratedJointStrainGauge(StepprAnkle.RIGHT.getShankSensor());
      
      parentRegistry.addChild(registry);
   }

   private StrainSensor getCalibratedJointStrainGauge(StrainGaugeInformation sensorInfo)
   {
      if (sensorInfo != null)
      {
         StrainSensor strainSensor = actuatorStates.get(sensorInfo.getStrainSensorBoard()).getStrainGuage(sensorInfo.getStrainSensorConnectorId());
         strainSensor.setCalibration(sensorInfo.getStrainSensorGain(), sensorInfo.getStrainSensorOffset());
         return strainSensor;
      }
      else
      {
         return null;
      }
   }

   private void createJoints()
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         if (joint.isLinear())
         {
            StepprActuatorState actuator = actuatorStates.get(joint.getActuators()[0]);
            StrainSensor strainSensor = getCalibratedJointStrainGauge(joint.getStrainGaugeInformation());
            jointStates.put(joint, new StepprLinearTransmissionJointState(joint.getSdfName(), joint.getRatio(), joint.hasOutputEncoder(), actuator,
                  strainSensor, registry));
         }
      }

      // Create knees

      StepprKneeJointState leftKnee = new StepprKneeJointState(StepprJoint.LEFT_KNEE_Y, actuatorStates.get(StepprActuator.LEFT_KNEE),
            actuatorStates.get(StepprActuator.LEFT_ANKLE_LEFT), getCalibratedJointStrainGauge(StepprJoint.LEFT_KNEE_Y.getStrainGaugeInformation()), registry);
      StepprKneeJointState rightKnee = new StepprKneeJointState(StepprJoint.RIGHT_KNEE_Y, actuatorStates.get(StepprActuator.RIGHT_KNEE),
            actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT), getCalibratedJointStrainGauge(StepprJoint.RIGHT_KNEE_Y.getStrainGaugeInformation()), registry);

      jointStates.put(StepprJoint.LEFT_KNEE_Y, leftKnee);
      jointStates.put(StepprJoint.RIGHT_KNEE_Y, rightKnee);

      // Create ankles
      StepprAnkleJointState leftAnkle = new StepprAnkleJointState(RobotSide.LEFT, actuatorStates.get(StepprActuator.LEFT_ANKLE_RIGHT),
            actuatorStates.get(StepprActuator.LEFT_ANKLE_LEFT), getCalibratedJointStrainGauge(StepprJoint.LEFT_ANKLE_Y.getStrainGaugeInformation()), getCalibratedJointStrainGauge(StepprJoint.LEFT_ANKLE_X.getStrainGaugeInformation()), registry);
      jointStates.put(StepprJoint.LEFT_ANKLE_Y, leftAnkle.ankleY());
      jointStates.put(StepprJoint.LEFT_ANKLE_X, leftAnkle.ankleX());

      StepprAnkleJointState rightAnkle = new StepprAnkleJointState(RobotSide.RIGHT, actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT),
            actuatorStates.get(StepprActuator.RIGHT_ANKLE_LEFT), getCalibratedJointStrainGauge(StepprJoint.RIGHT_ANKLE_Y.getStrainGaugeInformation()), getCalibratedJointStrainGauge(StepprJoint.RIGHT_ANKLE_X.getStrainGaugeInformation()), registry);
      jointStates.put(StepprJoint.RIGHT_ANKLE_Y, rightAnkle.ankleY());
      jointStates.put(StepprJoint.RIGHT_ANKLE_X, rightAnkle.ankleX());

   }

   private void createActuators()
   {
      for (StepprActuator actuatorName : StepprActuator.values)
      {
         actuatorStates
               .put(actuatorName, new StepprActuatorState(actuatorName.getName(), actuatorName.getKt(), actuatorName.getSensedCurrentToTorqueDirection(), registry));
      }
   }

   public void update(ByteBuffer buffer, long timestamp) throws IOException
   {
      timeSincePreviousPacket.set(timestamp - lastReceivedTime.getLongValue());
      lastReceivedTime.set(timestamp);

      stateCollectionStartTime.set(buffer.getLong());
      stateCollectionFinishTime.set(buffer.getLong());
      for (StepprActuator actuatorName : StepprActuator.values)
      {
         actuatorStates.get(actuatorName).update(buffer);
      }
      powerDistributionState.update(buffer);
      xsens.update(buffer);


      stepprUpperBodyOffsetCalculator.update();

      for (StepprJoint joint : StepprJoint.values)
      {
         jointStates.get(joint).update();
      }

      if (updateOffsets.getBooleanValue())
      {
         stepprUpperBodyOffsetCalculator.updateOffsets();
         for (StepprJoint joint : StepprJoint.values)
         {
            jointStates.get(joint).updateOffsets();
         }

         updateOffsets.set(false);
      }
      
      updateFootForceSensor();
    
      updateMotorPower();
      
   }

   private void updateFootForceSensor()
   {
      
      if(tareSensors.getBooleanValue())
      {
         tarePressureSensors();
         leftFootStrainGauge.tare();
         rightFootStrainGauge.tare();
         //jointStates.get(StepprJoint.LEFT_ANKLE_X).tare();
         tareSensors.set(false);
      }

      
      double leftFootForce = 0;
      double rightFootForce = 0;
   
      StepprActuatorState leftFootSensorState = actuatorStates.get(StepprActuator.LEFT_ANKLE_RIGHT);
      StepprActuatorState rightFootSensorState = actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT);
      if (USE_STRAIN_GAUGES_FOR_Z_FORCE)
      {
         rightFootForce = rightFootStrainGauge.getCalibratedValue();
         leftFootForce = leftFootStrainGauge.getCalibratedValue();
   
      }
      else
      {
         double leftHeelForce = 0.0, rightHeelForce = 0.0;
         for (int sensor : leftFootForceSensorsToUse)
         {
            leftHeelForce += ((PressureSensor) leftFootSensorState.getPressureSensor(sensor)).getValue();
         }
         
         for (int sensor : rightFootForceSensorsToUse)
         {
            rightHeelForce += ((PressureSensor) rightFootSensorState.getPressureSensor(sensor)).getValue();
         }
         
         //calculate toe Z force based on ankle
         final double distHeelFromAnkle = 0.035+2.0*0.0254-0.001;
         final double distToeFromAnkle = 0.16+0.001;       
         double leftAnkleYTau = jointStates.get(StepprJoint.LEFT_ANKLE_Y).getTau();
         double leftToeForce = leftHeelForce*distHeelFromAnkle + leftAnkleYTau / distToeFromAnkle;         
         double rightAnkleYTau = jointStates.get(StepprJoint.RIGHT_ANKLE_Y).getTau();
         double rightToeForce = rightHeelForce*distHeelFromAnkle + rightAnkleYTau / distToeFromAnkle;
         
         leftFootForce = leftHeelForce + leftToeForce;
         rightFootForce = rightHeelForce + rightToeForce;
      }
      
      
      footWrenches.get(RobotSide.LEFT).set(5, leftFootForce);
      footWrenches.get(RobotSide.RIGHT).set(5, rightFootForce);
      
   }
   
   private void updateMotorPower()
   {
	   double accTotalMotorPower = 0;
	   
	   for (StepprActuator actuatorName : StepprActuator.values)
	   {
	      accTotalMotorPower += actuatorStates.get(actuatorName).getMotorPower();
	   }
	   totalMotorPower.set(accTotalMotorPower);
   }

   private void tarePressureSensors()
   {
      for (StepprActuatorState actuatorState : actuatorStates.values())
      {
         for (StepprSlowSensor sensor : actuatorState.getSlowSensors())
         {
            if (sensor instanceof PressureSensor)
            {
               ((PressureSensor) sensor).tare();
            }
         }
      }
      
   }

   public StepprJointState getJointState(StepprJoint joint)
   {
      return jointStates.get(joint);
   }

   public void updateRawSensorData(StepprJoint joint, RawJointSensorDataHolder dataHolder)
   {
      StepprJointState stepprJointState = jointStates.get(joint);
      dataHolder.setQ_raw(stepprJointState.getQ());
      dataHolder.setQd_raw(stepprJointState.getQd());
      for (int i = 0; i < stepprJointState.getNumberOfActuators(); i++)
      {
         dataHolder.setMotorAngle(i, stepprJointState.getMotorAngle(i));
      }
   }

   public StepprXSensState getXSensState()
   {
      return xsens;
   }

   public DenseMatrix64F getFootWrench(RobotSide robotSide)
   {
      return footWrenches.get(robotSide);
   }
}
