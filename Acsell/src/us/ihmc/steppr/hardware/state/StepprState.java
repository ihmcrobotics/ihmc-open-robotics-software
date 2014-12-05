package us.ihmc.steppr.hardware.state;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.EnumMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.state.slowSensors.PressureSensor;
import us.ihmc.steppr.hardware.state.slowSensors.StepprSlowSensor;
import us.ihmc.steppr.hardware.state.slowSensors.StrainSensor;
import us.ihmc.utilities.math.UnitConversions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class StepprState
{
   private final YoVariableRegistry registry = new YoVariableRegistry("Steppr");

   private final LongYoVariable lastReceivedTime = new LongYoVariable("lastReceivedTime", registry);
   private final LongYoVariable timeSincePreviousPacket = new LongYoVariable("timeSincePreviousPacket", registry);

   private final LongYoVariable stateCollectionStartTime = new LongYoVariable("stateCollectionStartTime", registry);
   private final LongYoVariable stateCollectionFinishTime = new LongYoVariable("stateCollectionFinishTime", registry);

   private final EnumMap<StepprActuator, StepprActuatorState> actuatorStates = new EnumMap<>(StepprActuator.class);
   private final StepprPowerDistributionADCState powerDistributionState = new StepprPowerDistributionADCState("powerDistribution", registry);
   private final StepprXSensState xsens = new StepprXSensState("xsens", registry);

   private final EnumMap<StepprJoint, StepprJointState> jointStates = new EnumMap<>(StepprJoint.class);

   private final SideDependentList<DenseMatrix64F> footWrenches = new SideDependentList<>();

   private final StepprUpperBodyOffsetCalculator stepprUpperBodyOffsetCalculator;
   
   
   private final BooleanYoVariable updateOffsets = new BooleanYoVariable("updateOffsets", registry);
   private final BooleanYoVariable tarePressureSensors = new BooleanYoVariable("tarePressureSensors", registry);
   // X: footLength, Y: footWidth
   private final double forceSensorXOffset = 4.2 * UnitConversions.INCH_TO_METER;
   private final double forceSensorYOffset = 2.2 * UnitConversions.INCH_TO_METER;
   private final double AnkleToCenterOfFoot = 3.0 * UnitConversions.INCH_TO_METER;
   private final double forceSensorToeXOffset = forceSensorXOffset + AnkleToCenterOfFoot;
   private final double forceSensorHeelXOffset = forceSensorXOffset - AnkleToCenterOfFoot;
   
   private final boolean useToeSensorsOnlyForZForce=true;
   
   public StepprState(double dt, YoVariableRegistry parentRegistry)
   {
      createActuators();
      createJoints();
      

      stepprUpperBodyOffsetCalculator = new StepprUpperBodyOffsetCalculator(actuatorStates.get(StepprActuator.TORSO_Z),
            actuatorStates.get(StepprActuator.TORSO_X), actuatorStates.get(StepprActuator.TORSO_Y), actuatorStates.get(StepprActuator.TORSO_Z), xsens, dt, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         footWrenches.put(robotSide, new DenseMatrix64F(6, 1));
      }

      parentRegistry.addChild(registry);
   }
   
   
   private StrainSensor getCalibratedJoint(StepprJoint joint)
   {
      StrainSensor strainSensor=null;
      if(joint.getStrainSensorBoard()!=null)
      {
         strainSensor = actuatorStates.get(joint.getStrainSensorBoard()).getStrainGuage(joint.getStrainSensorConnectorId());
         strainSensor.setCalibration(joint.getStrainSensorGain(), joint.getStrainSensorOffset());
      }
      return strainSensor;
   }


   private void createJoints()
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         if (joint.isLinear())
         {
            StepprActuatorState actuator = actuatorStates.get(joint.getActuators()[0]);
            StrainSensor strainSensor = getCalibratedJoint(joint);
            jointStates.put(joint, new StepprLinearTransmissionJointState(joint.getSdfName(), joint.getRatio(), joint.hasOutputEncoder(), actuator, strainSensor, registry));
         }
      }

      // Create knees
     
      
      StepprKneeJointState leftKnee = new StepprKneeJointState(StepprJoint.LEFT_KNEE_Y, actuatorStates.get(StepprActuator.LEFT_KNEE),
            actuatorStates.get(StepprActuator.LEFT_ANKLE_LEFT), getCalibratedJoint(StepprJoint.LEFT_KNEE_Y),registry);
      StepprKneeJointState rightKnee = new StepprKneeJointState(StepprJoint.RIGHT_KNEE_Y, actuatorStates.get(StepprActuator.RIGHT_KNEE),
            actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT), getCalibratedJoint(StepprJoint.RIGHT_KNEE_Y), registry);

      jointStates.put(StepprJoint.LEFT_KNEE_Y, leftKnee);
      jointStates.put(StepprJoint.RIGHT_KNEE_Y, rightKnee);

      // Create ankles
      StepprAnkleJointState leftAnkle = new StepprAnkleJointState(RobotSide.LEFT, actuatorStates.get(StepprActuator.LEFT_ANKLE_RIGHT),
            actuatorStates.get(StepprActuator.LEFT_ANKLE_LEFT), registry);
      jointStates.put(StepprJoint.LEFT_ANKLE_Y, leftAnkle.ankleY());
      jointStates.put(StepprJoint.LEFT_ANKLE_X, leftAnkle.ankleX());

      StepprAnkleJointState rightAnkle = new StepprAnkleJointState(RobotSide.RIGHT, actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT),
            actuatorStates.get(StepprActuator.RIGHT_ANKLE_LEFT), registry);
      jointStates.put(StepprJoint.RIGHT_ANKLE_Y, rightAnkle.ankleY());
      jointStates.put(StepprJoint.RIGHT_ANKLE_X, rightAnkle.ankleX());

   }
   
  


   private void createActuators()
   {
      for (StepprActuator actuatorName : StepprActuator.values)
      {
         actuatorStates.put(actuatorName, new StepprActuatorState(actuatorName.getName(), actuatorName.getKt(), actuatorName.getKinematicDirection(), registry));
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
      
      

      StepprActuatorState leftFootSensorState = actuatorStates.get(StepprActuator.LEFT_ANKLE_RIGHT);
      double leftForceLeftHeel = ((PressureSensor) leftFootSensorState.getPressureSensor(0)).getValue();
      double leftForceRightHeel = ((PressureSensor) leftFootSensorState.getPressureSensor(1)).getValue();
      double leftForceLeftToe = ((PressureSensor) leftFootSensorState.getPressureSensor(2)).getValue();
      double leftForceRightToe = ((PressureSensor) leftFootSensorState.getPressureSensor(3)).getValue();
      
      footWrenches.get(RobotSide.LEFT).set(0, 
            -(leftForceLeftHeel+leftForceLeftToe)*forceSensorYOffset 
            + (leftForceRightHeel+leftForceRightToe)*forceSensorYOffset);
      footWrenches.get(RobotSide.LEFT).set(1, 
            -(leftForceLeftHeel+leftForceRightHeel)*forceSensorHeelXOffset 
            +(leftForceLeftToe+leftForceRightToe)*forceSensorToeXOffset);
      footWrenches.get(RobotSide.LEFT).set(5, leftForceLeftHeel+leftForceRightHeel+leftForceLeftToe+leftForceRightToe);

      StepprActuatorState rightFootSensorState = actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT);
      double rightForceLeftHeel = ((PressureSensor) rightFootSensorState.getPressureSensor(0)).getValue();
      double rightForceRightHeel = ((PressureSensor) rightFootSensorState.getPressureSensor(1)).getValue();
      double rightForceLeftToe = ((PressureSensor) rightFootSensorState.getPressureSensor(2)).getValue();
      double rightForceRightToe = ((PressureSensor) rightFootSensorState.getPressureSensor(3)).getValue();
      
      
      
      footWrenches.get(RobotSide.RIGHT).set(0, 
            - (rightForceLeftHeel+rightForceLeftToe)*forceSensorYOffset 
            + (rightForceRightHeel+rightForceRightToe)*forceSensorYOffset);
      footWrenches.get(RobotSide.RIGHT).set(1, 
            +(rightForceLeftHeel+rightForceRightHeel)*forceSensorHeelXOffset 
            -(rightForceLeftToe+rightForceRightToe)*forceSensorToeXOffset);
      footWrenches.get(RobotSide.RIGHT).set(5, rightForceLeftHeel+rightForceRightHeel+rightForceLeftToe+rightForceRightToe);
      
      if(useToeSensorsOnlyForZForce)
      {
         footWrenches.get(RobotSide.LEFT).set(5, leftForceLeftHeel+leftForceRightHeel);
//         footWrenches.get(RobotSide.RIGHT).set(5, rightForceLeftHeel+rightForceRightHeel);         
         footWrenches.get(RobotSide.RIGHT).set(5, rightForceLeftHeel*2);         
      }

      
      if(tarePressureSensors.getBooleanValue())
      {
         tarePressureSensors();
         tarePressureSensors.set(false);        
      }


      stepprUpperBodyOffsetCalculator.update();
      
      for (StepprJoint joint : StepprJoint.values)
      {
         jointStates.get(joint).update();
      }

      if(updateOffsets.getBooleanValue())
      {
         stepprUpperBodyOffsetCalculator.updateOffsets();
         for (StepprJoint joint : StepprJoint.values)
         {
            jointStates.get(joint).updateOffsets();
         }
         
         updateOffsets.set(false);
      }
      
   }

   private void tarePressureSensors()
   {
      for(StepprActuatorState actuatorState:actuatorStates.values())
      {
         for(StepprSlowSensor sensor:actuatorState.getSlowSensors())
         {
            if(sensor instanceof PressureSensor)
            {
               ((PressureSensor)sensor).tare();
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
