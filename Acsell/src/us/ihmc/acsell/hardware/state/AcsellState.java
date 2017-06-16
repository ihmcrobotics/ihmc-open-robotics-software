package us.ihmc.acsell.hardware.state;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.EnumMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.acsell.hardware.AcsellJoint;
import us.ihmc.acsell.hardware.configuration.AcsellRobot;
import us.ihmc.acsell.hardware.configuration.StrainGaugeInformation;
import us.ihmc.acsell.hardware.state.slowSensors.StrainSensor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.steppr.hardware.state.StepprPowerDistributionADCState;
import us.ihmc.wanderer.hardware.state.WandererPowerDistributionADCState;

public abstract class AcsellState<ACTUATOR extends Enum<ACTUATOR> & AcsellActuator, JOINT extends Enum<JOINT> & AcsellJoint>
{

   protected final YoVariableRegistry registry;

   private final YoLong lastReceivedTime;
   private final YoLong timeSincePreviousPacket;

   private final YoLong stateCollectionStartTime;
   private final YoLong stateCollectionFinishTime;

   protected final EnumMap<ACTUATOR, AcsellActuatorState> actuatorStates;
   private final AcsellPowerDistributionADCState powerDistributionState;
   private final YoDouble totalMotorPower;
   protected final AcsellXSensState xsens;

   protected final EnumMap<JOINT, AcsellJointState> jointStates;

   protected final SideDependentList<DenseMatrix64F> footWrenches = new SideDependentList<>();

   public AcsellState(String name, double dt, AcsellRobot robot, YoVariableRegistry parentRegistry)
   {
      registry  = new YoVariableRegistry(name);
      lastReceivedTime = new YoLong("lastReceivedTime", registry);
      timeSincePreviousPacket = new YoLong("timeSincePreviousPacket", registry);
      stateCollectionStartTime = new YoLong("stateCollectionStartTime", registry);
      stateCollectionFinishTime = new YoLong("stateCollectionFinishTime", registry);

      if(robot==AcsellRobot.WANDERER)
         this.powerDistributionState = new WandererPowerDistributionADCState("powerDistribution", registry);
      else
         this.powerDistributionState = new StepprPowerDistributionADCState("powerDistribution", registry);
      totalMotorPower = new YoDouble("totalMotorPower", registry);
      xsens = new AcsellXSensState("xsens", robot, registry);
      
      actuatorStates = createActuators();
      jointStates = createJoints();

      for (RobotSide robotSide : RobotSide.values)
      {
         footWrenches.put(robotSide, new DenseMatrix64F(6, 1));
      }

      parentRegistry.addChild(registry);
   }

   protected StrainSensor getCalibratedJointStrainGauge(StrainGaugeInformation sensorInfo)
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

   protected abstract EnumMap<JOINT, AcsellJointState> createJoints();

   protected abstract EnumMap<ACTUATOR, AcsellActuatorState> createActuators();

   protected abstract ACTUATOR[] getActuators();

   protected abstract JOINT[] getJoints();

   public void update(ByteBuffer buffer, long timestamp) throws IOException
   {
      timeSincePreviousPacket.set(timestamp - lastReceivedTime.getLongValue());
      lastReceivedTime.set(timestamp);

      stateCollectionStartTime.set(buffer.getLong());
      stateCollectionFinishTime.set(buffer.getLong());
      for (ACTUATOR actuatorName : getActuators())
      {
         actuatorStates.get(actuatorName).update(buffer);
      }
      powerDistributionState.update(buffer);
      xsens.update(buffer);

      for (JOINT joint : getJoints())
      {
         jointStates.get(joint).update();
      }


      updateMotorPower();

   }

  

   private void updateMotorPower()
   {
      double accTotalMotorPower = 0;

      for (ACTUATOR actuatorName : getActuators())
      {
         accTotalMotorPower += actuatorStates.get(actuatorName).getMotorPower();
      }
      totalMotorPower.set(accTotalMotorPower);
   }


   public AcsellJointState getJointState(JOINT joint)
   {
      return jointStates.get(joint);
   }

   public void updateRawSensorData(JOINT joint, RawJointSensorDataHolder dataHolder)
   {
      AcsellJointState acsellJointState = jointStates.get(joint);
      dataHolder.setQ_raw(acsellJointState.getQ());
      dataHolder.setQd_raw(acsellJointState.getQd());
      for (int i = 0; i < acsellJointState.getNumberOfActuators(); i++)
      {
         dataHolder.setMotorAngle(i, acsellJointState.getMotorAngle(i));
      }
   }

   public AcsellXSensState getXSensState()
   {
      return xsens;
   }

   public DenseMatrix64F getFootWrench(RobotSide robotSide)
   {
      return footWrenches.get(robotSide);
   }

   public AcsellPowerDistributionADCState getPowerDistributionState()
   {
      return powerDistributionState;
   }
}
