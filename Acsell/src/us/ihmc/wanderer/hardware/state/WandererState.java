package us.ihmc.wanderer.hardware.state;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.EnumMap;

import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.acsell.hardware.configuration.AcsellRobot;
import us.ihmc.acsell.hardware.state.AcsellActuatorState;
import us.ihmc.acsell.hardware.state.AcsellAnkleJointState;
import us.ihmc.acsell.hardware.state.AcsellJointState;
import us.ihmc.acsell.hardware.state.AcsellLinearTransmissionJointState;
import us.ihmc.acsell.hardware.state.AcsellState;
import us.ihmc.acsell.hardware.state.slowSensors.StrainSensor;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.hardware.configuration.WandererAnkleKinematicParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class WandererState extends AcsellState<WandererActuator, WandererJoint>
{
   public WandererState(double dt, YoVariableRegistry parentRegistry)
   {
      super("Wanderer", dt, AcsellRobot.WANDERER, parentRegistry);
   }
   
   @Override
   public void update(ByteBuffer buffer, long timestamp) throws IOException
   {
      super.update(buffer, timestamp);
      
      throw new RuntimeException("TODO: Add foot force sensors and set super.footWrenches");
      /*
       * 
       * for(RobotSide robotSide : RobotSide.values)
       * {
       *    footWrenches.get(robotSide).set();
       * }
       * 
       * 
       */

   }
   protected EnumMap<WandererJoint, AcsellJointState> createJoints()
   {
      EnumMap<WandererJoint, AcsellJointState> jointStates = new EnumMap<>(WandererJoint.class);
      for (WandererJoint joint : WandererJoint.values)
      {
         if (joint.isLinear())
         {
            AcsellActuatorState actuator = actuatorStates.get(joint.getActuators()[0]);
            StrainSensor strainSensor = getCalibratedJointStrainGauge(joint.getStrainGaugeInformation());
            jointStates.put(joint, new AcsellLinearTransmissionJointState(joint.getSdfName(), joint.getRatio(), joint.hasOutputEncoder(), actuator,
                  strainSensor, registry));
         }
      }

      AcsellAnkleKinematicParameters parameters = new WandererAnkleKinematicParameters();
      // Create ankles
      AcsellAnkleJointState leftAnkle = new AcsellAnkleJointState(parameters, RobotSide.LEFT, actuatorStates.get(WandererActuator.LEFT_ANKLE_RIGHT),
            actuatorStates.get(WandererActuator.LEFT_ANKLE_LEFT), getCalibratedJointStrainGauge(WandererJoint.LEFT_ANKLE_Y.getStrainGaugeInformation()),
            getCalibratedJointStrainGauge(WandererJoint.LEFT_ANKLE_X.getStrainGaugeInformation()), registry);
      jointStates.put(WandererJoint.LEFT_ANKLE_Y, leftAnkle.ankleY());
      jointStates.put(WandererJoint.LEFT_ANKLE_X, leftAnkle.ankleX());

      AcsellAnkleJointState rightAnkle = new AcsellAnkleJointState(parameters, RobotSide.RIGHT, actuatorStates.get(WandererActuator.RIGHT_ANKLE_RIGHT),
            actuatorStates.get(WandererActuator.RIGHT_ANKLE_LEFT), getCalibratedJointStrainGauge(WandererJoint.RIGHT_ANKLE_Y.getStrainGaugeInformation()),
            getCalibratedJointStrainGauge(WandererJoint.RIGHT_ANKLE_X.getStrainGaugeInformation()), registry);
      jointStates.put(WandererJoint.RIGHT_ANKLE_Y, rightAnkle.ankleY());
      jointStates.put(WandererJoint.RIGHT_ANKLE_X, rightAnkle.ankleX());

      return jointStates;
   }

   protected EnumMap<WandererActuator, AcsellActuatorState> createActuators()
   {
      EnumMap<WandererActuator, AcsellActuatorState> actuatorStates = new EnumMap<>(WandererActuator.class);
      for (WandererActuator actuatorName : WandererActuator.values)
      {
         actuatorStates.put(actuatorName,
               new AcsellActuatorState(actuatorName.getName(), new WandererSlowSensorConstants(), actuatorName.getKt(), actuatorName.getSensedCurrentToTorqueDirection(), registry));
      }
      return actuatorStates;
   }

   @Override
   protected WandererActuator[] getActuators()
   {
      return WandererActuator.values;
   }

   @Override
   protected WandererJoint[] getJoints()
   {
      return WandererJoint.values;
   }
}
