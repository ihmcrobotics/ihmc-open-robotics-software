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
import us.ihmc.acsell.hardware.state.slowSensors.PressureSensor;
import us.ihmc.acsell.hardware.state.slowSensors.StrainSensor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.hardware.configuration.WandererAnkleKinematicParameters;
import us.ihmc.wanderer.parameters.WandererPhysicalProperties;

public class WandererState extends AcsellState<WandererActuator, WandererJoint>
{
   private final WandererFootSensorManager footSensorManager;
   private final WandererBatteryMonitor batteryMonitor;
   
   public WandererState(double dt, YoVariableRegistry parentRegistry)
   {
      super("Wanderer", dt, AcsellRobot.WANDERER, parentRegistry);
      footSensorManager = new WandererFootSensorManager(footWrenches, jointStates, actuatorStates, parentRegistry);
      batteryMonitor = new WandererBatteryMonitor(actuatorStates, parentRegistry);
   }
   
   @Override
   public void update(ByteBuffer buffer, long timestamp) throws IOException
   {
      super.update(buffer, timestamp);
      
      //TODO: Add foot force sensors and set super.footWrenches
//      updateFootForceSensor();//TODO: Fix this!

      footSensorManager.update();
      batteryMonitor.update();
      
   }
   
   private void updateFootForceSensor()
   {
      double leftFootForce = 0;
      double rightFootForce = 0;

      AcsellActuatorState leftFootSensorState = actuatorStates.get(WandererActuator.LEFT_KNEE);
      AcsellActuatorState rightFootSensorState = actuatorStates.get(WandererActuator.RIGHT_KNEE);


         double leftHeelForce = 0.0, rightHeelForce = 0.0;
         for (int sensor=0; sensor<4; sensor++)
         {
            leftHeelForce += ((PressureSensor) leftFootSensorState.getPressureSensor(sensor)).getValue();
         }

         for (int sensor=0; sensor<4; sensor++)
         {
            rightHeelForce += ((PressureSensor) rightFootSensorState.getPressureSensor(sensor)).getValue();
         }

         //calculate toe Z force based on ankle
         final double distHeelFromAnkle = WandererPhysicalProperties.footBack - 0.5*0.0254;
         final double distToeFromAnkle = WandererPhysicalProperties.footForward - 0.5*0.0254;
         double leftAnkleYTau = jointStates.get(WandererJoint.LEFT_ANKLE_Y).getTau();
         double leftToeForce = leftHeelForce * distHeelFromAnkle + leftAnkleYTau / distToeFromAnkle;
         double rightAnkleYTau = jointStates.get(WandererJoint.RIGHT_ANKLE_Y).getTau();
         double rightToeForce = rightHeelForce * distHeelFromAnkle + rightAnkleYTau / distToeFromAnkle;

         leftFootForce = leftHeelForce + leftToeForce;
         rightFootForce = rightHeelForce + rightToeForce;


      footWrenches.get(RobotSide.LEFT).set(5, leftFootForce);
      footWrenches.get(RobotSide.RIGHT).set(5, rightFootForce);
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
      for (WandererActuator actuator : WandererActuator.values)
      {
         actuatorStates.put(actuator,
               new AcsellActuatorState(actuator, AcsellRobot.WANDERER, new WandererSlowSensorConstants(), registry));
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
