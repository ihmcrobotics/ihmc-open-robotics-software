package us.ihmc.steppr.hardware.state;

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
import us.ihmc.acsell.hardware.state.slowSensors.AcsellSlowSensor;
import us.ihmc.acsell.hardware.state.slowSensors.PressureSensor;
import us.ihmc.acsell.hardware.state.slowSensors.StrainSensor;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprAnkle;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.configuration.StepprAnkleKinematicParameters;
import us.ihmc.steppr.hardware.state.slowSensors.StepprSlowSensorConstants;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;

public class StepprState extends AcsellState<StepprActuator, StepprJoint>
{
   private static final int[] leftFootForceSensorsToUse = { 0, 1, 2, 3 }; //leftMost sensor0 is broken
   private static final int[] rightFootForceSensorsToUse = { 0, 1, 2, 3 }; //rightMost sensor3 is broken
   private static final boolean USE_STRAIN_GAUGES_FOR_Z_FORCE = false;
   
   private final StepprUpperBodyOffsetCalculator stepprUpperBodyOffsetCalculator;
   private final BooleanYoVariable updateOffsets;
   
   private final BooleanYoVariable tareSensors;
   
   private final StrainSensor leftFootStrainGauge; 
   private final StrainSensor rightFootStrainGauge;


   public StepprState(double dt, YoVariableRegistry parentRegistry)
   {
      super("Steppr", dt, AcsellRobot.STEPPR, parentRegistry);
      updateOffsets = new BooleanYoVariable("updateOffsets", registry);
      tareSensors = new BooleanYoVariable("tareSensors", registry);
      
      stepprUpperBodyOffsetCalculator = new StepprUpperBodyOffsetCalculator(actuatorStates.get(StepprActuator.TORSO_Z),
            actuatorStates.get(StepprActuator.TORSO_X), actuatorStates.get(StepprActuator.TORSO_Y), actuatorStates.get(StepprActuator.TORSO_Z), xsens, dt,
            registry);
      
      leftFootStrainGauge = getCalibratedJointStrainGauge(StepprAnkle.LEFT.getShankSensor());
      rightFootStrainGauge = getCalibratedJointStrainGauge(StepprAnkle.RIGHT.getShankSensor());
   }

   @Override
   public void update(ByteBuffer buffer, long timestamp) throws IOException
   {
      super.update(buffer, timestamp);
      stepprUpperBodyOffsetCalculator.update();
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


   }
   
   private void updateFootForceSensor()
   {

      if (tareSensors.getBooleanValue())
      {
         tarePressureSensors();
         leftFootStrainGauge.tare();
         rightFootStrainGauge.tare();
         //jointStates.get(StepprJoint.LEFT_ANKLE_X).tare();
         tareSensors.set(false);
      }

      double leftFootForce = 0;
      double rightFootForce = 0;

      AcsellActuatorState leftFootSensorState = actuatorStates.get(StepprActuator.LEFT_ANKLE_RIGHT);
      AcsellActuatorState rightFootSensorState = actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT);
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
         final double distHeelFromAnkle = 0.035 + 2.0 * 0.0254 - 0.001;
         final double distToeFromAnkle = 0.16 + 0.001;
         double leftAnkleYTau = jointStates.get(StepprJoint.LEFT_ANKLE_Y).getTau();
         double leftToeForce = leftHeelForce * distHeelFromAnkle + leftAnkleYTau / distToeFromAnkle;
         double rightAnkleYTau = jointStates.get(StepprJoint.RIGHT_ANKLE_Y).getTau();
         double rightToeForce = rightHeelForce * distHeelFromAnkle + rightAnkleYTau / distToeFromAnkle;

         leftFootForce = leftHeelForce + leftToeForce;
         rightFootForce = rightHeelForce + rightToeForce;
      }

      footWrenches.get(RobotSide.LEFT).set(5, leftFootForce);
      footWrenches.get(RobotSide.RIGHT).set(5, rightFootForce);

   }
   
   private void tarePressureSensors()
   {
      for (AcsellActuatorState actuatorState : actuatorStates.values())
      {
         for (AcsellSlowSensor sensor : actuatorState.getSlowSensors())
         {
            if (sensor instanceof PressureSensor)
            {
               ((PressureSensor) sensor).tare();
            }
         }
      }

   }


   protected EnumMap<StepprJoint, AcsellJointState> createJoints()
   {
      EnumMap<StepprJoint, AcsellJointState> jointStates = new EnumMap<>(StepprJoint.class);
      for (StepprJoint joint : StepprJoint.values)
      {
         if (joint.isLinear())
         {
            AcsellActuatorState actuator = actuatorStates.get(joint.getActuators()[0]);
            StrainSensor strainSensor = getCalibratedJointStrainGauge(joint.getStrainGaugeInformation());
            jointStates.put(joint, new AcsellLinearTransmissionJointState(joint.getSdfName(), joint.getRatio(), joint.hasOutputEncoder(), actuator,
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

      
      AcsellAnkleKinematicParameters parameters = new StepprAnkleKinematicParameters();
      // Create ankles
      AcsellAnkleJointState leftAnkle = new AcsellAnkleJointState(parameters, RobotSide.LEFT, actuatorStates.get(StepprActuator.LEFT_ANKLE_RIGHT),
            actuatorStates.get(StepprActuator.LEFT_ANKLE_LEFT), getCalibratedJointStrainGauge(StepprJoint.LEFT_ANKLE_Y.getStrainGaugeInformation()), getCalibratedJointStrainGauge(StepprJoint.LEFT_ANKLE_X.getStrainGaugeInformation()), registry);
      jointStates.put(StepprJoint.LEFT_ANKLE_Y, leftAnkle.ankleY());
      jointStates.put(StepprJoint.LEFT_ANKLE_X, leftAnkle.ankleX());

      AcsellAnkleJointState rightAnkle = new AcsellAnkleJointState(parameters, RobotSide.RIGHT, actuatorStates.get(StepprActuator.RIGHT_ANKLE_RIGHT),
            actuatorStates.get(StepprActuator.RIGHT_ANKLE_LEFT), getCalibratedJointStrainGauge(StepprJoint.RIGHT_ANKLE_Y.getStrainGaugeInformation()), getCalibratedJointStrainGauge(StepprJoint.RIGHT_ANKLE_X.getStrainGaugeInformation()), registry);
      jointStates.put(StepprJoint.RIGHT_ANKLE_Y, rightAnkle.ankleY());
      jointStates.put(StepprJoint.RIGHT_ANKLE_X, rightAnkle.ankleX());

      return jointStates;
   }

   protected EnumMap<StepprActuator, AcsellActuatorState> createActuators()
   {
      EnumMap<StepprActuator, AcsellActuatorState> actuatorStates = new EnumMap<>(StepprActuator.class);
      for (StepprActuator actuator : StepprActuator.values)
      {
         actuatorStates
               .put(actuator, new AcsellActuatorState(actuator, AcsellRobot.STEPPR, new StepprSlowSensorConstants(), registry));
      }
      return actuatorStates;
   }


   @Override
   protected StepprActuator[] getActuators()
   {
      return StepprActuator.values;
   }


   @Override
   protected StepprJoint[] getJoints()
   {
      return StepprJoint.values;
   }
}
