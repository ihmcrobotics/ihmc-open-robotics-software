package us.ihmc.exampleSimulations.m2;

import us.ihmc.exampleSimulations.m2.Output.PerfectProcessedOutputs;
import us.ihmc.exampleSimulations.m2.Sensors.PerfectSensorProcessing;
import us.ihmc.exampleSimulations.m2.Sensors.ProcessedSensors;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;

public class M2ProcessedSensorsControllerWithStateTransitions implements SimulationDoneCriterion, RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("M2ClawarController");

   // These are the control variables which need to be created now:

   // Things we set
   private final YoDouble control_dt = new YoDouble("control_dt", registry);

   private final YoDouble ff_z = new YoDouble("ff_z", registry);

   private final YoDouble yd_gain = new YoDouble("yd_gain", registry);
   private final YoDouble yd_ankle_gain = new YoDouble("yd_ankle_gain", registry);
   private final YoDouble lateral_yaw_mul = new YoDouble("lateral_yaw_mul", registry);

   private final YoDouble ceny_gain = new YoDouble("ceny_gain", registry);

   // Body orientation control during stance:
   private final YoDouble bodyYawGain = new YoDouble("bodyYawGain", registry);
   private final YoDouble bodyYawDamping = new YoDouble("bodyYawDamping", registry);

   private final YoDouble desiredBodyPitch = new YoDouble("desiredBodyPitch", registry);
   private final YoDouble bodyPitchGain = new YoDouble("bodyPitchGain", registry);
   private final YoDouble bodyPitchDamping = new YoDouble("bodyPitchDamping", registry);

   private final YoDouble bodyRollGain = new YoDouble("bodyRollGain", registry);
   private final YoDouble bodyRollDamping = new YoDouble("bodyRollDamping", registry);

   private final YoDouble hipSwingAngle = new YoDouble("hipSwingAngle", "Desired hip swing angle", registry);

   private final YoDouble hipSwingGain = new YoDouble("hipSwingGain", "Proportional gain for hip during swing", registry);
   private final YoDouble hipSwingDamp = new YoDouble("hipSwingDamp", "Damping gain for hip during swing", registry);
   private final YoDouble hip_k = new YoDouble("hip_k", registry);
   private final YoDouble hip_hold = new YoDouble("hip_hold", registry);

   private final YoDouble knee_d = new YoDouble("knee_d", registry);
   private final YoDouble knee_gain = new YoDouble("knee_gain", registry);
   private final YoDouble knee_damp = new YoDouble("knee_damp", registry);

   private final YoDouble ankle_gain = new YoDouble("ankle_gain", registry);
   private final YoDouble ankle_damp = new YoDouble("ankle_damp", registry);

   private final YoDouble ankle_d = new YoDouble("ankle_d", registry);
   private final YoDouble ankle_k = new YoDouble("ankle_k", registry);
   private final YoDouble ankle_b = new YoDouble("ankle_b", registry);

   private final YoDouble hip_b = new YoDouble("hip_b", registry);
   private final YoDouble footForceAlpha = new YoDouble("footForceAlpha", "Alpha for filter the foot forces", registry);
   private final YoDouble ankle_limit_set = new YoDouble("ankle_limit_set", registry);
   private final YoDouble ankle_limit_gain = new YoDouble("ankle_limit_gain", registry);
   private final YoDouble flat_ankle_roll = new YoDouble("flat_ankle_roll", registry);

   private final YoDouble swing_roll_off = new YoDouble("swing_roll_off", registry);

   private final YoDouble swing_gain_knee = new YoDouble("swing_gain_knee", registry);
   private final YoDouble swing_damp_knee1 = new YoDouble("swing_damp_knee1", registry);
   private final YoDouble swing_damp_knee = new YoDouble("swing_damp_knee", registry);

   private final YoDouble push_set = new YoDouble("push_set", registry);
   private final YoDouble push_gain = new YoDouble("push_gain", registry);
   private final YoDouble push_damp = new YoDouble("push_damp", registry);

   private final YoDouble tran_time = new YoDouble("tran_time", registry);
   private final YoDouble swing_time = new YoDouble("swing_time", registry);
   private final YoDouble min_support_time = new YoDouble("min_support_time", registry);

   private final YoDouble max_hip_torque = new YoDouble("max_hip_torque", registry);

   private final YoDouble force_thresh = new YoDouble("force_thresh", registry);

   private final YoDouble decay_freq = new YoDouble("decay_freq", registry);

   // The following variables are computed as part of the controller. They are not user set parameters.
   private final YoDouble q_d_left_hip_pitch = new YoDouble("q_d_left_hip_pitch", "Desired left hip pitch angle [rad]", registry);
   private final YoDouble q_d_right_hip_pitch = new YoDouble("q_d_right_hip_pitch", "Desired right hip pitch angle [rad]", registry);


   // private final YoVariable right_hip_set = new YoVariable("right_hip_set", "Desired left hip pitch angle [rad]", registry);
   private final YoDouble k_right_hip_yaw = new YoDouble("k_right_hip_yaw", registry);
   private final YoDouble k_right_hip_roll = new YoDouble("k_right_hip_roll", registry);
   private final YoDouble k_right_ankle_roll = new YoDouble("k_right_ankle_roll", registry);
   private final YoDouble k_left_ankle_roll = new YoDouble("k_left_ankle_roll", registry);
   private final YoDouble ff_right_knee = new YoDouble("ff_right_knee", registry);
   private final YoDouble ff_right_hip_pitch = new YoDouble("ff_right_hip_pitch", registry);
   private final YoDouble ff_right_ankle_pitch = new YoDouble("ff_right_ankle_pitch", registry);
   private final YoDouble ff_left_knee = new YoDouble("ff_left_knee", registry);
   private final YoDouble ff_left_hip_yaw = new YoDouble("ff_left_hip_yaw", registry);
   private final YoDouble ff_left_hip_roll = new YoDouble("ff_left_hip_roll", registry);
   private final YoDouble ff_left_hip_pitch = new YoDouble("ff_left_hip_pitch", registry);
   private final YoDouble ff_left_ankle_roll = new YoDouble("ff_left_ankle_roll", registry);
   private final YoDouble b_right_hip_yaw = new YoDouble("b_right_hip_yaw", registry);
   private final YoDouble b_right_hip_roll = new YoDouble("b_right_hip_roll", registry);
   private final YoDouble b_right_ankle_roll = new YoDouble("b_right_ankle_roll", registry);
   private final YoDouble b_left_ankle_roll = new YoDouble("b_left_ankle_roll", registry);
   private final YoDouble act_right_ankle = new YoDouble("act_right_ankle", registry);
   private final YoDouble right_heel = new YoDouble("right_heel", registry);
   private final YoDouble ramp_right = new YoDouble("ramp_right", registry);
   private final YoDouble ramp_left = new YoDouble("ramp_left", registry);
   private final YoDouble left_heel = new YoDouble("left_heel", registry);
   private final YoDouble ff_hip_roll = new YoDouble("ff_hip_roll", registry);

   private final YoDouble ff_right_ankle_roll_speed = new YoDouble("ff_right_ankle_roll_speed", "Feedforward ankle speed, [rad/s]", registry);
   private final YoDouble ff_left_ankle_roll_speed = new YoDouble("ff_left_ankle_roll_speed", "Feedforward ankle speed, [rad/s]", registry);
   private final YoDouble k_left_hip_yaw = new YoDouble("k_left_hip_yaw", registry);
   private final YoDouble k_left_hip_roll = new YoDouble("k_left_hip_roll", registry);
   private final YoDouble ff_right_hip_yaw = new YoDouble("ff_right_hip_yaw", registry);
   private final YoDouble ff_right_hip_roll = new YoDouble("ff_right_hip_roll", registry);
   private final YoDouble ff_right_ankle_roll = new YoDouble("ff_right_ankle_roll", registry);
   private final YoDouble ff_left_ankle_pitch = new YoDouble("ff_left_ankle_pitch", registry);
   private final YoDouble b_left_hip_yaw = new YoDouble("b_left_hip_yaw", registry);
   private final YoDouble b_left_hip_roll = new YoDouble("b_left_hip_roll", registry);
   private final YoDouble act_left_ankle = new YoDouble("act_left_ankle", registry);
   private final YoDouble pas_right_ankle = new YoDouble("pas_right_ankle", registry);
   private final YoDouble pas_left_ankle = new YoDouble("pas_left_ankle", registry);

   private final YoDouble leftCopPercentFromHeel = new YoDouble("leftCopPercentFromHeel",
                                                        "The location of the left foot COP from the heel. 0.0=all on heel, " + "1,0= all on  toes", registry);
   private final YoDouble rightCopPercentFromHeel = new YoDouble("rightCopPercentFromHeel",
                                                         "The location of the right foot COP from the heel. 0.0=all on heel, " + "1,0= all on  toes", registry);

   // Desired for feedback control
   private final YoDouble q_d_right_hip_roll = new YoDouble("q_d_right_hip_roll", "Desired right hip roll angle [rad]", registry);
   private final YoDouble q_d_left_hip_roll = new YoDouble("q_d_left_hip_roll", "Desired left hip roll angle [rad]", registry);
   private final YoDouble left_ankle_roll_error = new YoDouble("left_ankle_roll_error", "Intermeidate error term", registry);
   private final YoDouble right_ankle_roll_error = new YoDouble("right_ankle_roll_error", "Intermeidate error term [rad]", registry);

   // These are calculate variables related to state of the robot
   private final YoDouble rightHeelToBodyCenterInWorldY = new YoDouble("rightHeelToBodyCenterInWorldY",
                                                               "Distance from Right Heel To Body Center In Point" + " In World Y [m]", registry);
   private final AlphaFilteredYoVariable rightFootForceFiltered = new AlphaFilteredYoVariable("rightFootForceFiltered", registry, footForceAlpha);
   private final AlphaFilteredYoVariable leftFootForceFiltered = new AlphaFilteredYoVariable("leftFootForceFiltered", registry, footForceAlpha);
   private final YoDouble q_d_yaw = new YoDouble("q_d_yaw", registry);
   private final YoDouble q_d_right_hip_yaw = new YoDouble("q_d_right_hip_yaw", registry);
   private final YoDouble q_d_left_hip_yaw = new YoDouble("q_d_left_hip_yaw", registry);
   private final YoDouble bodyVelocityInWorldX = new YoDouble("bodyVelocityInWorldX", registry);
   private final YoDouble bodyVelocityInBodyFrameY = new YoDouble("bodyVelocityInBodyFrameY", registry);
   private final YoDouble bodyVelocityInBodyFrameX = new YoDouble("xd", registry);
   private final YoDouble bodyCenterToLeftHeelInWorldY = new YoDouble("bodyCenterToLeftHeelInWorldY",
                                                              "Distance from Body Center to Left Heel In Point" + " In World Y [m]", registry);
   private final YoDouble velocityHeadingInBodyFrame = new YoDouble("velocityHeadingInBodyFrame",
                                                            "This is the heading angle of the body velocity relative to body frame x", registry);

   private final YoBoolean doneWithStateRight = new YoBoolean("doneWithStateRight", registry);
   private final YoBoolean doneWithStateLeft = new YoBoolean("doneWithStateLeft", registry);

   private final SideDependentList<YoDouble> ffHipYaw = new SideDependentList<YoDouble>(ff_left_hip_yaw, ff_right_hip_yaw);
   private final SideDependentList<YoDouble> ffHipRoll = new SideDependentList<YoDouble>(ff_left_hip_roll, ff_right_hip_roll);
   private final SideDependentList<YoDouble> ffHipPitch = new SideDependentList<YoDouble>(ff_left_hip_pitch, ff_right_hip_pitch);
   private final SideDependentList<YoDouble> ffKnee = new SideDependentList<YoDouble>(ff_left_knee, ff_right_knee);
//   private final SideDependentList<YoVariable> ffAnklePitch = new SideDependentList<YoVariable>(ff_left_ankle_pitch, ff_right_ankle_pitch);
   private final SideDependentList<YoDouble> ffAnkleRoll = new SideDependentList<YoDouble>(ff_left_ankle_roll, ff_right_ankle_roll);

   private final SideDependentList<YoDouble> ankleRollError = new SideDependentList<YoDouble>(left_ankle_roll_error, right_ankle_roll_error);
   private final SideDependentList<YoDouble> ffAnkleRollSpeed = new SideDependentList<YoDouble>(ff_left_ankle_roll_speed, ff_right_ankle_roll_speed);
   private final SideDependentList<YoDouble> footForceFiltered = new SideDependentList<YoDouble>(leftFootForceFiltered, rightFootForceFiltered);
   private final SideDependentList<YoBoolean> doneWithState = new SideDependentList<YoBoolean>(doneWithStateLeft, doneWithStateRight);
   private final SideDependentList<YoDouble> activeAnkle = new SideDependentList<YoDouble>(act_left_ankle, act_right_ankle);
   private final SideDependentList<YoDouble> passiveAnkle = new SideDependentList<YoDouble>(pas_left_ankle, pas_right_ankle);

   private final SideDependentList<YoDouble> ramp = new SideDependentList<YoDouble>(ramp_left, ramp_right);
   private final SideDependentList<YoDouble> q_d_hipYaw = new SideDependentList<YoDouble>(q_d_left_hip_yaw, q_d_right_hip_yaw);
   private final SideDependentList<YoDouble> q_d_hipRoll = new SideDependentList<YoDouble>(q_d_left_hip_roll, q_d_right_hip_roll);
   private final SideDependentList<YoDouble> k_hipYaw = new SideDependentList<YoDouble>(k_left_hip_yaw, k_right_hip_yaw);
   private final SideDependentList<YoDouble> b_hipYaw = new SideDependentList<YoDouble>(b_left_hip_yaw, b_right_hip_yaw);
   private final SideDependentList<YoDouble> kAnkleRoll = new SideDependentList<YoDouble>(k_left_ankle_roll, k_right_ankle_roll);
   private final SideDependentList<YoDouble> bAnkleRoll = new SideDependentList<YoDouble>(b_left_ankle_roll, b_right_ankle_roll);
   private final SideDependentList<YoDouble> kHipRoll = new SideDependentList<YoDouble>(k_left_hip_roll, k_right_hip_roll);
   private final SideDependentList<YoDouble> bHipRoll = new SideDependentList<YoDouble>(b_left_hip_roll, b_right_hip_roll);
   private final SideDependentList<YoDouble> q_d_hipPitch = new SideDependentList<YoDouble>(q_d_left_hip_pitch, q_d_right_hip_pitch);
   private final SideDependentList<YoDouble> heel = new SideDependentList<YoDouble>(left_heel, right_heel);
   private final SideDependentList<YoDouble> bodyCenterToHeelInWorldY = new SideDependentList<YoDouble>(bodyCenterToLeftHeelInWorldY,
                                                                             rightHeelToBodyCenterInWorldY);


   private StateMachine<States> leftStateMachine, rightStateMachine;
   private SideDependentList<StateMachine<States>> stateMachines;

   private final PerfectSensorProcessing perfectSensorProcessing;
   private final PerfectProcessedOutputs processedOutputs;
   private final ProcessedSensors processedSensors;

   StateTransitionCondition doneWithStateBooleanRight = new StateTransitionCondition()
   {
      public boolean checkCondition()
      {
         return doneWithStateRight.getBooleanValue();
      }
   };

   StateTransitionCondition doneWithStateBooleanLeft = new StateTransitionCondition()
   {
      public boolean checkCondition()
      {
         return doneWithStateLeft.getBooleanValue();
      }
   };


   private enum States {SUPPORT, TOE_OFF, SWING, STRAIGHTEN}

   private final M2Parameters m2Parameters;

   private String name;

   public M2ProcessedSensorsControllerWithStateTransitions(M2Parameters m2Parameters, ProcessedSensors pS, PerfectSensorProcessing perfectSensorProcessing,
           PerfectProcessedOutputs pO, String name)
   {
      this.name = name;
      this.m2Parameters = m2Parameters;

      this.processedSensors = pS;
      this.perfectSensorProcessing = perfectSensorProcessing;
      this.processedOutputs = pO;
      initControl();

      if (M2Simulation.USE_HEAVY_M2)
         initControlForHeavyM2();

      setupStateMachines();
   }

   public void initControl()
   {
      control_dt.set(0.0004);

      bodyPitchGain.set(100);
      bodyPitchDamping.set(20);

      bodyRollGain.set(200);
      bodyRollDamping.set(20);

      bodyYawGain.set(30);
      bodyYawDamping.set(4);

      ff_z.set(241.961);

      ankle_limit_set.set(0);
      ankle_limit_gain.set(400);

      knee_d.set(0);
      knee_gain.set(30);
      knee_damp.set(10);

      yd_ankle_gain.set(282.745);
      ankle_k.set(4);
      ankle_b.set(0.1);

      hipSwingAngle.set(0.624706);
      hip_hold.set(0.49);

      footForceAlpha.set(1.0 - 0.04);

      flat_ankle_roll.set(-0.087451);
      yd_gain.set(0.358824);

      desiredBodyPitch.set(0);


      decay_freq.set(100);
      push_gain.set(30);
      push_set.set(0.3);
      push_damp.set(0);
      tran_time.set(0.2);
      lateral_yaw_mul.set(0);
      hip_k.set(92.0784);
      hip_b.set(22.8431);
      hipSwingGain.set(23.7176);
      hipSwingDamp.set(2.37255);
      max_hip_torque.set(20);
      swing_damp_knee1.set(0.25);
      ankle_gain.set(4);
      ankle_d.set(0);
      ankle_damp.set(0.5);
      swing_roll_off.set(-0.047255);
      ceny_gain.set(2);
      swing_gain_knee.set(1);
      swing_damp_knee.set(1.2);
      force_thresh.set(100);
      min_support_time.set(0.2);
      swing_time.set(0.3);
   }

   public void initControlForHeavyM2()
   {
      bodyPitchGain.set(100 * 6.0);    // 10.0;
      bodyPitchDamping.set(20 * 6.0);    // 10.0;

      bodyRollGain.set(200 * 6.0);    // 10.0;
      bodyRollDamping.set(20 * 6.0);    // 10.0;

      bodyYawGain.set(30 * 4.0);
      bodyYawDamping.set(4 * 4.0);

      ff_z.set(241.961 * 6.0);    // 10.0;

      knee_gain.set(30.0 * 10.0);
      knee_damp.set(10.0 * 10.0);

      yd_ankle_gain.set(282.745 * 6.0);
      ankle_k.set(4.0 * 6.0);
      ankle_b.set(0.1 * 6.0);

      hipSwingAngle.set(0.8);    // 0.5;
      hip_hold.set(0.4);

      ankle_limit_gain.set(400.0 * 10.0);    // 10.0;
      push_gain.set(30.0 * 10.0);
      push_set.set(0.3);
      push_damp.set(0);


      force_thresh.set(100 * 10.0);

      hip_k.set(800.0);    // 92.0784;
      swing_time.set(0.45);    // 0.3;
   }


   public void doControl()
   {
      perfectSensorProcessing.update();
      balistic_walking_state_machine();

      // Set the torques.
      processedOutputs.setJointTorque(RobotSide.LEFT, JointName.HIP_YAW, ff_left_hip_yaw.getDoubleValue());
      processedOutputs.setJointTorque(RobotSide.LEFT, JointName.HIP_ROLL, ff_left_hip_roll.getDoubleValue());
      processedOutputs.setJointTorque(RobotSide.LEFT, JointName.HIP_PITCH, ff_left_hip_pitch.getDoubleValue());

      processedOutputs.setJointTorque(RobotSide.LEFT, JointName.KNEE, ff_left_knee.getDoubleValue());

      processedOutputs.setJointTorque(RobotSide.LEFT, JointName.ANKLE_ROLL, ff_left_ankle_roll.getDoubleValue());
      processedOutputs.setJointTorque(RobotSide.LEFT, JointName.ANKLE_PITCH, ff_left_ankle_pitch.getDoubleValue());

      processedOutputs.setJointTorque(RobotSide.RIGHT, JointName.HIP_YAW, ff_right_hip_yaw.getDoubleValue());
      processedOutputs.setJointTorque(RobotSide.RIGHT, JointName.HIP_ROLL, ff_right_hip_roll.getDoubleValue());
      processedOutputs.setJointTorque(RobotSide.RIGHT, JointName.HIP_PITCH, ff_right_hip_pitch.getDoubleValue());

      processedOutputs.setJointTorque(RobotSide.RIGHT, JointName.KNEE, ff_right_knee.getDoubleValue());

      processedOutputs.setJointTorque(RobotSide.RIGHT, JointName.ANKLE_ROLL, ff_right_ankle_roll.getDoubleValue());
      processedOutputs.setJointTorque(RobotSide.RIGHT, JointName.ANKLE_PITCH, ff_right_ankle_pitch.getDoubleValue());

   }

   private void setupStateMachines()
   {
      // States and Actions:

      State<States> leftSupportState = new SupportState(RobotSide.LEFT, States.SUPPORT);
      State<States> leftToeOffState = new ToeOffState(RobotSide.LEFT, States.TOE_OFF);
      State<States> leftSwingState = new SwingState(RobotSide.LEFT, States.SWING);
      State<States> leftStraightenState = new StraightenState(RobotSide.LEFT, States.STRAIGHTEN);


      State<States> rightSupportState = new SupportState(RobotSide.RIGHT, States.SUPPORT);
      State<States> rightToeOffState = new ToeOffState(RobotSide.RIGHT, States.TOE_OFF);
      State<States> rightSwingState = new SwingState(RobotSide.RIGHT, States.SWING);
      State<States> rightStraightenState = new StraightenState(RobotSide.RIGHT, States.STRAIGHTEN);


      // Create the state machines:
      leftStateMachine = new StateMachine<States>("left_state", "left_switch_time_state_machine", States.class, processedSensors.getTimeYoVariable(), registry);
      rightStateMachine = new StateMachine<States>("right_state", "right_switch_time_state_machine", States.class, processedSensors.getTimeYoVariable(), registry);
      stateMachines = new SideDependentList<StateMachine<States>>(leftStateMachine, rightStateMachine);

      // left State Transitions:

      StateTransition<States> leftSupportToToeOff = new StateTransition<States>(leftToeOffState.getStateEnum(), doneWithStateBooleanLeft);
      leftSupportState.addStateTransition(leftSupportToToeOff);
      StateTransition<States> leftToeOffToSwing = new StateTransition<States>(leftSwingState.getStateEnum(), doneWithStateBooleanLeft);
      leftToeOffState.addStateTransition(leftToeOffToSwing);
      StateTransition<States> leftSwingToStraighten = new StateTransition<States>(leftStraightenState.getStateEnum(), doneWithStateBooleanLeft);
      leftSwingState.addStateTransition(leftSwingToStraighten);
      StateTransition<States> leftStraightenToSupport = new StateTransition<States>(leftSupportState.getStateEnum(), doneWithStateBooleanLeft);
      leftStraightenState.addStateTransition(leftStraightenToSupport);

      // right State Transitions:
      StateTransition<States> rightSupportToToeOff = new StateTransition<States>(rightToeOffState.getStateEnum(), doneWithStateBooleanRight);
      rightSupportState.addStateTransition(rightSupportToToeOff);
      StateTransition<States> rightToeOffToSwing = new StateTransition<States>(rightSwingState.getStateEnum(), doneWithStateBooleanRight);
      rightToeOffState.addStateTransition(rightToeOffToSwing);
      StateTransition<States> rightSwingToStraighten = new StateTransition<States>(rightStraightenState.getStateEnum(), doneWithStateBooleanRight);
      rightSwingState.addStateTransition(rightSwingToStraighten);
      StateTransition<States> rightStraightenToSupport = new StateTransition<States>(rightSupportState.getStateEnum(), doneWithStateBooleanRight);
      rightStraightenState.addStateTransition(rightStraightenToSupport);

      // Assemble the left State Machine:
      leftStateMachine.addState(leftSupportState);
      leftStateMachine.addState(leftToeOffState);
      leftStateMachine.addState(leftSwingState);
      leftStateMachine.addState(leftStraightenState);

      // Assemble the right State Machine:
      rightStateMachine.addState(rightSupportState);
      rightStateMachine.addState(rightToeOffState);
      rightStateMachine.addState(rightSwingState);
      rightStateMachine.addState(rightStraightenState);

      // Set the Initial States:

      leftStateMachine.setCurrentState(States.SUPPORT);
      rightStateMachine.setCurrentState(States.SWING);
   }

   void balistic_walking_state_machine()
   {
      defaultActionsIntoStates();

      leftStateMachine.doAction();
      rightStateMachine.doAction();

      leftStateMachine.checkTransitionConditions();
      rightStateMachine.checkTransitionConditions();

      defaultActionsOutOfStates();
   }

   private void defaultActionsIntoStates()
   {
      // check to see if this should be a vector of x and y velocity
      bodyVelocityInWorldX.set(processedSensors.getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis.X));

      // pS.getRobotYawPitchOrRoll()

      bodyVelocityInBodyFrameY
         .set(processedSensors.getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis.Y)
                * Math.cos(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW)) - processedSensors
                   .getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis.X) * Math.sin(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW)));
      bodyVelocityInBodyFrameX
         .set(processedSensors.getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis.X)
                * Math.cos(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW)) + processedSensors
                   .getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis.Y) * Math.sin(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW)));

      // if (Math.abs(bodyVelocityInBodyFrameX.val) > 0.01)
      // velocityHeadingInBodyFrame.val = Math.atan(bodyVelocityInBodyFrameY.val / bodyVelocityInBodyFrameX.val);
      // else if (Math.abs(bodyVelocityInBodyFrameY.val) > 0.02)
      // velocityHeadingInBodyFrame.val = Math.PI / 2.0 + Math.atan(bodyVelocityInBodyFrameX.val / bodyVelocityInBodyFrameY.val);
      // else
      // velocityHeadingInBodyFrame.val = 0.0;

      if (Math.abs(bodyVelocityInBodyFrameX.getDoubleValue()) > 0.01)
         velocityHeadingInBodyFrame.set(Math.atan2(bodyVelocityInBodyFrameY.getDoubleValue(), bodyVelocityInBodyFrameX.getDoubleValue()));
      else
         velocityHeadingInBodyFrame.set(0.0);


      /* Center distance from foot to body */
      bodyCenterToLeftHeelInWorldY.set(-processedSensors.getRobotBodyPositionComponentInWorldCoordinates(RobotAxis.Y)
                                         + processedSensors.getGroundContactPointsPositions(RobotSide.LEFT, ContactPointName.HEEL_IN).getY());
      rightHeelToBodyCenterInWorldY.set(processedSensors.getRobotBodyPositionComponentInWorldCoordinates(RobotAxis.Y)
              - processedSensors.getGroundContactPointsPositions(RobotSide.RIGHT, ContactPointName.HEEL_IN).getY());

      /* Calculate forces on the feet */
      double totalLeftFootForceUnfiltered = processedSensors.getTotalFootForce(RobotSide.LEFT);
      leftFootForceFiltered.update(totalLeftFootForceUnfiltered);

      if (leftFootForceFiltered.getDoubleValue() > 5.0)
      {
         leftCopPercentFromHeel.set((processedSensors.getGroundContactPointForces(RobotSide.LEFT, ContactPointName.TOE_IN).getZ()
                                       + processedSensors.getGroundContactPointForces(RobotSide.LEFT,
                                          ContactPointName.TOE_OUT).getZ()) / leftFootForceFiltered.getDoubleValue());
         leftCopPercentFromHeel.set(MathTools.clamp(leftCopPercentFromHeel.getDoubleValue(), 0.0, 1.0));
      }
      else
         leftCopPercentFromHeel.set(0.5);

      // Make sure that

      double totalRightFootForceUnfiltered = processedSensors.getTotalFootForce(RobotSide.RIGHT);

      rightFootForceFiltered.update(totalRightFootForceUnfiltered);

      // right_force.val = (1.0 - force_alpha.val) * right_force.val +
      // force_alpha.val * totalRightFootForceUnfiltered;

      if (rightFootForceFiltered.getDoubleValue() > 5.0)
      {
         rightCopPercentFromHeel.set((processedSensors.getGroundContactPointForces(RobotSide.RIGHT, ContactPointName.TOE_IN).getZ()
                                        + processedSensors.getGroundContactPointForces(RobotSide.RIGHT,
                                           ContactPointName.TOE_OUT).getZ()) / rightFootForceFiltered.getDoubleValue());
         rightCopPercentFromHeel.set(MathTools.clamp(rightCopPercentFromHeel.getDoubleValue(), 0.0, 1.0));
      }
      else
         rightCopPercentFromHeel.set(0.5);
   }

   private void defaultActionsOutOfStates()
   {
      ff_left_ankle_pitch.set(act_left_ankle.getDoubleValue() + pas_left_ankle.getDoubleValue());
      ff_right_ankle_pitch.set(act_right_ankle.getDoubleValue() + pas_right_ankle.getDoubleValue());
   }

   double passive_ankle_torques(double pos, double vel)
   {
      if (pos < ankle_limit_set.getDoubleValue())
         return (ankle_limit_gain.getDoubleValue() * (ankle_limit_set.getDoubleValue() - pos) * (ankle_limit_set.getDoubleValue() - pos));
      else
         return (0.0);
   }

   public boolean isSimulationDone()
   {
      return (processedSensors.getTime() > 0.5);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private class SupportState extends State<States>
   {
      protected RobotSide robotSide;
      protected double minusForRightSide = 1.0;

      public SupportState(RobotSide robotSide, States stateEnum)
      {
         super(stateEnum);
         this.robotSide = robotSide;

         if (robotSide == RobotSide.RIGHT)
         {
            minusForRightSide = -1.0;
         }
      }

      public void doAction()
      {
         ankleRollError.get(robotSide).set(-minusForRightSide * flat_ankle_roll.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.ANKLE_ROLL));
         ffAnkleRollSpeed.get(robotSide).set(yd_ankle_gain.getDoubleValue() * (ankleRollError.get(robotSide).getDoubleValue() - yd_gain.getDoubleValue() * bodyVelocityInBodyFrameY.getDoubleValue()));
         ffAnkleRoll.get(robotSide).set(ffAnkleRollSpeed.get(robotSide).getDoubleValue() + ankle_k.getDoubleValue() * ankleRollError.get(robotSide).getDoubleValue()
                                          - ankle_b.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_ROLL));

         /* Use hip to servo pitch, roll, and yaw */
         if (footForceFiltered.get(robotSide).getDoubleValue() > 5.0)
         {
            ffHipPitch.get(robotSide).set(-bodyPitchGain.getDoubleValue() * (desiredBodyPitch.getDoubleValue() - processedSensors.getRobotYawPitchOrRoll(RobotOrientation.PITCH))
                                            + bodyPitchDamping.getDoubleValue() * processedSensors.getRobotYawPitchOrRollVelocity(RobotOrientation.PITCH));

            ff_hip_roll.set((m2Parameters.HIP_OFFSET_Y.value * Math.cos(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL))
                               + minusForRightSide * m2Parameters.BODY_CG_Z.value
                                 * Math.sin(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL))) * ff_z.getDoubleValue());
            ffHipRoll.get(robotSide).set(bodyRollGain.getDoubleValue() * processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL)
                                           + bodyRollDamping.getDoubleValue() * processedSensors.getRobotYawPitchOrRollVelocity(RobotOrientation.ROLL)
                                           + minusForRightSide * ff_hip_roll.getDoubleValue());
            ffHipYaw.get(robotSide).set(bodyYawGain.getDoubleValue() * (processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW) - q_d_yaw.getDoubleValue())
                                          + bodyYawDamping.getDoubleValue() * processedSensors.getRobotYawPitchOrRollVelocity(RobotOrientation.YAW));
         }
         else
         {
            ffHipPitch.get(robotSide).set(0.0);
            ffHipRoll.get(robotSide).set(0.0);
            ffHipYaw.get(robotSide).set(0.0);
         }

         /* Keep knee straight */
         ffKnee.get(robotSide).set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.KNEE))
                                     - knee_damp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.KNEE));

         /* Use ankle to servo speed, position */
         activeAnkle.get(robotSide).set(0.0);

         /* Ankle limit to go on toes and maintain cop */
         passiveAnkle.get(robotSide).set(passive_ankle_torques(processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH),
                 processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH)));


         if (heelForceIsLessThanThreshold(robotSide) && (robotIsMovingForward() && heelIsBehindBody(robotSide)) && footIsRearFoot(robotSide)
                 && (stateMachines.get(robotSide).timeInCurrentState() > min_support_time.getDoubleValue()))
         {
            doneWithState.get(robotSide).set(true);
         }
      }

      public void doTransitionIntoAction()
      {
         doneWithState.get(robotSide).set(false);
      }

      public void doTransitionOutOfAction()
      {
      }
   }


   private boolean heelForceIsLessThanThreshold(RobotSide robotSide)
   {
      return (processedSensors.getGroundContactPointForces(robotSide, ContactPointName.HEEL_IN).getZ()
              + processedSensors.getGroundContactPointForces(robotSide, ContactPointName.HEEL_OUT).getZ() < force_thresh.getDoubleValue());
   }

   private boolean robotIsMovingForward()
   {
      return processedSensors.getRobotBodyVelocityComponentInWorldCoordinates(RobotAxis.X) > 0.0;
   }

   private boolean heelIsBehindBody(RobotSide robotSide)
   {
      return processedSensors.getGroundContactPointsPositions(robotSide, ContactPointName.HEEL_IN).getX()
             < processedSensors.getRobotBodyPositionComponentInWorldCoordinates(RobotAxis.X);
   }

   private boolean footIsRearFoot(RobotSide robotSide)
   {
      return processedSensors.getGroundContactPointsPositions(robotSide, ContactPointName.HEEL_IN).getX()
             < processedSensors.getGroundContactPointsPositions(robotSide.getOppositeSide(), ContactPointName.HEEL_IN).getX();
   }


   private class ToeOffState extends State<States>
   {
      protected RobotSide robotSide;
      protected double minusForRightSide = 1.0;

      public ToeOffState(RobotSide robotSide, States stateEnum)
      {
         super(stateEnum);
         this.robotSide = robotSide;

         if (robotSide == RobotSide.RIGHT)
         {
            minusForRightSide = -1.0;
         }
      }

      public void doAction()
      {
         ankleRollError.get(robotSide).set(-minusForRightSide * flat_ankle_roll.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.ANKLE_ROLL));

         /* Decay roll instead of setting straight to zero */
         ffAnkleRollSpeed.get(robotSide).set((1.0 - decay_freq.getDoubleValue() * control_dt.getDoubleValue()) * ffAnkleRollSpeed.get(robotSide).getDoubleValue());
         ffAnkleRoll.get(robotSide).set(ffAnkleRollSpeed.get(robotSide).getDoubleValue() + ankle_k.getDoubleValue() * ankleRollError.get(robotSide).getDoubleValue()
                                          - ankle_b.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_ROLL));

         ff_hip_roll.set((m2Parameters.HIP_OFFSET_Y.value * Math.cos(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL))
                            + minusForRightSide * m2Parameters.BODY_CG_Z.value
                              * Math.sin(processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL))) * ff_z.getDoubleValue());
         ffHipRoll.get(robotSide).set(bodyRollGain.getDoubleValue() * processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL)
                                        + bodyRollDamping.getDoubleValue() * processedSensors.getRobotYawPitchOrRollVelocity(RobotOrientation.ROLL)
                                        + minusForRightSide * ff_hip_roll.getDoubleValue());

         if ((processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.TOE_IN) > 0.9)
                 && (processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.TOE_OUT) > 0.9))
            ffHipYaw.get(robotSide).set(bodyYawGain.getDoubleValue() * (processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW) - q_d_yaw.getDoubleValue())
                                          + bodyYawDamping.getDoubleValue() * processedSensors.getRobotYawPitchOrRollVelocity(RobotOrientation.YAW));
         else
         {
            ffHipYaw.get(robotSide).set(0.0);
         }

         /* Use hip to servo pitch */
         ffHipPitch.get(robotSide).set(-bodyPitchGain.getDoubleValue() * (desiredBodyPitch.getDoubleValue() - processedSensors.getRobotYawPitchOrRoll(RobotOrientation.PITCH))
                                         + bodyPitchDamping.getDoubleValue() * processedSensors.getRobotYawPitchOrRollVelocity(RobotOrientation.PITCH));

         /* Keep knee straight */
         ffKnee.get(robotSide).set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.KNEE))
                                     - knee_damp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.KNEE));

         /* Ankle limit to go on toes and maintain cop */
         passiveAnkle.get(robotSide).set(passive_ankle_torques(processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH),
                 processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH)));

         /* Ankle push off */
         activeAnkle.get(robotSide).set(push_gain.getDoubleValue() * (push_set.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH))
                                          - push_damp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH));

         if ((stateMachines.get(robotSide.getOppositeSide()).getCurrentStateEnum() == States.SUPPORT)
                 && (footForceFiltered.get(robotSide).getDoubleValue() < force_thresh.getDoubleValue()))
         {
            doneWithState.get(robotSide).set(true);
         }
      }

      public void doTransitionIntoAction()
      {
         doneWithState.get(robotSide).set(false);
      }

      public void doTransitionOutOfAction()
      {
      }
   }


   private class SwingState extends State<States>
   {
      private final RobotSide robotSide;

      public SwingState(RobotSide robotSide, States stateEnum)
      {
         super(stateEnum);

         this.robotSide = robotSide;
      }

      public void doAction()
      {
         ramp.get(robotSide).set((stateMachines.get(robotSide).timeInCurrentState()) / tran_time.getDoubleValue());
         if (ramp.get(robotSide).getDoubleValue() > 1.0)
            ramp.get(robotSide).set(1.0);

         q_d_hipYaw.get(robotSide).set(-processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW) + lateral_yaw_mul.getDoubleValue() * velocityHeadingInBodyFrame.getDoubleValue());
         k_hipYaw.get(robotSide).set(ramp.get(robotSide).getDoubleValue() * hip_k.getDoubleValue() / 10.0);
         b_hipYaw.get(robotSide).set(ramp.get(robotSide).getDoubleValue() * hip_b.getDoubleValue() / 4.0);

         ffHipYaw.get(robotSide).set(k_hipYaw.get(robotSide).getDoubleValue() * (q_d_hipYaw.get(robotSide).getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.HIP_YAW))
            - b_hipYaw.get(robotSide).getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.HIP_YAW));

         if ((processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.TOE_IN) < 0.1)
                 && (processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.TOE_OUT) < 0.1))
            kHipRoll.get(robotSide).set(ramp.get(robotSide).getDoubleValue() * hip_k.getDoubleValue());
         else
            kHipRoll.get(robotSide).set(0.0);
         bHipRoll.get(robotSide).set(ramp.get(robotSide).getDoubleValue() * hip_b.getDoubleValue());
         ffHipRoll.get(robotSide)
            .set(kHipRoll.get(robotSide).getDoubleValue()
               * (-processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL) - processedSensors.getJointPosition(robotSide, JointName.HIP_ROLL)) - bHipRoll
                  .get(robotSide).getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.HIP_ROLL));

         kAnkleRoll.get(robotSide).set(ramp.get(robotSide).getDoubleValue() * ankle_k.getDoubleValue());
         bAnkleRoll.get(robotSide).set(ramp.get(robotSide).getDoubleValue() * ankle_b.getDoubleValue());
         ffAnkleRoll.get(robotSide).set(kAnkleRoll.get(robotSide).getDoubleValue() * (0.0 - processedSensors.getJointPosition(robotSide, JointName.ANKLE_ROLL))
                                          - bAnkleRoll.get(robotSide).getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_ROLL));

         /* Servo hip up */
         q_d_hipPitch.get(robotSide).set(-hipSwingAngle.getDoubleValue() - processedSensors.getRobotYawPitchOrRoll(RobotOrientation.PITCH));
         ffHipPitch.get(robotSide).set(hipSwingGain.getDoubleValue() * (q_d_hipPitch.get(robotSide).getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.HIP_PITCH))
            - hipSwingDamp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.HIP_PITCH));
         if (ffHipPitch.get(robotSide).getDoubleValue() > max_hip_torque.getDoubleValue())
            ffHipPitch.get(robotSide).set(max_hip_torque.getDoubleValue());
         if (ffHipPitch.get(robotSide).getDoubleValue() < -max_hip_torque.getDoubleValue())
            ffHipPitch.get(robotSide).set(-max_hip_torque.getDoubleValue());

         /* Damp the knee */
         ffKnee.get(robotSide).set(-swing_damp_knee1.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.KNEE));

         /* Continue toe off until toe leaves ground */
         if (processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.TOE_IN) > 0.1)
         {
            passiveAnkle.get(robotSide).set(passive_ankle_torques(processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH),
                    processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH)));
            activeAnkle.get(robotSide).set(push_gain.getDoubleValue() * (push_set.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH))
                                             - push_damp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH));
         }
         else
         {
            /* Servo ankle level to the ground */
            heel.get(robotSide).set(-processedSensors.getRobotYawPitchOrRoll(RobotOrientation.PITCH)
                                      - processedSensors.getJointPosition(robotSide, JointName.HIP_PITCH)
                                      - processedSensors.getJointPosition(robotSide, JointName.KNEE)
                                      - processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH));
            activeAnkle.get(robotSide).set(-ankle_gain.getDoubleValue() * (-ankle_d.getDoubleValue() - heel.get(robotSide).getDoubleValue())
                                             - ankle_damp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH));

            /*
             * passiveAnkle.get(robotSide).set(* passive_ankle_torques(pS.getLimbJointPosition
             * (robotSide, LimbJointName.ANKLE_PITCH),
             * pS.getLimbJointVelocity(robotSide,
             * LimbJointName.ANKLE_PITCH)));
             */
            passiveAnkle.get(robotSide).set(0.0);
         }

         ffHipPitch.get(robotSide.getOppositeSide()).set(ffHipPitch.get(robotSide.getOppositeSide()).getDoubleValue() - ffHipPitch.get(robotSide).getDoubleValue());

         if (stateMachines.get(robotSide).timeInCurrentState() > swing_time.getDoubleValue())
         {
            doneWithState.get(robotSide).set(true);
         }

      }

      public void doTransitionIntoAction()
      {
         doneWithState.get(robotSide).set(false);
      }

      public void doTransitionOutOfAction()
      {
      }

   }


   private class StraightenState extends State<States>
   {
      private final RobotSide robotSide;
      protected double minusForRightSide = 1.0;

      public StraightenState(RobotSide robotSide, States stateEnum)
      {
         super(stateEnum);

         this.robotSide = robotSide;
         if (robotSide == RobotSide.RIGHT)
            minusForRightSide = -1.0;
      }

      public void doAction()
      {
         q_d_hipYaw.get(robotSide).set(-processedSensors.getRobotYawPitchOrRoll(RobotOrientation.YAW) + lateral_yaw_mul.getDoubleValue() * velocityHeadingInBodyFrame.getDoubleValue());
         ffHipYaw.get(robotSide).set(hip_k.getDoubleValue() / 10.0 * (q_d_hipYaw.get(robotSide).getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.HIP_YAW))
                                       - hip_b.getDoubleValue() / 4.0 * processedSensors.getJointVelocity(robotSide, JointName.HIP_YAW));

         q_d_hipRoll.get(robotSide).set(-processedSensors.getRobotYawPitchOrRoll(RobotOrientation.ROLL) + minusForRightSide * swing_roll_off.getDoubleValue());
         q_d_hipRoll.get(robotSide).set(q_d_hipRoll.get(robotSide).getDoubleValue() + yd_gain.getDoubleValue() * bodyVelocityInBodyFrameY.getDoubleValue()
                                           + minusForRightSide * ceny_gain.getDoubleValue() * bodyCenterToHeelInWorldY.get(robotSide.getOppositeSide()).getDoubleValue());    // rightHeelToBodyCenterInWorldY.val;
         ffHipRoll.get(robotSide).set(hip_k.getDoubleValue() * (q_d_hipRoll.get(robotSide).getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.HIP_ROLL))
                                        - hip_b.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.HIP_ROLL));

         ffAnkleRoll.get(robotSide).set(ankle_k.getDoubleValue() * (0.0 - processedSensors.getJointPosition(robotSide, JointName.ANKLE_ROLL))
                                          - ankle_b.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_ROLL));

         /* Servo hip to a more shallow angle */
         q_d_hipPitch.get(robotSide).set(-hip_hold.getDoubleValue() - processedSensors.getRobotYawPitchOrRoll(RobotOrientation.PITCH));
         ffHipPitch.get(robotSide).set(hipSwingGain.getDoubleValue() * (q_d_hipPitch.get(robotSide).getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.HIP_PITCH))
            - hipSwingDamp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.HIP_PITCH));

         /* Keep knee straight */
         ffKnee.get(robotSide).set(swing_gain_knee.getDoubleValue() * (knee_d.getDoubleValue() - processedSensors.getJointPosition(robotSide, JointName.KNEE))
                                     - swing_damp_knee.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.KNEE));

         /*
          * Couple knee torque to hip so that leg doesn't go swinging
          * forward when knee is stopped...
          */
         ffHipPitch.get(robotSide).set(ffHipPitch.get(robotSide).getDoubleValue() + ffKnee.get(robotSide).getDoubleValue());


         /* Servo ankle level to the ground */
         heel.get(robotSide).set(-processedSensors.getRobotYawPitchOrRoll(RobotOrientation.PITCH)
                                   - processedSensors.getJointPosition(robotSide, JointName.HIP_PITCH)
                                   - processedSensors.getJointPosition(robotSide, JointName.KNEE)
                                   - processedSensors.getJointPosition(robotSide, JointName.ANKLE_PITCH));
         activeAnkle.get(robotSide).set(-ankle_gain.getDoubleValue() * (-ankle_d.getDoubleValue() - heel.get(robotSide).getDoubleValue())
                                          - ankle_damp.getDoubleValue() * processedSensors.getJointVelocity(robotSide, JointName.ANKLE_PITCH));
         passiveAnkle.get(robotSide).set(0.0);

         // if (((processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.TOE_IN) == 1.0) ||
         // (processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.HEEL_IN) == 1.0)) &&
         // (processedSensors.getGroundContactPointsPositions(robotSide, ContactPointName.HEEL_IN).getX() >
         // processedSensors.getGroundContactPointsPositions(RobotSide.RIGHT, ContactPointName.HEEL_IN).getX()))

         if (((processedSensors.getGroundContactPointFootSwitch(robotSide, ContactPointName.HEEL_IN) > 0.5))
                 && (processedSensors.getGroundContactPointsPositions(robotSide, ContactPointName.HEEL_IN).getX()
                     > processedSensors.getGroundContactPointsPositions(robotSide.getOppositeSide(), ContactPointName.HEEL_IN).getX()))
         {
            doneWithState.get(robotSide).set(true);
         }

      }

      public void doTransitionIntoAction()
      {
         doneWithState.get(robotSide).set(false);
      }

      public void doTransitionOutOfAction()
      {
      }

   }
   
   public String getName()
   {
      return this.name;
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }


}
