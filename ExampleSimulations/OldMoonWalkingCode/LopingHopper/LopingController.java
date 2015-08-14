package us.ihmc.moonwalking.models.LopingHopper;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public class LopingController implements RobotController
{
	private final YoVariableRegistry registry = new YoVariableRegistry("controller");

	private enum States
	{
		SUPPORT, PUSH_OFF, SWING
	};

	private StateMachine backStateMachine, frontStateMachine;
	private final YoVariable swing_time = new YoVariable("swing_time", registry);
	private final YoVariable leg_gain = new YoVariable("k_leg", registry);
	private final YoVariable leg_damp = new YoVariable("b_leg", registry);

	private final YoVariable hip_gain = new YoVariable("k_hip", registry);
	private final YoVariable hip_damp = new YoVariable("b_hip", registry);

	private final YoVariable q_d_frontHipJoint = new YoVariable("q_d_frontHipJoint", registry);
	private final YoVariable q_d_backHipJoint = new YoVariable("q_d_backHipJoint", registry);

	private final YoVariable q_d_frontLegJoint = new YoVariable("q_d_frontLegJoint", registry);
	private final YoVariable q_d_backLegJoint = new YoVariable("q_d_backLegJoint", registry);

	private final YoVariable qd_d_frontHipJoint = new YoVariable("qd_d_frontHipJoint", registry);
	private final YoVariable qd_d_backHipJoint = new YoVariable("qd_d_backHipJoint", registry);

	private final YoVariable qd_d_frontLegJoint = new YoVariable("qd_d_frontLegJoint", registry);
	private final YoVariable qd_d_backLegJoint = new YoVariable("qd_d_backLegJoint", registry);

	private YoVariable t;


	// private YoVariable act_backLeg = new YoVariable("act_backLeg", registry);
	// private YoVariable act_frontLeg = new YoVariable("act_frontLeg",
	// registry);

	// private YoVariable act_backHip = new YoVariable("act_backHip", registry);
	// private YoVariable act_frontHip = new YoVariable("act_frontHip",
	// registry);

	private YoVariable frontLegLengthExtend = new YoVariable("frontLegLengthExtend", registry);
	private YoVariable frontLegLengthRetract = new YoVariable("frontLegLengthRetract", registry);

	private YoVariable backLegLengthExtend = new YoVariable("backLegLengthExtend", registry);
	private YoVariable backLegLengthRetract = new YoVariable("backLegLengthRetract", registry);

	private YoVariable frontHipForword = new YoVariable("frontHipForword", registry);
	private YoVariable frontHipBackword = new YoVariable("frontHipBackword", registry);

	private YoVariable backHipForword = new YoVariable("backHipForword", registry);
	private YoVariable backHipBackword = new YoVariable("backHipBackword", registry);

	private YoVariable min_support_time = new YoVariable("min_support_time", registry);

	private YoVariable errorFrontLeg = new YoVariable("errorFrontLeg", registry);
	private YoVariable errorFrontLegD = new YoVariable("errorFrontLegD", registry);

	private YoVariable errorBackLeg = new YoVariable("errorBackLeg", registry);
	private YoVariable errorBackLegD = new YoVariable("errorBackLegD", registry);

	private YoVariable errorFrontHip = new YoVariable("errorFrontHip", registry);
	private YoVariable errorFrontHipD = new YoVariable("errorFrontHipD", registry);

	private YoVariable errorBackHip = new YoVariable("errorBackHip", registry);
	private YoVariable errorBackHipD = new YoVariable("errorBackHipD", registry);

	private final LopingHopperRobot robot;
	private double gravity;

	LopingController(LopingHopperRobot robot, double gravity)
	{
		this.robot = robot;
		this.gravity = gravity;
		t = robot.getVariable("t");

	
		swing_time.val = 0.418039;
		leg_gain.val = 2000.0;
		leg_damp.val = 300.0;

		hip_gain.val = 2000.0;
		hip_damp.val = 300.0;

		q_d_frontHipJoint.val = -0.2;
		q_d_backHipJoint.val = 0.2;
		q_d_frontLegJoint.val = 1.14;
		q_d_backLegJoint.val = 1.14;

		frontHipForword.val = -0.3;
		frontHipBackword.val = -0.2;

		backHipForword.val = 0.3;
		backHipBackword.val = 0.4;

		frontLegLengthExtend.val = -0.0;
		frontLegLengthRetract.val = 0.2;

		backLegLengthExtend.val = -0.05;
		backLegLengthRetract.val = 0.2;
		min_support_time.val = 0.2;
		setupStateMachines();
	}

	private double servoFrontLegLength(double desiredLegLength, double desiredLegLengthDot)
	{
		errorFrontLeg.val = desiredLegLength - robot.getFrontLegLength();

		errorFrontLegD.val = desiredLegLengthDot - robot.getFrontLegLengthDot();

		double tau = leg_gain.val * (errorFrontLeg.val) + leg_damp.val * (errorFrontLegD.val);
		return tau;
	}

	private double servoBackLegLength(double desiredLegLength, double desiredLegLengthDot)
	{
		errorBackLeg.val = desiredLegLength - robot.getBackLegLength();

		errorBackLegD.val = desiredLegLengthDot - robot.getBackLegLengthDot();

		double tau = leg_gain.val * (errorBackLeg.val) + leg_damp.val * (errorBackLegD.val);
		return tau;
	}

	private double servoFrontHipAngle(double desiredHipAngle, double desiredHipAngleDot)
	{
		errorFrontHip.val = desiredHipAngle - robot.getFrontHipAngle();

		errorFrontHipD.val = desiredHipAngleDot - robot.getFrontHipAngleDot();

		double tau = hip_gain.val * (errorFrontHip.val) + hip_damp.val * (errorFrontHipD.val);
		return tau;
	}

	

	private double servoBackHipAngle(double desiredHipAngle, double desiredHipAngleDot)
	{
		errorBackHip.val = desiredHipAngle - robot.getBackHipAngle();

		errorBackHipD.val = desiredHipAngleDot - robot.getBackHipAngleDot();

		double tau = hip_gain.val * (errorBackHip.val) + hip_damp.val * (errorBackHipD.val);
		return tau;
	}

	public void doControl()
	{
		balistic_walking_state_machine();
	}

	private void setupStateMachines()
	{
		// States and Actions:

		State backSupportState = new State(States.SUPPORT)
		{
			public void doAction()
			{
				q_d_backLegJoint.val = backLegLengthRetract.val;
				qd_d_backLegJoint.val = 0.0;
				// act_backLeg.val = leg_gain.val * (q_d_backLegJoint.val -
				// robot.getBackLegLength()) - leg_damp.val *
				// robot.getBackLegLengthDot();

				q_d_backHipJoint.val = backHipBackword.val + robot.getBodyPitch();
				qd_d_backHipJoint.val = 0.0;
				// act_backHip.val = hip_gain.val * (q_d_backHipJoint.val -
				// robot.getBackHipAngle()) - hip_damp.val *
				// robot.getBackHipAngleDot();
			}

			public void doTransitionIntoAction()
			{
			}

			public void doTransitionOutOfAction()
			{
			}
		};

		State backPushOffState = new State(States.PUSH_OFF)
		{
			public void doAction()
			{
				q_d_backLegJoint.val = backLegLengthExtend.val;
				qd_d_backLegJoint.val = 0.0;
				// act_backLeg.val = leg_gain.val * (q_d_backLegJoint.val -
				// robot.getBackLegLength()) - leg_damp.val *
				// robot.getBackLegLengthDot();

				q_d_backHipJoint.val = backHipBackword.val + robot.getBodyPitch();
				qd_d_backHipJoint.val = 0.0;
				// act_backHip.val = hip_gain.val * (q_d_backHipJoint.val -
				// robot.getBackHipAngle()) - hip_damp.val *
				// robot.getBackHipAngleDot();
				// act_backHip.val = -hip_gain.val * (q_d_backHip.val -
				// robot.q_pitch.val) + hip_damp.val * qd_pitch.val;
			}

			public void doTransitionIntoAction()
			{
			}

			public void doTransitionOutOfAction()
			{
			}
		};

		State backSwingState = new State(States.SWING)
		{
			public void doAction()
			{
				q_d_backLegJoint.val = backLegLengthRetract.val;
				qd_d_backLegJoint.val = 0.0;
				// act_backLeg.val = leg_gain.val / 10 * (q_d_backLegJoint.val -
				// robot.getBackLegLength()) - leg_damp.val *
				// robot.getBackLegLengthDot();

				q_d_backHipJoint.val = backHipForword.val - robot.getBodyPitch();// backHipBackword.val;
				qd_d_backHipJoint.val = 0.0;
				// act_backHip.val = hip_gain.val * (q_d_backHipJoint.val -
				// robot.getBackHipAngle()) - hip_damp.val *
				// robot.getBackHipAngleDot();

			}

			public void doTransitionIntoAction()
			{
			}

			public void doTransitionOutOfAction()
			{
			}
		};

		State frontSupportState = new State(States.SUPPORT)
		{
			public void doAction()
			{
				q_d_frontLegJoint.val = frontLegLengthRetract.val;
				qd_d_frontLegJoint.val = 0.0;
				// act_frontLeg.val = leg_gain.val * (q_d_frontLegJoint.val -
				// robot.getFrontLegLength()) - leg_damp.val *
				// robot.getFrontLegLengthDot();

				q_d_frontHipJoint.val = frontHipBackword.val + robot.getBodyPitch();
				qd_d_frontHipJoint.val = 0.0;
				// act_frontHip.val = hip_gain.val * (q_d_frontHipJoint.val -
				// robot.getFrontHipAngle()) - hip_damp.val *
				// robot.getFrontHipAngleDot();
				// act_frontHip.val = -hip_gain.val * (q_d_frontHip.val -
				// robot.q_pitch.val) + hip_damp.val * qd_pitch.val;
			}

			public void doTransitionIntoAction()
			{
			}

			public void doTransitionOutOfAction()
			{
			}
		};

		State frontPushOffState = new State(States.PUSH_OFF)
		{
			public void doAction()
			{
				q_d_frontLegJoint.val = frontLegLengthExtend.val;
				qd_d_frontLegJoint.val = 0.0;
				// act_frontLeg.val = leg_gain.val * (q_d_frontLegJoint.val -
				// robot.getFrontLegLength()) - leg_damp.val *
				// robot.getFrontLegLengthDot();

				q_d_frontHipJoint.val = frontHipBackword.val + robot.getBodyPitch();
				qd_d_frontHipJoint.val = 0.0;
				// act_frontHip.val = hip_gain.val * (q_d_frontHipJoint.val -
				// robot.getFrontHipAngle()) - hip_damp.val *
				// robot.getFrontHipAngleDot();
				// act_frontHip.val = -hip_gain.val * (q_d_frontHip.val -
				// robot.q_pitch.val) + hip_damp.val * qd_pitch.val;
			}

			public void doTransitionIntoAction()
			{
			}

			public void doTransitionOutOfAction()
			{
			}
		};

		State frontSwingState = new State(States.SWING)
		{
			public void doAction()
			{
				q_d_frontLegJoint.val = frontLegLengthRetract.val;
				qd_d_frontLegJoint.val = 0.0;
				// act_frontLeg.val = leg_gain.val / 10 * (q_d_frontLegJoint.val
				// - robot.getFrontLegLength()) - leg_damp.val *
				// robot.getFrontLegLengthDot();

				q_d_frontHipJoint.val = frontHipForword.val - robot.getBodyPitch();// frontHipBackword.val;
				qd_d_frontHipJoint.val = 0.0;
				// act_frontHip.val = hip_gain.val * (q_d_frontHipJoint.val -
				// robot.getFrontHipAngle()) - hip_damp.val *
				// robot.getFrontHipAngleDot();
			}

			public void doTransitionIntoAction()
			{
			}

			public void doTransitionOutOfAction()
			{
			}
		};

		// Create the state machines:
		backStateMachine = new StateMachine("back_state", "back_switch_time", States.values(), t, registry);
		frontStateMachine = new StateMachine("front_state", "front_switch_time", States.values(), t, registry);

		// Transition Conditions:

		StateTransitionCondition backFootUnloaded = new BackFootUnloadedCondition();
		StateTransitionCondition backFootTouchedDown = new BackFootTouchedDownCondition();

		StateTransitionCondition frontFootUnloaded = new FrontFootUnloadedCondition();
		StateTransitionCondition frontFootTouchedDown = new FrontFootTouchedDownCondition();

		// Back State Transitions:

		StateTransition backSupportToPushOff = new StateTransition(States.PUSH_OFF, backFootTouchedDown);
		backSupportToPushOff.addTimePassedCondition(min_support_time);
		backSupportState.addStateTransition(backSupportToPushOff);

		StateTransition backPushOffToSwing = new StateTransition(States.SWING, backFootUnloaded);
		backPushOffState.addStateTransition(backPushOffToSwing);

		StateTransition backSwingToSupport = new StateTransition(States.SUPPORT, backFootTouchedDown);
		backSwingState.addStateTransition(backSwingToSupport);

		// Front State Transitions:
		StateTransition frontSupportToPushOff = new StateTransition(States.PUSH_OFF, frontFootTouchedDown);
		frontSupportToPushOff.addTimePassedCondition(min_support_time);
		frontSupportState.addStateTransition(frontSupportToPushOff);

		StateTransition frontPushOffToSwing = new StateTransition(States.SWING, frontFootUnloaded);
		// frontPushOffToSwing.addTimePassedCondition(min_support_time);
		frontPushOffState.addStateTransition(frontPushOffToSwing);

		StateTransition frontSwingToSupport = new StateTransition(States.SUPPORT, frontFootTouchedDown);
		frontSwingState.addStateTransition(frontSwingToSupport);

		// Assemble the Back State Machine:
		backStateMachine.addState(backSupportState);
		backStateMachine.addState(backPushOffState);
		backStateMachine.addState(backSwingState);

		// Assemble the Front State Machine:
		frontStateMachine.addState(frontSupportState);
		frontStateMachine.addState(frontPushOffState);
		frontStateMachine.addState(frontSwingState);

		// Set the Initial States:

		backStateMachine.setCurrentState(States.SWING);
		frontStateMachine.setCurrentState(States.SWING);
	}

	private void balistic_walking_state_machine()
	{

		// Actions in Each State

		backStateMachine.doAction();
		frontStateMachine.doAction();

		// Communication between legs for now...

		// Transition Conditions:

		backStateMachine.checkTransitionConditions();
		frontStateMachine.checkTransitionConditions();

		// Torques at the joints:
		
		

		

		double servoFrontLegTorque = servoFrontLegLength(q_d_frontLegJoint.val, qd_d_frontLegJoint.val);// +
		// getGravityCompensationTorque(robot.getFrontHipAngle());
		double servoBackLegTorque = servoBackLegLength(q_d_backLegJoint.val, qd_d_backLegJoint.val);// +
		// getGravityCompensationTorque(robot.getBackHipAngle());

		double servoFrontHipTorque = servoFrontHipAngle(q_d_frontHipJoint.val, qd_d_frontHipJoint.val);
		double servoBackHipTorque = servoBackHipAngle(q_d_backHipJoint.val, qd_d_backHipJoint.val);

		robot.setFrontLegTorque(servoFrontLegTorque);
		robot.setBackLegTorque(servoBackLegTorque);

		robot.setFrontHipTorque(servoFrontHipTorque);
		robot.setBackHipTorque(servoBackHipTorque);

	}

	public class BackFootUnloadedCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			return (!robot.isBackFootOnGround());
		}
	}

	public class FrontFootUnloadedCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			return (!robot.isFrontFootOnGround());
		}
	}

	public class BackFootTouchedDownCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			return (robot.isBackFootOnGround());
		}

	}

	public class FrontFootTouchedDownCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			return (robot.isFrontFootOnGround());
		}
	}

	public class LeanCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			return (robot.getBodyPitch() > -0.2);
		}
	}

	public YoVariableRegistry getYoVariableRegistry()
	{
		return registry;
	}
}
