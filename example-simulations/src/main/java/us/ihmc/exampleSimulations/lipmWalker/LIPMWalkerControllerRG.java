// GMN: TODO, DONE, Notes, answered & unanswered Q's:
//
// Questions:
// =========
// Q: What about CoM height servo, instead of body-height?
// Q: What would it mean to have a clock-less controller?
// Q: Try some AM compensation or Natural Posture control?
// Q: [FOR JERRY] How to make larger time window for interactive sim playback?
//    Q: [FOR JERRY] How to save above as reloaded config?
//
// TODO:
// ====
// + ELIMINATE DUPLICATE CODE BTW LIFT & SWING STATES
// + I have some controller state that is not rewindable - need to fix
//   + YoVariables
//   + check out YoMatrix
// + Don't start var names with caps
// + Make human-readable var names
//
// + Check-in code
// + IMPROVE LOGGING SO CAN DEBUG BETTER
//   + Q: How well hitting desired TD?
//   + Q: How well hitting prescribed timing?
// + General cleanup
// + Predictive TDLO
//   + Add pole-placement to TDLO
//   + Get closer to desired pole-placement step dynamics
// + Ground-speed matching at TD
// + Tune-up swing tracking, both joints and TD (e.g. ground-speed matching)
//   + Double-check swing IK control
// + Get away from z-cubic-spline for SWING
// + Learn how to use Yo-variables
// + More learning of SCS plotting/video tools
// + Can we fix .getCurrentState() for these state machines?
//   + (get rid of inLiftState flag)
//
// DONE:
// ====
// + Access to matrices? Examples? 
// + Need Jacobians
// + Need to figure out how state machine works
// + Body-height servo
// + Body-pitch servo
// + Swing IK
// + TDLO implementation
// + Replace "robotSide" with just "side" everywhere

package us.ihmc.exampleSimulations.lipmWalker;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerControllerRG implements RobotController
{
   private final LIPMWalkerRobot robot;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private enum States
   {
      // GMN: DOUBLE-SUPPORT ???
      //      A: "LIFT" is effectively double-support...
      SUPPORT, LIFT, SWING;
   }

   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final double controlDT;
   private final double g = 9.81;

   private final YoDouble desiredBodyHeight = new YoDouble("desiredBodyHeight", registry);
   private final YoDouble stepDuration = new YoDouble("stepDuration", registry);
   private final YoDouble omega = new YoDouble("omega", registry);
   private final YoDouble tdloPole1 = new YoDouble("tdloPole1", registry);
   private final YoDouble tdloPole2 = new YoDouble("tdloPole2", registry);
   private final YoDouble tdloStiffness = new YoDouble("tdloStiffness", registry);
   private final YoDouble tdloDamping = new YoDouble("tdloDamping", registry);

   private final YoFramePoint3D stancePositionAtStartOfSupport;  // GMN: Need to restructure this...

   private final YoDouble swingHeight = new YoDouble("swingHeight", registry);
   private final YoDouble midSwingFraction = new YoDouble("midSwingFraction", registry);
   private final YoDouble touchdownVelocity = new YoDouble("touchdownVelocity", registry);
   private final YoDouble touchdownHeight = new YoDouble("touchdownHeight", registry);

   private final YoDouble heightServoStiffness = new YoDouble("heightServoStiffness", registry);
   private final YoDouble heightServoDamping = new YoDouble("heightServoDamping", registry);
   private final YoDouble feedForwardHeightForce = new YoDouble("feedForwardHeightForce", registry);
   private final YoDouble feedBackHeightForce = new YoDouble("feedBackHeightForce", registry);
   private final PDController heightServoController = new PDController(heightServoStiffness, heightServoDamping, "heightController", registry);

   private final YoDouble timeInSwing = new YoDouble("timeInSwing", registry);
   private final YoDouble desiredFootHeight = new YoDouble("desiredFootHeight", registry);
   private final YoDouble desiredFootZVelocity = new YoDouble("desiredFootZVelocity", registry);

   private final YoDouble bodyServoStiffness = new YoDouble("bodyServoStiffness", registry);
   private final YoDouble bodyServoDamping = new YoDouble("bodyServoDamping", registry);
   private final YoDouble feedBackBodyTorque = new YoDouble("feedBackBodyTorque", registry);
   private final PDController bodyServoController = new PDController(bodyServoStiffness, bodyServoDamping, "bodyController", registry);

   private final YoDouble swingJointStiffness = new YoDouble("swingJointStiffness", registry);
   private final YoDouble swingJointDamping = new YoDouble("swingJointDamping", registry);
   private final PDController swingHipController = new PDController(swingJointStiffness, swingJointDamping, "swingHipController", registry);
   private final PDController swingKneeController = new PDController(swingJointStiffness, swingJointDamping, "swingKneeController", registry);

   private final YoDouble netHeightForce = new YoDouble("netHeightForce", registry);
   private final YoDouble netHorizontalForce = new YoDouble("netHorizontalForce", registry);
   private final YoDouble netBodyTorque = new YoDouble("netBodyTorque", registry);

   private final SideDependentList<YoDouble> desiredSwingFootPositions = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSwingFootVelocities = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSwingHipAngles = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSwingHipVelocities = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSwingKneeAngles = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSwingKneeVelocities = new SideDependentList<>();

   private final SideDependentList<YoDouble> timeInStanceState = new SideDependentList<>();

   private final SideDependentList<Polynomial> swingLegLengthTrajectories = new SideDependentList<>();

   // For logging & debugging:
   private final SideDependentList<YoDouble> predictedTouchdownLocation = new SideDependentList<>();
   private final LIPMWalkerStateEstimator stateEstimator;

   public LIPMWalkerControllerRG(LIPMWalkerRobot robot, double controlDT)
   {
      this.controlDT = controlDT;
      this.robot = robot;

      stateEstimator = new LIPMWalkerStateEstimator(robot);

      desiredBodyHeight.set(0.8);
      stepDuration.set(0.65);
      omega.set(Math.sqrt(g / desiredBodyHeight.getValue()));
      tdloPole1.set(0.08);
      tdloPole2.set(0.08);

      desiredBodyHeight.addListener((v) -> omega.set(Math.sqrt(g / desiredBodyHeight.getValue())));
      omega.addListener((v) -> updateTDLOGains());
      stepDuration.addListener((v) -> updateTDLOGains());
      tdloPole1.addListener((v) -> updateTDLOGains());
      tdloPole2.addListener((v) -> updateTDLOGains());

      stancePositionAtStartOfSupport = new YoFramePoint3D("stancePositionAtStartOfSupport", stateEstimator.getCoMReferenceFrame(), registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         desiredSwingFootPositions.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredFootPosition", registry));
         desiredSwingFootVelocities.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredFootVelocity", registry));

         desiredSwingHipAngles.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredSwingHipAngle", registry));
         desiredSwingKneeAngles.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredSwingKneeAngle", registry));
         desiredSwingHipVelocities.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredSwingHipVelocities", registry));
         desiredSwingKneeVelocities.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredSwingKneeVelocities", registry));

         predictedTouchdownLocation.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "PredictedTouchdownLocation", registry));

         timeInStanceState.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "TimeInStanceState", registry));

         swingLegLengthTrajectories.put(robotSide, new Polynomial(4));
      }

      heightServoStiffness.set(500.0);
      heightServoDamping.set(50.0);

      bodyServoStiffness.set(100.0);
      bodyServoDamping.set(10.0);

      swingJointStiffness.set(800.0);
      swingJointDamping.set(80.0);

      // swing parameters
      swingHeight.set(0.12);
      midSwingFraction.set(0.5);
      touchdownHeight.set(0.0);
      touchdownVelocity.set(-1.0);

      initialize();

      stateMachines = setupStateMachines();
   }

   @Override
   public void initialize()
   {
      updateTDLOGains();
   }

   private void updateTDLOGains()
   {
      //      // Dead-beat TDLO: (all discrete poles = 0)
      //      kx_TDLO = (1+Math.exp(2*LIPTipFreq*stepTime))/(2*Math.pow(-1+Math.exp(LIPTipFreq*stepTime),2));
      //      bx_TDLO = (1+Math.exp(2*LIPTipFreq*stepTime))/(2*Math.pow( 1+Math.exp(LIPTipFreq*stepTime),2));

      // Pole-placement:   GMN - Can't handle imaginary part yet!
      double p1 = tdloPole1.getDoubleValue();
      double p2 = tdloPole2.getDoubleValue();
      double k = omega.getDoubleValue();
      double Ts = stepDuration.getDoubleValue();
      tdloStiffness.set((1 + Math.exp(2 * k * Ts) + 2 * Math.exp(k * Ts) * (-p1 - p2 + p1 * p2)) / (2 * Math.pow(-1 + Math.exp(k * Ts), 2)));
      tdloDamping.set((1 + Math.exp(2 * k * Ts) - 2 * Math.exp(k * Ts) * (p1 + p2 + p1 * p2)) / (2 * Math.pow(1 + Math.exp(k * Ts), 2)));
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      // Basic TDLO list:
      // + Need body servo to:
      //   + control height (CoM? Body?)
      //   + control body pitch
      // + Implicit force control in stance leg: tau = J-transpose * F
      // + Need analytical or diff IK to generate joint angles for swing leg
      // + Need task-space traj generation for swing leg towards some desired TD and at desired time
      // + Joint servos with desired values
      // + State machine; step on a timer

      stateEstimator.update();

      for (RobotSide side : RobotSide.values)
      {
         stateMachines.get(side).doActionAndTransition();
      }
   }


   private void controlSwingLeg(RobotSide side, double timeInSwing, double dT)
   {
      double thd_body = robot.getBodyPitchAngularVelocity();

      // Dead-reckoning on foot x pos to desired:
      double dt_denom = Math.max((stepDuration.getDoubleValue() - 0.1) - timeInSwing, 0.001); // GMN: Added 0.1 sec arrive early...
      //      double dt_denom = stepTime - timeInSwing; // GMN: Added 0.1 sec arrive early...

      desiredSwingFootVelocities.get(side).set(0.0);
      //      if (timeInSwing > 0.05) // GMN: try to prevent dragging while lifting; refactor so this happens in LIFT only; also really needs to be r.t. world
      //      {
      desiredSwingFootVelocities.get(side).set((predictedTouchdownLocation.get(side).getValue() - desiredSwingFootPositions.get(side).getValue()) / dt_denom); // GMN
      //      }
      double desiredSwingFootVelocity = desiredSwingFootVelocities.get(side).getDoubleValue();
      desiredSwingFootPositions.get(side).add(desiredSwingFootVelocity * dT);

      // get zd_d_foot:
      this.timeInSwing.set(timeInSwing);
      swingLegLengthTrajectories.get(side).compute(timeInSwing);
      desiredFootHeight.set(swingLegLengthTrajectories.get(side).getValue());
      desiredFootZVelocity.set(swingLegLengthTrajectories.get(side).getVelocity());

      // Do diffIK; use measured th_body as FF term?
      DMatrixRMaj Jleg = robot.getlegJacobian(robot.getBodyPitchAngle(),  // includes body-pitch
                                              desiredSwingHipAngles.get(side).getDoubleValue(),
                                              desiredSwingKneeAngles.get(side).getDoubleValue());
      DMatrixRMaj Jq = new DMatrixRMaj(2, 2); // just leg joints
      Jq.set(0, 0, Jleg.get(0, 1));
      Jq.set(0, 1, Jleg.get(0, 2));
      Jq.set(1, 0, Jleg.get(1, 1));
      Jq.set(1, 1, Jleg.get(1, 2));
      CommonOps_DDRM.invert(Jq);  // Jq now holds its inverse
      DMatrixRMaj RHS = new DMatrixRMaj(2, 1);
      RHS.set(0, 0, desiredSwingFootVelocity - 0 * Jleg.get(0, 0) * thd_body); // GMN: including body-pitch rate bias
      RHS.set(1, 0, desiredFootZVelocity.getValue() - 0 * Jleg.get(1, 0) * thd_body); // GMN: including body-pitch rate bias
      DMatrixRMaj qd_d = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(Jq, RHS, qd_d); // resolved-rate IK
      desiredSwingHipAngles.get(side).add(dT * qd_d.get(0, 0));
      desiredSwingKneeAngles.get(side).add(dT * qd_d.get(1, 0));
      desiredSwingHipVelocities.get(side).set(qd_d.get(0, 0));
      desiredSwingKneeVelocities.get(side).set(qd_d.get(1, 0));
      //      LRq_d.get(side).set(0,0,LRq_d.get(side).get(0,0)+qd_d.get(0,0)*dT); // Easier way?????
      //      LRq_d.get(side).set(1,0,LRq_d.get(side).get(1,0)+qd_d.get(1,0)*dT); // Easier way?????


      // Servo joints to the desired joint trajs:
      robot.setHipTorque(side,
                         swingHipController.compute(robot.getHipAngle(side),
                                                    desiredSwingHipAngles.get(side).getDoubleValue(),
                                                    robot.getHipVelocity(side),
                                                    desiredSwingHipVelocities.get(side).getDoubleValue()));
      robot.setKneeForce(side,
                         swingKneeController.compute(robot.getKneeLength(side),
                                                     desiredSwingKneeAngles.get(side).getDoubleValue(),
                                                     robot.getKneeVelocity(side),
                                                     desiredSwingKneeVelocities.get(side).getDoubleValue()));
   }

   // Get current foot x pos r.t. body wrt world:
   double getXcop(RobotSide side)
   {
      Point3DReadOnly pos_foot = robot.getFootPosition(side);
      return pos_foot.getX() - robot.getBodyXPosition(); // xcop r.t. body
   }

   double simpleTDLOStepLocation(double xLO)
   {
      // Simple TDLO:
      return (tdloStiffness.getValue() * (stancePositionAtStartOfSupport.getX() - xLO) - tdloDamping.getValue() * (stancePositionAtStartOfSupport.getX() + xLO));
   }

   // predicted-LO TDLO (sort of like "Black Diamond" TDLO)
   double predictedTDLOStepLocation(double t, double xTDk, double xCoP)
   {
      double k = omega.getDoubleValue();
      double Ts = stepDuration.getDoubleValue();
      if (t < 0.2 * Ts)
      {
         t = 0.2 * Ts;
      }

      // See reMarkable notes Feb 23, 2022:
      double cTD = (Math.exp(k * Ts) - Math.exp(2 * k * (t - Ts))) / (1 - Math.exp(2 * k * t));
      double cCoP = Math.exp(k * (t - Ts)) * (Math.exp(2 * k * Ts) - 1) / (Math.exp(2 * k * t) - 1);

      // predicted upcoming LO location:
      double xLOk = cTD * xTDk + cCoP * xCoP;

      // predicted final TDLO TD location r.t. body:
      double xTDk1 = tdloStiffness.getDoubleValue() * (xTDk - xLOk) - tdloDamping.getValue() * (xTDk + xLOk);

      //      // desired TD location: (attempt to get a location ideally fixed in the world)
      //      return (xCoP - xLOk + xTDk1);
      return xTDk1;
   }


   private SideDependentList<StateMachine<States, State>> setupStateMachines()
   {
      // States and Actions:
      StateMachineFactory<States, State> leftFactory = new StateMachineFactory<>(States.class);
      StateMachineFactory<States, State> rightFactory = new StateMachineFactory<>(States.class);

      leftFactory.setNamePrefix("leftState").setRegistry(registry).buildYoClock(robot.getRobot().getYoTime());
      rightFactory.setNamePrefix("rightState").setRegistry(registry).buildYoClock(robot.getRobot().getYoTime());

      // Left State Transitions:
      leftFactory.addTransition(States.SUPPORT, States.LIFT, new OtherHeelOnGroundCondition(RobotSide.LEFT));
      leftFactory.addTransition(States.LIFT, States.SWING, new HeelOffGroundCondition(RobotSide.LEFT));
      leftFactory.addTransition(States.SWING, States.SUPPORT, new HeelOnGroundCondition(RobotSide.LEFT));

      // Right State Transitions:
      rightFactory.addTransition(States.SUPPORT, States.LIFT, new OtherHeelOnGroundCondition(RobotSide.RIGHT));
      rightFactory.addTransition(States.LIFT, States.SWING, new HeelOffGroundCondition(RobotSide.RIGHT));
      rightFactory.addTransition(States.SWING, States.SUPPORT, new HeelOnGroundCondition(RobotSide.RIGHT));

      // Assemble the Left State Machine:
      leftFactory.addState(States.SUPPORT, new SupportState(RobotSide.LEFT));
      leftFactory.addState(States.LIFT, new LiftState(RobotSide.LEFT));
      leftFactory.addState(States.SWING, new SwingState(RobotSide.LEFT));

      // Assemble the Right State Machine:
      rightFactory.addState(States.SUPPORT, new SupportState(RobotSide.RIGHT));
      rightFactory.addState(States.LIFT, new LiftState(RobotSide.RIGHT));
      rightFactory.addState(States.SWING, new SwingState(RobotSide.RIGHT));

      // Create the state machines and set their initial states:
      return new SideDependentList<>(leftFactory.build(States.SUPPORT), rightFactory.build(States.LIFT));
   }

   private class SupportState implements State
   {
      private final RobotSide side;

      public SupportState(RobotSide side)
      {
         this.side = side;
      }

      @Override
      public void onEntry()
      {
         // Record stance position for TDLO:
         stancePositionAtStartOfSupport.setFromReferenceFrame(stateEstimator.getFootFrame(side));
      }

      @Override
      public void doAction(double timeInState)
      {
         // Support Side:
         controlSupportLeg(side);
         timeInStanceState.get(side).set(timeInState);
      }

      @Override
      public void onExit(double timeInState)
      {

      }

      private void controlSupportLeg(RobotSide side)
      {
         // GMN: Would seem we would do body-servo here.
         // Steps?
         // + Do body-servo:  Gives Fx and Fz at foot
         //   + Need measured height (assume body) & veloc
         //   + Need measured pitch & veloc
         // + Do tau = J-transpose * F
         // + Assign leg actuator torques/forces


         // The body-servo:
         feedBackHeightForce.set(heightServoController.compute(robot.getBodyZPosition(), desiredBodyHeight.getDoubleValue(), robot.getBodyZVelocity(), 0.0));
         feedForwardHeightForce.set(0.999 * robot.getMass() * g);
         netHeightForce.set(feedForwardHeightForce.getDoubleValue() + feedBackHeightForce.getDoubleValue()); // GMN

         feedBackBodyTorque.set(bodyServoController.compute(robot.getBodyPitchAngle(), 0.0, robot.getBodyPitchAngularVelocity(), 0.0));
         netBodyTorque.set(feedBackBodyTorque.getDoubleValue()); // GMN

         FramePoint3D foot = new FramePoint3D(stateEstimator.getFootFrame(side));
         foot.changeFrame(stateEstimator.getCoMReferenceFrame());
         double xcop = foot.getX();

         netHorizontalForce.set(-1.0 / robot.getBodyZPosition() * (netHeightForce.getValue() * xcop + netBodyTorque.getDoubleValue()));  // Fx based on body-servo & current foot position

         // Do implicit force control:
         DMatrixRMaj desiredFootForce = new DMatrixRMaj(2, 1);
         desiredFootForce.set(0, 0, -netHorizontalForce.getValue()); // set desired foot-force (applied by foot to ground)
         desiredFootForce.set(1, 0, -netHeightForce.getValue()); // set desired foot-force (applied by foot to ground)
         DMatrixRMaj jacobianToLeg = robot.getlegJacobian(robot.getBodyPitchAngle(),
                                                          robot.getHipAngle(side),
                                                          Math.abs(robot.getKneeLength(side))); // get jacobian (includes body pitch)
         DMatrixRMaj jacobianTranspose = new DMatrixRMaj(3, 2);
         CommonOps_DDRM.transpose(jacobianToLeg, jacobianTranspose);  // transpose the jacobian
         DMatrixRMaj jointTorqueVector = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(jacobianTranspose, desiredFootForce, jointTorqueVector); // tau = J-transpose * F

         //      /* Compute and set orbital energy YoDouble. Based on virtual spring-mass system. */
         //      double orbitalEnergyValue = 0.5 * mass * centerOfMassVelocity.lengthSquared() - 0.5 * g / desiredHeight.getDoubleValue() * centerOfMassPosition.distanceFromOriginSquared();
         //      orbitalEnergy.set(orbitalEnergyValue);

         // Skip 0 element as that is moment on body
         robot.setHipTorque(side, jointTorqueVector.get(1, 0));
         robot.setKneeForce(side, jointTorqueVector.get(2, 0));
      }
   }

   private class LiftState implements State
   {
      private final RobotSide side;

      public LiftState(RobotSide side)
      {
         this.side = side;
      }

      @Override
      public void onEntry()
      {
         // Build the z-spline:
         Point3DReadOnly pos_foot = robot.getFootPosition(side);
         swingLegLengthTrajectories.get(side).setCubicWithIntermediatePositionAndFinalVelocityConstraint(0.0,
                                                                                                         midSwingFraction.getDoubleValue() * stepDuration.getValue(),
                                                                                                         stepDuration.getDoubleValue(),
                                                                                                         pos_foot.getZ(),
                                                                                                         swingHeight.getDoubleValue(),
                                                                                                         touchdownHeight.getDoubleValue(),
                                                                                                         touchdownVelocity.getDoubleValue());

         // Initialize the diff IK:
         desiredSwingHipAngles.get(side).set(robot.getHipAngle(side));
         desiredSwingKneeAngles.get(side).set(robot.getKneeLength(side));
         desiredSwingFootPositions.get(side).set(getXcop(side));
      }

      @Override
      public void doAction(double timeInState)
      {
         double xCoP = getXcop(side.getOppositeSide());
         predictedTouchdownLocation.get(side).set(simpleTDLOStepLocation(xCoP)); // simple TDLO
         //         double desiredTD = predictedTDLOStepLocation(timeInState,xTD,xCoP); // predicted TDLO
         controlSwingLeg(side, timeInState, controlDT);
         //         double stanceTime = timeInStance.get(side.getOppositeSide()).getValue();
         //         controlSwingLeg(side,stanceTime,dT,desiredTD);
      }

      @Override
      public void onExit(double timeInState)
      {
      }
   }

   private class SwingState implements State
   {
      private final RobotSide side;

      public SwingState(RobotSide side)
      {
         this.side = side;
      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {
         double xCoP = getXcop(side.getOppositeSide());
         predictedTouchdownLocation.get(side).set(simpleTDLOStepLocation(xCoP)); // simple TDLO
         //         double desiredTD = predictedTDLOStepLocation(timeInState,xTD,xCoP); // predicted TDLO
         //       controlSwingLeg(side,timeInState,dT,desiredTD);
         double stanceTime = timeInStanceState.get(side.getOppositeSide()).getValue();
         controlSwingLeg(side, stanceTime, controlDT);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   // LIFT -> SWING: The LIFTING foot has cleared the ground:
   private class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide side;

      public HeelOffGroundCondition(RobotSide side)
      {
         this.side = side;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         return robot.getFootPosition(side).getZ() > swingHeight.getDoubleValue() / 4.0;
      }
   }

   // SUPPORT -> LIFT: The current STANCE foot detects that the OTHER foot has hit the ground:
   private class OtherHeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide side;

      public OtherHeelOnGroundCondition(RobotSide side)
      {
         this.side = side;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         // GMN: Is there a way to find & look at the contact state of the OTHER side of the robot???
         // GMN: Make the state transition simply be when the OTHER side is on the ground:
         return ((robot.getFootZForce(this.side.getOppositeSide()) > 10) && (stateMachines.get(side.getOppositeSide()).getCurrentStateKey() != States.LIFT));
      }
   }

   // SWING -> SUPPORT: The current SWING foot hits the ground:
   private class HeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide side;

      public HeelOnGroundCondition(RobotSide side)
      {
         this.side = side;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         return (robot.getFootZForce(this.side) > 10);
      }
   }
}