// GMN: TODO, DONE, Notes, answered & unanswered Q's:
//
// Questions:
// =========
// Q: What about CoM height servo, instead of body-height?
// Q: What would it mean to have a clock-less controller?
// Q: Try some AM compensation or Natural Posture control?
// Q: [FOR JERRY] How to make larger time window for interactive sim playback?
//
// TODO:
// ====
// + I have some controller state that is not rewindable - need to fix
//   + YoVariables
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

 

package us.ihmc.exampleSimulations.lipmWalker;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.util.RobotController;

public class LIPMWalkerControllerGMN implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
//   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
//   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
//   private final YoDouble kpHip = new YoDouble("kpHip", registry);
//   private final YoDouble kdHip = new YoDouble("kdHip", registry);
   
//   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
//   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
//   private final YoDouble q_d_leftHip = new YoDouble("q_d_leftHip", registry);
//   private final YoDouble q_d_rightHip = new YoDouble("q_d_rightHip", registry);

//   private final YoDouble q_rightHipWorld = new YoDouble("q_rightHipWorld", registry);
//   private final YoDouble q_leftHipWorld = new YoDouble("q_leftHipWorld", registry);

//   private final YoDouble comHeight = new YoDouble("comHeight", registry);
//   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
//   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);

   private enum States
   {
      // GMN: DOUBLE-SUPPORT ???
      //      A: "LIFT" is effectively double-support...
      SUPPORT, LIFT, SWING;
   }

   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final double g  = 9.81;
   public double xTD;  // GMN: Need to restructure this...
   private double desiredBodyHeight = 0.8;
   private double stepTime = 0.65;
   private double LIPTipFreq;
   private double kx_TDLO, bx_TDLO;
   private double SwingHeight = 0.12;
   
   private SideDependentList<Double> LRlastTime = new SideDependentList<>(-1.0,-1.0);
   private SideDependentList<DMatrixRMaj> LRSwingCoeffs = new SideDependentList<>(new DMatrixRMaj(3,1), new DMatrixRMaj(3,1));
   private SideDependentList<Double> LRx_d_foot = new SideDependentList<>(0.0,0.0);
   private SideDependentList<DMatrixRMaj> LRq_d = new SideDependentList<>(new DMatrixRMaj(2,1), new DMatrixRMaj(2,1));
   private SideDependentList<Boolean> inLiftState = new SideDependentList<>(false, false); // GMN: work-around for incompatible "State" types, UGH!

   public LIPMWalkerControllerGMN(LIPMWalkerRobot robot)
   {
      this.robot = robot;
      initialize();
      
      stateMachines = setupStateMachines();
   }

   @Override
   public void initialize()
   {
//    kpHip.set(971.25);
//    kdHip.set(80.0);
//      kpKnee.set(1000.0);
//      kdKnee.set(100.0);
      
      LIPTipFreq = Math.sqrt(g/desiredBodyHeight);
      
      // Dead-beat TDLO: (all discrete poles = 0)
      kx_TDLO = (1+Math.exp(2*LIPTipFreq*stepTime))/(2*Math.pow(-1+Math.exp(LIPTipFreq*stepTime),2));
      bx_TDLO = (1+Math.exp(2*LIPTipFreq*stepTime))/(2*Math.pow( 1+Math.exp(LIPTipFreq*stepTime),2));
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
      
//      // Debugging control:
//      controlSupportLeg(RobotSide.LEFT);
//      
//      double q_hip = robot.getHipAngle(RobotSide.RIGHT);
//      double qd_hip = robot.getHipVelocity(RobotSide.RIGHT);
//      double q_knee = robot.getKneeLength(RobotSide.RIGHT);
//      double qd_knee = robot.getKneeVelocity(RobotSide.RIGHT);
//      // Servo joints to the desired joint trajs:
//      double Tauhip  = 10*(0 - q_hip)  + 1*(0 - qd_hip);
//      double Tauknee = 10*(0.6 - q_knee) + 1*(0 - qd_knee);
//      robot.setHipTorque(RobotSide.RIGHT, Tauhip);
//      robot.setKneeForce(RobotSide.RIGHT, Tauknee);
      
      
      
      for(RobotSide robotSide : RobotSide.values())
      {
         stateMachines.get(robotSide).doAction();
         stateMachines.get(robotSide).doTransitions();
      }

//      LogTools.info("StateMachine: Left:{} Right:{}", stateMachines.get(RobotSide.LEFT).getCurrentStateKey(), stateMachines.get(RobotSide.RIGHT).getCurrentStateKey());
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
      
//      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
//      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();
      
      double z_body_w = robot.getBodyZPosition();
      double zd_body_w = robot.getBodyZVelocity();
      double th_body = robot.getBodyPitchAngle();
      double thd_body = robot.getBodyPitchAngularVelocity();
  
      // The body-servo:
      double Fz = 50*(desiredBodyHeight - z_body_w) - 5*zd_body_w + 0.999*mass*9.81; // GMN
      double My = 50*(0 - th_body) - 5*thd_body; // GMN
      
      double xcop = getXcop(side);
      
      double Fx = -1/z_body_w*(Fz*xcop + My);  // Fx based on body-servo & current foot position

      // Do implicit force control:
      DMatrixRMaj Ffoot = new DMatrixRMaj(2,1);
      Ffoot.set(0,0,-Fx); // set desired foot-force (applied by foot to ground)
      Ffoot.set(1,0,-Fz); // set desired foot-force (applied by foot to ground)
      DMatrixRMaj Jleg = robot.getlegJacobian(robot.getBodyPitchAngle(),
                                              robot.getHipAngle(side),
                                              Math.abs(robot.getKneeLength(side))); // get jacobian (includes body pitch)
      CommonOps_DDRM.transpose(Jleg);  // transpose the jacobian
      DMatrixRMaj Tau = new DMatrixRMaj(3,1);
      CommonOps_DDRM.mult(Jleg,Ffoot,Tau); // tau = J-transpose * F
      
//      /* Compute and set orbital energy YoDouble. Based on virtual spring-mass system. */
//      double orbitalEnergyValue = 0.5 * mass * centerOfMassVelocity.lengthSquared() - 0.5 * g / desiredHeight.getDoubleValue() * centerOfMassPosition.distanceFromOriginSquared();
//      orbitalEnergy.set(orbitalEnergyValue);

      // Skip 0 element as that is moment on body
      robot.setHipTorque(side, Tau.get(1, 0));
      robot.setKneeForce(side, Tau.get(2, 0));
   }

   private void controlSwingLeg(RobotSide side, double timeInSwing, double dT, double desiredTD)
   {
      double thd_body = robot.getBodyPitchAngularVelocity();
      
      // Dead-reckoning on foot x pos to desired:
      double dt_denom = (stepTime-0.1) - timeInSwing; // GMN: Added 0.1 sec arrive early...
      if (dt_denom < 0.01)
      {
         dt_denom = 0.01; // GMN
      }
      double xd_d_foot = (desiredTD - LRx_d_foot.get(side))/dt_denom; // GMN
      LRx_d_foot.set(side,LRx_d_foot.get(side) + xd_d_foot*dT);
      
      // get zd_d_foot:
      double zd_d_foot = calculateSwingZdot(side,timeInSwing);
      
      // Do diffIK; use measured th_body as FF term?
      DMatrixRMaj Jleg = robot.getlegJacobian(robot.getBodyPitchAngle(),  // includes body-pitch
                                              LRq_d.get(side).get(0,0),
                                              LRq_d.get(side).get(1,0)); 
      DMatrixRMaj Jq = new DMatrixRMaj(2,2); // just leg joints
      Jq.set(0,0,Jleg.get(0,1));
      Jq.set(0,1,Jleg.get(0,2)); 
      Jq.set(1,0,Jleg.get(1,1));
      Jq.set(1,1,Jleg.get(1,2));
      CommonOps_DDRM.invert(Jq);  // Jq now holds its inverse
      DMatrixRMaj RHS = new DMatrixRMaj(2,1);
      RHS.set(0,0,xd_d_foot - Jleg.get(0,0)*thd_body); // GMN: including body-pitch rate bias
      RHS.set(1,0,zd_d_foot - Jleg.get(1,0)*thd_body); // GMN: including body-pitch rate bias
      DMatrixRMaj qd_d = new DMatrixRMaj(2,1);
      CommonOps_DDRM.mult(Jq, RHS, qd_d); // resolved-rate IK
      LRq_d.get(side).set(0,0,LRq_d.get(side).get(0,0)+qd_d.get(0,0)*dT); // Easier way?????
      LRq_d.get(side).set(1,0,LRq_d.get(side).get(1,0)+qd_d.get(1,0)*dT); // Easier way?????
      
      double q_hip = robot.getHipAngle(side);
      double qd_hip = robot.getHipVelocity(side);
      double q_knee = robot.getKneeLength(side);
      double qd_knee = robot.getKneeVelocity(side);

      // Servo joints to the desired joint trajs:
      double Tauhip  = 100*(LRq_d.get(side).get(0,0) - q_hip)  + 10*(qd_d.get(0,0) - qd_hip);
      double Tauknee = 100*(LRq_d.get(side).get(1,0) - q_knee) + 10*(qd_d.get(1,0) - qd_knee);
      
      robot.setHipTorque(side, Tauhip);
      robot.setKneeForce(side, Tauknee);
   }

   // Get current foot x pos r.t. body ewrt world:
   double getXcop(RobotSide side)
   {
      Point3D pos_foot = robot.getFootPosition(side);
      double xcop = pos_foot.getX() - robot.getBodyXPosition(); // xcop r.t. body
      
      return xcop;
   }
   
   double calculateStepLocation(double xLO)
   {
      // TDLO here......
      // + a desired x position on the ground
      
      return (kx_TDLO*(xTD-xLO)-bx_TDLO*(xTD+xLO));      
//      return (kx_TDLO*(xTD-xLO)-bx_TDLO*(xTD+xLO) - 0.04);      
   }
   
   // See reMarkable notes: 2/21/2022
   DMatrixRMaj buildSwingZdot(double zLO)
   {
      double zSH = SwingHeight ; // GMN
      double zTD = 0;
      double zdTD = -1; //-1; // GMN
      
      double tSH = 0.5*stepTime;
      double tTD = stepTime;

      // Cubic-spline fit:
      double data[][] = new double[][] {
         {  Math.pow(tSH,3), Math.pow(tSH,2), tSH },
         {  Math.pow(tTD,3), Math.pow(tTD,2), tTD },
         {3*Math.pow(tTD,2), 2*tTD, 1}
      };
      
      // Solve for spline coeffs:
      DMatrixRMaj m = new DMatrixRMaj(data);
      CommonOps_DDRM.invert(m);
      DMatrixRMaj LHS = new DMatrixRMaj(3,1);
      LHS.set(0,0,zSH - zLO);
      LHS.set(1,0,zTD - zLO);
      LHS.set(2,0,zdTD);
      DMatrixRMaj result = new DMatrixRMaj(3,1);
      CommonOps_DDRM.mult(m, LHS, result);
      
      return result;
   }
   
   double calculateSwingZdot(RobotSide side, double t)
   {
      // This is z-dot:
      DMatrixRMaj coeffs = LRSwingCoeffs.get(side);
      return (3*coeffs.get(0,0)*Math.pow(t,2) + 2*coeffs.get(1,0)*t + coeffs.get(2,0));
   }

   private SideDependentList<StateMachine<States, State>> setupStateMachines()
   {
      // States and Actions:
      StateMachineFactory<States, State> leftFactory = new StateMachineFactory<>(States.class);
      StateMachineFactory<States, State> rightFactory = new StateMachineFactory<>(States.class);

      leftFactory.setNamePrefix("leftState");
      rightFactory.setNamePrefix("rightState");
      
      leftFactory.setRegistry(registry);
      rightFactory.setRegistry(registry);
      
      leftFactory.buildClock(robot.getRobot().getYoTime());
      rightFactory.buildClock(robot.getRobot().getYoTime());
 
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
      private final RobotSide robotSide;

      public SupportState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {
         // Record xTD for TDLO:
         xTD = getXcop(robotSide);
      }

      @Override
      public void doAction(double timeInState)
      {
         // Support Side:
         controlSupportLeg(robotSide);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   private class LiftState implements State
   {
      private final RobotSide robotSide;

      public LiftState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {
         LRlastTime.set(robotSide,-1.0);

         // Build the z-spline:
         Point3D pos_foot = robot.getFootPosition(robotSide);
         LRSwingCoeffs.set(robotSide,buildSwingZdot(pos_foot.getZ()));
         
         // Initialize the diff IK:
         LRq_d.get(robotSide).set(0,0,robot.getHipAngle(robotSide));
         LRq_d.get(robotSide).set(1,0,robot.getKneeLength(robotSide));
         LRx_d_foot.set(robotSide,getXcop(robotSide));
         
         inLiftState.set(robotSide,true);
      }

      @Override
      public void doAction(double timeInState)
      {
         if (LRlastTime.get(robotSide) < 0)
         {
            LRlastTime.set(robotSide,timeInState);
         }
         double dT = timeInState - LRlastTime.get(robotSide);
         LRlastTime.set(robotSide,timeInState);
         
         double xLO = getXcop(robotSide.getOppositeSide());
         double desiredTD = calculateStepLocation(xLO); // TDLO
         controlSwingLeg(robotSide,timeInState,dT,desiredTD);
      }

      @Override
      public void onExit(double timeInState)
      {
         inLiftState.set(robotSide,false);
      }
   }

   private class SwingState implements State
   {
      private final RobotSide robotSide;

      public SwingState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {
         
      }

      @Override
      public void doAction(double timeInState)
      {
         double dT = timeInState - LRlastTime.get(robotSide);
         LRlastTime.set(robotSide,timeInState);
         
         double xLO = getXcop(robotSide.getOppositeSide());
         double desiredTD = calculateStepLocation(xLO); // TDLO
         controlSwingLeg(robotSide,timeInState,dT,desiredTD);
      }

      @Override
      public void onExit(double timeInState)
      {
         
      }
   }

   // LIFT -> SWING: The LIFTING foot has cleared the ground:
   private class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         Point3D pos_foot = robot.getFootPosition(robotSide); // GMN: What is behind this function?  Super-sensing?
         boolean result = (pos_foot.getZ() > SwingHeight/4); 
         return (result);
      }
   }

   // SUPPORT -> LIFT: The current STANCE foot detects that the OTHER foot has hit the ground:
   private class OtherHeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public OtherHeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         // GMN: Is there a way to find & look at the contact state of the OTHER side of the robot???
         // GMN: Make the state transition simply be when the OTHER side is on the ground:
         boolean result = ((robot.getFootZForce(this.robotSide.getOppositeSide()) > 10) && 
                           (inLiftState.get(this.robotSide.getOppositeSide()) == false));
         return (result);
      }
   }

   // SWING -> SUPPORT: The current SWING foot hits the ground:
   private class HeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         boolean result = ((robot.getFootZForce(this.robotSide) > 10) &&
                           (inLiftState.get(this.robotSide) == false));
         return (result);
         
//         LogTools.info("Side: {} getKneeForce(): {}", robotSide, robot.getKneeForce(robotSide));
      }
   }
}