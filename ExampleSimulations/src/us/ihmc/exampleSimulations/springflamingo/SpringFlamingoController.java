package us.ihmc.exampleSimulations.springflamingo;

import java.awt.Container;

import javax.swing.BoxLayout;
import javax.swing.JFrame;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachinesJPanel;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;


/**
 * <p>Title: SpringFlamingoController</p>
 *
 * <p>Description: Simple balistic walking controller for the SpringFlamingo simulation model.
 * Controls body pitch through hip torque, velocity through ankle torque. </p>
 *
 * <p>Copyright: Copyright (c) 2001</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SpringFlamingoController implements RobotController
{
   /**
    * Initialization 
    */
   
   private final YoVariableRegistry registry = new YoVariableRegistry("SpringFlamingoController");
   
   private double comPosX, comPosZ, comVelX, icpPos; //TODO modified
//   private ICPVisualizer icpVisualizer;
   
   // State Machine:
   private enum States {SUPPORT, TOE_OFF, SWING, STRAIGHTEN}

   private final StateMachine<States> leftStateMachine, rightStateMachine;
   private final SideDependentList<StateMachine<States>> stateMachines;

   // Control Parameters:
   private final DoubleYoVariable stand_gain = new DoubleYoVariable("stand_gain", "Gain for torquing the support ankle based on an error in desired x", registry);
   private final DoubleYoVariable x_d = new DoubleYoVariable("x_d", "Desired x location. Controlled with stand_gain", registry);

   private final DoubleYoVariable v_nom = new DoubleYoVariable("v_nom", "Nominal velocity", registry);
   private final DoubleYoVariable vtp_gain = new DoubleYoVariable("vtp_gain", "Gain on velocity error to support ankle torque.", registry);
   private final DoubleYoVariable desiredBodyPitch = new DoubleYoVariable("t_d", "Desired body pitch", registry);
   private final DoubleYoVariable t_gain = new DoubleYoVariable("t_gain", "Gain on Pitch angle to hip torque", registry);
   private final DoubleYoVariable t_damp = new DoubleYoVariable("t_damp", "Hip damping", registry);

   private final DoubleYoVariable knee_d = new DoubleYoVariable("knee_d", registry);
   private final DoubleYoVariable knee_gain = new DoubleYoVariable("knee_gain", registry);
   private final DoubleYoVariable knee_damp = new DoubleYoVariable("knee_damp", registry);
   private final DoubleYoVariable hip_d = new DoubleYoVariable("hip_d", registry);
   private final DoubleYoVariable hip_gain = new DoubleYoVariable("hip_gain", registry);
   private final DoubleYoVariable hip_damp = new DoubleYoVariable("hip_damp", registry);
   private final DoubleYoVariable hip_hold = new DoubleYoVariable("hip_hold", registry);
   private final DoubleYoVariable swing_gain_knee = new DoubleYoVariable("swing_gain_knee", registry);
   private final DoubleYoVariable swing_damp_knee = new DoubleYoVariable("swing_damp_knee", registry);
   private final DoubleYoVariable force_thresh = new DoubleYoVariable("force_thresh", registry);

   private final DoubleYoVariable min_support_time = new DoubleYoVariable("min_support_time", registry);
   private final DoubleYoVariable swing_time = new DoubleYoVariable("swing_time", registry);
   private final DoubleYoVariable toe_off_time = new DoubleYoVariable("toe_off_time", registry);
   private final DoubleYoVariable ankle_d = new DoubleYoVariable("ankle_d", "Swing ankle desired pitch", registry);
   private final DoubleYoVariable ankle_gain = new DoubleYoVariable("ankle_gain", "Gain on ankle ankle to ankle torque", registry);
   private final DoubleYoVariable ankle_damp = new DoubleYoVariable("ankle_damp", "Gain on ankle velocity to ankle torque", registry);

   private final DoubleYoVariable ankle_limit_set = new DoubleYoVariable("ankle_limit_set", registry);
   private final DoubleYoVariable ankle_limit_gain = new DoubleYoVariable("ankle_limit_gain", registry);

   private final DoubleYoVariable push_set = new DoubleYoVariable("push_set", "Ankle angle to toe off to", registry);
   private final DoubleYoVariable push_gain = new DoubleYoVariable("push_gain", "Gain on ankle angle to ankle torque when toeing off", registry);
   private final DoubleYoVariable push_damp = new DoubleYoVariable("push_damp", "Damping on ankle angle to ankle torque when toeing off", registry);

   private final DoubleYoVariable knee_collapse = new DoubleYoVariable("knee_collapse", registry);

   // Active and Passive Torques:
   private final DoubleYoVariable act_lh = new DoubleYoVariable("act_lh", registry);
   private final DoubleYoVariable act_lk = new DoubleYoVariable("act_lk", registry);
   private final DoubleYoVariable act_la = new DoubleYoVariable("act_la", registry);
   private final DoubleYoVariable pas_la = new DoubleYoVariable("pas_la", registry);
   private final DoubleYoVariable act_rh = new DoubleYoVariable("act_rh", registry);
   private final DoubleYoVariable act_rk = new DoubleYoVariable("act_rk", registry);
   private final DoubleYoVariable act_ra = new DoubleYoVariable("act_ra", registry);
   private final DoubleYoVariable pas_ra = new DoubleYoVariable("pas_ra", registry);
   private final DoubleYoVariable pas_lh = new DoubleYoVariable("pas_lh", registry);
   private final DoubleYoVariable pas_lk = new DoubleYoVariable("pas_lk", registry);
   private final DoubleYoVariable pas_rh = new DoubleYoVariable("pas_rh", registry);
   private final DoubleYoVariable pas_rk = new DoubleYoVariable("pas_rk", registry);
   private final DoubleYoVariable max_hip_torque = new DoubleYoVariable("max_hip_torque", registry);

   private final SideDependentList<DoubleYoVariable> actHip = new SideDependentList<DoubleYoVariable>(act_lh, act_rh);
   private final SideDependentList<DoubleYoVariable> actKnee = new SideDependentList<DoubleYoVariable>(act_lk, act_rk);
   private final SideDependentList<DoubleYoVariable> actAnkle = new SideDependentList<DoubleYoVariable>(act_la, act_ra);
   private final SideDependentList<DoubleYoVariable> pasHip = new SideDependentList<DoubleYoVariable>(pas_lh, pas_rh);
   private final SideDependentList<DoubleYoVariable> pasKnee = new SideDependentList<DoubleYoVariable>(pas_lk, pas_rk);
   private final SideDependentList<DoubleYoVariable> pasAnkle = new SideDependentList<DoubleYoVariable>(pas_la, pas_ra);
   private final SideDependentList<DoubleYoVariable> tauHip;
   private final SideDependentList<DoubleYoVariable> tauKnee;
   private final SideDependentList<DoubleYoVariable> tauAnkle;

   private final SideDependentList<DoubleYoVariable> qAnkle;
   private final SideDependentList<DoubleYoVariable> qHip;
   private final SideDependentList<DoubleYoVariable> qKnee;

   private final SideDependentList<DoubleYoVariable> qdAnkle;
   private final SideDependentList<DoubleYoVariable> qdHip;
   private final SideDependentList<DoubleYoVariable> qdKnee;

   private final SideDependentList<DoubleYoVariable> gcHeel_fs;
   private final SideDependentList<DoubleYoVariable> gcHeel_fz;
   private final SideDependentList<DoubleYoVariable> gcHeel_x;
   private final SideDependentList<DoubleYoVariable> gcToe_fs;
   private final SideDependentList<DoubleYoVariable> gcToe_fz;

   // Computed State Variables:
   private final DoubleYoVariable vel = new DoubleYoVariable("vel", "Actual Velocity", registry);
   private final DoubleYoVariable left_heel = new DoubleYoVariable("left_heel", "Orientation of the left foot", registry);
   private final DoubleYoVariable right_heel = new DoubleYoVariable("right_heel", "Orientation of the right foot", registry);

   private final DoubleYoVariable left_force = new DoubleYoVariable("left_force", "Total vertical ground reaction force on the left foot", registry);
   private final DoubleYoVariable left_cop = new DoubleYoVariable("left_cop", "Location of the left foots Center of Pressure", registry);
   private final DoubleYoVariable right_force = new DoubleYoVariable("right_force", "Total vertical ground reaction force on the left foot", registry);
   private final DoubleYoVariable right_cop = new DoubleYoVariable("right_cop", "Location of the right foots Center of Pressure", registry);

   private final DoubleYoVariable left_hip_set = new DoubleYoVariable("left_hip_set", registry);
   private final DoubleYoVariable right_hip_set = new DoubleYoVariable("right_hip_set", registry);

   private final SideDependentList<DoubleYoVariable> force = new SideDependentList<DoubleYoVariable>(left_force, right_force);
   private final SideDependentList<DoubleYoVariable> cops = new SideDependentList<DoubleYoVariable>(left_cop, right_cop);
   private final SideDependentList<DoubleYoVariable> heel = new SideDependentList<DoubleYoVariable>(left_heel, right_heel);
   private final SideDependentList<DoubleYoVariable> hipSet = new SideDependentList<DoubleYoVariable>(left_hip_set, right_hip_set);

   private final SpringFlamingoRobot robot;

   private String name;
   
   private final SimpleMovingAverageFilteredYoVariable average_qd_x ; //TODO what is this? 

   /**
    * Constructor
    */
   
   public SpringFlamingoController(SpringFlamingoRobot robot, String name) // TODO SpringFlamingoController(SpringFlamingoRobot robot, String name, ICPVisualizer icpVisualizer) 
   {
      this.name = name;
      this.robot = robot;

      qAnkle = new SideDependentList<DoubleYoVariable>(robot.q_la, robot.q_ra);
      qHip = new SideDependentList<DoubleYoVariable>(robot.q_lh, robot.q_rh);
      qKnee = new SideDependentList<DoubleYoVariable>(robot.q_lk, robot.q_rk);
      qdAnkle = new SideDependentList<DoubleYoVariable>(robot.qd_la, robot.qd_ra);
      qdHip = new SideDependentList<DoubleYoVariable>(robot.qd_lh, robot.qd_rh);
      qdKnee = new SideDependentList<DoubleYoVariable>(robot.qd_lk, robot.qd_rk);
      gcHeel_fs = new SideDependentList<DoubleYoVariable>(robot.gc_lheel_fs, robot.gc_rheel_fs);
      gcHeel_fz = new SideDependentList<DoubleYoVariable>(robot.gc_lheel_fz, robot.gc_rheel_fz);
      gcHeel_x = new SideDependentList<DoubleYoVariable>(robot.gc_lheel_x, robot.gc_rheel_x);
      gcToe_fs = new SideDependentList<DoubleYoVariable>(robot.gc_ltoe_fs, robot.gc_rtoe_fs);
      gcToe_fz = new SideDependentList<DoubleYoVariable>(robot.gc_ltoe_fz, robot.gc_rtoe_fz);

      tauHip = new SideDependentList<DoubleYoVariable>(robot.tau_lh, robot.tau_rh);
      tauKnee = new SideDependentList<DoubleYoVariable>(robot.tau_lk, robot.tau_rk);
      tauAnkle = new SideDependentList<DoubleYoVariable>(robot.tau_la, robot.tau_ra);

      average_qd_x = new SimpleMovingAverageFilteredYoVariable("average_qd_x", 100, robot.qd_x, registry );
      
      // Create the state machines:
      leftStateMachine = new StateMachine<States>("leftState", "leftSwitchTime", States.class, robot.t, registry);
      rightStateMachine = new StateMachine<States>("rightState", "rightSwitchTime", States.class, robot.t, registry);
      stateMachines = new SideDependentList<StateMachine<States>>(leftStateMachine, rightStateMachine);

      setupStateMachines();
      createStateMachineWindow();
      initControl();
      
   }
   
   
   //////////////////////////////////////////////////////
   public void createStateMachineWindow()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         public void run()
         {
            createStateMachineWindowLocal();
         }
      });
   }

   public void createStateMachineWindowLocal()
   {
      JFrame jFrame = new JFrame("Spring Flamingo State Machines");
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.X_AXIS));

      StateMachinesJPanel<States> leftStateMachinePanel = new StateMachinesJPanel<States>(leftStateMachine, false);
      StateMachinesJPanel<States> rightStateMachinePanel = new StateMachinesJPanel<States>(rightStateMachine, true);

      jFrame.getContentPane().add(leftStateMachinePanel);
      jFrame.getContentPane().add(rightStateMachinePanel);

      jFrame.pack();
      jFrame.setSize(450, 300);
      jFrame.setAlwaysOnTop(false);
      jFrame.setVisible(true);

      // Doing the following will cause redraw when the state changes, but not during replay or rewind:
      leftStateMachine.attachStateChangedListener(leftStateMachinePanel);
      rightStateMachine.attachStateChangedListener(rightStateMachinePanel);

      // Doing this will cause redraw every specified milliseconds:
      //    leftStateMachinePanel.createUpdaterThread(250);
      //    rightStateMachinePanel.createUpdaterThread(250);
   }
   //////////////////////////////////////////////////////////
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void initControl()
   {
      // Initialize the variables.
      robot.initializeForBallisticWalking();

      vel.set(0.0);
      stand_gain.set(0.0);
      x_d.set(0.0);
      v_nom.set(-0.4);
      vtp_gain.set(0.0);
      desiredBodyPitch.set(0.0);
      t_gain.set(100.0);
      t_damp.set(20.0);
      knee_d.set(0.0);
      knee_gain.set(30.0);      
      knee_damp.set(10.0);
      hip_d.set(0.587059);
      hip_gain.set(15.9804);
      hip_damp.set(2.01177);
      hip_hold.set(0.358627);
      swing_gain_knee.set(1.81176);
      swing_damp_knee.set(0.403529);
      force_thresh.set(13.6078);
      min_support_time.set(0.2);
      swing_time.set(0.418039);
      toe_off_time.set(0.05);
      ankle_d.set(0.0);
      ankle_gain.set(4.0);
      ankle_damp.set(1.0);
      ankle_limit_set.set(0.0);
      ankle_limit_gain.set(250.98);

      left_force.set(0.0);
      left_cop.set(0.5);
      right_force.set(131.83);
      right_cop.set(0.557913);
      left_hip_set.set(0.358627);

      left_heel.set(-0.392589);
      right_hip_set.set(0.358627);

      right_heel.set(-0.231515);
      push_gain.set(9.56078);
      knee_collapse.set(0.0);
      push_set.set(-0.236471);
      push_damp.set(0.0);
      max_hip_torque.set(0.0);

   }

   public void doControl()
   {
      balistic_walking_state_machine();
      average_qd_x.update();
   }

//   private double passive_ankle_torques(double pos, double vel)
//   {
//      if (pos > ankle_limit_set.getDoubleValue())
//         return (-ankle_limit_gain.getDoubleValue() * (ankle_limit_set.getDoubleValue() - pos) * (ankle_limit_set.getDoubleValue() - pos));
//      else
//         return (0.0);
//   }

   private double passive_ankle_torques(double pos, double vel)
   {
       if (pos > ankle_limit_set.getDoubleValue())
         return (ankle_limit_gain.getDoubleValue() * (ankle_limit_set.getDoubleValue() - pos));
       else
          return (0.0);
   }   
   
   private double toe_off_ankle_torques(double pos, double vel)
   {
      return (push_gain.getDoubleValue() * (push_set.getDoubleValue() - pos) - push_damp.getDoubleValue() * vel);
   }

   private void setupStateMachines()
   {
      // States and Actions:

      State<States> leftSupportState = new SupportState(RobotSide.LEFT, States.SUPPORT);
      State<States> rightSupportState = new SupportState(RobotSide.RIGHT, States.SUPPORT);  
      State<States> leftToeOffState = new ToeOffState(RobotSide.LEFT, States.TOE_OFF);
      State<States> rightToeOffState = new ToeOffState(RobotSide.RIGHT, States.TOE_OFF);
      State<States> leftSwingState = new SwingState(RobotSide.LEFT, States.SWING);
      State<States> rightSwingState = new SwingState(RobotSide.RIGHT, States.SWING);
      State<States> leftStraightenState = new StraightenState(RobotSide.LEFT, States.STRAIGHTEN);
      State<States> rightStraightenState = new StraightenState(RobotSide.RIGHT, States.STRAIGHTEN);

      // Transition Conditions:
      StateTransitionCondition leftHealUnloaded = new HeelOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftFootUnloaded = new ToeOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftFootTouchedDown = new HeelOnGroundCondition(RobotSide.LEFT);

      StateTransitionCondition rightHealUnloaded = new HeelOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightFootUnloaded = new ToeOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightFootTouchedDown = new HeelOnGroundCondition(RobotSide.RIGHT);

      // Left State Transitions:
      StateTransition<States> leftSupportToToeOff = new StateTransition<States>(States.TOE_OFF, leftHealUnloaded);
      leftSupportToToeOff.addTimePassedCondition(min_support_time);
      leftSupportState.addStateTransition(leftSupportToToeOff);

      StateTransition<States> leftToeOffToSwing = new StateTransition<States>(States.SWING, leftFootUnloaded);
      leftToeOffState.addStateTransition(leftToeOffToSwing);

      StateTransition<States> leftSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swing_time);
      leftSwingState.addStateTransition(leftSwingToStraighten);

      StateTransition<States> leftStraightenToSupport = new StateTransition<States>(States.SUPPORT, leftFootTouchedDown);
      leftStraightenState.addStateTransition(leftStraightenToSupport);

      // Right State Transitions:
      StateTransition<States> rightSupportToToeOff = new StateTransition<States>(States.TOE_OFF, rightHealUnloaded);
      rightSupportToToeOff.addTimePassedCondition(min_support_time);
      rightSupportState.addStateTransition(rightSupportToToeOff);

      StateTransition<States> rightToeOffToSwing = new StateTransition<States>(States.SWING, rightFootUnloaded);
      rightToeOffState.addStateTransition(rightToeOffToSwing);

      StateTransition<States> rightSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swing_time);
      rightSwingState.addStateTransition(rightSwingToStraighten);

      StateTransition<States> rightStraightenToSupport = new StateTransition<States>(States.SUPPORT, rightFootTouchedDown);
      rightStraightenState.addStateTransition(rightStraightenToSupport);


      // Assemble the Left State Machine:
      leftStateMachine.addState(leftSupportState);
      leftStateMachine.addState(leftToeOffState);
      leftStateMachine.addState(leftSwingState);
      leftStateMachine.addState(leftStraightenState);


      // Assemble the Right State Machine:
      rightStateMachine.addState(rightSupportState);
      rightStateMachine.addState(rightToeOffState);
      rightStateMachine.addState(rightSwingState);
      rightStateMachine.addState(rightStraightenState);

      // Set the Initial States:
      leftStateMachine.setCurrentState(States.STRAIGHTEN);
      rightStateMachine.setCurrentState(States.SUPPORT);
   } 
   
   //////////////////////////////////////////////  (1) Support
   private class SupportState extends State<States>
   {
      private final RobotSide robotSide;

      public SupportState(RobotSide robotSide, States stateEnum)
      {
         super(States.SUPPORT);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         // Use hip to servo pitch
         if (force.get(robotSide).getDoubleValue() > 20.0)
            actHip.get(robotSide).set(-t_gain.getDoubleValue() * (desiredBodyPitch.getDoubleValue() - robot.q_pitch.getDoubleValue())
                  + t_damp.getDoubleValue() * robot.qd_pitch.getDoubleValue());

         // Keep knee straight
         actKnee.get(robotSide).set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - qKnee.get(robotSide).getDoubleValue())
               - knee_damp.getDoubleValue() * qdKnee.get(robotSide).getDoubleValue());

         // Use ankle to servo speed, position
         actAnkle.get(robotSide).set(-stand_gain.getDoubleValue() * (x_d.getDoubleValue() - robot.q_x.getDoubleValue())
               - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - robot.qd_x.getDoubleValue()));

         // Ankle limit to go on toes and maintain cop
         pasAnkle.get(robotSide).set(passive_ankle_torques(qAnkle.get(robotSide).getDoubleValue(), qdAnkle.get(robotSide).getDoubleValue()));
      }

      public void doTransitionIntoAction()
      {
      }
      public void doTransitionOutOfAction()
      {
      }
   }

   //////////////////////////////////////////////  (2) Toe Off
   private class ToeOffState extends State<States>
   {
      private final RobotSide robotSide;

      public ToeOffState(RobotSide robotSide, States stateEnum)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         // Use hip to servo pitch
         actHip.get(robotSide).set(-t_gain.getDoubleValue() * (desiredBodyPitch.getDoubleValue() - robot.q_pitch.getDoubleValue())
               + t_damp.getDoubleValue() * robot.qd_pitch.getDoubleValue());

         // Keep knee straight
         actKnee.get(robotSide).set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - qKnee.get(robotSide).getDoubleValue())
               - knee_damp.getDoubleValue() * qdKnee.get(robotSide).getDoubleValue());

         // Use ankle to servo speed, position
         actAnkle.get(robotSide).set(-stand_gain.getDoubleValue() * (x_d.getDoubleValue() - robot.q_x.getDoubleValue())
               - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - robot.qd_x.getDoubleValue()));

         // Ankle limit to go on toes and maintain cop
         pasAnkle.get(robotSide).set(passive_ankle_torques(qAnkle.get(robotSide).getDoubleValue(), qdAnkle.get(robotSide).getDoubleValue()));

         // Ankle push off
         actAnkle.get(robotSide).set(actAnkle.get(robotSide).getDoubleValue() + toe_off_ankle_torques(qAnkle.get(robotSide).getDoubleValue(), qdAnkle.get(robotSide).getDoubleValue()));

      }

      public void doTransitionIntoAction()
      {
      }
      public void doTransitionOutOfAction()
      {
      }
   }

   //////////////////////////////////////////////  (3) Swing
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
         // Servo hip up
         hipSet.get(robotSide).set(hip_d.getDoubleValue());
         actHip.get(robotSide).set(hip_gain.getDoubleValue() * (hipSet.get(robotSide).getDoubleValue() - qHip.get(robotSide).getDoubleValue())
               - hip_damp.getDoubleValue() * qdHip.get(robotSide).getDoubleValue());

         // Damp the knee
         actKnee.get(robotSide).set(-swing_damp_knee.getDoubleValue() * qdKnee.get(robotSide).getDoubleValue());

         // act_lk.val = 0.0;

         // Damp and Toggle the knee:
         actKnee.get(robotSide).set(actKnee.get(robotSide).getDoubleValue() + knee_collapse.getDoubleValue());

         // Continue toe off until toe leaves ground
         if (gcToe_fs.get(robotSide).getDoubleValue() > 0.1)
         {
            pasAnkle.get(robotSide).set(passive_ankle_torques(qAnkle.get(robotSide).getDoubleValue(), qdAnkle.get(robotSide).getDoubleValue()));
            actAnkle.get(robotSide).set(toe_off_ankle_torques(qAnkle.get(robotSide).getDoubleValue(), qdAnkle.get(robotSide).getDoubleValue()));
         }
         else
         {
            // Servo ankle level to the ground
            heel.get(robotSide).set(-robot.q_pitch.getDoubleValue() - qHip.get(robotSide).getDoubleValue() - qKnee.get(robotSide).getDoubleValue() - qAnkle.get(robotSide).getDoubleValue());
            actAnkle.get(robotSide).set(-ankle_gain.getDoubleValue() * (ankle_d.getDoubleValue() - heel.get(robotSide).getDoubleValue())
                  - ankle_damp.getDoubleValue() * qdAnkle.get(robotSide).getDoubleValue());
         }
      }

      public void doTransitionIntoAction()
      {
      }
      public void doTransitionOutOfAction()
      {
      }
   }

   //////////////////////////////////////////////  (4) Straighten
   private class StraightenState extends State<States>
   {
      private final RobotSide robotSide;

      public StraightenState(RobotSide robotSide, States stateEnum)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         // Servo hip to a more shallow angle
         hipSet.get(robotSide).set(hip_hold.getDoubleValue());
         actHip.get(robotSide).set(hip_gain.getDoubleValue() * (hipSet.get(robotSide).getDoubleValue() - qHip.get(robotSide).getDoubleValue())
               - hip_damp.getDoubleValue() * qdHip.get(robotSide).getDoubleValue());

         // Keep knee straight
         actKnee.get(robotSide).set(swing_gain_knee.getDoubleValue() * (knee_d.getDoubleValue() - qKnee.get(robotSide).getDoubleValue())
               - swing_damp_knee.getDoubleValue() * qdKnee.get(robotSide).getDoubleValue());

         // Servo ankle level to the ground
         heel.get(robotSide).set(-robot.q_pitch.getDoubleValue() - qHip.get(robotSide).getDoubleValue() - qKnee.get(robotSide).getDoubleValue() - qAnkle.get(robotSide).getDoubleValue());
         actAnkle.get(robotSide).set(-ankle_gain.getDoubleValue() * (ankle_d.getDoubleValue() - heel.get(robotSide).getDoubleValue())
               - ankle_damp.getDoubleValue() * qdAnkle.get(robotSide).getDoubleValue());
      }

      public void doTransitionIntoAction()
      {
      }
      public void doTransitionOutOfAction()
      {
      }
   }

   ////////////////////////////////////////////////////  STATE MACHINE
   private void balistic_walking_state_machine()
   {   
      // Robot happens to walk in negative x direction.  Set vel positive just so it makes intuitive sense.
      vel.set(-robot.qd_x.getDoubleValue());

      // Calculate forces on the feet
      for (RobotSide robotSide : RobotSide.values())
      {
         //System.out.println("gc heel force: " + gcHeel_fz.get(robotSide).getDoubleValue());
         force.get(robotSide).set(gcHeel_fz.get(robotSide).getDoubleValue() + gcToe_fz.get(robotSide).getDoubleValue());
         if (force.get(robotSide).getDoubleValue() > 5.0)
            cops.get(robotSide).set(gcToe_fz.get(robotSide).getDoubleValue() / force.get(robotSide).getDoubleValue());
         else
            cops.get(robotSide).set(0.5);
      }

      // Actions in Each State
      for (RobotSide robotSide : RobotSide.values())
         stateMachines.get(robotSide).doAction();

      for (RobotSide robotSide : RobotSide.values())
      {
         StateMachine<States> stateMachine = stateMachines.get(robotSide);
         // Communication between legs for now...

         // Subtract swing hip torque from the stance leg so less disturbance on the body
         if (stateMachine.isCurrentState(States.SWING))
            actHip.get(robotSide.getOppositeSide()).sub(actHip.get(robotSide).getDoubleValue());

         // Transition Conditions:
         stateMachine.checkTransitionConditions();
      }
      
      // Torques at the joints:
      for (RobotSide robotSide : RobotSide.values())
      {
         tauHip.get(robotSide).set(actHip.get(robotSide).getDoubleValue() + pasHip.get(robotSide).getDoubleValue());
         tauKnee.get(robotSide).set(actKnee.get(robotSide).getDoubleValue() + pasKnee.get(robotSide).getDoubleValue());
         tauAnkle.get(robotSide).set(actAnkle.get(robotSide).getDoubleValue() + pasAnkle.get(robotSide).getDoubleValue());
      }
         
   }

   ////////////////////////////////////////////////////////////////////////
   public class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }
      public boolean checkCondition()
      {      
         return (gcHeel_fz.get(robotSide).getDoubleValue() < force_thresh.getDoubleValue())
               && ((robot.qd_x.getDoubleValue() < 0.0) && (gcHeel_x.get(robotSide).getDoubleValue() > robot.q_x.getDoubleValue()))
               && (gcHeel_x.get(robotSide).getDoubleValue() >   gcHeel_x.get(robotSide.getOppositeSide()).getDoubleValue());
      }
   }  

   public class ToeOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public ToeOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }   

      public boolean checkCondition()
      {
         return (force.get(robotSide).getDoubleValue() < force_thresh.getDoubleValue());
      }
   }

   public class HeelOnGroundCondition implements StateTransitionCondition   
   {
      private final RobotSide robotSide;

      public HeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         return (gcToe_fs.get(robotSide).getDoubleValue() == 1.0) || (gcHeel_fs.get(robotSide).getDoubleValue() == 1.0);
      }
   }
   
   //////////////////////////////////////////////////////////////////////////////
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }
   
   ///////////////////////////////////////////////////////////////////////////////
//   private double getICP()
//   {
//      //Calculation
//      comVelX = robot.getBodyVelocityX();
//      comPosX = robot.getBodyPositionX();
//      double omega0 = Math.sqrt(9.81 / comPosZ);
//      
//      FramePoint2d capturePoint = new FramePoint2d();
//      FramePoint2d centerOfMassInWorld = new FramePoint2d(ReferenceFrame.getWorldFrame(), comPosX, 0.0);
//      FrameVector2d centerOfMassVelocityInWorld = new FrameVector2d(ReferenceFrame.getWorldFrame(), comVelX, 0.0);
//      CapturePointCalculator.computeCapturePoint(capturePoint, centerOfMassInWorld, centerOfMassVelocityInWorld, omega0);
//      icpPos = capturePoint.getX();
//      
//      //Visualization
//      icpVisualizer.setLocation(icpPos, 0.0, 0.0);
//      
//      return icpPos;
//   }
}
