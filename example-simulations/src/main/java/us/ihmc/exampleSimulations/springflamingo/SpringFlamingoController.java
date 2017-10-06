package us.ihmc.exampleSimulations.springflamingo;

import java.awt.Container;

import javax.swing.BoxLayout;
import javax.swing.JFrame;

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
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>Title: SpringFlamingoController</p>
 *
 * <p>Description: Simple balistic walking controller for the SpringFlamingo simulation model.
 * Controls body pitch through hip torque, velocity through ankle torque. </p>
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
   private final DoubleParameter stand_gain = new DoubleParameter("stand_gain", "Gain for torquing the support ankle based on an error in desired x", registry, 0.0);
   private final DoubleParameter x_d = new DoubleParameter("x_d", "Desired x location. Controlled with stand_gain", registry, 0.0);

   private final DoubleParameter v_nom = new DoubleParameter("v_nom", "Nominal velocity", registry, -0.4);
   private final DoubleParameter vtp_gain = new DoubleParameter("vtp_gain", "Gain on velocity error to support ankle torque.", registry, 0.0);
   private final DoubleParameter desiredBodyPitch = new DoubleParameter("t_d", "Desired body pitch", registry, 0.0);
   private final DoubleParameter t_gain = new DoubleParameter("t_gain", "Gain on Pitch angle to hip torque", registry, 100.0);
   private final DoubleParameter t_damp = new DoubleParameter("t_damp", "Hip damping", registry, 20.0);

   private final DoubleParameter knee_d = new DoubleParameter("knee_d", registry, 0.0);
   private final DoubleParameter knee_gain = new DoubleParameter("knee_gain", registry, 30.0);
   private final DoubleParameter knee_damp = new DoubleParameter("knee_damp", registry, 10.0);
   private final DoubleParameter hip_d = new DoubleParameter("hip_d", registry, 0.587059);
   private final DoubleParameter hip_gain = new DoubleParameter("hip_gain", registry, 15.9804);
   private final DoubleParameter hip_damp = new DoubleParameter("hip_damp", registry, 2.01177);
   private final DoubleParameter hip_hold = new DoubleParameter("hip_hold", registry, 0.358627);
   private final DoubleParameter swing_gain_knee = new DoubleParameter("swing_gain_knee", registry, 1.81176);
   private final DoubleParameter swing_damp_knee = new DoubleParameter("swing_damp_knee", registry, 0.403529);
   private final DoubleParameter force_thresh = new DoubleParameter("force_thresh", registry, 13.6078);

   private final DoubleParameter min_support_time = new DoubleParameter("min_support_time", registry, 0.2);
   private final DoubleParameter swing_time = new DoubleParameter("swing_time", registry, 0.418039);
   private final DoubleParameter toe_off_time = new DoubleParameter("toe_off_time", registry, 0.05);
   private final DoubleParameter ankle_d = new DoubleParameter("ankle_d", "Swing ankle desired pitch", registry, 0.0);
   private final DoubleParameter ankle_gain = new DoubleParameter("ankle_gain", "Gain on ankle ankle to ankle torque", registry, 4.0);
   private final DoubleParameter ankle_damp = new DoubleParameter("ankle_damp", "Gain on ankle velocity to ankle torque", registry, 1.0);

   private final DoubleParameter ankle_limit_set = new DoubleParameter("ankle_limit_set", registry, 0.0);
   private final DoubleParameter ankle_limit_gain = new DoubleParameter("ankle_limit_gain", registry, 250.98);

   private final DoubleParameter push_set = new DoubleParameter("push_set", "Ankle angle to toe off to", registry, -0.236471);
   private final DoubleParameter push_gain = new DoubleParameter("push_gain", "Gain on ankle angle to ankle torque when toeing off", registry, 9.56078);
   private final DoubleParameter push_damp = new DoubleParameter("push_damp", "Damping on ankle angle to ankle torque when toeing off", registry, 0.0);

   private final DoubleParameter knee_collapse = new DoubleParameter("knee_collapse", registry, 0.0);

   // Active and Passive Torques:
   private final YoDouble act_lh = new YoDouble("act_lh", registry);
   private final YoDouble act_lk = new YoDouble("act_lk", registry);
   private final YoDouble act_la = new YoDouble("act_la", registry);
   private final YoDouble pas_la = new YoDouble("pas_la", registry);
   private final YoDouble act_rh = new YoDouble("act_rh", registry);
   private final YoDouble act_rk = new YoDouble("act_rk", registry);
   private final YoDouble act_ra = new YoDouble("act_ra", registry);
   private final YoDouble pas_ra = new YoDouble("pas_ra", registry);
   private final YoDouble pas_lh = new YoDouble("pas_lh", registry);
   private final YoDouble pas_lk = new YoDouble("pas_lk", registry);
   private final YoDouble pas_rh = new YoDouble("pas_rh", registry);
   private final YoDouble pas_rk = new YoDouble("pas_rk", registry);
   private final YoDouble max_hip_torque = new YoDouble("max_hip_torque", registry);

   private final SideDependentList<YoDouble> actHip = new SideDependentList<YoDouble>(act_lh, act_rh);
   private final SideDependentList<YoDouble> actKnee = new SideDependentList<YoDouble>(act_lk, act_rk);
   private final SideDependentList<YoDouble> actAnkle = new SideDependentList<YoDouble>(act_la, act_ra);
   private final SideDependentList<YoDouble> pasHip = new SideDependentList<YoDouble>(pas_lh, pas_rh);
   private final SideDependentList<YoDouble> pasKnee = new SideDependentList<YoDouble>(pas_lk, pas_rk);
   private final SideDependentList<YoDouble> pasAnkle = new SideDependentList<YoDouble>(pas_la, pas_ra);
   private final SideDependentList<YoDouble> tauHip;
   private final SideDependentList<YoDouble> tauKnee;
   private final SideDependentList<YoDouble> tauAnkle;

   private final SideDependentList<YoDouble> qAnkle;
   private final SideDependentList<YoDouble> qHip;
   private final SideDependentList<YoDouble> qKnee;

   private final SideDependentList<YoDouble> qdAnkle;
   private final SideDependentList<YoDouble> qdHip;
   private final SideDependentList<YoDouble> qdKnee;

   private final SideDependentList<YoDouble> gcHeel_fs;
   private final SideDependentList<YoDouble> gcHeel_fz;
   private final SideDependentList<YoDouble> gcHeel_x;
   private final SideDependentList<YoDouble> gcToe_fs;
   private final SideDependentList<YoDouble> gcToe_fz;

   // Computed State Variables:
   private final YoDouble vel = new YoDouble("vel", "Actual Velocity", registry);
   private final YoDouble left_heel = new YoDouble("left_heel", "Orientation of the left foot", registry);
   private final YoDouble right_heel = new YoDouble("right_heel", "Orientation of the right foot", registry);

   private final YoDouble left_force = new YoDouble("left_force", "Total vertical ground reaction force on the left foot", registry);
   private final YoDouble left_cop = new YoDouble("left_cop", "Location of the left foots Center of Pressure", registry);
   private final YoDouble right_force = new YoDouble("right_force", "Total vertical ground reaction force on the left foot", registry);
   private final YoDouble right_cop = new YoDouble("right_cop", "Location of the right foots Center of Pressure", registry);

   private final YoDouble left_hip_set = new YoDouble("left_hip_set", registry);
   private final YoDouble right_hip_set = new YoDouble("right_hip_set", registry);

   private final SideDependentList<YoDouble> force = new SideDependentList<YoDouble>(left_force, right_force);
   private final SideDependentList<YoDouble> cops = new SideDependentList<YoDouble>(left_cop, right_cop);
   private final SideDependentList<YoDouble> heel = new SideDependentList<YoDouble>(left_heel, right_heel);
   private final SideDependentList<YoDouble> hipSet = new SideDependentList<YoDouble>(left_hip_set, right_hip_set);

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

      qAnkle = new SideDependentList<YoDouble>(robot.q_la, robot.q_ra);
      qHip = new SideDependentList<YoDouble>(robot.q_lh, robot.q_rh);
      qKnee = new SideDependentList<YoDouble>(robot.q_lk, robot.q_rk);
      qdAnkle = new SideDependentList<YoDouble>(robot.qd_la, robot.qd_ra);
      qdHip = new SideDependentList<YoDouble>(robot.qd_lh, robot.qd_rh);
      qdKnee = new SideDependentList<YoDouble>(robot.qd_lk, robot.qd_rk);
      gcHeel_fs = new SideDependentList<YoDouble>(robot.gc_lheel_fs, robot.gc_rheel_fs);
      gcHeel_fz = new SideDependentList<YoDouble>(robot.gc_lheel_fz, robot.gc_rheel_fz);
      gcHeel_x = new SideDependentList<YoDouble>(robot.gc_lheel_x, robot.gc_rheel_x);
      gcToe_fs = new SideDependentList<YoDouble>(robot.gc_ltoe_fs, robot.gc_rtoe_fs);
      gcToe_fz = new SideDependentList<YoDouble>(robot.gc_ltoe_fz, robot.gc_rtoe_fz);

      tauHip = new SideDependentList<YoDouble>(robot.tau_lh, robot.tau_rh);
      tauKnee = new SideDependentList<YoDouble>(robot.tau_lk, robot.tau_rk);
      tauAnkle = new SideDependentList<YoDouble>(robot.tau_la, robot.tau_ra);

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

      left_force.set(0.0);
      left_cop.set(0.5);
      right_force.set(131.83);
      right_cop.set(0.557913);
      left_hip_set.set(0.358627);

      left_heel.set(-0.392589);
      right_hip_set.set(0.358627);

      right_heel.set(-0.231515);
      max_hip_torque.set(0.0);
      
      
      DefaultParameterReader defaultParameterReader = new DefaultParameterReader();
      defaultParameterReader.readParametersInRegistry(registry);

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
       if (pos > ankle_limit_set.getValue())
         return (ankle_limit_gain.getValue() * (ankle_limit_set.getValue() - pos));
       else
          return (0.0);
   }   
   
   private double toe_off_ankle_torques(double pos, double vel)
   {
      return (push_gain.getValue() * (push_set.getValue() - pos) - push_damp.getValue() * vel);
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
            actHip.get(robotSide).set(-t_gain.getValue() * (desiredBodyPitch.getValue() - robot.q_pitch.getDoubleValue())
                  + t_damp.getValue() * robot.qd_pitch.getDoubleValue());

         // Keep knee straight
         actKnee.get(robotSide).set(knee_gain.getValue() * (knee_d.getValue() - qKnee.get(robotSide).getDoubleValue())
               - knee_damp.getValue() * qdKnee.get(robotSide).getValue());

         // Use ankle to servo speed, position
         actAnkle.get(robotSide).set(-stand_gain.getValue() * (x_d.getValue() - robot.q_x.getDoubleValue())
               - vtp_gain.getValue() * (v_nom.getValue() - robot.qd_x.getDoubleValue()));

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
         actHip.get(robotSide).set(-t_gain.getValue() * (desiredBodyPitch.getValue() - robot.q_pitch.getDoubleValue())
               + t_damp.getValue() * robot.qd_pitch.getDoubleValue());

         // Keep knee straight
         actKnee.get(robotSide).set(knee_gain.getValue() * (knee_d.getValue() - qKnee.get(robotSide).getDoubleValue())
               - knee_damp.getValue() * qdKnee.get(robotSide).getDoubleValue());

         // Use ankle to servo speed, position
         actAnkle.get(robotSide).set(-stand_gain.getValue() * (x_d.getValue() - robot.q_x.getDoubleValue())
               - vtp_gain.getValue() * (v_nom.getValue() - robot.qd_x.getDoubleValue()));

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
         hipSet.get(robotSide).set(hip_d.getValue());
         actHip.get(robotSide).set(hip_gain.getValue() * (hipSet.get(robotSide).getDoubleValue() - qHip.get(robotSide).getDoubleValue())
               - hip_damp.getValue() * qdHip.get(robotSide).getDoubleValue());

         // Damp the knee
         actKnee.get(robotSide).set(-swing_damp_knee.getValue() * qdKnee.get(robotSide).getDoubleValue());

         // act_lk.val = 0.0;

         // Damp and Toggle the knee:
         actKnee.get(robotSide).set(actKnee.get(robotSide).getDoubleValue() + knee_collapse.getValue());

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
            actAnkle.get(robotSide).set(-ankle_gain.getValue() * (ankle_d.getValue() - heel.get(robotSide).getDoubleValue())
                  - ankle_damp.getValue() * qdAnkle.get(robotSide).getDoubleValue());
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
         hipSet.get(robotSide).set(hip_hold.getValue());
         actHip.get(robotSide).set(hip_gain.getValue() * (hipSet.get(robotSide).getDoubleValue() - qHip.get(robotSide).getDoubleValue())
               - hip_damp.getValue() * qdHip.get(robotSide).getDoubleValue());

         // Keep knee straight
         actKnee.get(robotSide).set(swing_gain_knee.getValue() * (knee_d.getValue() - qKnee.get(robotSide).getDoubleValue())
               - swing_damp_knee.getValue() * qdKnee.get(robotSide).getDoubleValue());

         // Servo ankle level to the ground
         heel.get(robotSide).set(-robot.q_pitch.getDoubleValue() - qHip.get(robotSide).getDoubleValue() - qKnee.get(robotSide).getDoubleValue() - qAnkle.get(robotSide).getDoubleValue());
         actAnkle.get(robotSide).set(-ankle_gain.getValue() * (ankle_d.getValue() - heel.get(robotSide).getDoubleValue())
               - ankle_damp.getValue() * qdAnkle.get(robotSide).getDoubleValue());
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
         return (gcHeel_fz.get(robotSide).getDoubleValue() < force_thresh.getValue())
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
         return (force.get(robotSide).getDoubleValue() < force_thresh.getValue());
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
