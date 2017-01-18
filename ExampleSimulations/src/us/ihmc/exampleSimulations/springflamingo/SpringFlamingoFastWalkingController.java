package us.ihmc.exampleSimulations.springflamingo;

import java.awt.Container;

import javax.swing.BoxLayout;
import javax.swing.JFrame;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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
 * <p>Description: Simple FastWalking controller for the SpringFlamingo simulation model.
 * Adapted from controller that was used in Jerry Pratt PhD Thesis.
 * Walk fast exploiting natural dynamics including ballistic swing Algorithm.
 *  Swing hip by tracking a minjerk trajectory. </p>
 *
 * <p>Copyright: Copyright (c) 2001</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SpringFlamingoFastWalkingController implements RobotController
{
   
   private final boolean CREATE_STATE_MACHINE_WINDOW = true;

   private final double CONTROL_DT = 0.001;

   private final YoVariableRegistry registry = new YoVariableRegistry("FastWalkingController");

   private final double LPF_DOWN_TAU = (1.0 - 10.0 * CONTROL_DT);

   private StateMachine leftStateMachine, rightStateMachine;

   // Control Parameters:
   private final DoubleYoVariable v_nom = new DoubleYoVariable("v_nom", "Desired walking velocity.", registry);
   private final DoubleYoVariable x_d = new DoubleYoVariable("x_d", "Desired x location when standing and balancing.", registry);
   private final DoubleYoVariable t_d = new DoubleYoVariable("t_d", "Desired body pitch.", registry);

   private final DoubleYoVariable vtp_gain = new DoubleYoVariable("vtp_gain", "Gain from velocity error to change in virtual toe point.", registry);
   private final DoubleYoVariable t_gain = new DoubleYoVariable("t_gain", "Proportional gain for body pitch controller.", registry);
   private final DoubleYoVariable t_damp = new DoubleYoVariable("t_damp", "Damping for body pitch controller.", registry);

// private final YoVariable s_d2s = new YoVariable("s_d2s", "Distance to new front support foot before transition to it from double support", registry);
   private final DoubleYoVariable s_d2s_trail = new DoubleYoVariable("s_d2s_trail",
                                             "If distance from traling leg is this much, then transition to front leg from double support.", registry);
   private final DoubleYoVariable supportTransitionGain = new DoubleYoVariable("supportTransitionGain",
                                                       "Gain from velocity error to distance from new support leg before transition to single support.",
                                                       registry);
   private final DoubleYoVariable captureRatio = new DoubleYoVariable("captureRatio", "Ratio from velocity to support transition for capture point.", registry);

   private final DoubleYoVariable toeOffTorque = new DoubleYoVariable("toeOffTorque", "Toe off torque", registry);
   private final DoubleYoVariable toeOffGain = new DoubleYoVariable("toeOffGain", "Gain from velocity to toe off torque", registry);
   private final DoubleYoVariable toeOffOffset = new DoubleYoVariable("toeOffOffset", "Toe off torque offset.", registry);
   private final DoubleYoVariable maxToeOffTorque = new DoubleYoVariable("maxToeOffTorque", "Maximum toe off torque.", registry);

   private final DoubleYoVariable couple_tau_percent = new DoubleYoVariable("couple_tau_percent", "", registry);
   private final DoubleYoVariable foot_switch_force = new DoubleYoVariable("foot_switch_force", "", registry);

   private final DoubleYoVariable swingTimeMultiplier = new DoubleYoVariable("swingTimeMultiplier", "Multiplier to slow down swing when robot is going slowly.", registry);
   private final DoubleYoVariable tot_swing_time = new DoubleYoVariable("tot_swing_time", "Total swing time.", registry);
   private final DoubleYoVariable hip_swing_time = new DoubleYoVariable("hip_swing_time", "Time to swing hip before straightening.", registry);
   private final DoubleYoVariable straighten_delay = new DoubleYoVariable("straighten_delay", "Time to collapse knee.", registry);


   private final DoubleYoVariable swing_damp_knee1 = new DoubleYoVariable("swing_damp_knee1", "", registry);
   private final DoubleYoVariable sw_force_thresh = new DoubleYoVariable("sw_force_thresh", "", registry);


   private final DoubleYoVariable hip_d = new DoubleYoVariable("hip_d", "Desired hip angle for initial swing.", registry);
   private final DoubleYoVariable hip_hold = new DoubleYoVariable("hip_hold", "Desired hip angle to hold for straightening.", registry);
   private final DoubleYoVariable hip_down = new DoubleYoVariable("hip_down", "", registry);

   private final DoubleYoVariable hip_down_torque = new DoubleYoVariable("hip_down_torque", "", registry);

   private final DoubleYoVariable sh_bash_pos = new DoubleYoVariable("sh_bash_pos", "", registry);
   private final DoubleYoVariable sh_bash_vel = new DoubleYoVariable("sh_bash_vel", "", registry);

   private final DoubleYoVariable knee_damp = new DoubleYoVariable("knee_damp", "", registry);
   private final DoubleYoVariable knee_d = new DoubleYoVariable("knee_d", "", registry);
   private final DoubleYoVariable knee_gain = new DoubleYoVariable("knee_gain", "", registry);
//   private final YoVariable stand_gain = new YoVariable("stand_gain", "", registry);

   private final DoubleYoVariable heel_thresh = new DoubleYoVariable("heel_thresh", "", registry);

   private final DoubleYoVariable knee_straight_torque = new DoubleYoVariable("knee_straight_torque", "", registry);
   private final DoubleYoVariable pitch_gain = new DoubleYoVariable("pitch_gain", "", registry);
   private final DoubleYoVariable doub_time = new DoubleYoVariable("doub_time", "", registry);

   private final DoubleYoVariable toff_knee_gain = new DoubleYoVariable("toff_knee_gain", "", registry);
   private final DoubleYoVariable toff_knee_d = new DoubleYoVariable("toff_knee_d", "", registry);

   private final DoubleYoVariable ff_z = new DoubleYoVariable("ff_z", "", registry);

   private final DoubleYoVariable swing_damp_knee = new DoubleYoVariable("swing_damp_knee", "", registry);
   private final DoubleYoVariable swing_gain_knee = new DoubleYoVariable("swing_gain_knee", "", registry);

   private final DoubleYoVariable f_add = new DoubleYoVariable("f_add", "", registry);
   private final DoubleYoVariable f_mul = new DoubleYoVariable("f_mul", "", registry);
   private final DoubleYoVariable f_min = new DoubleYoVariable("f_min", "", registry);
   private final DoubleYoVariable f_max = new DoubleYoVariable("f_max", "", registry);

   private final DoubleYoVariable lambda1 = new DoubleYoVariable("lambda1", "", registry);
   private final DoubleYoVariable lambda2 = new DoubleYoVariable("lambda2", "", registry);
//   private final YoVariable lambda3 = new YoVariable("lambda3", "", registry);
//   private final YoVariable lambda4 = new YoVariable("lambda4", "", registry);
//   private final YoVariable lambda5 = new YoVariable("lambda5", "", registry);

   private final DoubleYoVariable a1 = new DoubleYoVariable("a1", "", registry);
   private final DoubleYoVariable a2 = new DoubleYoVariable("a2", "", registry);
   private final DoubleYoVariable a3 = new DoubleYoVariable("a3", "", registry);
   private final DoubleYoVariable a4 = new DoubleYoVariable("a4", "", registry);
   private final DoubleYoVariable a5 = new DoubleYoVariable("a5", "", registry);

   private final DoubleYoVariable kd1 = new DoubleYoVariable("kd1", "", registry);
   private final DoubleYoVariable kd2 = new DoubleYoVariable("kd2", "", registry);

   private final DoubleYoVariable ankle_limit_set = new DoubleYoVariable("ankle_limit_set", "", registry);
   private final DoubleYoVariable ankle_limit_gain = new DoubleYoVariable("ankle_limit_gain", "", registry);
   private final DoubleYoVariable push_gain = new DoubleYoVariable("push_gain", "", registry);
   private final DoubleYoVariable push_set = new DoubleYoVariable("push_set", "", registry);
   private final DoubleYoVariable push_damp = new DoubleYoVariable("push_damp", "", registry);

   // Computed variables:
   private final DoubleYoVariable supportTransitionDistance = new DoubleYoVariable("supportTransitionDistance",
                                                           "Total distance from new support leg before transition to single support.", registry);

   private final DoubleYoVariable swingExtraTime = new DoubleYoVariable("swingExtraTime", "Extra time to swing when going slowly.", registry);

   private final DoubleYoVariable thigh_d = new DoubleYoVariable("thigh_d", "", registry);
   private final DoubleYoVariable thighd_d = new DoubleYoVariable("thighd_d", "", registry);
   private final DoubleYoVariable thighdd_d = new DoubleYoVariable("thighdd_d", "", registry);
   private final DoubleYoVariable shin_d = new DoubleYoVariable("shin_d", "", registry);
   private final DoubleYoVariable shind_d = new DoubleYoVariable("shind_d", "", registry);
   private final DoubleYoVariable shindd_d = new DoubleYoVariable("shindd_d", "", registry);

   private final DoubleYoVariable lthigh_d = new DoubleYoVariable("lthigh_d", "", registry);
   private final DoubleYoVariable lthighd_d = new DoubleYoVariable("lthighd_d", "", registry);
   private final DoubleYoVariable lthighdd_d = new DoubleYoVariable("lthighdd_d", "", registry);

   private final DoubleYoVariable lshin_d = new DoubleYoVariable("lshin_d", "", registry);
   private final DoubleYoVariable lshind_d = new DoubleYoVariable("lshind_d", "", registry);
   private final DoubleYoVariable lshindd_d = new DoubleYoVariable("lshindd_d", "", registry);

   private final DoubleYoVariable rthigh_d = new DoubleYoVariable("rthigh_d", "", registry);
   private final DoubleYoVariable rthighd_d = new DoubleYoVariable("rthighd_d", "", registry);
   private final DoubleYoVariable rthighdd_d = new DoubleYoVariable("rthighdd_d", "", registry);

   private final DoubleYoVariable rshin_d = new DoubleYoVariable("rshin_d", "", registry);
   private final DoubleYoVariable rshind_d = new DoubleYoVariable("rshind_d", "", registry);
   private final DoubleYoVariable rshindd_d = new DoubleYoVariable("rshindd_d", "", registry);

   private final DoubleYoVariable qdd_1r = new DoubleYoVariable("qdd_1r", "", registry);
   private final DoubleYoVariable qdd_2r = new DoubleYoVariable("qdd_2r", "", registry);

   private final DoubleYoVariable qd_1r = new DoubleYoVariable("qd_1r", "", registry);
   private final DoubleYoVariable qd_2r = new DoubleYoVariable("qd_2r", "", registry);

   private final DoubleYoVariable s1 = new DoubleYoVariable("s1", "", registry);
   private final DoubleYoVariable s2 = new DoubleYoVariable("s2", "", registry);

   private final DoubleYoVariable Y11 = new DoubleYoVariable("Y11", "", registry);
   private final DoubleYoVariable Y12 = new DoubleYoVariable("Y12", "", registry);
   private final DoubleYoVariable Y13 = new DoubleYoVariable("Y13", "", registry);
   private final DoubleYoVariable Y14 = new DoubleYoVariable("Y14", "", registry);
   private final DoubleYoVariable Y15 = new DoubleYoVariable("Y15", "", registry);

   private final DoubleYoVariable Y21 = new DoubleYoVariable("Y21", "", registry);
   private final DoubleYoVariable Y22 = new DoubleYoVariable("Y22", "", registry);
   private final DoubleYoVariable Y23 = new DoubleYoVariable("Y23", "", registry);
   private final DoubleYoVariable Y24 = new DoubleYoVariable("Y24", "", registry);
   private final DoubleYoVariable Y25 = new DoubleYoVariable("Y25", "", registry);

   private final DoubleYoVariable transfer_ratio = new DoubleYoVariable("transfer_ratio", "", registry);
   private final DoubleYoVariable leftVirtualToePoint = new DoubleYoVariable("leftVirtualToePoint",
                                                     "The virtual toe point (i.e. desired Center of Pressure) for the left leg.", registry);
   private final DoubleYoVariable rightVirtualToePoint = new DoubleYoVariable("rightVirtualToePoint",
                                                      "The virtual toe point (i.e. desired Center of Pressure) for the right leg.", registry);

   private final DoubleYoVariable fxl = new DoubleYoVariable("fxl", "", registry);
   private final DoubleYoVariable fzl = new DoubleYoVariable("fzl", "", registry);
   private final DoubleYoVariable ftl = new DoubleYoVariable("ftl", "", registry);

   private final DoubleYoVariable fxr = new DoubleYoVariable("fxr", "", registry);
   private final DoubleYoVariable fzr = new DoubleYoVariable("fzr", "", registry);
   private final DoubleYoVariable ftr = new DoubleYoVariable("ftr", "", registry);

   // Kinematic Computed Variables:
   private final DoubleYoVariable q_lthigh = new DoubleYoVariable("q_lthigh", "", registry);
   private final DoubleYoVariable qd_lthigh = new DoubleYoVariable("qd_lthigh", "", registry);

   private final DoubleYoVariable q_lshin = new DoubleYoVariable("q_lshin", "", registry);
   private final DoubleYoVariable qd_lshin = new DoubleYoVariable("qd_lshin", "", registry);
   private final DoubleYoVariable q_rthigh = new DoubleYoVariable("q_rthigh", "", registry);

   private final DoubleYoVariable qd_rthigh = new DoubleYoVariable("qd_rthigh", "", registry);

   private final DoubleYoVariable q_rshin = new DoubleYoVariable("q_rshin", "", registry);
   private final DoubleYoVariable qd_rshin = new DoubleYoVariable("qd_rshin", "", registry);

   private final DoubleYoVariable x_lf = new DoubleYoVariable("x_lf", "", registry);
   private final DoubleYoVariable z_lf = new DoubleYoVariable("z_lf", "", registry);
   private final DoubleYoVariable xd_lf = new DoubleYoVariable("xd_lf", "", registry);
   private final DoubleYoVariable zd_lf = new DoubleYoVariable("zd_lf", "", registry);

   private final DoubleYoVariable x_rf = new DoubleYoVariable("x_rf", "", registry);
   private final DoubleYoVariable z_rf = new DoubleYoVariable("z_rf", "", registry);
   private final DoubleYoVariable xd_rf = new DoubleYoVariable("xd_rf", "", registry);
   private final DoubleYoVariable zd_rf = new DoubleYoVariable("zd_rf", "", registry);

   private final DoubleYoVariable lshin_start = new DoubleYoVariable("lshin_start", "", registry);
   private final DoubleYoVariable lshind_start = new DoubleYoVariable("lshind_start", "", registry);

   //
   private final DoubleYoVariable down_lh = new DoubleYoVariable("down_lh", "", registry);
   private final DoubleYoVariable down_rh = new DoubleYoVariable("down_rh", "", registry);
   SideDependentList<DoubleYoVariable> hipDownTorques = new SideDependentList<DoubleYoVariable>(down_lh, down_rh);

   private final DoubleYoVariable knee_lock_torque = new DoubleYoVariable("knee_lock_torque", "", registry);

   private final DoubleYoVariable left_heel = new DoubleYoVariable("left_heel", "", registry);
   private final DoubleYoVariable right_heel = new DoubleYoVariable("right_heel", "", registry);
   private final SideDependentList<DoubleYoVariable> heels = new SideDependentList<DoubleYoVariable>(left_heel, right_heel);

   private final DoubleYoVariable max_hip_torque = new DoubleYoVariable("max_hip_torque", "", registry);

//   private final YoVariable left_knee_set = new YoVariable("left_knee_set", "", registry);
//   private final YoVariable lk_set_d = new YoVariable("lk_set_d", "", registry);
//
//   private final YoVariable right_knee_set = new YoVariable("right_knee_set", "", registry);
//   private final YoVariable rk_set_d = new YoVariable("rk_set_d", "", registry);

//   private final SideDependentList<YoVariable> kneeSetPoints = new SideDependentList<YoVariable>(left_knee_set, right_knee_set);
//   private final SideDependentList<YoVariable> kneeSetVelocities = new SideDependentList<YoVariable>(lk_set_d, rk_set_d);

   private final DoubleYoVariable ankle_gain = new DoubleYoVariable("ankle_gain", "", registry);
   private final DoubleYoVariable ankle_desired = new DoubleYoVariable("ankle_desired", "", registry);
   private final DoubleYoVariable ankle_damp = new DoubleYoVariable("ankle_damp", "", registry);

   private final DoubleYoVariable ft_right = new DoubleYoVariable("ft_right", "", registry);

   private final DoubleYoVariable r_swing_flag = new DoubleYoVariable("r_swing_flag", "", registry);
   private final DoubleYoVariable rshin_start = new DoubleYoVariable("rshin_start", "", registry);
   private final DoubleYoVariable rshind_start = new DoubleYoVariable("rshind_start", "", registry);

   private final DoubleYoVariable rthigh_start = new DoubleYoVariable("rthigh_start", "", registry);

// private final YoVariable gc_lheel = new YoVariable("gc_lheel", "", registry);
   private final DoubleYoVariable force_thresh = new DoubleYoVariable("force_thresh", "", registry);
   private final DoubleYoVariable min_support_time = new DoubleYoVariable("min_support_time", "", registry);

   private final DoubleYoVariable gc_ltoe = new DoubleYoVariable("gc_ltoe", "", registry);

   private final DoubleYoVariable ff_hip = new DoubleYoVariable("ff_hip", "", registry);
   private final DoubleYoVariable ff_knee = new DoubleYoVariable("ff_knee", "", registry);
   private final DoubleYoVariable ff_ankle = new DoubleYoVariable("ff_ankle", "", registry);

// private final YoVariable gc_rheel = new YoVariable("gc_rheel", "", registry);
   private final DoubleYoVariable gc_rtoe = new DoubleYoVariable("gc_rtoe", "", registry);

//   private final YoVariable offset = new YoVariable("offset", "", registry);
//   private final YoVariable amp = new YoVariable("amp", "", registry);
//   private final YoVariable freq = new YoVariable("freq", "", registry);

// private final YoVariable left_state = new YoVariable("left_state", "", States.values(), registry);
   private final DoubleYoVariable ft_left = new DoubleYoVariable("ft_left", "", registry);

   private final DoubleYoVariable la_int = new DoubleYoVariable("la_int", "", registry);
   private final DoubleYoVariable ra_int = new DoubleYoVariable("ra_int", "", registry);

   private final DoubleYoVariable l_swing_flag = new DoubleYoVariable("l_swing_flag", "", registry);

   private final DoubleYoVariable lthigh_start = new DoubleYoVariable("lthigh_start", "", registry);

   private final DoubleYoVariable left_force = new DoubleYoVariable("left_force", "", registry);

   // Active Joint Torques:
   private final DoubleYoVariable act_lh = new DoubleYoVariable("act_lh", "", registry);
   private final DoubleYoVariable act_lk = new DoubleYoVariable("act_lk", "", registry);
   private final DoubleYoVariable act_la = new DoubleYoVariable("act_la", "", registry);

   private final DoubleYoVariable act_rh = new DoubleYoVariable("act_rh", "", registry);
   private final DoubleYoVariable act_rk = new DoubleYoVariable("act_rk", "", registry);
   private final DoubleYoVariable act_ra = new DoubleYoVariable("act_ra", "", registry);

   // Passive Joint Torques:
   private final DoubleYoVariable pas_lh = new DoubleYoVariable("pas_lh", "", registry);
   private final DoubleYoVariable pas_lk = new DoubleYoVariable("pas_lk", "", registry);
   private final DoubleYoVariable pas_la = new DoubleYoVariable("pas_la", "", registry);

   private final DoubleYoVariable pas_rh = new DoubleYoVariable("pas_rh", "", registry);
   private final DoubleYoVariable pas_rk = new DoubleYoVariable("pas_rk", "", registry);
   private final DoubleYoVariable pas_ra = new DoubleYoVariable("pas_ra", "", registry);

   private final DoubleYoVariable vel = new DoubleYoVariable("vel", "", registry);
   private final DoubleYoVariable left_cop = new DoubleYoVariable("left_cop", "", registry);
   private final DoubleYoVariable right_force = new DoubleYoVariable("right_force", "", registry);
   private final DoubleYoVariable right_cop = new DoubleYoVariable("right_cop", "", registry);

   private final SideDependentList<DoubleYoVariable> desiredThighAngles = new SideDependentList<DoubleYoVariable>(lthigh_d, rthigh_d);
   private final SideDependentList<DoubleYoVariable> desiredThighVelocities = new SideDependentList<DoubleYoVariable>(lthighd_d, rthighd_d);
   private final SideDependentList<DoubleYoVariable> desiredThighAccelerations = new SideDependentList<DoubleYoVariable>(lthighdd_d, rthighdd_d);

   private final SideDependentList<DoubleYoVariable> desiredShinAngles = new SideDependentList<DoubleYoVariable>(lshin_d, rshin_d);
   private final SideDependentList<DoubleYoVariable> desiredShinVelocities = new SideDependentList<DoubleYoVariable>(lshind_d, rshind_d);
   private final SideDependentList<DoubleYoVariable> desiredShinAccelerations = new SideDependentList<DoubleYoVariable>(lshindd_d, rshindd_d);

   private final SideDependentList<DoubleYoVariable> thighAngles = new SideDependentList<DoubleYoVariable>(q_lthigh, q_rthigh);
   private final SideDependentList<DoubleYoVariable> thighVelocities = new SideDependentList<DoubleYoVariable>(qd_lthigh, qd_rthigh);
   private final SideDependentList<DoubleYoVariable> shinAngles = new SideDependentList<DoubleYoVariable>(q_lshin, q_rshin);
   private final SideDependentList<DoubleYoVariable> shinVelocities = new SideDependentList<DoubleYoVariable>(qd_lshin, qd_rshin);

   private final SideDependentList<DoubleYoVariable> thighStartAngles = new SideDependentList<DoubleYoVariable>(lthigh_start, rthigh_start);

// private final SideDependentList<YoVariable> thighStartVelocities = new SideDependentList<YoVariable>(lthighd_start, rthighd_start);
   private final SideDependentList<DoubleYoVariable> shinStartAngles = new SideDependentList<DoubleYoVariable>(lshin_start, rshin_start);
   private final SideDependentList<DoubleYoVariable> shinStartVelocities = new SideDependentList<DoubleYoVariable>(lshind_start, rshind_start);

   private final SideDependentList<DoubleYoVariable> activeTorqueAtHip = new SideDependentList<DoubleYoVariable>(act_lh, act_rh);
   private final SideDependentList<DoubleYoVariable> activeTorqueAtKnee = new SideDependentList<DoubleYoVariable>(act_lk, act_rk);
   private final SideDependentList<DoubleYoVariable> activeTorqueAtAnkle = new SideDependentList<DoubleYoVariable>(act_la, act_ra);

//   private final SideDependentList<YoVariable> passiveTorqueAtHip = new SideDependentList<YoVariable>(pas_lh, pas_rh);
//   private final SideDependentList<YoVariable> passiveTorqueAtKnee = new SideDependentList<YoVariable>(pas_lk, pas_rk);
   private final SideDependentList<DoubleYoVariable> passiveTorqueAtAnkle = new SideDependentList<DoubleYoVariable>(pas_la, pas_ra);

   private final SpringFlamingoRobot robot;

   private String name;

   public SpringFlamingoFastWalkingController(SpringFlamingoRobot robot, double gravity, String name)
   {
      this.name = name;
      this.robot = robot;
      initControl(gravity);
      setupStateMachines();

      if (CREATE_STATE_MACHINE_WINDOW)
         createStateMachineWindow();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

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
      StateMachinesJPanel leftStateMachinePanel = new StateMachinesJPanel(leftStateMachine, true);
      StateMachinesJPanel rightStateMachinePanel = new StateMachinesJPanel(rightStateMachine, true);

      jFrame.getContentPane().add(leftStateMachinePanel);
      jFrame.getContentPane().add(rightStateMachinePanel);

      jFrame.pack();
      jFrame.setSize(450, 300);
      jFrame.setAlwaysOnTop(true);
      jFrame.setVisible(true);

      // Doing the following will cause redraw when the state changes, but not during replay or rewind:
      leftStateMachine.attachStateChangedListener(leftStateMachinePanel);
      rightStateMachine.attachStateChangedListener(rightStateMachinePanel);

      // Doing this will cause redraw every specified miliseconds:
//      leftStateMachinePanel.createUpdaterThread(250);
//      rightStateMachinePanel.createUpdaterThread(250);
   }

   private void initControl(double gravity)
   {
      robot.initializeForFastWalking(RobotSide.LEFT);
      robot.t.set(0.0);    // 10.00;

      init_walking_params(gravity);

//    setWalkingParametersForMoonGravity();
   }

// void setWalkingParametersForMoonGravity()
// {
//   init_walking_params(SpringFlamingoSimulation.MOON_GRAVITY);
//   // Tuning overwrites:
// }


   void init_walking_params(double gravity)
   {
      if (gravity < 0.0)
         gravity = -gravity;
      double gravityScaling = gravity / 9.81;
      System.out.println("gravityScaling = " + gravityScaling);

      swingTimeMultiplier.set(0.6 * (1.0 - gravityScaling));
      if (swingTimeMultiplier.getDoubleValue() < 0.0)
         swingTimeMultiplier.set(0.0);

      toeOffGain.set(4.0 * gravityScaling);
      toeOffOffset.set(10.0 * gravityScaling);
      maxToeOffTorque.set(15.0 * gravityScaling);

      couple_tau_percent.set(0.8);

      foot_switch_force.set(15.0 * gravityScaling);

      tot_swing_time.set(0.45);    // 0.52;
      hip_swing_time.set(0.3);    // 0.35;
      straighten_delay.set(0.19);


      hip_d.set(0.45);    // 0.4;
      hip_hold.set(0.35);    // 0.28;
      hip_down.set(0.25);    // 0.0;
      hip_down_torque.set(9.0 * gravityScaling);

      sh_bash_vel.set(0.0);
      sh_bash_pos.set(0.15);

//    swing_ang.val = 0.0;

//    ankle_push.val = 0.0;
      x_d.set(0.04);

      v_nom.set(0.8 * Math.sqrt(gravityScaling) + 0.5);

      /* v_nom.set(1.4); */
      /* vtp_gain.set(0.0); */

      vtp_gain.set(0.03 / Math.sqrt(gravityScaling));
      supportTransitionGain.set(0.1 / Math.sqrt(gravityScaling));
      captureRatio.set(0.3 / Math.sqrt(gravityScaling));

      /* pitch_gain.set(0.06); */

//    pitch_gain.val = 0.04;
      pitch_gain.set(0.0);
      ff_z.set(140.0 * gravityScaling);
      t_d.set(-0.04);
      t_gain.set(70.0);
      t_damp.set(12.0);
      knee_d.set(0.2);
      knee_gain.set(0.0);
      knee_damp.set(1.2);
      knee_straight_torque.set(2.0);

//    hip_gain.val = 30.0;
//    hip_damp.val = 2.5;
      max_hip_torque.set(12.0);
      swing_gain_knee.set(2.0);
      swing_damp_knee.set(0.6);
      swing_damp_knee1.set(0.5);

//    knee_collapse.val = 0.4;
      knee_lock_torque.set(2.0);
      min_support_time.set(0.25);
      doub_time.set(0.06);

//    tran_time.val = 0.2;

//    s_d2s.val = 0.04;



      s_d2s_trail.set(0.4);    // 0.24;

      push_set.set(-0.08);
      z_d.set(0.88);
      push_gain.set(0.0);
      push_damp.set(0.0);

//    push_ff.val = 0.0;
      ankle_desired.set(-0.2);
      ankle_desired.set(-0.2);
      ankle_gain.set(6.0);    // 1.5;
      ankle_damp.set(0.1);
      heel_thresh.set(0.18);

      /* f_min.set(-0.12); */
      f_min.set(-0.11);
      f_max.set(0.03);
      f_mul.set(0.8);
      f_add.set(0.0);
      vel.set(0.0452136);
      kd1.set(1.0);
      lambda1.set(16.0);
      kd2.set(0.7);
      lambda2.set(10.0);
      toff_knee_d.set(0.15);
      toff_knee_gain.set(0.0);
      force_thresh.set(20.0);
      sw_force_thresh.set(55.0);    // 75.0;

//    fs_thresh.val = 10.0;
      ankle_limit_set.set(0.0);
      ankle_limit_gain.set(0.0);

//    toe_off_ankle_thresh.val = 0.0;



      a1.set(0.21);
      a2.set(0.11);
      a3.set(0.12);
      a4.set(4.5);
      a5.set(2.8);

   }

   public void doControl()
   {
      fast_walking_state_machine();
   }

   public double passive_ankle_torques(double pos, double vel)
   {
      if (pos > ankle_limit_set.getDoubleValue())
         return (-ankle_limit_gain.getDoubleValue() * (ankle_limit_set.getDoubleValue() - pos) * (ankle_limit_set.getDoubleValue() - pos));
      else
         return (0.0);
   }

   DoubleYoVariable z_d = new DoubleYoVariable("z_d", "", registry);

   private double toe_off_ankle_torques(double anklePosition, double ankleVelocity)
   {
//    *int_error = *int_error + (ls.ankle_push - pos);

      /* return (ls.push_gain * *int_error); */

//    return (push_gain.val * (push_set.val - pos) - push_damp.val * vel);

      // todo: Really?
//    return (push_gain.val * (z_d.val - robot.q_z.val));

      toeOffTorque.set(-toeOffOffset.getDoubleValue() - toeOffGain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue()));
      if (toeOffTorque.getDoubleValue() > 0.0)
         toeOffTorque.set(0.0);
      if (toeOffTorque.getDoubleValue() < -maxToeOffTorque.getDoubleValue())
         toeOffTorque.set(-maxToeOffTorque.getDoubleValue());

      return toeOffTorque.getDoubleValue();
   }

   private void fast_walking_state_machine()
   {
      // Robot happens to walk in negative x direction.  Set vel positive just
      // so it makes intuitive sense.

      calculate_kinematics();

//    v_nom.val = offset.val + amp.val/2.0 + amp.val/2.0 * Math.sin(2.0 * Math.PI * freq.val * robot.t.val);

      vel.set(-robot.qd_x.getDoubleValue());

      // Calculate forces on the feet

      left_force.set(robot.gc_lheel_fz.getDoubleValue() + robot.gc_ltoe_fz.getDoubleValue());
      if (left_force.getDoubleValue() > 5.0)
         left_cop.set(robot.gc_ltoe_fz.getDoubleValue() / left_force.getDoubleValue());
      else
         left_cop.set(0.5);

      right_force.set(robot.gc_rheel_fz.getDoubleValue() + robot.gc_rtoe_fz.getDoubleValue());
      if (right_force.getDoubleValue() > 5.0)
         right_cop.set(robot.gc_rtoe_fz.getDoubleValue() / right_force.getDoubleValue());
      else
         right_cop.set(0.5);

      // Actions in Each State

      leftStateMachine.doAction();
      rightStateMachine.doAction();

      // Communication between legs for now...



      /* Communication between legs for now... */

      /*
       *  Subtract swing hip torque from the stance leg so less disturbance
       *  on the body
       */
      if ((leftStateMachine.getCurrentStateEnum() == WalkingState.SWING))    // || (leftStateMachine.getCurrentStateEnum() == States.STRAIGHTEN))
      {
         single_support_vtp(RobotSide.RIGHT, 0.0, act_lh.getDoubleValue(), ff_hip, ff_knee, ff_ankle);

         /*
          * act_rh.set(act_rh.getDoubleValue() - act_lh.getDoubleValue());
          *      act_rk.set(act_rk.getDoubleValue() - act_lh.getDoubleValue());
          *      act_ra.set(act_ra.getDoubleValue() - act_lh.getDoubleValue());
          */

         act_rh.set(act_rh.getDoubleValue() + couple_tau_percent.getDoubleValue() * ff_hip.getDoubleValue());

         /* if (ff_knee.getDoubleValue() > 0.0) */
         act_rk.set(act_rk.getDoubleValue() + couple_tau_percent.getDoubleValue() * ff_knee.getDoubleValue());
         act_ra.set(act_ra.getDoubleValue() + couple_tau_percent.getDoubleValue() * ff_ankle.getDoubleValue());
      }

      if ((rightStateMachine.getCurrentStateEnum() == WalkingState.SWING))    // || (rightStateMachine.getCurrentStateEnum() ==  States.STRAIGHTEN))
      {
         single_support_vtp(RobotSide.LEFT, 0.0, act_rh.getDoubleValue(), ff_hip, ff_knee, ff_ankle);

         /*
          * act_lh.set(act_lh.getDoubleValue() - act_rh.getDoubleValue());
          *      act_lk.set(act_lk.getDoubleValue() - act_rh.getDoubleValue());
          *      act_la.set(act_la.getDoubleValue() - act_rh.getDoubleValue());
          */

         act_lh.set(act_lh.getDoubleValue() + couple_tau_percent.getDoubleValue() * ff_hip.getDoubleValue());

         /* if (ff_knee.getDoubleValue() > 0.0) */
         act_lk.set(act_lk.getDoubleValue() + couple_tau_percent.getDoubleValue() * ff_knee.getDoubleValue());
         act_la.set(act_la.getDoubleValue() + couple_tau_percent.getDoubleValue() * ff_ankle.getDoubleValue());
      }

//    // Subtract swing hip torque from the stance leg so less disturbance
//    //   on the body
//
//    if (leftStateMachine.isCurrentState(States.SWING))
//    {
//      act_rh.val = act_rh.val - act_lh.val;
//    }
//
//if   (rightStateMachine.isCurrentState(States.SWING))
//{
  
//    act_lh.val = act_lh.val - act_rh.val;
//}
  

// T  ransition Conditions:

      leftStateMachine.checkTransitionConditions();
      rightStateMachine.checkTransitionConditions();

      // Torques at the joints:

      robot.tau_lh.set(act_lh.getDoubleValue() + pas_lh.getDoubleValue());
      robot.tau_lk.set(act_lk.getDoubleValue() + pas_lk.getDoubleValue());
      robot.tau_la.set(act_la.getDoubleValue() + pas_la.getDoubleValue());

      robot.tau_rh.set(act_rh.getDoubleValue() + pas_rh.getDoubleValue());
      robot.tau_rk.set(act_rk.getDoubleValue() + pas_rk.getDoubleValue());
      robot.tau_ra.set(act_ra.getDoubleValue() + pas_ra.getDoubleValue());
   }

   private void body_kinematics_ankles(double foot_x, double foot_z, double ankle, double knee, double hip, DoubleYoVariable body_x, DoubleYoVariable body_z,
           double body_theta, double ankle_dtheta, double knee_dtheta, double hip_dtheta, DoubleYoVariable body_dx, DoubleYoVariable body_dz, double body_dtheta)

   {
      double toe = -body_theta - ankle - knee - hip;
      double toe_dtheta = -body_dtheta - ankle_dtheta - knee_dtheta - hip_dtheta;

      body_x.set(foot_x - SpringFlamingoRobot.FOOT_FORWARD + SpringFlamingoRobot.FOOT_FORWARD * Math.cos(toe) 
      			   - SpringFlamingoRobot.FOOT_H * Math.sin(toe) - SpringFlamingoRobot.LOWER_LINK_LENGTH * Math.sin(toe + ankle) 
      			   - SpringFlamingoRobot.UPPER_LINK_LENGTH * Math.sin(toe + ankle + knee));

      /*
       *  +++JEP Hack:  If toe is up (toe>0.0),
       *  then calculate the height from the heel rather than
       *  the toe
       */

      if (toe > 0.0)
         body_z.set(foot_z - SpringFlamingoRobot.FOOT_BEHIND * Math.sin(toe) + SpringFlamingoRobot.FOOT_H * Math.cos(toe) 
         			  + SpringFlamingoRobot.LOWER_LINK_LENGTH * Math.cos(toe + ankle)
                      + SpringFlamingoRobot.UPPER_LINK_LENGTH * Math.cos(toe + ankle + knee));
      else
         body_z.set(foot_z - SpringFlamingoRobot.FOOT_FORWARD * Math.sin(toe) + SpringFlamingoRobot.FOOT_H * Math.cos(toe) 
         			  + SpringFlamingoRobot.LOWER_LINK_LENGTH * Math.cos(toe + ankle)
                      + SpringFlamingoRobot.UPPER_LINK_LENGTH * Math.cos(toe + ankle + knee));

      double J3 = -Math.cos(toe + ankle + knee) * SpringFlamingoRobot.UPPER_LINK_LENGTH;
      double J6 = -Math.sin(toe + ankle + knee) * SpringFlamingoRobot.UPPER_LINK_LENGTH;

      double J2 = -Math.cos(toe + ankle) * SpringFlamingoRobot.LOWER_LINK_LENGTH + J3;
      double J5 = -Math.sin(toe + ankle) * SpringFlamingoRobot.LOWER_LINK_LENGTH + J6;

      /*
       *   J1 = ls.lf*Math.sin(toe)-robot.FOOT_H*Math.cos(toe) + J2;
       *    J4 = -ls.lf*Math.cos(toe)-robot.FOOT_H*Math.sin(toe) + J5;
       */

      double J1 = SpringFlamingoRobot.FOOT_FORWARD * Math.sin(toe) - SpringFlamingoRobot.FOOT_H * Math.cos(toe) + J2;
      double J4 = -SpringFlamingoRobot.FOOT_FORWARD * Math.cos(toe) - SpringFlamingoRobot.FOOT_H * Math.sin(toe) + J5;

      body_dx.set(J1 * toe_dtheta + J2 * ankle_dtheta + J3 * knee_dtheta);

      /*
       *  +++JEP Hack:  If toe is up (toe>0.0),
       *  then calculate the height velocity as if the foot is
       *  pushed to be flat (toe = 0.0) and the toe isn't moving.
       */

      if (toe > 0.0)
         body_dz.set(J4 * 0.0 + J5 * ankle_dtheta + J6 * knee_dtheta);
      else
         body_dz.set(J4 * toe_dtheta + J5 * ankle_dtheta + J6 * knee_dtheta);

   }

   void com_kinematics(double foot_x, double foot_z, double ankle, double knee, double hip, DoubleYoVariable body_x, DoubleYoVariable body_z, double body_theta,
                       double ankle_dtheta, double knee_dtheta, double hip_dtheta, DoubleYoVariable body_dx, DoubleYoVariable body_dz, double body_dtheta)
   {
      double toe = -body_theta - ankle - knee - hip;
      double toe_dtheta = -body_dtheta - ankle_dtheta - knee_dtheta - hip_dtheta;

      body_x.set(foot_x - SpringFlamingoRobot.FOOT_FORWARD + SpringFlamingoRobot.FOOT_FORWARD * Math.cos(toe) 
      			   - SpringFlamingoRobot.FOOT_H * Math.sin(toe) - SpringFlamingoRobot.LOWER_LINK_LENGTH * Math.sin(toe + ankle) 
      			   - SpringFlamingoRobot.UPPER_LINK_LENGTH * Math.sin(toe + ankle + knee)
                   - SpringFlamingoRobot.BODY_CG_Z * Math.sin(toe + ankle + knee + hip));

      /*
       *  +++JEP Hack:  If toe is up (toe>0.0),
       *  then calculate the height from the heel rather than
       *  the toe
       */

      if (toe > 0.0)
         body_z.set(foot_z - SpringFlamingoRobot.FOOT_BEHIND * Math.sin(toe) 
         			  + SpringFlamingoRobot.FOOT_H * Math.cos(toe) + SpringFlamingoRobot.LOWER_LINK_LENGTH * Math.cos(toe + ankle)
                      + SpringFlamingoRobot.UPPER_LINK_LENGTH * Math.cos(toe + ankle + knee) 
                      + SpringFlamingoRobot.BODY_CG_Z * Math.cos(toe + ankle + knee + hip));
      else
         body_z.set(foot_z - SpringFlamingoRobot.FOOT_FORWARD * Math.sin(toe) + SpringFlamingoRobot.FOOT_H * Math.cos(toe) 
         			  + SpringFlamingoRobot.LOWER_LINK_LENGTH * Math.cos(toe + ankle)
                      + SpringFlamingoRobot.UPPER_LINK_LENGTH * Math.cos(toe + ankle + knee) 
                      + SpringFlamingoRobot.BODY_CG_Z * Math.cos(toe + ankle + knee + hip));

      double J3 = -Math.cos(toe + ankle + knee) * SpringFlamingoRobot.UPPER_LINK_LENGTH;
      double J6 = -Math.sin(toe + ankle + knee) * SpringFlamingoRobot.UPPER_LINK_LENGTH;

      double J2 = -Math.cos(toe + ankle) * SpringFlamingoRobot.LOWER_LINK_LENGTH + J3;
      double J5 = -Math.sin(toe + ankle) * SpringFlamingoRobot.LOWER_LINK_LENGTH + J6;

      /*
       *   J1 = ls.lf*Math.sin(toe)-robot.FOOT_H*Math.cos(toe) + J2;
       *    J4 = -ls.lf*Math.cos(toe)-robot.FOOT_H*Math.sin(toe) + J5;
       */

      double J1 = SpringFlamingoRobot.FOOT_FORWARD * Math.sin(toe) - SpringFlamingoRobot.FOOT_H * Math.cos(toe) + J2;
      double J4 = -SpringFlamingoRobot.FOOT_FORWARD * Math.cos(toe) - SpringFlamingoRobot.FOOT_H * Math.sin(toe) + J5;

      body_dx.set(J1 * toe_dtheta + J2 * ankle_dtheta + J3 * knee_dtheta + SpringFlamingoRobot.BODY_CG_Z * Math.cos(body_theta) * body_dtheta);

      /*
       *  +++JEP Hack:  If toe is up (toe>0.0),
       *  then calculate the height velocity as if the foot is
       *  pushed to be flat (toe = 0.0) and the toe isn't moving.
       */

      if (toe > 0.0)
         body_dz.set(J4 * 0.0 + J5 * ankle_dtheta + J6 * knee_dtheta - SpringFlamingoRobot.BODY_CG_Z * Math.sin(body_theta) * body_dtheta);
      else
         body_dz.set(J4 * toe_dtheta + J5 * ankle_dtheta + J6 * knee_dtheta - SpringFlamingoRobot.BODY_CG_Z * Math.sin(body_theta) * body_dtheta);

   }

   void calculate_kinematics()
   {
      q_rthigh.set(robot.q_pitch.getDoubleValue() + robot.q_rh.getDoubleValue());
      qd_rthigh.set(robot.qd_pitch.getDoubleValue() + robot.qd_rh.getDoubleValue());

      q_rshin.set(robot.q_pitch.getDoubleValue() + robot.q_rh.getDoubleValue() + robot.q_rk.getDoubleValue());
      qd_rshin.set(robot.qd_pitch.getDoubleValue() + robot.qd_rh.getDoubleValue() + robot.qd_rk.getDoubleValue());

      q_lthigh.set(robot.q_pitch.getDoubleValue() + robot.q_lh.getDoubleValue());
      qd_lthigh.set(robot.qd_pitch.getDoubleValue() + robot.qd_lh.getDoubleValue());

      q_lshin.set(robot.q_pitch.getDoubleValue() + robot.q_lh.getDoubleValue() + robot.q_lk.getDoubleValue());
      qd_lshin.set(robot.qd_pitch.getDoubleValue() + robot.qd_lh.getDoubleValue() + robot.qd_lk.getDoubleValue());

      body_kinematics_ankles(0.0, 0.0, robot.q_la.getDoubleValue(), robot.q_lk.getDoubleValue(), robot.q_lh.getDoubleValue(), x_lf, z_lf, robot.q_pitch.getDoubleValue(), robot.qd_la.getDoubleValue(), robot.qd_lk.getDoubleValue(),
                             robot.qd_lh.getDoubleValue(), xd_lf, zd_lf, robot.qd_pitch.getDoubleValue());

      body_kinematics_ankles(0.0, 0.0, robot.q_ra.getDoubleValue(), robot.q_rk.getDoubleValue(), robot.q_rh.getDoubleValue(), x_rf, z_rf, robot.q_pitch.getDoubleValue(), robot.qd_ra.getDoubleValue(), robot.qd_rk.getDoubleValue(),
                             robot.qd_rh.getDoubleValue(), xd_rf, zd_rf, robot.qd_pitch.getDoubleValue());
   }

   private void setupStateMachines()
   {
      // Create the state machines:
      leftStateMachine = new StateMachine("leftState", "leftSwitchTime", WalkingState.class, robot.t, registry);
      rightStateMachine = new StateMachine("rightState", "rightSwitchTime", WalkingState.class, robot.t, registry);

      // Create the states:

      State leftBalanceState = new BalanceState(WalkingState.BALANCE, RobotSide.LEFT);
      State rightBalanceState = new BalanceState(WalkingState.BALANCE, RobotSide.RIGHT);

      State leftSupportState = new SupportState(WalkingState.SUPPORT, RobotSide.LEFT);
      State rightSupportState = new SupportState(WalkingState.SUPPORT, RobotSide.RIGHT);

      State leftSwingState = new SwingState(WalkingState.SWING, RobotSide.LEFT);
      State rightSwingState = new SwingState(WalkingState.SWING, RobotSide.RIGHT);

//    State leftStraightenState = new StraightenState(States.STRAIGHTEN, RobotSide.LEFT);
//    State rightStraightenState = new StraightenState(States.STRAIGHTEN, RobotSide.RIGHT);

      State leftToeOffState = new ToeOffState(WalkingState.TOE_OFF, RobotSide.LEFT);
      State rightToeOffState = new ToeOffState(WalkingState.TOE_OFF, RobotSide.RIGHT);

      // Create the transition conditions.

      ReadyToStartWalkingCondition readyToStartWalkingLeftSupport = new ReadyToStartWalkingCondition(RobotSide.LEFT);
      ReadyToStartWalkingCondition readyToStartWalkingRightSupport = new ReadyToStartWalkingCondition(RobotSide.RIGHT);

      SupportToToeOffCondition leftSupportToToeOffCondition = new SupportToToeOffCondition(RobotSide.LEFT);
      SupportToToeOffCondition rightSupportToToeOffCondition = new SupportToToeOffCondition(RobotSide.RIGHT);

      ToeOffToSwingCondition leftToeOffToSwingCondition = new ToeOffToSwingCondition(RobotSide.LEFT);
      ToeOffToSwingCondition rightToeOffToSwingCondition = new ToeOffToSwingCondition(RobotSide.RIGHT);

      SwingToSupportCondition leftSwingToSupportCondition = new SwingToSupportCondition(RobotSide.LEFT);
      SwingToSupportCondition rightSwingToSupportCondition = new SwingToSupportCondition(RobotSide.RIGHT);

      // Create the transitions
      // Left State Machine:
      StateTransition startWalkingLeftSupport = new StateTransition(WalkingState.SUPPORT, readyToStartWalkingLeftSupport);
      StateTransition startWalkingLeftSwing = new StateTransition(WalkingState.SWING, readyToStartWalkingRightSupport);

      StateTransition leftSupportToToeOffTransition = new StateTransition(WalkingState.TOE_OFF, leftSupportToToeOffCondition);
      StateTransition leftToeOffToSwingTransition = new StateTransition(WalkingState.SWING, leftToeOffToSwingCondition);
      StateTransition leftSwingToSupportTransition = new StateTransition(WalkingState.SUPPORT, leftSwingToSupportCondition);

      leftBalanceState.addStateTransition(startWalkingLeftSupport);
      leftBalanceState.addStateTransition(startWalkingLeftSwing);
      leftSupportState.addStateTransition(leftSupportToToeOffTransition);
      leftToeOffState.addStateTransition(leftToeOffToSwingTransition);
      leftSwingState.addStateTransition(leftSwingToSupportTransition);

      // Right State Machine:
      StateTransition startWalkingRightSupport = new StateTransition(WalkingState.SUPPORT, readyToStartWalkingRightSupport);
      StateTransition startWalkingRightSwing = new StateTransition(WalkingState.SWING, readyToStartWalkingLeftSupport);

      StateTransition rightSupportToToeOffTransition = new StateTransition(WalkingState.TOE_OFF, rightSupportToToeOffCondition);
      StateTransition rightToeOffToSwingTransition = new StateTransition(WalkingState.SWING, rightToeOffToSwingCondition);
      StateTransition rightSwingToSupportTransition = new StateTransition(WalkingState.SUPPORT, rightSwingToSupportCondition);

      rightBalanceState.addStateTransition(startWalkingRightSupport);
      rightBalanceState.addStateTransition(startWalkingRightSwing);
      rightSupportState.addStateTransition(rightSupportToToeOffTransition);
      rightToeOffState.addStateTransition(rightToeOffToSwingTransition);
      rightSwingState.addStateTransition(rightSwingToSupportTransition);

      // Add the states to the state machines
      leftStateMachine.addState(leftBalanceState);
      leftStateMachine.addState(leftSupportState);
      leftStateMachine.addState(leftSwingState);

//    leftStateMachine.addState(leftStraightenState);
      leftStateMachine.addState(leftToeOffState);

      rightStateMachine.addState(rightBalanceState);
      rightStateMachine.addState(rightSupportState);
      rightStateMachine.addState(rightSwingState);

//    rightStateMachine.addState(rightStraightenState);
      rightStateMachine.addState(rightToeOffState);

   }

   private class BalanceState extends State
   {
      private final RobotSide robotSide;
      private SideDependentList<DoubleYoVariable> bodyPitchTorque = new SideDependentList<DoubleYoVariable>(ft_left, ft_right);

      public BalanceState(WalkingState stateEnum, RobotSide robotSide)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         /* Use hip to servo pitch */
         bodyPitchTorque.get(robotSide).set(0.65 * (t_gain.getDoubleValue() * (t_d.getDoubleValue() - robot.q_pitch.getDoubleValue()) - t_damp.getDoubleValue() * robot.qd_pitch.getDoubleValue()));

         /* Virtual Model Control Stuff for now... */
         setVTPBalance(robotSide);

         single_support_vtp(robotSide, 0.65 * ff_z.getDoubleValue(), bodyPitchTorque.get(robotSide).getDoubleValue(), activeTorqueAtHip.get(robotSide),
                            activeTorqueAtKnee.get(robotSide), activeTorqueAtAnkle.get(robotSide));

         /* Keep knee straight */
         activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - robot.getKneeAngle(robotSide)));
         activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + -knee_damp.getDoubleValue() * robot.getKneeVelocity(robotSide) + knee_straight_torque.getDoubleValue());

         /* Ankle limit to go on toes and maintain cop */
         passiveTorqueAtAnkle.get(robotSide).set(passive_ankle_torques(robot.getAnkleAngle(robotSide), robot.getAnkleVelocity(robotSide)));
      }

      public void doTransitionIntoAction()
      {
      }

      public void doTransitionOutOfAction()
      {
//       robot.qd_x.val = -0.2;

      }

   }


   private double getTimeInCurrentState(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return leftStateMachine.timeInCurrentState();
      else
         return rightStateMachine.timeInCurrentState();
   }

   private WalkingState getStateMachineState(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return (WalkingState) leftStateMachine.getCurrentStateEnum();
      else
         return (WalkingState) rightStateMachine.getCurrentStateEnum();
   }

   private class SupportState extends State
   {
      private final RobotSide robotSide;
      private SideDependentList<DoubleYoVariable> bodyPitchTorque = new SideDependentList<DoubleYoVariable>(ft_left, ft_right);

      public SupportState(WalkingState stateEnum, RobotSide robotSide)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         double timeInCurrentState = SpringFlamingoFastWalkingController.this.getTimeInCurrentState(robotSide);

         /* Use hip to servo pitch */
         bodyPitchTorque.get(robotSide).set(t_gain.getDoubleValue() * ((t_d.getDoubleValue() - pitch_gain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue())) - robot.q_pitch.getDoubleValue())
                 - t_damp.getDoubleValue() * robot.qd_pitch.getDoubleValue());
         if (timeInCurrentState < doub_time.getDoubleValue())
            bodyPitchTorque.get(robotSide).set((timeInCurrentState) / doub_time.getDoubleValue() * bodyPitchTorque.get(robotSide).getDoubleValue());

         /* Virtual Model Control Stuff for now... */
         setVTP(robotSide);
         single_support_vtp(robotSide, ff_z.getDoubleValue(), bodyPitchTorque.get(robotSide).getDoubleValue(), activeTorqueAtHip.get(robotSide), activeTorqueAtKnee.get(robotSide),
                            activeTorqueAtAnkle.get(robotSide));

         /* Keep knee straight */
         activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - robot.getKneeAngle(robotSide)));
         if (timeInCurrentState < 0.06)
            activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + (timeInCurrentState) / 0.06
                    * (-knee_damp.getDoubleValue() * robot.getKneeVelocity(robotSide) + knee_straight_torque.getDoubleValue()));
         else
            activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + -knee_damp.getDoubleValue() * robot.getKneeVelocity(robotSide) + knee_straight_torque.getDoubleValue());

         /* Use ankle to servo speed, position */
         /* act_la.set(act_la.getDoubleValue() + vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - qd.x)); */

         /* Ankle limit to go on toes and maintain cop */
         passiveTorqueAtAnkle.get(robotSide).set(passive_ankle_torques(robot.getAnkleAngle(robotSide), robot.getAnkleVelocity(robotSide)));
      }

      public void doTransitionIntoAction()
      {
         if (robotSide == RobotSide.RIGHT)
         {
            down_rh.set(0.0);
            r_swing_flag.set(0.0);
            rthigh_start.set(robot.q_pitch.getDoubleValue() + robot.q_rh.getDoubleValue());
         }

         else
         {
            down_lh.set(0.0);
            l_swing_flag.set(0.0);
            lthigh_start.set(robot.q_pitch.getDoubleValue() + robot.q_lh.getDoubleValue());
         }
      }

      public void doTransitionOutOfAction()
      {
      }
   }


   private class ToeOffState extends State
   {
      private final RobotSide robotSide;
      private SideDependentList<DoubleYoVariable> bodyPitchTorque = new SideDependentList<DoubleYoVariable>(ft_left, ft_right);

      public ToeOffState(WalkingState stateEnum, RobotSide robotSide)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
//         double timeInCurrentState = getTimeInCurrentState(robotSide);
         double timeInOtherStateMachineState = SpringFlamingoFastWalkingController.this.getTimeInCurrentState(robotSide.getOppositeSide());
         WalkingState otherStateMachineState = getStateMachineState(robotSide.getOppositeSide());

         /* Use hip to servo pitch */
         bodyPitchTorque.get(robotSide).set(t_gain.getDoubleValue() * ((t_d.getDoubleValue() - pitch_gain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue())) - robot.q_pitch.getDoubleValue())
                 - t_damp.getDoubleValue() * robot.qd_pitch.getDoubleValue());

         if (otherStateMachineState == WalkingState.SUPPORT)
         {
            if (timeInOtherStateMachineState < doub_time.getDoubleValue())
            {
               transfer_ratio.set((doub_time.getDoubleValue() - (timeInOtherStateMachineState)) / doub_time.getDoubleValue());
               bodyPitchTorque.get(robotSide).set((doub_time.getDoubleValue() - (timeInOtherStateMachineState)) / doub_time.getDoubleValue() * bodyPitchTorque.get(robotSide).getDoubleValue());
            }
            else
            {
               bodyPitchTorque.get(robotSide).set(0.0);
               transfer_ratio.set(0.0);
            }
         }

         /* Virtual Model Control Stuff for now... */
         setVTP(robotSide);

         if (otherStateMachineState == WalkingState.SUPPORT)
         {
            set_transfer_ratio();
            single_support_vtp(robotSide, (transfer_ratio.getDoubleValue()) * ff_z.getDoubleValue(), bodyPitchTorque.get(robotSide).getDoubleValue(), activeTorqueAtHip.get(robotSide),
                               activeTorqueAtKnee.get(robotSide), activeTorqueAtAnkle.get(robotSide));
         }
         else
            single_support_vtp(robotSide, ff_z.getDoubleValue(), bodyPitchTorque.get(robotSide).getDoubleValue(), activeTorqueAtHip.get(robotSide), activeTorqueAtKnee.get(robotSide),
                               activeTorqueAtAnkle.get(robotSide));

         /* Keep knee straight */
         /* act_lk.set(act_lk.getDoubleValue() + knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - robot.q_lk.getDoubleValue()) - knee_damp.getDoubleValue() * robot.qd_lk.getDoubleValue()); */
         /* act_lk.set(act_lk.getDoubleValue() - knee_damp.getDoubleValue() * robot.qd_lk.getDoubleValue() + knee_straight_torque.getDoubleValue()); */

         /* Bend knee a little bit to prepare for swing... */
         activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + -toff_knee_gain.getDoubleValue() * (toff_knee_d.getDoubleValue() - robot.getKneeAngle(robotSide))
                 - knee_damp.getDoubleValue() * robot.getKneeVelocity(robotSide) + knee_straight_torque.getDoubleValue());

         /* Use ankle to servo speed, position */
         /* act_la.set(act_la.getDoubleValue() + vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - qd.x)); */

         /* Ankle limit to go on toes and maintain cop */
         passiveTorqueAtAnkle.get(robotSide).set(passive_ankle_torques(robot.getAnkleAngle(robotSide), robot.getAnkleVelocity(robotSide)));

         /* Ankle push off */
         activeTorqueAtAnkle.get(robotSide).set(activeTorqueAtAnkle.get(robotSide).getDoubleValue() + toe_off_ankle_torques(robot.getAnkleAngle(robotSide), robot.getAnkleVelocity(robotSide)));
      }

      public void doTransitionIntoAction()
      {
         if (robotSide == RobotSide.LEFT)
         {
            la_int.set(0.0);
         }
         else
         {
            ra_int.set(0.0);
         }
      }

      public void doTransitionOutOfAction()
      {
      }
   }


   private class SwingState extends State
   {
      private final RobotSide robotSide;
      private final SideDependentList<DoubleYoVariable> swingFlags = new SideDependentList<DoubleYoVariable>(l_swing_flag, r_swing_flag);

      public SwingState(WalkingState stateEnum, RobotSide robotSide)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         double timeInCurrentState = SpringFlamingoFastWalkingController.this.getTimeInCurrentState(robotSide);
         DoubleYoVariable swingFlag = swingFlags.get(robotSide);

         /* swing flag transitions */
         if (swingFlag.getDoubleValue() < 0.5)
         {
            /* if (q_lthigh.getDoubleValue() > swing_ang.getDoubleValue()) */
            if (timeInCurrentState > straighten_delay.getDoubleValue())
            {
               swingFlag.set(1.0);
               shinStartAngles.get(robotSide).set(shinAngles.get(robotSide).getDoubleValue());
               shinStartVelocities.get(robotSide).set(shinVelocities.get(robotSide).getDoubleValue());
            }
         }
         else if (swingFlag.getDoubleValue() < 1.5)
         {
            if (timeInCurrentState > hip_swing_time.getDoubleValue() + swingExtraTime.getDoubleValue())
            {
               swingFlag.set(2.0);
               thighStartAngles.get(robotSide).set(hip_d.getDoubleValue());
            }
         }

         else if (swingFlag.getDoubleValue() < 2.5)
         {
            if (timeInCurrentState > tot_swing_time.getDoubleValue() + swingExtraTime.getDoubleValue())
            {
               swingFlag.set(3.0);
            }
         }

         /* Which part of swing?? */
         if (swingFlag.getDoubleValue() < 0.5)
         {
            /* Minimum Jerk Trajectory for hip */
            minjerk_equation(thighStartAngles.get(robotSide).getDoubleValue(), hip_d.getDoubleValue(), hip_swing_time.getDoubleValue(), timeInCurrentState, desiredThighAngles.get(robotSide),
                             desiredThighVelocities.get(robotSide), desiredThighAccelerations.get(robotSide));

            /* Damp the knee */
            if (robot.getKneeVelocity(robotSide) < 0.0)
               activeTorqueAtKnee.get(robotSide).set(-swing_damp_knee1.getDoubleValue() * robot.getKneeVelocity(robotSide));
            else
               activeTorqueAtKnee.get(robotSide).set(0.0);

            /* Damp and Toggle the knee: */
            /* activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + knee_collapse.getDoubleValue()); */

            adaptiveControlThigh(robotSide);
         }

         else if (swingFlag.getDoubleValue() < 1.5)
         {
            /* Minimum Jerk Trajectory for hip */
            minjerk_equation(thighStartAngles.get(robotSide).getDoubleValue(), hip_d.getDoubleValue(), hip_swing_time.getDoubleValue(), timeInCurrentState, desiredThighAngles.get(robotSide),
                             desiredThighVelocities.get(robotSide), desiredThighAccelerations.get(robotSide));

            cubic_equation(shinStartAngles.get(robotSide).getDoubleValue(), hip_hold.getDoubleValue() + sh_bash_pos.getDoubleValue(), shinStartVelocities.get(robotSide).getDoubleValue(), sh_bash_vel.getDoubleValue(),
                           tot_swing_time.getDoubleValue() + swingExtraTime.getDoubleValue() - straighten_delay.getDoubleValue(), timeInCurrentState + swingExtraTime.getDoubleValue() - straighten_delay.getDoubleValue(),
                           desiredShinAngles.get(robotSide), desiredShinVelocities.get(robotSide), desiredShinAccelerations.get(robotSide));

//          activeTorqueAtKnee.get(robotSide).val  = 0.0;
//          activeTorqueAtKnee.get(robotSide).val  = knee_lock_torque.val -swing_damp_knee.val * robot.getKneeVelocity(robotSide);

            adaptiveControl(robotSide);

         }

         else if (swingFlag.getDoubleValue() < 2.5)
         {
            /* Minimum Jerk Trajectory for hip */
            minjerk_equation(thighStartAngles.get(robotSide).getDoubleValue(), hip_hold.getDoubleValue(), (tot_swing_time.getDoubleValue() - hip_swing_time.getDoubleValue()) * 2.0,
                             timeInCurrentState - (hip_swing_time.getDoubleValue() + swingExtraTime.getDoubleValue()), desiredThighAngles.get(robotSide),
                             desiredThighVelocities.get(robotSide), desiredThighAccelerations.get(robotSide));

            cubic_equation(shinStartAngles.get(robotSide).getDoubleValue(), hip_hold.getDoubleValue() + sh_bash_pos.getDoubleValue(), shinStartVelocities.get(robotSide).getDoubleValue(), sh_bash_vel.getDoubleValue(),
                           tot_swing_time.getDoubleValue() - straighten_delay.getDoubleValue(), timeInCurrentState - straighten_delay.getDoubleValue(), desiredShinAngles.get(robotSide),
                           desiredShinVelocities.get(robotSide), desiredShinAccelerations.get(robotSide));

            /* Hack for now to prevent instantaneous change in acceleration */
            if (timeInCurrentState > tot_swing_time.getDoubleValue() - 0.1)
            {
               desiredShinAccelerations.get(robotSide).set(desiredShinAccelerations.get(robotSide).getDoubleValue() * (tot_swing_time.getDoubleValue() + swingExtraTime.getDoubleValue() - timeInCurrentState) / 0.1);
            }

            adaptiveControl(robotSide);

            // activeTorqueAtKnee.get(robotSide).val = knee_lock_torque.val -swing_damp_knee.val * robot.getKneeVelocity(robotSide);


            hipDownTorques.get(robotSide).set(LPF_DOWN_TAU * hipDownTorques.get(robotSide).getDoubleValue() + (1.0 - LPF_DOWN_TAU) * hip_down_torque.getDoubleValue());
            activeTorqueAtHip.get(robotSide).set(activeTorqueAtHip.get(robotSide).getDoubleValue() + -hipDownTorques.get(robotSide).getDoubleValue());
         }

         else
         {
            /* Hold in place */
            desiredThighAngles.get(robotSide).set(hip_hold.getDoubleValue());
            desiredThighVelocities.get(robotSide).set(0.0);
            desiredThighAccelerations.get(robotSide).set(0.0);

            desiredShinAngles.get(robotSide).set(thighAngles.get(robotSide).getDoubleValue());    /* hip_hold.getDoubleValue(); */
            desiredShinVelocities.get(robotSide).set(0.0);
            desiredShinAccelerations.get(robotSide).set(0.0);    /* lshindd_d.getDoubleValue() * (1.0 - 8.0*control_dt.getDoubleValue()); */

            adaptiveControl(robotSide);
            activeTorqueAtKnee.get(robotSide).set(activeTorqueAtKnee.get(robotSide).getDoubleValue() + knee_lock_torque.getDoubleValue());

            /* JEP: see what happens if hip goes limp... */
            hipDownTorques.get(robotSide).set(LPF_DOWN_TAU * hipDownTorques.get(robotSide).getDoubleValue() + (1.0 - LPF_DOWN_TAU) * hip_down_torque.getDoubleValue());
            activeTorqueAtHip.get(robotSide).set(-hipDownTorques.get(robotSide).getDoubleValue());

            /*
             * activeTorqueAtKnee.get(robotSide).set(knee_lock_torque.getDoubleValue() -swing_damp_knee.getDoubleValue() * robot.getKneeVelocity(robotSide));
             *           if (thighAngle(robotSide).getDoubleValue() < desiredThighAngles(robotSide).getDoubleValue()) desiredThighAngles(robotSide).set(thighAngle(robotSide).getDoubleValue());
             */

         }

         /* Servo ankle level to the ground */
         heels.get(robotSide).set(-robot.q_pitch.getDoubleValue() - robot.getHipAngle(robotSide) - robot.getKneeAngle(robotSide) - robot.getAnkleAngle(robotSide));
         activeTorqueAtAnkle.get(robotSide).set(-ankle_gain.getDoubleValue() * (ankle_desired.getDoubleValue() - heels.get(robotSide).getDoubleValue())
                 - ankle_damp.getDoubleValue() * robot.getAnkleVelocity(robotSide));
         passiveTorqueAtAnkle.get(robotSide).set(0.0);

         DoubleYoVariable activeHipTorque = activeTorqueAtHip.get(robotSide);
         DoubleYoVariable activeKneeTorque = activeTorqueAtKnee.get(robotSide);

         if (activeHipTorque.getDoubleValue() > max_hip_torque.getDoubleValue())
            activeHipTorque.set(max_hip_torque.getDoubleValue());
         if (activeHipTorque.getDoubleValue() < -max_hip_torque.getDoubleValue())
            activeHipTorque.set(-max_hip_torque.getDoubleValue());

         if (activeKneeTorque.getDoubleValue() > max_hip_torque.getDoubleValue())
            activeKneeTorque.set(max_hip_torque.getDoubleValue());
         if (activeKneeTorque.getDoubleValue() < -max_hip_torque.getDoubleValue())
            activeKneeTorque.set(-max_hip_torque.getDoubleValue());
      }

      public void doTransitionIntoAction()
      {
         hipDownTorques.get(robotSide).set(0.0);
         swingFlags.get(robotSide).set(0.0);
         thighStartAngles.get(robotSide).set(robot.q_pitch.getDoubleValue() + robot.getHipAngle(robotSide));

         swingExtraTime.set(swingTimeMultiplier.getDoubleValue() * (1.0 - vel.getDoubleValue()));
         if (swingExtraTime.getDoubleValue() < 0.0)
            swingExtraTime.set(0.0);
      }

      public void doTransitionOutOfAction()
      {
      }
   }


   /*private class StraightenState extends State
   {
      private final RobotSide robotSide;

      public StraightenState(WalkingState stateEnum, RobotSide robotSide)
      {
         super(stateEnum);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         double timeInCurrentState = getTimeInCurrentState(robotSide);
         minjerk_equation(hip_d.getDoubleValue(), hip_hold.getDoubleValue(), 0.1, timeInCurrentState, desiredThighAngles.get(robotSide), desiredThighVelocities.get(robotSide),
                          desiredThighAccelerations.get(robotSide));

         if (robot.getKneeAngle(robotSide) > -0.2)
            desiredThighAngles.get(robotSide).set(desiredThighAngles.get(robotSide).getDoubleValue() + -hip_down.getDoubleValue());

          Keep knee straight and damp it 
         YoVariable kneeSetPoint = kneeSetPoints.get(robotSide);
         YoVariable kneeSetVelocity = kneeSetVelocities.get(robotSide);

         minjerk_equation(-0.6, knee_d.getDoubleValue(), 0.2, timeInCurrentState, kneeSetPoint, kneeSetVelocity, desiredShinAccelerations.get(robotSide));
         activeTorqueAtKnee.get(robotSide).set(swing_gain_knee.getDoubleValue() * (kneeSetPoint.getDoubleValue() - robot.getKneeAngle(robotSide))
                 + swing_damp_knee.getDoubleValue() * (kneeSetVelocity.getDoubleValue() - robot.getKneeVelocity(robotSide)) + knee_lock_torque.getDoubleValue());

         adaptiveControlThigh(robotSide);

          Servo ankle level to the ground 
         heels.get(robotSide).set(-robot.q_pitch.getDoubleValue() - robot.getHipAngle(robotSide) - robot.getKneeAngle(robotSide) - robot.getAnkleAngle(robotSide));
         activeTorqueAtAnkle.get(robotSide).set(-ankle_gain.getDoubleValue() * (ankle_desired.getDoubleValue() - heels.get(robotSide).getDoubleValue())
                 - ankle_damp.getDoubleValue() * robot.getAnkleVelocity(robotSide));

      }

      public void doTransitionIntoAction()
      {
      }

      public void doTransitionOutOfAction()
      {
      }
   }
*/

   private class ReadyToStartWalkingCondition implements StateTransitionCondition
   {
      private final RobotSide supportSide;
      private final BooleanYoVariable enoughTimePassed, supportLegIsBack;

      public ReadyToStartWalkingCondition(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         enoughTimePassed = new BooleanYoVariable("enoughTimePassed_RTS_" + supportSide, "Check if enough time passed to start walking",
                                           registry);
         supportLegIsBack = new BooleanYoVariable("supportLegIsBack_RTS_" + supportSide, "Check if the support leg for this transition is the reward leg",
                                           registry);
      }

      public boolean checkCondition()
      {
         enoughTimePassed.set(robot.t.getDoubleValue() > 2.0);
         supportLegIsBack.set(false);

         if (supportSide == RobotSide.LEFT)
         {
            supportLegIsBack.set(x_lf.getDoubleValue() >= x_rf.getDoubleValue());
         }
         else if (supportSide == RobotSide.RIGHT)
         {
            supportLegIsBack.set(x_rf.getDoubleValue() > x_lf.getDoubleValue());
         }

         return (enoughTimePassed.getBooleanValue() && supportLegIsBack.getBooleanValue());
      }

   }


   private class SupportToToeOffCondition implements StateTransitionCondition
   {
      private final RobotSide supportSide;
      private final BooleanYoVariable enoughTimePassed, supportLegIsBackAndUnloaded, otherSideInSupport;

      public SupportToToeOffCondition(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         enoughTimePassed = new BooleanYoVariable("enoughTimePassed_STTO_" + supportSide, "Check if enough time passed to start walking",
                                           registry);
         otherSideInSupport = new BooleanYoVariable("otherSideInSupport_TOTS_" + supportSide, "Check if other side is in support before going to toe off",
                 registry);

         supportLegIsBackAndUnloaded = new BooleanYoVariable("supportLegIsBackAndUnloaded_STTO_" + supportSide,
                 "Check if the support leg for this transition is the reward leg", registry);
      }

      public boolean checkCondition()
      {
         double timeInState;
         if (supportSide == RobotSide.LEFT)
            timeInState = leftStateMachine.timeInCurrentState();
         else
            timeInState = rightStateMachine.timeInCurrentState();

         enoughTimePassed.set(timeInState > min_support_time.getDoubleValue());

         otherSideInSupport.set(getStateMachineState(supportSide.getOppositeSide()) == WalkingState.SUPPORT);


         if (supportSide == RobotSide.LEFT)
         {
            supportLegIsBackAndUnloaded.set((x_lf.getDoubleValue() < x_rf.getDoubleValue()) && (robot.gc_lheel_fz.getDoubleValue() < force_thresh.getDoubleValue()));
         }
         else
         {
            supportLegIsBackAndUnloaded.set((x_rf.getDoubleValue() < x_lf.getDoubleValue()) && (robot.gc_rheel_fz.getDoubleValue() < force_thresh.getDoubleValue()));
         }

         return (enoughTimePassed.getBooleanValue() && supportLegIsBackAndUnloaded.getBooleanValue() && otherSideInSupport.getBooleanValue());
      }

   }


   private class ToeOffToSwingCondition implements StateTransitionCondition
   {
      private final RobotSide supportSide;
      private final BooleanYoVariable enoughTimePassed, supportLegIsBackAndUnloaded;

      public ToeOffToSwingCondition(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         enoughTimePassed = new BooleanYoVariable("enoughTimePassed_TOTS_" + supportSide, "Check if enough time passed to start walking", 
                                           registry);
         supportLegIsBackAndUnloaded = new BooleanYoVariable("supportLegIsBackAndUnloaded_TOTS_" + supportSide,
                 "Check if the support leg for this transition is the reward leg", registry);
      }

      public boolean checkCondition()
      {
         double timeInState;
         if (supportSide == RobotSide.LEFT)
            timeInState = leftStateMachine.timeInCurrentState();
         else
            timeInState = rightStateMachine.timeInCurrentState();

         enoughTimePassed.set(timeInState > doub_time.getDoubleValue());

         supportTransitionDistance.set(captureRatio.getDoubleValue() * vel.getDoubleValue() - supportTransitionGain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue()));    // +  s_d2s.val ;
         if (supportTransitionDistance.getDoubleValue() < 0.01)
            supportTransitionDistance.set(0.01);
         if (supportTransitionDistance.getDoubleValue() > 0.2)
            supportTransitionDistance.set(0.2);


         if (supportSide == RobotSide.LEFT)
         {
            boolean condition1 = (x_rf.getDoubleValue() < supportTransitionDistance.getDoubleValue()) || (-x_lf.getDoubleValue() > s_d2s_trail.getDoubleValue());
            boolean condition2 = (gc_ltoe.getDoubleValue() < sw_force_thresh.getDoubleValue());
            boolean condition3 = rightStateMachine.isCurrentState(WalkingState.SUPPORT);
            supportLegIsBackAndUnloaded.set(condition1 && condition2 && condition3);
         }
         else
         {
            boolean condition1 = (x_lf.getDoubleValue() < supportTransitionDistance.getDoubleValue()) || (-x_rf.getDoubleValue() > s_d2s_trail.getDoubleValue());
            boolean condition2 = (gc_rtoe.getDoubleValue() < sw_force_thresh.getDoubleValue());
            boolean condition3 = leftStateMachine.isCurrentState(WalkingState.SUPPORT);
            supportLegIsBackAndUnloaded.set(condition1 && condition2 && condition3);
         }

         return (enoughTimePassed.getBooleanValue() && supportLegIsBackAndUnloaded.getBooleanValue());
      }
   }


   private class SwingToSupportCondition implements StateTransitionCondition
   {
      private final RobotSide supportSide;
      private final BooleanYoVariable enoughTimePassed, swingFootHitGround;

      public SwingToSupportCondition(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         enoughTimePassed = new BooleanYoVariable("enoughTimePassed_STS_" + supportSide, "Check if enough time passed to ensure swing is done",
                                            registry);
         swingFootHitGround = new BooleanYoVariable("swingFootHitGround_STS_" + supportSide, "Check if the swing foot hit the ground", 
                 registry);
      }

      public boolean checkCondition()
      {
         double timeInState;
         if (supportSide == RobotSide.LEFT)
            timeInState = leftStateMachine.timeInCurrentState();
         else
            timeInState = rightStateMachine.timeInCurrentState();

         enoughTimePassed.set(timeInState > tot_swing_time.getDoubleValue() + swingExtraTime.getDoubleValue());

         if (supportSide == RobotSide.LEFT)
         {
            boolean condition1 = (robot.gc_lheel_fz.getDoubleValue() > foot_switch_force.getDoubleValue());
            boolean condition2 = (robot.q_lk.getDoubleValue() > -0.6) && (robot.qd_lk.getDoubleValue() < 0.5);
            swingFootHitGround.set(condition1 && condition2);
         }
         else
         {
            boolean condition1 = (robot.gc_rheel_fz.getDoubleValue() > foot_switch_force.getDoubleValue());
            boolean condition2 = (robot.q_rk.getDoubleValue() > -0.6) && (robot.qd_rk.getDoubleValue() < 0.5);
            swingFootHitGround.set(condition1 && condition2);
         }

         return (enoughTimePassed.getBooleanValue() && swingFootHitGround.getBooleanValue());
      }

   }


   /*
    *  Zero Torque about virtual toe point (VTP) defined
    *  by lf.getDoubleValue(), rf.getDoubleValue() which specify point relative
    *  to the ankle (positive being forward) that zero
    *  torque will be applied.  X force cannot be specified.
    *  Rather it is computed based on fz,ft,rf,lf.
    */

   private void single_support_vtp(RobotSide support_leg, double fz, double ft, DoubleYoVariable ff_hip, DoubleYoVariable ff_knee, DoubleYoVariable ff_ankle)
   {
      double J1, J2, J3;
      double J4, J5, J6;

      double lt = -robot.q_pitch.getDoubleValue() - robot.q_lh.getDoubleValue() - robot.q_lk.getDoubleValue() - robot.q_la.getDoubleValue();
      double rt = -robot.q_pitch.getDoubleValue() - robot.q_rh.getDoubleValue() - robot.q_rk.getDoubleValue() - robot.q_ra.getDoubleValue();

      if (support_leg == RobotSide.LEFT)
      {
         J3 = -Math.cos(lt + robot.q_la.getDoubleValue() + robot.q_lk.getDoubleValue()) * SpringFlamingoRobot.UPPER_LINK_LENGTH;
         J6 = -Math.sin(lt + robot.q_la.getDoubleValue() + robot.q_lk.getDoubleValue()) * SpringFlamingoRobot.UPPER_LINK_LENGTH;

         J2 = J3 - Math.cos(lt + robot.q_la.getDoubleValue()) * SpringFlamingoRobot.LOWER_LINK_LENGTH;
         J5 = J6 - Math.sin(lt + robot.q_la.getDoubleValue()) * SpringFlamingoRobot.LOWER_LINK_LENGTH;

         J1 = J2 + leftVirtualToePoint.getDoubleValue() * Math.sin(lt) - SpringFlamingoRobot.FOOT_H * Math.cos(lt);
         J4 = J5 - leftVirtualToePoint.getDoubleValue() * Math.cos(lt) - SpringFlamingoRobot.FOOT_H * Math.sin(lt);

         fzl.set(fz);
         ftl.set(ft);
         fxl.set((1.0 / J1) * (ft - J4 * fz));
         fzr.set(0.0);
         ftr.set(0.0);
         fxr.set(0.0);
      }

      else if (support_leg == RobotSide.RIGHT)
      {
         J3 = -Math.cos(rt + robot.q_ra.getDoubleValue() + robot.q_rk.getDoubleValue()) * SpringFlamingoRobot.UPPER_LINK_LENGTH;
         J6 = -Math.sin(rt + robot.q_ra.getDoubleValue() + robot.q_rk.getDoubleValue()) * SpringFlamingoRobot.UPPER_LINK_LENGTH;

         J2 = J3 - Math.cos(rt + robot.q_ra.getDoubleValue()) * SpringFlamingoRobot.LOWER_LINK_LENGTH;
         J5 = J6 - Math.sin(rt + robot.q_ra.getDoubleValue()) * SpringFlamingoRobot.LOWER_LINK_LENGTH;

         J1 = J2 + rightVirtualToePoint.getDoubleValue() * Math.sin(rt) - SpringFlamingoRobot.FOOT_H * Math.cos(rt);
         J4 = J5 - rightVirtualToePoint.getDoubleValue() * Math.cos(rt) - SpringFlamingoRobot.FOOT_H * Math.sin(rt);

         fzr.set(fz);
         ftr.set(ft);
         fxr.set((1.0 / J1) * (ft - J4 * fz));
         fzl.set(0.0);
         ftl.set(0.0);
         fxl.set(0.0);
      }

      else
         throw new RuntimeException("Shouldn't get here!");

      ff_ankle.set((-J2 * J4 / J1 + J5) * fz + (J2 / J1 - 1) * ft);
      ff_knee.set((-J3 * J4 / J1 + J6) * fz + (J3 / J1 - 1) * ft);
      ff_hip.set(-ft);

   }




   /* For fitting a cubic equation: page 234 of Craig */
   void cubic_equation(double b0, double bf, double dot_b0, double dot_bf, double tf, double t, DoubleYoVariable b, DoubleYoVariable b_dot, DoubleYoVariable b_ddot)
   {
      double a0, a1, a2, a3;

      a0 = b0;
      a1 = dot_b0;
      a2 = 3.0 / (tf * tf) * (bf - b0) - 2.0 / tf * dot_b0 - 1.0 / tf * dot_bf;
      a3 = -2.0 / Math.pow(tf, 3.0) * (bf - b0) + 1.0 / (tf * tf) * (dot_bf + dot_b0);

      b.set(a0 + a1 * t + a2 * (t * t) + a3 * (t * t * t));

      b_dot.set(a1 + a2 * 2.0 * t + a3 * 3.0 * t * t);

      b_ddot.set(a2 * 2.0 + a3 * 6.0 * t);

      if (t > tf)
      {
         b.set(bf);
         b_dot.set(dot_bf);
         b_ddot.set(0.0);
      }

      if (t < 0)
      {
         b.set(b0);
         b_dot.set(dot_b0);
         b_ddot.set(0.0);
      }

   }

   /* For fitting a minimum jerk trajectory.  From Flash and Hogan 1985 */
   void minjerk_equation(double b0, double bf, double tf, double t, DoubleYoVariable b, DoubleYoVariable b_dot, DoubleYoVariable b_ddot)

   {
      double tau;

      tau = t / tf;

      b.set(b0 + (b0 - bf) * (15.0 * tau * tau * tau * tau - 6.0 * tau * tau * tau * tau * tau - 10.0 * tau * tau * tau));

      b_dot.set((1.0 / tf) * (b0 - bf) * (60.0 * tau * tau * tau - 30.0 * tau * tau * tau * tau - 30.0 * tau * tau));

      b_ddot.set((1.0 / (tf * tf)) * (b0 - bf) * (180.0 * tau * tau - 120.0 * tau * tau * tau - 60.0 * tau));

      if (t > tf)
      {
         b.set(bf);
         b_dot.set(0.0);
         b_ddot.set(0.0);
      }

      if (t < 0.0)
      {
         b.set(b0);
         b_dot.set(0.0);
         b_ddot.set(0.0);
      }

   }

   private void setVTP(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         set_left_vtp();
      }
      else
      {
         set_right_vtp();
      }
   }

   void set_left_vtp()
   {
      leftVirtualToePoint.set(x_lf.getDoubleValue() * f_mul.getDoubleValue() + f_add.getDoubleValue());
      leftVirtualToePoint.set(leftVirtualToePoint.getDoubleValue() + vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue()));

      /* lf.set(lf.getDoubleValue() - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue())); */


      if (leftVirtualToePoint.getDoubleValue() > f_max.getDoubleValue())
         leftVirtualToePoint.set(f_max.getDoubleValue());
      if (leftVirtualToePoint.getDoubleValue() < f_min.getDoubleValue())
         leftVirtualToePoint.set(f_min.getDoubleValue());

      if ((x_lf.getDoubleValue() > x_rf.getDoubleValue()) && (leftVirtualToePoint.getDoubleValue() > 0.0))
         leftVirtualToePoint.set(0.0);    // Don't put weight on the front leg's heels.
   }

   void set_right_vtp()
   {
      rightVirtualToePoint.set(x_rf.getDoubleValue() * f_mul.getDoubleValue() + f_add.getDoubleValue());
      rightVirtualToePoint.set(rightVirtualToePoint.getDoubleValue() + vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue()));

      /* rf.set(rf.getDoubleValue() - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - vel.getDoubleValue())); */

      if (rightVirtualToePoint.getDoubleValue() > f_max.getDoubleValue())
         rightVirtualToePoint.set(f_max.getDoubleValue());
      if (rightVirtualToePoint.getDoubleValue() < f_min.getDoubleValue())
         rightVirtualToePoint.set(f_min.getDoubleValue());

      if ((x_rf.getDoubleValue() > x_lf.getDoubleValue()) && (rightVirtualToePoint.getDoubleValue() > 0.0))
         rightVirtualToePoint.set(0.0);    // Don't put weight on the front leg's heels.
   }

   private void setVTPBalance(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         set_left_vtp_balance();
      else
         set_right_vtp_balance();
   }

   void set_left_vtp_balance()
   {
      leftVirtualToePoint.set(x_lf.getDoubleValue() * 1.2 + f_add.getDoubleValue());
      leftVirtualToePoint.set(leftVirtualToePoint.getDoubleValue() + vtp_gain.getDoubleValue() * (0.0 - vel.getDoubleValue()));

      if (leftVirtualToePoint.getDoubleValue() > f_max.getDoubleValue())
         leftVirtualToePoint.set(f_max.getDoubleValue());
      if (leftVirtualToePoint.getDoubleValue() < f_min.getDoubleValue())
         leftVirtualToePoint.set(f_min.getDoubleValue());
   }

   void set_right_vtp_balance()
   {
      rightVirtualToePoint.set(x_rf.getDoubleValue() * 1.2 + f_add.getDoubleValue());
      rightVirtualToePoint.set(rightVirtualToePoint.getDoubleValue() + vtp_gain.getDoubleValue() * (0.0 - vel.getDoubleValue()));

      if (rightVirtualToePoint.getDoubleValue() > f_max.getDoubleValue())
         rightVirtualToePoint.set(f_max.getDoubleValue());
      if (rightVirtualToePoint.getDoubleValue() < f_min.getDoubleValue())
         rightVirtualToePoint.set(f_min.getDoubleValue());
   }

   void right_adaptive_control_thigh()
   {
      double act_rk_temp;

      /* Assumes act_rk.getDoubleValue() already set before entering */

      act_rk_temp = act_rk.getDoubleValue();

      rshin_d.set(q_rshin.getDoubleValue());
      rshind_d.set(qd_rshin.getDoubleValue());

      rshindd_d.set((1.0 / a3.getDoubleValue())
                      * (-a2.getDoubleValue() * Math.cos(robot.q_rk.getDoubleValue()) * thighdd_d.getDoubleValue() - a2.getDoubleValue() * Math.sin(robot.q_rk.getDoubleValue()) * qd_rthigh.getDoubleValue() * qd_rthigh.getDoubleValue()
                         - a5.getDoubleValue() * Math.sin(q_rshin.getDoubleValue()) + act_rk.getDoubleValue()));

      adaptiveControl(RobotSide.RIGHT);
      act_rk.set(act_rk_temp);
   }

   void adaptiveControl(RobotSide robotSide)
   {
//    RobotSide robotSide = RobotSide.LEFT;

      thighdd_d.set(desiredThighAccelerations.get(robotSide).getDoubleValue());
      thighd_d.set(desiredThighVelocities.get(robotSide).getDoubleValue());
      thigh_d.set(desiredThighAngles.get(robotSide).getDoubleValue());

      shindd_d.set(desiredShinAccelerations.get(robotSide).getDoubleValue());
      shind_d.set(desiredShinVelocities.get(robotSide).getDoubleValue());
      shin_d.set(desiredShinAngles.get(robotSide).getDoubleValue());

      double thighAngle = thighAngles.get(robotSide).getDoubleValue();
      double thighVelocity = thighVelocities.get(robotSide).getDoubleValue();

      double shinAngle = shinAngles.get(robotSide).getDoubleValue();
      double shinVelocity = shinVelocities.get(robotSide).getDoubleValue();

      double hipAngle = robot.getHipAngle(robotSide);
      double kneeAngle = robot.getKneeAngle(robotSide);

      double kneeVelocity = robot.getKneeVelocity(robotSide);
      double hipVelocity = robot.getHipVelocity(robotSide);

      qdd_1r.set(thighdd_d.getDoubleValue() - lambda1.getDoubleValue() * (thighVelocity - thighd_d.getDoubleValue()));
      qdd_2r.set(shindd_d.getDoubleValue() - lambda2.getDoubleValue() * (shinVelocity - shind_d.getDoubleValue()));

      /* JEP 2/21/2000: Found bug! Sign error here!!! */

      qd_1r.set(thighd_d.getDoubleValue() - lambda1.getDoubleValue() * (thighAngle - thigh_d.getDoubleValue()));
      qd_2r.set(shind_d.getDoubleValue() - lambda2.getDoubleValue() * (shinAngle - shin_d.getDoubleValue()));

      s1.set((thighVelocity - thighd_d.getDoubleValue()) + lambda1.getDoubleValue() * (thighAngle - thigh_d.getDoubleValue()));
      s2.set((shinVelocity - shind_d.getDoubleValue()) + lambda2.getDoubleValue() * (shinAngle - shin_d.getDoubleValue()));

      Y11.set(qdd_1r.getDoubleValue());
      Y12.set(Math.cos(kneeAngle) * (qdd_1r.getDoubleValue() + qdd_2r.getDoubleValue())
                + Math.sin(kneeAngle) * (qd_1r.getDoubleValue() * (robot.qd_pitch.getDoubleValue() + hipVelocity) - qd_2r.getDoubleValue() * (robot.qd_pitch.getDoubleValue() + hipVelocity + kneeVelocity)));
      Y13.set(qdd_2r.getDoubleValue());
      Y14.set(Math.sin(robot.q_pitch.getDoubleValue() + hipAngle));
      Y15.set(Math.sin(robot.q_pitch.getDoubleValue() + hipAngle + kneeAngle));

      Y21.set(0.0);
      Y22.set(Math.cos(kneeAngle) * qdd_1r.getDoubleValue() + Math.sin(kneeAngle) * qd_1r.getDoubleValue() * (robot.qd_pitch.getDoubleValue() + hipVelocity));
      Y23.set(qdd_2r.getDoubleValue());
      Y24.set(0.0);
      Y25.set(Math.sin(robot.q_pitch.getDoubleValue() + hipAngle + kneeAngle));

      /*
       * a1.set(J0+J2-J3);
       *      a2.set(J1);
       *      a3.set(J3);
       *      a4.set(G1);
       *      a5.set(G2);
       */

      activeTorqueAtHip.get(robotSide).set(Y11.getDoubleValue() * a1.getDoubleValue() + Y12.getDoubleValue() * a2.getDoubleValue() + Y13.getDoubleValue() * a3.getDoubleValue() + Y14.getDoubleValue() * a4.getDoubleValue() + Y15.getDoubleValue() * a5.getDoubleValue() - kd1.getDoubleValue() * s1.getDoubleValue());
      activeTorqueAtKnee.get(robotSide).set(Y21.getDoubleValue() * a1.getDoubleValue() + Y22.getDoubleValue() * a2.getDoubleValue() + Y23.getDoubleValue() * a3.getDoubleValue() + Y24.getDoubleValue() * a4.getDoubleValue() + Y25.getDoubleValue() * a5.getDoubleValue() - kd2.getDoubleValue() * s2.getDoubleValue());

      /*
       *   a1_dot.set(-gam1.getDoubleValue() * (Y11.getDoubleValue() * s1.getDoubleValue() + Y21.getDoubleValue() * s2.getDoubleValue()));
       *      a2_dot.set(-gam2.getDoubleValue() * (Y12.getDoubleValue() * s1.getDoubleValue() + Y22.getDoubleValue() * s2.getDoubleValue()));
       *      a3_dot.set(-gam3.getDoubleValue() * (Y13.getDoubleValue() * s1.getDoubleValue() + Y23.getDoubleValue() * s2.getDoubleValue()));
       *      a4_dot.set(-gam4.getDoubleValue() * (Y14.getDoubleValue() * s1.getDoubleValue() + Y24.getDoubleValue() * s2.getDoubleValue()));
       *      a5_dot.set(-gam5.getDoubleValue() * (Y15.getDoubleValue() * s1.getDoubleValue() + Y25.getDoubleValue() * s2.getDoubleValue()));
       *
       *      if ((robot.t.getDoubleValue() > start_time.getDoubleValue()) && (!ramp_off))
       * {
       *   a1.set(a1.getDoubleValue() + control_dt.getDoubleValue() * a1_dot.getDoubleValue());
       *   a2.set(a2.getDoubleValue() + control_dt.getDoubleValue() * a2_dot.getDoubleValue());
       *   a3.set(a3.getDoubleValue() + control_dt.getDoubleValue() * a3_dot.getDoubleValue());
       *   a4.set(a4.getDoubleValue() + control_dt.getDoubleValue() * a4_dot.getDoubleValue());
       *   a5.set(a5.getDoubleValue() + control_dt.getDoubleValue() * a5_dot.getDoubleValue());
       * }
       */

      /*
       * adap_test1.set(Y11.getDoubleValue() * a1.getDoubleValue() + Y12.getDoubleValue() * a2.getDoubleValue() + Y13.getDoubleValue() * a3.getDoubleValue() + Y14.getDoubleValue() * a4.getDoubleValue() + Y15.getDoubleValue() * a5.getDoubleValue() - tau.lh);
       *      adap_test2.set(Y21.getDoubleValue() * a1.getDoubleValue() + Y22.getDoubleValue() * a2.getDoubleValue() + Y23.getDoubleValue() * a3.getDoubleValue() + Y24.getDoubleValue() * a4.getDoubleValue() + Y25.getDoubleValue() * a5.getDoubleValue() - tau.lk);
       */
   }

// void right_adaptive_control()
// {
//   left_adaptive_control(RobotSide.RIGHT);
//   if (1==1) return;
// }


   private void adaptiveControlThigh(RobotSide robotSide)
   {
      /* Assumes act_lk.getDoubleValue() already set before entering */
      double tempActiveKneeTorque = activeTorqueAtKnee.get(robotSide).getDoubleValue();

      double kneeAngle = robot.getKneeAngle(robotSide);
      double shinAngle = shinAngles.get(robotSide).getDoubleValue();
      double thighVelocity = thighVelocities.get(robotSide).getDoubleValue();

      DoubleYoVariable activeKneeTorque = activeTorqueAtKnee.get(robotSide);

      desiredShinAngles.get(robotSide).set(shinAngles.get(robotSide).getDoubleValue());
      desiredShinVelocities.get(robotSide).set(shinVelocities.get(robotSide).getDoubleValue());

      desiredShinAccelerations.get(robotSide).set((1.0 / a3.getDoubleValue())
              * (-a2.getDoubleValue() * Math.cos(kneeAngle) * thighdd_d.getDoubleValue() - a2.getDoubleValue() * Math.sin(kneeAngle) * thighVelocity * thighVelocity - a5.getDoubleValue() * Math.sin(shinAngle)
                 + activeKneeTorque.getDoubleValue()));

      adaptiveControl(robotSide);
      activeKneeTorque.set(tempActiveKneeTorque);
   }

   void set_transfer_ratio()
   {
      double min_ratio, max_ratio, range;
      double l_lf, l_rf;

      /* If feet really close just split up */
      if (Math.abs(x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue() - x_rf.getDoubleValue() + rightVirtualToePoint.getDoubleValue()) < 0.01)
         transfer_ratio.set(0.5);

         /* Standard: 1.0 means full on REAR leg.  0.0 means full on FRONT leg. */
      else
      {
         l_lf = Math.sqrt((x_rf.getDoubleValue() - rightVirtualToePoint.getDoubleValue()) * (x_rf.getDoubleValue() - rightVirtualToePoint.getDoubleValue()) + z_rf.getDoubleValue() * z_rf.getDoubleValue());
         l_rf = Math.sqrt((x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue()) * (x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue()) + z_lf.getDoubleValue() * z_lf.getDoubleValue());

         if ((x_rf.getDoubleValue() - rightVirtualToePoint.getDoubleValue()) > (x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue()))
            transfer_ratio.set(((x_rf.getDoubleValue() - rightVirtualToePoint.getDoubleValue()) * l_lf)
                                 / ((x_rf.getDoubleValue() - rightVirtualToePoint.getDoubleValue()) * l_lf - (x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue()) * l_rf));
         else
            transfer_ratio.set(((x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue()) * l_rf)
                                 / (-(x_rf.getDoubleValue() - rightVirtualToePoint.getDoubleValue()) * l_lf + (x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue()) * l_rf));

      }

      /* As feet get closer, limit range get smaller */

      range = Math.abs(x_lf.getDoubleValue() - leftVirtualToePoint.getDoubleValue() - x_rf.getDoubleValue() + rightVirtualToePoint.getDoubleValue()) / 0.30;
      if (range > 0.8)
         range = 0.8;
      max_ratio = 0.5 + range / 2.0;
      min_ratio = 0.5 - range / 2.0;

      if (transfer_ratio.getDoubleValue() < min_ratio)
         transfer_ratio.set(min_ratio);
      if (transfer_ratio.getDoubleValue() > max_ratio)
         transfer_ratio.set(max_ratio);

      /* But still make sure it doesn't go crazy */

      if (transfer_ratio.getDoubleValue() < 0.1)
         transfer_ratio.set(0.1);
      if (transfer_ratio.getDoubleValue() > 0.9)
         transfer_ratio.set(0.9);

   }

   private enum WalkingState {BALANCE, SUPPORT, TOE_OFF, SWING} // , STRAIGHTEN};

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
   
}
