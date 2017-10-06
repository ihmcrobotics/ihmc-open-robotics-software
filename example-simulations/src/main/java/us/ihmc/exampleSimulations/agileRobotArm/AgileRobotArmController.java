package us.ihmc.exampleSimulations.agileRobotArm;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;

public class AgileRobotArmController implements RobotController
{
   private static final long serialVersionUID = 362874479072760137L;

   private final YoVariableRegistry registry = new YoVariableRegistry("AgileRobotArmController");


   protected AgileRobotArmRobot rob;

   // These are the variables that are automatically created when the robot is created:
   private final YoDouble t, q_shoulder_yaw, qd_shoulder_yaw, tau_shoulder_yaw, q_shoulder_pitch, qd_shoulder_pitch, tau_shoulder_pitch, q_elbow_pitch;
   private final YoDouble qd_elbow_pitch, tau_elbow_pitch, q_wrist_pitch, qd_wrist_pitch, tau_wrist_pitch, q_wrist_yaw, qd_wrist_yaw;
   private final YoDouble tau_wrist_yaw, q_wrist_roll, qd_wrist_roll, tau_wrist_roll;

   private static final int
      NULL_TORQUES = 0, POSITION_CONTROL = 1, ANTI_GRAVITY = 2, DAMPING_PLUS_ANTI_GRAVITY = 3, POSITION_PLUS_ANTI_GRAVITY = 4, SINE_TRAJECTORY = 5;

   private final YoDouble mode = new YoDouble("mode", registry);

   private final YoDouble
      v1 = new YoDouble("v1", registry), v2 = new YoDouble("v2", registry), v3 = new YoDouble("v3", registry);
   private final YoDouble
      v4 = new YoDouble("v4", registry), v5 = new YoDouble("v5", registry), v6 = new YoDouble("v6", registry);


   private final YoDouble shoulder_yaw_amp = new YoDouble("shoulder_yaw_amp", registry);
   private final YoDouble shoulder_yaw_freq = new YoDouble("shoulder_yaw_freq", registry);
   private final YoDouble shoulder_yaw_phase = new YoDouble("shoulder_yaw_phase", registry);
   private final YoDouble q_d_shoulder_yaw = new YoDouble("q_d_shoulder_yaw", registry);

   private final YoDouble k_shoulder_yaw = new YoDouble("k_shoulder_yaw", registry);
   private final YoDouble b_shoulder_yaw = new YoDouble("b_shoulder_yaw", registry);

   private final YoDouble shoulder_pitch_amp = new YoDouble("shoulder_pitch_amp", registry);
   private final YoDouble shoulder_pitch_freq = new YoDouble("shoulder_pitch_freq", registry);
   private final YoDouble shoulder_pitch_phase = new YoDouble("shoulder_pitch_phase", registry);
   private final YoDouble q_d_shoulder_pitch = new YoDouble("q_d_shoulder_pitch", registry);

   private final YoDouble k_shoulder_pitch = new YoDouble("k_shoulder_pitch", registry);
   private final YoDouble b_shoulder_pitch = new YoDouble("b_shoulder_pitch", registry);

   private final YoDouble elbow_pitch_amp = new YoDouble("elbow_pitch_amp", registry);
   private final YoDouble elbow_pitch_freq = new YoDouble("elbow_pitch_freq", registry);
   private final YoDouble elbow_pitch_phase = new YoDouble("elbow_pitch_phase", registry);
   private final YoDouble q_d_elbow_pitch = new YoDouble("q_d_elbow_pitch", registry);

   private final YoDouble k_elbow_pitch = new YoDouble("k_elbow_pitch", registry);
   private final YoDouble b_elbow_pitch = new YoDouble("b_elbow_pitch", registry);

   private final YoDouble wrist_pitch_amp = new YoDouble("wrist_pitch_amp", registry);
   private final YoDouble wrist_pitch_freq = new YoDouble("wrist_pitch_freq", registry);
   private final YoDouble wrist_pitch_phase = new YoDouble("wrist_pitch_phase", registry);
   private final YoDouble q_d_wrist_pitch = new YoDouble("q_d_wrist_pitch", registry);

   private final YoDouble k_wrist_pitch = new YoDouble("k_wrist_pitch", registry);
   private final YoDouble b_wrist_pitch = new YoDouble("b_wrist_pitch", registry);

   private final YoDouble wrist_yaw_amp = new YoDouble("wrist_yaw_amp", registry);
   private final YoDouble wrist_yaw_freq = new YoDouble("wrist_yaw_freq", registry);
   private final YoDouble wrist_yaw_phase = new YoDouble("wrist_yaw_phase", registry);
   private final YoDouble q_d_wrist_yaw = new YoDouble("q_d_wrist_yaw", registry);

   private final YoDouble k_wrist_yaw = new YoDouble("k_wrist_yaw", registry);
   private final YoDouble b_wrist_yaw = new YoDouble("b_wrist_yaw", registry);

   private final YoDouble wrist_roll_amp = new YoDouble("wrist_roll_amp", registry);
   private final YoDouble wrist_roll_offset = new YoDouble("wrist_roll_offset", registry);
   private final YoDouble wrist_roll_freq = new YoDouble("wrist_roll_freq", registry);
   private final YoDouble wrist_roll_phase = new YoDouble("wrist_roll_phase", registry);
   private final YoDouble q_d_wrist_roll = new YoDouble("q_d_wrist_roll", registry);

   private final YoDouble k_wrist_roll = new YoDouble("k_wrist_roll", registry);
   private final YoDouble b_wrist_roll = new YoDouble("b_wrist_roll", registry);

   private final YoDouble a1 = new YoDouble("a1", registry);
   private final YoDouble a2 = new YoDouble("a2", registry);
   private final YoDouble a3 = new YoDouble("a3", registry);
   private final YoDouble a4 = new YoDouble("a4", registry);
   private final YoDouble a5 = new YoDouble("a5", registry);

   public AgileRobotArmController(AgileRobotArmRobot rob)
   {
      this.rob = rob;

      t = (YoDouble) rob.getVariable("t");
      q_shoulder_yaw = (YoDouble) rob.getVariable("q_shoulder_yaw");
      qd_shoulder_yaw = (YoDouble) rob.getVariable("qd_shoulder_yaw");
      tau_shoulder_yaw = (YoDouble) rob.getVariable("tau_shoulder_yaw");
      q_shoulder_pitch = (YoDouble) rob.getVariable("q_shoulder_pitch");
      qd_shoulder_pitch = (YoDouble) rob.getVariable("qd_shoulder_pitch");
      tau_shoulder_pitch = (YoDouble) rob.getVariable("tau_shoulder_pitch");
      q_elbow_pitch = (YoDouble) rob.getVariable("q_elbow_pitch");
      qd_elbow_pitch = (YoDouble) rob.getVariable("qd_elbow_pitch");
      tau_elbow_pitch = (YoDouble) rob.getVariable("tau_elbow_pitch");
      q_wrist_pitch = (YoDouble) rob.getVariable("q_wrist_pitch");
      qd_wrist_pitch = (YoDouble) rob.getVariable("qd_wrist_pitch");
      tau_wrist_pitch = (YoDouble) rob.getVariable("tau_wrist_pitch");
      q_wrist_yaw = (YoDouble) rob.getVariable("q_wrist_yaw");
      qd_wrist_yaw = (YoDouble) rob.getVariable("qd_wrist_yaw");
      tau_wrist_yaw = (YoDouble) rob.getVariable("tau_wrist_yaw");
      q_wrist_roll = (YoDouble) rob.getVariable("q_wrist_roll");
      qd_wrist_roll = (YoDouble) rob.getVariable("qd_wrist_roll");
      tau_wrist_roll = (YoDouble) rob.getVariable("tau_wrist_roll");


      initControl();
   }

   public void initControl()
   {
      doNullTorques();

      mode.set(SINE_TRAJECTORY);    // NULL_TORQUES; //POSITION_CONTROL; //ANTI_GRAVITY;

      v1.set(1.0);
      v2.set(2.0);
      v3.set(0.5);
      v4.set(0.75);
      v5.set(0.3);
      v6.set(3.0);

      shoulder_yaw_amp.set(1.0);    // 0.3;
      shoulder_yaw_freq.set(0.2);

      shoulder_pitch_amp.set(1.0);    // 0.3;
      shoulder_pitch_freq.set(0.2);
      shoulder_pitch_phase.set(Math.PI / 2.0);

      elbow_pitch_amp.set(1.0);    // 0.0;
      elbow_pitch_freq.set(0.3);    // 1.0;
      elbow_pitch_phase.set(0.0);

      wrist_pitch_amp.set(0.3);
      wrist_pitch_freq.set(0.75);    // 1.0;
      wrist_pitch_phase.set(0.0);

      wrist_yaw_amp.set(0.3);
      wrist_yaw_freq.set(2.0);
      wrist_yaw_phase.set(0.0);

      wrist_roll_amp.set(1.0);
      wrist_roll_freq.set(0.5);
      wrist_roll_phase.set(0.0);


      k_shoulder_yaw.set(150.0);    // 20.0; //200.0;
      b_shoulder_yaw.set(12.0);    // 2.0; //6.0;

      k_shoulder_pitch.set(150.0);    // 20.0; //200.0;
      b_shoulder_pitch.set(12.0);    // 2.0; //6.0;

      k_elbow_pitch.set(40.0);    // 60.0; //10.0;
      b_elbow_pitch.set(0.5);    // 0.8; //0.3;

      k_wrist_pitch.set(10.0);    // 5.0;
      b_wrist_pitch.set(0.3);

      k_wrist_yaw.set(10.0);    // 5.0;
      b_wrist_yaw.set(0.3);

      k_wrist_roll.set(1.0);
      b_wrist_roll.set(0.1);

      // Anti-gravity
      a1.set(3.0);
      a2.set(20.0);
      a3.set(3.5);
      a4.set(0.3);
      a5.set(0.8);
   }

   public void doControl()
   {
      doNullTorques();

      int controlMode = (int) mode.getDoubleValue();

      if (controlMode == NULL_TORQUES)
      {
      }
      else if (controlMode == POSITION_CONTROL)
         doPositionControl();
      else if (controlMode == ANTI_GRAVITY)
         doAntiGravityControl();
      else if (controlMode == DAMPING_PLUS_ANTI_GRAVITY)
         doDampingPlusAntiGravityControl();
      else if (controlMode == POSITION_PLUS_ANTI_GRAVITY)
         doPositionPlusAntiGravityControl();
      else if (controlMode == SINE_TRAJECTORY)
         doSineTrajectoryControl();

      limitTorques();
   }

   public void doNullTorques()
   {
      tau_shoulder_yaw.set(0.0);
      tau_shoulder_pitch.set(0.0);
      tau_elbow_pitch.set(0.0);
      tau_wrist_pitch.set(0.0);
      tau_wrist_yaw.set(0.0);
      tau_wrist_roll.set(0.0);
   }

   public void rampTorques()
   {
      // Ramp up torques over 5 seconds:
      if ((t.getDoubleValue() < 5.0) && (t.getDoubleValue() > 0.0))
      {
         double percent_ramp = t.getDoubleValue() / 5.0;

         tau_shoulder_yaw.set(tau_shoulder_yaw.getDoubleValue() * percent_ramp);
         tau_shoulder_pitch.set(tau_shoulder_pitch.getDoubleValue() * percent_ramp);
         tau_elbow_pitch.set(tau_elbow_pitch.getDoubleValue() * percent_ramp);
         tau_wrist_pitch.set(tau_wrist_pitch.getDoubleValue() * percent_ramp);
         tau_wrist_yaw.set(tau_wrist_yaw.getDoubleValue() * percent_ramp);
         tau_wrist_roll.set(tau_wrist_roll.getDoubleValue() * percent_ramp);
      }

   }

   public void doInverseGravity()
   {
      double s1 = Math.sin(q_shoulder_yaw.getDoubleValue());
      double c1 = Math.cos(q_shoulder_yaw.getDoubleValue());
      double s2 = Math.sin(q_shoulder_pitch.getDoubleValue());
      double c2 = Math.cos(q_shoulder_pitch.getDoubleValue());
      double s23 = Math.sin(q_shoulder_pitch.getDoubleValue() + q_elbow_pitch.getDoubleValue());
      double c23 = Math.cos(q_shoulder_pitch.getDoubleValue() + q_elbow_pitch.getDoubleValue());
      double s234 = Math.sin(q_shoulder_pitch.getDoubleValue() + q_elbow_pitch.getDoubleValue() + q_wrist_pitch.getDoubleValue());
      double c234 = Math.cos(q_shoulder_pitch.getDoubleValue() + q_elbow_pitch.getDoubleValue() + q_wrist_pitch.getDoubleValue());
      double s5 = Math.sin(q_wrist_yaw.getDoubleValue());
      double c5 = Math.cos(q_wrist_yaw.getDoubleValue());

      double tau5 = -a5.getDoubleValue() * c1 * c234 * s5 - a5.getDoubleValue() * s1 * c5;
      double tau4 = -a4.getDoubleValue() * c1 * s234 - a5.getDoubleValue() * c1 * s234 * c5;
      double tau3 = tau4 - a3.getDoubleValue() * c1 * s23;
      double tau2 = tau3 - a2.getDoubleValue() * c1 * s2;
      double tau1 = -a1.getDoubleValue() * s1 - a2.getDoubleValue() * s1 * c2 - a3.getDoubleValue() * s1 * c23 - a4.getDoubleValue() * s1 * c234 * c5
                    - a5.getDoubleValue() * c1 * s5;

      tau_shoulder_yaw.add(tau1);
      tau_shoulder_pitch.add(tau2);
      tau_elbow_pitch.add(tau3);
      tau_wrist_pitch.add(tau4);
      tau_wrist_yaw.add(tau5);

   }

   public void doDampingControl()
   {
      tau_shoulder_pitch.add(-b_shoulder_pitch.getDoubleValue() * qd_shoulder_pitch.getDoubleValue());
      tau_shoulder_yaw.add(-b_shoulder_yaw.getDoubleValue() * qd_shoulder_yaw.getDoubleValue());
      tau_elbow_pitch.add(-b_elbow_pitch.getDoubleValue() * qd_elbow_pitch.getDoubleValue());
      tau_wrist_pitch.add(-b_wrist_pitch.getDoubleValue() * qd_wrist_pitch.getDoubleValue());
      tau_wrist_yaw.add(-b_wrist_yaw.getDoubleValue() * qd_wrist_yaw.getDoubleValue());
      tau_wrist_roll.add(-b_wrist_roll.getDoubleValue() * qd_wrist_roll.getDoubleValue());
   }

   public void doPDPositionControl()
   {
      tau_shoulder_pitch.add(k_shoulder_pitch.getDoubleValue() * (q_d_shoulder_pitch.getDoubleValue() - q_shoulder_pitch.getDoubleValue())
                             - b_shoulder_pitch.getDoubleValue() * qd_shoulder_pitch.getDoubleValue());
      tau_shoulder_yaw.add(k_shoulder_yaw.getDoubleValue() * (q_d_shoulder_yaw.getDoubleValue() - q_shoulder_yaw.getDoubleValue())
                           - b_shoulder_yaw.getDoubleValue() * qd_shoulder_yaw.getDoubleValue());
      tau_elbow_pitch.add(k_elbow_pitch.getDoubleValue() * (q_d_elbow_pitch.getDoubleValue() - q_elbow_pitch.getDoubleValue())
                          - b_elbow_pitch.getDoubleValue() * qd_elbow_pitch.getDoubleValue());
      tau_wrist_pitch.add(k_wrist_pitch.getDoubleValue() * (q_d_wrist_pitch.getDoubleValue() - q_wrist_pitch.getDoubleValue())
                          - b_wrist_pitch.getDoubleValue() * qd_wrist_pitch.getDoubleValue());
      tau_wrist_yaw.add(k_wrist_yaw.getDoubleValue() * (q_d_wrist_yaw.getDoubleValue() - q_wrist_yaw.getDoubleValue())
                        - b_wrist_yaw.getDoubleValue() * qd_wrist_yaw.getDoubleValue());
      tau_wrist_roll.add(k_wrist_roll.getDoubleValue() * (q_d_wrist_roll.getDoubleValue() - q_wrist_roll.getDoubleValue())
                         - b_wrist_roll.getDoubleValue() * qd_wrist_roll.getDoubleValue());
   }

   public void doSineDesiredPositions()
   {
      q_d_shoulder_yaw.set(shoulder_yaw_amp.getDoubleValue()
                           * Math.sin(2.0 * Math.PI * shoulder_yaw_freq.getDoubleValue() * t.getDoubleValue() + shoulder_yaw_phase.getDoubleValue()));
      q_d_shoulder_pitch.set(shoulder_pitch_amp.getDoubleValue()
                             * Math.sin(2.0 * Math.PI * shoulder_pitch_freq.getDoubleValue() * t.getDoubleValue() + shoulder_pitch_phase.getDoubleValue()));

      q_d_elbow_pitch.set(elbow_pitch_amp.getDoubleValue()
                          * Math.sin(2.0 * Math.PI * elbow_pitch_freq.getDoubleValue() * t.getDoubleValue() + elbow_pitch_phase.getDoubleValue()));

      q_d_wrist_pitch.set(wrist_pitch_amp.getDoubleValue()
                          * Math.sin(2.0 * Math.PI * wrist_pitch_freq.getDoubleValue() * t.getDoubleValue() + wrist_pitch_phase.getDoubleValue()));
      q_d_wrist_yaw.set(wrist_yaw_amp.getDoubleValue()
                        * Math.sin(2.0 * Math.PI * wrist_yaw_freq.getDoubleValue() * t.getDoubleValue() + wrist_yaw_phase.getDoubleValue()));

      q_d_wrist_roll.set(wrist_roll_offset.getDoubleValue()
                         + wrist_roll_amp.getDoubleValue()
                           * Math.sin(2.0 * Math.PI * wrist_roll_freq.getDoubleValue() * t.getDoubleValue() + wrist_roll_phase.getDoubleValue()));
   }

   public void doAntiGravityControl()
   {
      // Inverse gravity:
      doInverseGravity();
      rampTorques();
   }

   public void doPositionControl()
   {
      // PD control:
      doPDPositionControl();

      // Inverse gravity:
      // doInverseGravity();

      rampTorques();
   }

   public void doDampingPlusAntiGravityControl()
   {
      // Damping control:
      doDampingControl();

      // Inverse gravity:
      doInverseGravity();

      rampTorques();
   }

   public void doPositionPlusAntiGravityControl()
   {
      // PD control:
      doPDPositionControl();

      // Inverse gravity:
      doInverseGravity();

      rampTorques();
   }

   public void doSineTrajectoryControl()
   {
      doSineDesiredPositions();

      // Plus PD control:
      doPDPositionControl();

      // Inverse gravity:
      doInverseGravity();

      rampTorques();
   }

   private static final double
      MAX_SHOULDER_TAU = 40.0, MAX_ELBOW_TAU = 25.0, MAX_WRIST_TAU = 25.0;
   private static final double MAX_WRIST_ROLL_TAU = 1.2;

   public void limitTorques()
   {
      if (tau_shoulder_yaw.getDoubleValue() > MAX_SHOULDER_TAU)
         tau_shoulder_yaw.set(MAX_SHOULDER_TAU);
      if (tau_shoulder_yaw.getDoubleValue() < -MAX_SHOULDER_TAU)
         tau_shoulder_yaw.set(-MAX_SHOULDER_TAU);

      if (tau_shoulder_pitch.getDoubleValue() > MAX_SHOULDER_TAU)
         tau_shoulder_pitch.set(MAX_SHOULDER_TAU);
      if (tau_shoulder_pitch.getDoubleValue() < -MAX_SHOULDER_TAU)
         tau_shoulder_pitch.set(-MAX_SHOULDER_TAU);

      if (tau_elbow_pitch.getDoubleValue() > MAX_ELBOW_TAU)
         tau_elbow_pitch.set(MAX_ELBOW_TAU);
      if (tau_elbow_pitch.getDoubleValue() < -MAX_ELBOW_TAU)
         tau_elbow_pitch.set(-MAX_ELBOW_TAU);

      if (tau_wrist_yaw.getDoubleValue() > MAX_WRIST_TAU)
         tau_wrist_yaw.set(MAX_WRIST_TAU);
      if (tau_wrist_yaw.getDoubleValue() < -MAX_WRIST_TAU)
         tau_wrist_yaw.set(-MAX_WRIST_TAU);

      if (tau_wrist_pitch.getDoubleValue() > MAX_WRIST_TAU)
         tau_wrist_pitch.set(MAX_WRIST_TAU);
      if (tau_wrist_pitch.getDoubleValue() < -MAX_WRIST_TAU)
         tau_wrist_pitch.set(-MAX_WRIST_TAU);

      if (tau_wrist_roll.getDoubleValue() > MAX_WRIST_ROLL_TAU)
         tau_wrist_roll.set(MAX_WRIST_ROLL_TAU);
      if (tau_wrist_roll.getDoubleValue() < -MAX_WRIST_ROLL_TAU)
         tau_wrist_roll.set(-MAX_WRIST_ROLL_TAU);

   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }
}
