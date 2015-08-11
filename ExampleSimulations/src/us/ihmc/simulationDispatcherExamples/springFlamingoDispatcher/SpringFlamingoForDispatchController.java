package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class SpringFlamingoForDispatchController implements RobotController
{
   // private SpringFlamingoRobot rob;

   // These are the variables that are automatically created when the robot is created:
   DoubleYoVariable t;
   DoubleYoVariable q_x, q_z, q_pitch, q_rh, q_rk, q_ra, q_lh, q_lk, q_la;
   DoubleYoVariable qd_x, qd_z, qd_pitch, qd_rh, qd_rk, qd_ra, qd_lh, qd_lk, qd_la;
   DoubleYoVariable tau_rh, tau_rk, tau_ra, tau_lh, tau_lk, tau_la;

   DoubleYoVariable gc_lheel_x, gc_lheel_fz, gc_lheel_fs;
   DoubleYoVariable gc_ltoe_x, gc_ltoe_fz, gc_ltoe_fs;
   DoubleYoVariable gc_rheel_x, gc_rheel_fz, gc_rheel_fs;
   DoubleYoVariable gc_rtoe_x, gc_rtoe_fz, gc_rtoe_fs;

   // These are the control variables which need to be created now:

   YoVariableRegistry registry = new YoVariableRegistry("SpringFlamingoController");

   DoubleYoVariable vel = new DoubleYoVariable("vel", registry), stand_gain = new DoubleYoVariable("stand_gain", registry),
                    x_d = new DoubleYoVariable("x_d", registry), v_nom = new DoubleYoVariable("v_nom", registry);
   DoubleYoVariable vtp_gain = new DoubleYoVariable("vtp_gain", registry);
   DoubleYoVariable t_d = new DoubleYoVariable("t_d", registry), t_gain = new DoubleYoVariable("t_gain", registry),
                    t_damp = new DoubleYoVariable("t_damp", registry);
   DoubleYoVariable knee_d = new DoubleYoVariable("knee_d", registry), knee_gain = new DoubleYoVariable("knee_gain", registry),
                    knee_damp = new DoubleYoVariable("knee_damp", registry);
   DoubleYoVariable hip_d = new DoubleYoVariable("hip_d", registry), hip_gain = new DoubleYoVariable("hip_gain", registry),
                    hip_damp = new DoubleYoVariable("hip_damp", registry), hip_hold = new DoubleYoVariable("hip_hold", registry);
   DoubleYoVariable swing_gain_knee = new DoubleYoVariable("swing_gain_knee", registry), swing_damp_knee = new DoubleYoVariable("swing_damp_knee", registry),
                    force_thresh = new DoubleYoVariable("force_thresh", registry);
   DoubleYoVariable min_support_time = new DoubleYoVariable("min_support_time", registry);
   DoubleYoVariable swing_time = new DoubleYoVariable("swing_time", registry), toe_off_time = new DoubleYoVariable("toe_off_time", registry);
   DoubleYoVariable ankle_d = new DoubleYoVariable("ankle_d", registry), ankle_gain = new DoubleYoVariable("ankle_gain", registry),
                    ankle_damp = new DoubleYoVariable("ankle_damp", registry);
   DoubleYoVariable ankle_limit_set = new DoubleYoVariable("ankle_limit_set", registry), ankle_limit_gain = new DoubleYoVariable("ankle_limit_gain", registry);
   DoubleYoVariable left_state = new DoubleYoVariable("left_state", registry), right_state = new DoubleYoVariable("right_state", registry),
                    left_force = new DoubleYoVariable("left_force", registry), left_cop = new DoubleYoVariable("left_cop", registry);
   DoubleYoVariable right_force = new DoubleYoVariable("right_force", registry), right_cop = new DoubleYoVariable("right_cop", registry),
                    left_hip_set = new DoubleYoVariable("left_hip_set", registry);
   DoubleYoVariable left_switch_time = new DoubleYoVariable("left_switch_time", registry);
   DoubleYoVariable left_heel = new DoubleYoVariable("left_heel", registry), right_hip_set = new DoubleYoVariable("right_hip_set", registry),
                    right_switch_time = new DoubleYoVariable("right_switch_time", registry);
   DoubleYoVariable right_heel = new DoubleYoVariable("right_heel", registry);
   DoubleYoVariable toe_off_ankle_thresh = new DoubleYoVariable("toe_off_ankle_thresh", registry), ankle_push = new DoubleYoVariable("ankle_push", registry),
                    push_gain = new DoubleYoVariable("push_gain", registry);
   DoubleYoVariable knee_collapse = new DoubleYoVariable("knee_collapse", registry), push_set = new DoubleYoVariable("push_set", registry),
                    push_damp = new DoubleYoVariable("push_damp", registry);
   DoubleYoVariable pow_lh = new DoubleYoVariable("pow_lh", registry), pow_lk = new DoubleYoVariable("pow_lk", registry),
                    pow_la = new DoubleYoVariable("pow_la", registry);
   DoubleYoVariable pow_rh = new DoubleYoVariable("pow_rh", registry), pow_rk = new DoubleYoVariable("pow_rk", registry),
                    pow_ra = new DoubleYoVariable("pow_ra", registry);
   DoubleYoVariable power = new DoubleYoVariable("power", registry), energy = new DoubleYoVariable("energy", registry);
   DoubleYoVariable act_lh = new DoubleYoVariable("act_lh", registry), act_lk = new DoubleYoVariable("act_lk", registry),
                    act_la = new DoubleYoVariable("act_la", registry), pas_la = new DoubleYoVariable("pas_la", registry);
   DoubleYoVariable act_rh = new DoubleYoVariable("act_rh", registry), act_rk = new DoubleYoVariable("act_rk", registry),
                    act_ra = new DoubleYoVariable("act_ra", registry), pas_ra = new DoubleYoVariable("pas_ra", registry);
   DoubleYoVariable pas_lh = new DoubleYoVariable("pas_lh", registry), pas_lk = new DoubleYoVariable("pas_lk", registry),
                    pas_rh = new DoubleYoVariable("pas_rh", registry), pas_rk = new DoubleYoVariable("pas_rk", registry);
   DoubleYoVariable max_hip_torque = new DoubleYoVariable("max_hip_torque", registry);


   // These are the control variables which we want to have access with on the GUI:

   DoubleYoVariable[] controlVars = new DoubleYoVariable[]
   {
      vel, stand_gain, x_d, v_nom, vtp_gain, t_d, t_gain, t_damp, knee_d, knee_gain, knee_damp, hip_d, hip_gain, hip_damp, hip_hold, swing_gain_knee,
      swing_damp_knee, force_thresh, min_support_time, swing_time, toe_off_time, ankle_d, ankle_gain, ankle_damp, ankle_limit_set, ankle_limit_gain, left_state,
      right_state, left_force, left_cop, right_force, right_cop, left_hip_set, left_switch_time, left_heel, right_hip_set, right_switch_time, right_heel,
      toe_off_ankle_thresh, ankle_push, push_gain, knee_collapse, push_set, push_damp, pow_lh, pow_lk, pow_la, pow_rh, pow_rk, pow_ra, power, energy, act_lh,
      act_lk, act_la, pas_la, act_rh, act_rk, act_ra, pas_ra, pas_lh, pas_lk, pas_rh, pas_rk, max_hip_torque
   };

   private final int SUPPORT = 0, TOE_OFF = 1, SWING = 2, STRAIGHTEN = 3;

   public SpringFlamingoForDispatchController(SpringFlamingoForDispatchRobot rob)
   {
      // this.rob = rob;
      initControl(rob);
   }

   public DoubleYoVariable[] getControlVars()
   {
      return this.controlVars;
   }

   private void initControl(SpringFlamingoForDispatchRobot rob)
   {
      // Get the variables that are stored with the robot:

      t = (DoubleYoVariable) rob.getVariable("t");
      q_x = (DoubleYoVariable) rob.getVariable("q_x");
      q_z = (DoubleYoVariable) rob.getVariable("q_z");
      q_pitch = (DoubleYoVariable) rob.getVariable("q_pitch");
      q_rh = (DoubleYoVariable) rob.getVariable("q_rh");
      q_rk = (DoubleYoVariable) rob.getVariable("q_rk");
      q_ra = (DoubleYoVariable) rob.getVariable("q_ra");
      q_lh = (DoubleYoVariable) rob.getVariable("q_lh");
      q_lk = (DoubleYoVariable) rob.getVariable("q_lk");
      q_la = (DoubleYoVariable) rob.getVariable("q_la");

      qd_x = (DoubleYoVariable) rob.getVariable("qd_x");
      qd_z = (DoubleYoVariable) rob.getVariable("qd_z");
      qd_pitch = (DoubleYoVariable) rob.getVariable("qd_pitch");
      qd_rh = (DoubleYoVariable) rob.getVariable("qd_rh");
      qd_rk = (DoubleYoVariable) rob.getVariable("qd_rk");
      qd_ra = (DoubleYoVariable) rob.getVariable("qd_ra");
      qd_lh = (DoubleYoVariable) rob.getVariable("qd_lh");
      qd_lk = (DoubleYoVariable) rob.getVariable("qd_lk");
      qd_la = (DoubleYoVariable) rob.getVariable("qd_la");

      tau_rh = (DoubleYoVariable) rob.getVariable("tau_rh");
      tau_rk = (DoubleYoVariable) rob.getVariable("tau_rk");
      tau_ra = (DoubleYoVariable) rob.getVariable("tau_ra");
      tau_lh = (DoubleYoVariable) rob.getVariable("tau_lh");
      tau_lk = (DoubleYoVariable) rob.getVariable("tau_lk");
      tau_la = (DoubleYoVariable) rob.getVariable("tau_la");

      gc_lheel_x = (DoubleYoVariable) rob.getVariable("gc_lheel_x");
      gc_lheel_fz = (DoubleYoVariable) rob.getVariable("gc_lheel_fz");
      gc_lheel_fs = (DoubleYoVariable) rob.getVariable("gc_lheel_fs");
      gc_ltoe_x = (DoubleYoVariable) rob.getVariable("gc_ltoe_x");
      gc_ltoe_fz = (DoubleYoVariable) rob.getVariable("gc_ltoe_fz");
      gc_ltoe_fs = (DoubleYoVariable) rob.getVariable("gc_ltoe_fs");
      gc_rheel_x = (DoubleYoVariable) rob.getVariable("gc_rheel_x");
      gc_rheel_fz = (DoubleYoVariable) rob.getVariable("gc_rheel_fz");
      gc_rheel_fs = (DoubleYoVariable) rob.getVariable("gc_rheel_fs");
      gc_rtoe_x = (DoubleYoVariable) rob.getVariable("gc_rtoe_x");
      gc_rtoe_fz = (DoubleYoVariable) rob.getVariable("gc_rtoe_fz");
      gc_rtoe_fs = (DoubleYoVariable) rob.getVariable("gc_rtoe_fs");

      // Initialize the variables.  This will be put in state files later...

      vel.set(0.745599);
      tau_rh.set(7.48314);
      tau_rk.set(-0.0852298);
      tau_ra.set(-9.32007);
      tau_lh.set(-2.86766);
      tau_lk.set(1.05104);
      tau_la.set(0.377569);

      stand_gain.set(0);
      x_d.set(0);
      v_nom.set(-0.4);
      vtp_gain.set(0);
      t_d.set(0);
      t_gain.set(100);
      t_damp.set(20);
      knee_d.set(0);
      knee_gain.set(30);
      knee_damp.set(10);
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
      ankle_d.set(0);
      ankle_gain.set(4);
      ankle_damp.set(1);
      ankle_limit_set.set(0);
      ankle_limit_gain.set(250.98);
      left_state.set(3);
      right_state.set(0);
      left_force.set(0);
      left_cop.set(0.5);
      right_force.set(131.83);
      right_cop.set(0.557913);
      left_hip_set.set(0.358627);
      left_switch_time.set(9.8056);
      left_heel.set(-0.392589);
      right_hip_set.set(0.358627);
      right_switch_time.set(9.2432);
      right_heel.set(-0.231515);
      toe_off_ankle_thresh.set(0.1);
      ankle_push.set(-0.3);
      push_gain.set(9.56078);
      knee_collapse.set(0);
      push_set.set(-0.236471);
      push_damp.set(0);
      pow_lh.set(-2.20136);
      pow_lk.set(-2.31217);
      pow_la.set(-0.749179);
      pow_rh.set(-8.24497);
      pow_rk.set(0.00157622);
      pow_ra.set(-0);
      power.set(13.5093);
      energy.set(82.178);
      act_lh.set(-2.86766);
      act_lk.set(1.05104);
      act_la.set(0.377569);
      pas_la.set(0);
      act_rh.set(7.48314);
      act_rk.set(-0.0852298);
      act_ra.set(-0);
      pas_ra.set(-9.32007);
      pas_lh.set(0);
      pas_lk.set(0);
      pas_rh.set(0);
      pas_rk.set(0);
      max_hip_torque.set(8.57647);
      t.set(10.00);

      q_x.set(-6.68245);
      qd_x.set(-0.745821);
      q_z.set(0.859601);
      qd_z.set(-0.127678);
      q_pitch.set(0.0158429);
      qd_pitch.set(0.293524);
      q_rh.set(-0.210433);
      qd_rh.set(-1.10499);
      tau_rh.set(7.48314);
      q_rk.set(-1.43749e-05);
      qd_rk.set(0.0279385);
      tau_rk.set(-0.0852298);
      q_ra.set(0.19298);
      qd_ra.set(0.685901);
      tau_ra.set(-9.32007);
      q_lh.set(0.443428);
      qd_lh.set(0.727701);
      tau_lh.set(-2.86766);
      q_lk.set(-0.100803);
      qd_lk.set(-2.09991);
      tau_lk.set(1.05104);
      q_la.set(0.0329076);
      qd_la.set(-1.93702);
      tau_la.set(0.377569);
   }

   public void doControl()
   {
      balistic_walking_state_machine();
   }

   private double passive_ankle_torques(double pos, double vel)
   {
      if (pos > ankle_limit_set.getDoubleValue())
         return (-ankle_limit_gain.getDoubleValue() * (ankle_limit_set.getDoubleValue() - pos) * (ankle_limit_set.getDoubleValue() - pos));
      else
         return (0.0);
   }


   private double toe_off_ankle_torques(double pos, double vel)
   {
      return (push_gain.getDoubleValue() * (push_set.getDoubleValue() - pos) - push_damp.getDoubleValue() * vel);
   }

   private void balistic_walking_state_machine()
   {
      /*
       *  Robot happens to walk in negative x direction.  Set vel positive just
       *  so it makes intuitive sense.
       */

      vel.set(-qd_x.getDoubleValue());

      /* Calculate forces on the feet */

      left_force.set(gc_lheel_fz.getDoubleValue() + gc_ltoe_fz.getDoubleValue());
      if (left_force.getDoubleValue() > 5.0)
         left_cop.set(gc_ltoe_fz.getDoubleValue() / left_force.getDoubleValue());
      else
         left_cop.set(0.5);

      right_force.set(gc_rheel_fz.getDoubleValue() + gc_rtoe_fz.getDoubleValue());
      if (right_force.getDoubleValue() > 5.0)
         right_cop.set(gc_rtoe_fz.getDoubleValue() / right_force.getDoubleValue());
      else
         right_cop.set(0.5);


      /* Actions in Each State */


      if (left_state.getDoubleValue() == SUPPORT)
      {
         /* Use hip to servo pitch */
         if (left_force.getDoubleValue() > 20.0)
            act_lh.set(-t_gain.getDoubleValue() * (t_d.getDoubleValue() - q_pitch.getDoubleValue()) + t_damp.getDoubleValue() * qd_pitch.getDoubleValue());

         /* Keep knee straight */
         act_lk.set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - q_lk.getDoubleValue()) - knee_damp.getDoubleValue() * qd_lk.getDoubleValue());

         /* Use ankle to servo speed, position */
         act_la.set(-stand_gain.getDoubleValue() * (x_d.getDoubleValue() - q_x.getDoubleValue())
                    - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - qd_x.getDoubleValue()));

         /* Ankle limit to go on toes and maintain cop */
         pas_la.set(passive_ankle_torques(q_la.getDoubleValue(), qd_la.getDoubleValue()));

      }

      else if (left_state.getDoubleValue() == TOE_OFF)
      {
         /* Use hip to servo pitch */
         act_lh.set(-t_gain.getDoubleValue() * (t_d.getDoubleValue() - q_pitch.getDoubleValue()) + t_damp.getDoubleValue() * qd_pitch.getDoubleValue());

         /* Keep knee straight */
         act_lk.set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - q_lk.getDoubleValue()) - knee_damp.getDoubleValue() * qd_lk.getDoubleValue());

         /* Use ankle to servo speed, position */
         act_la.set(-stand_gain.getDoubleValue() * (x_d.getDoubleValue() - q_x.getDoubleValue())
                    - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - qd_x.getDoubleValue()));

         /* Ankle limit to go on toes and maintain cop */
         pas_la.set(passive_ankle_torques(q_la.getDoubleValue(), qd_la.getDoubleValue()));

         /* Ankle push off */
         act_la.set(act_la.getDoubleValue() + toe_off_ankle_torques(q_la.getDoubleValue(), qd_la.getDoubleValue()));
      }

      else if (left_state.getDoubleValue() == SWING)
      {
         /* Servo hip up */
         left_hip_set.set(hip_d.getDoubleValue());
         act_lh.set(hip_gain.getDoubleValue() * (left_hip_set.getDoubleValue() - q_lh.getDoubleValue()) - hip_damp.getDoubleValue() * qd_lh.getDoubleValue());

         /* Damp the knee */
         act_lk.set(-swing_damp_knee.getDoubleValue() * qd_lk.getDoubleValue());

         /* act_lk.set(0.0); */

         /* Damp and Toggle the knee: */
         act_lk.set(act_lk.getDoubleValue() + knee_collapse.getDoubleValue());

         /* Continue toe off until toe leaves ground */
         if (gc_ltoe_fs.getDoubleValue() > 0.1)
         {
            pas_la.set(passive_ankle_torques(q_la.getDoubleValue(), qd_la.getDoubleValue()));
            act_la.set(toe_off_ankle_torques(q_la.getDoubleValue(), qd_la.getDoubleValue()));
         }
         else
         {
            /* Servo ankle level to the ground */
            left_heel.set(-q_pitch.getDoubleValue() - q_lh.getDoubleValue() - q_lk.getDoubleValue() - q_la.getDoubleValue());
            act_la.set(-ankle_gain.getDoubleValue() * (ankle_d.getDoubleValue() - left_heel.getDoubleValue())
                       - ankle_damp.getDoubleValue() * qd_la.getDoubleValue());
         }
      }


      else if (left_state.getDoubleValue() == STRAIGHTEN)
      {
         /* Servo hip to a more shallow angle */
         left_hip_set.set(hip_hold.getDoubleValue());
         act_lh.set(hip_gain.getDoubleValue() * (left_hip_set.getDoubleValue() - q_lh.getDoubleValue()) - hip_damp.getDoubleValue() * qd_lh.getDoubleValue());

         /* Keep knee straight */
         act_lk.set(swing_gain_knee.getDoubleValue() * (knee_d.getDoubleValue() - q_lk.getDoubleValue())
                    - swing_damp_knee.getDoubleValue() * qd_lk.getDoubleValue());

         /* Servo ankle level to the ground */
         left_heel.set(-q_pitch.getDoubleValue() - q_lh.getDoubleValue() - q_lk.getDoubleValue() - q_la.getDoubleValue());
         act_la.set(-ankle_gain.getDoubleValue() * (ankle_d.getDoubleValue() - left_heel.getDoubleValue())
                    - ankle_damp.getDoubleValue() * qd_la.getDoubleValue());
      }


      if (right_state.getDoubleValue() == SUPPORT)
      {
         /* Use hip to servo pitch */
         if (right_force.getDoubleValue() > 20.0)
            act_rh.set(-t_gain.getDoubleValue() * (t_d.getDoubleValue() - q_pitch.getDoubleValue()) + t_damp.getDoubleValue() * qd_pitch.getDoubleValue());

         /* Keep knee straight */
         act_rk.set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - q_rk.getDoubleValue()) - knee_damp.getDoubleValue() * qd_rk.getDoubleValue());

         /* Use ankle to servo speed, position */
         act_ra.set(-stand_gain.getDoubleValue() * (x_d.getDoubleValue() - q_x.getDoubleValue())
                    - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - qd_x.getDoubleValue()));

         /* Ankle limit to go on toes and maintain cop */
         pas_ra.set(passive_ankle_torques(q_ra.getDoubleValue(), qd_ra.getDoubleValue()));

      }

      else if (right_state.getDoubleValue() == TOE_OFF)
      {
         /* Use hip to servo pitch */
         act_rh.set(-t_gain.getDoubleValue() * (t_d.getDoubleValue() - q_pitch.getDoubleValue()) + t_damp.getDoubleValue() * qd_pitch.getDoubleValue());

         /* Keep knee straight */
         act_rk.set(knee_gain.getDoubleValue() * (knee_d.getDoubleValue() - q_rk.getDoubleValue()) - knee_damp.getDoubleValue() * qd_rk.getDoubleValue());

         /* Use ankle to servo speed, position */
         act_ra.set(-stand_gain.getDoubleValue() * (x_d.getDoubleValue() - q_x.getDoubleValue())
                    - vtp_gain.getDoubleValue() * (v_nom.getDoubleValue() - qd_x.getDoubleValue()));

         /* Ankle limit to go on toes and maintain cop */
         pas_ra.set(passive_ankle_torques(q_ra.getDoubleValue(), qd_ra.getDoubleValue()));

         /* Ankle push off */
         act_ra.set(act_ra.getDoubleValue() + toe_off_ankle_torques(q_ra.getDoubleValue(), qd_ra.getDoubleValue()));
      }

      else if (right_state.getDoubleValue() == SWING)
      {
         /* Servo hip up */
         right_hip_set.set(hip_d.getDoubleValue());
         act_rh.set(hip_gain.getDoubleValue() * (right_hip_set.getDoubleValue() - q_rh.getDoubleValue()) - hip_damp.getDoubleValue() * qd_rh.getDoubleValue());

         /* Damp the knee */
         act_rk.set(-swing_damp_knee.getDoubleValue() * qd_rk.getDoubleValue());

         /* Damp and Toggle the knee */
         act_rk.set(act_rk.getDoubleValue() + knee_collapse.getDoubleValue());

         /* Continue toe off until toe leaves ground */
         if (gc_rtoe_fs.getDoubleValue() > 0.1)
         {
            pas_ra.set(passive_ankle_torques(q_ra.getDoubleValue(), qd_ra.getDoubleValue()));
            act_ra.set(toe_off_ankle_torques(q_ra.getDoubleValue(), qd_ra.getDoubleValue()));
         }
         else
         {
            /* Servo ankle level to the ground */
            right_heel.set(-q_pitch.getDoubleValue() - q_rh.getDoubleValue() - q_rk.getDoubleValue() - q_ra.getDoubleValue());
            act_ra.set(-ankle_gain.getDoubleValue() * (ankle_d.getDoubleValue() - right_heel.getDoubleValue())
                       - ankle_damp.getDoubleValue() * qd_ra.getDoubleValue());
         }
      }


      else if (right_state.getDoubleValue() == STRAIGHTEN)
      {
         /* Servo hip to a more shallow angle */
         right_hip_set.set(hip_hold.getDoubleValue());
         act_rh.set(hip_gain.getDoubleValue() * (right_hip_set.getDoubleValue() - q_rh.getDoubleValue()) - hip_damp.getDoubleValue() * qd_rh.getDoubleValue());

         /* Keep knee straight */
         act_rk.set(swing_gain_knee.getDoubleValue() * (knee_d.getDoubleValue() - q_rk.getDoubleValue())
                    - swing_damp_knee.getDoubleValue() * qd_rk.getDoubleValue());

         /* Servo ankle level to the ground */
         right_heel.set(-q_pitch.getDoubleValue() - q_rh.getDoubleValue() - q_rk.getDoubleValue() - q_ra.getDoubleValue());
         act_ra.set(-ankle_gain.getDoubleValue() * (ankle_d.getDoubleValue() - right_heel.getDoubleValue())
                    - ankle_damp.getDoubleValue() * qd_ra.getDoubleValue());

      }


      /* Communication between legs for now... */

      /*
       *  Subtract swing hip torque from the stance leg so less disturbance
       *  on the body
       */
      if (left_state.getDoubleValue() == SWING)
         act_rh.set(act_rh.getDoubleValue() - act_lh.getDoubleValue());

      if (right_state.getDoubleValue() == SWING)
         act_lh.set(act_lh.getDoubleValue() - act_rh.getDoubleValue());


      /* Transition Conditions */


      if (left_state.getDoubleValue() == SUPPORT)
      {
         /* Transition if unloaded and travelling away from the foot */
         if ((gc_lheel_fz.getDoubleValue() < force_thresh.getDoubleValue())
                 && ((qd_x.getDoubleValue() < 0.0) && (gc_lheel_x.getDoubleValue() > q_x.getDoubleValue()))
                 && ((gc_lheel_x.getDoubleValue() > gc_rheel_x.getDoubleValue()))
                 && (t.getDoubleValue() > left_switch_time.getDoubleValue() + min_support_time.getDoubleValue()))
         {
            left_state.set(TOE_OFF);
            left_switch_time.set(t.getDoubleValue());
         }
      }

      else if (left_state.getDoubleValue() == TOE_OFF)
      {
         if (left_force.getDoubleValue() < force_thresh.getDoubleValue())
         {
            left_state.set(SWING);
            left_switch_time.set(t.getDoubleValue());
         }
      }

      else if (left_state.getDoubleValue() == SWING)
      {
         if (t.getDoubleValue() > left_switch_time.getDoubleValue() + swing_time.getDoubleValue())
         {
            left_state.set(STRAIGHTEN);
            left_switch_time.set(t.getDoubleValue());
         }
      }

      else if (left_state.getDoubleValue() == STRAIGHTEN)
      {
         if ((gc_ltoe_fs.getDoubleValue() == 1.0) || (gc_lheel_fs.getDoubleValue() == 1.0))
         {
            left_state.set(SUPPORT);
            left_switch_time.set(t.getDoubleValue());
         }
      }



      if (right_state.getDoubleValue() == SUPPORT)
      {
         /* Transition if unloaded and travelling away from the foot */
         if ((gc_rheel_fz.getDoubleValue() < force_thresh.getDoubleValue())
                 && ((qd_x.getDoubleValue() < 0.0) && (gc_rheel_x.getDoubleValue() > q_x.getDoubleValue()))
                 && ((gc_rheel_x.getDoubleValue() > gc_lheel_x.getDoubleValue()))
                 && (t.getDoubleValue() > right_switch_time.getDoubleValue() + min_support_time.getDoubleValue()))
         {
            right_state.set(TOE_OFF);
            right_switch_time.set(t.getDoubleValue());
         }
      }

      else if (right_state.getDoubleValue() == TOE_OFF)
      {
         /* Once load on foot falls below a threshhold */
         if (right_force.getDoubleValue() < force_thresh.getDoubleValue())
         {
            right_state.set(SWING);
            right_switch_time.set(t.getDoubleValue());
         }
      }

      else if (right_state.getDoubleValue() == SWING)
      {
         if (t.getDoubleValue() > right_switch_time.getDoubleValue() + swing_time.getDoubleValue())
         {
            right_state.set(STRAIGHTEN);
            right_switch_time.set(t.getDoubleValue());
         }
      }

      else if (right_state.getDoubleValue() == STRAIGHTEN)
      {
         if ((gc_rtoe_fs.getDoubleValue() == 1.0) || (gc_rheel_fs.getDoubleValue() == 1.0))
         {
            right_state.set(SUPPORT);
            right_switch_time.set(t.getDoubleValue());
         }
      }

      /* Torques at the joints: */

      tau_lh.set(act_lh.getDoubleValue() + pas_lh.getDoubleValue());
      tau_lk.set(act_lk.getDoubleValue() + pas_lk.getDoubleValue());
      tau_la.set(act_la.getDoubleValue() + pas_la.getDoubleValue());

      tau_rh.set(act_rh.getDoubleValue() + pas_rh.getDoubleValue());
      tau_rk.set(act_rk.getDoubleValue() + pas_rk.getDoubleValue());
      tau_ra.set(act_ra.getDoubleValue() + pas_ra.getDoubleValue());

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return "SpringFlamingoController";
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }

}
