package us.ihmc.moonwalking.models.RaibertHopper;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class RaibertHopperController implements RobotController
{
  private enum States{LOADING, COMPRESSION, THRUST, UNLOADING, ASCENT, DESCENT};
  private final YoVariableRegistry yoVariableRegistry;
  private final RaibertHopperRobot raibertHopper;

  private final YoVariable t, raibertHopperState, switchTime;

  private final YoVariable q_d_knee_compress, q_d_knee_ascent, q_d_knee_thrust;


  private YoVariable hopping_gain_p;
  private YoVariable hopping_gain_v;
  private YoVariable speed_gain;
  private YoVariable hip_gain_p;
  private YoVariable hip_gain_v;
  private YoVariable pitch_gain_pc;
  private YoVariable pitch_gain_vc;
  private YoVariable spring_p;
  private YoVariable spring_v;

  private YoVariable gc_heel_z;
  private YoVariable gc_heel_fs;

  private YoVariable q_pitch;
  private YoVariable q_z;
  private YoVariable qd_x;
  private YoVariable qd_z;
  private YoVariable qdd_z;
  private YoVariable qd_pitch;

  private YoVariable q_hip;
  private YoVariable q_knee;
  private YoVariable qd_hip;
  private YoVariable qd_knee;
  private YoVariable tau_hip;
  private YoVariable tau_knee;

  private YoVariable stanceTime;
  private YoVariable desiredForwardVelocity;
  private YoVariable thighAngleAtTouchdown;
  private YoVariable bodyAngleAtTakeOff;

  public RaibertHopperController(RaibertHopperRobot raibertHopper)
  {
    t = raibertHopper.getVariable("t");

    yoVariableRegistry = new YoVariableRegistry("controller");
    this.raibertHopper = raibertHopper;
    this.raibertHopperState = new YoVariable("raibertHopperState", States.values(), yoVariableRegistry);
    this.switchTime = new YoVariable("switchTime", yoVariableRegistry);

    this.q_d_knee_compress = new YoVariable("q_d_knee_compress", yoVariableRegistry);
    this.q_d_knee_ascent = new YoVariable("q_d_knee_ascent", yoVariableRegistry);
    this.q_d_knee_thrust = new YoVariable("q_d_knee_thrust", yoVariableRegistry);

    this.hopping_gain_p = new YoVariable("hoping_gain_p", yoVariableRegistry);
    this.hopping_gain_v = new YoVariable("hopping_gain_v", yoVariableRegistry);
    this.speed_gain = new YoVariable("speed_gain", yoVariableRegistry);
    this.hip_gain_p = new YoVariable("hip_gain_p", yoVariableRegistry);
    this.hip_gain_v = new YoVariable("hip_gain_v", yoVariableRegistry);
    this.pitch_gain_pc = new YoVariable("pitch_gain_pc", yoVariableRegistry);
    this.pitch_gain_vc = new YoVariable("pitch_gain_vc", yoVariableRegistry);
    this.stanceTime = new YoVariable("stanceTime", yoVariableRegistry);
    this.spring_p = new YoVariable("spring_p", yoVariableRegistry);
    this.spring_v = new YoVariable("spring_v", yoVariableRegistry);
    this.desiredForwardVelocity = new YoVariable("desiredForwardVelocity", yoVariableRegistry);
    this.thighAngleAtTouchdown = new YoVariable("thighAngleAtTouchdown", yoVariableRegistry);
    this.bodyAngleAtTakeOff = new YoVariable("bodyAngleAtTakeOff", yoVariableRegistry);

    this.gc_heel_z = raibertHopper.getVariable("gc_heel_z");
    this.gc_heel_fs = raibertHopper.getVariable("gc_heel_fs");

    this.qdd_z = raibertHopper.getVariable("qdd_z");
    this.qd_z = raibertHopper.getVariable("qd_z");
    this.q_z = raibertHopper.getVariable("q_z");
    this.qd_x = raibertHopper.getVariable("qd_x");
    this.q_pitch = raibertHopper.getVariable("q_pitch");
    this.qd_pitch = raibertHopper.getVariable("qd_pitch");

    this.q_hip = raibertHopper.getVariable("q_hip");
    this.qd_hip = raibertHopper.getVariable("qd_hip");
    this.q_knee = raibertHopper.getVariable("q_knee");
    this.qd_knee = raibertHopper.getVariable("qd_knee");
    this.tau_hip = raibertHopper.getVariable("tau_hip");
    this.tau_knee = raibertHopper.getVariable("tau_knee");

    initControl();
  }

  private void initControl()
  {
    q_d_knee_compress.val = 0.02;
    q_d_knee_thrust.val = 0.0;
    q_d_knee_ascent.val = 0.0;

    spring_p.val = 5000.0;
    spring_v.val = 15.0;
    hopping_gain_p.val = 6500.0;//1500.0;
    hopping_gain_v.val = 60.0;//50.0;
    pitch_gain_pc.val = 153.0;
    pitch_gain_vc.val = 14.0;
    speed_gain.val = 0.03;
    hip_gain_p.val = 47.0;
    hip_gain_v.val = 1.26;
    stanceTime.val = 0.05;
    desiredForwardVelocity.val = 1.5;
    bodyAngleAtTakeOff.val = -0.05;
    thighAngleAtTouchdown.val = 0.0;

    raibertHopperState.set(States.DESCENT);
    switchTime.val = t.val;
  }

  public void doControl()
  {
    // Transition Conditions
    switch((States) raibertHopperState.getEnumValue())
    {
      case LOADING:
      {
        if(gc_heel_fs.val == 1.0)
        {
          raibertHopperState.set(States.COMPRESSION);
          switchTime.val = t.val;

        }
        break;
      }

      case COMPRESSION:
      {
        if((qd_knee.val < 0.0) && (t.val > switchTime.val + 0.05))
        {
          raibertHopperState.set(States.THRUST);
          switchTime.val = t.val;
        }
        break;
      }

      case THRUST:
      {
        if(q_z.val+0.2 > 0.65 && gc_heel_fs.val == 1.0)
        {
          raibertHopperState.set(States.UNLOADING);
          switchTime.val = t.val;
        }
        break;
      }

      case UNLOADING:
      {
        if(gc_heel_fs.val == 0.0 && gc_heel_z.val > 0.0020)
        {
          raibertHopperState.set(States.ASCENT);
          switchTime.val = t.val;
        }
        break;
      }

      case ASCENT:
      {
        if(qd_z.val<0.0)
        {
          raibertHopperState.set(States.DESCENT);
          switchTime.val = t.val;
        }
        break;
      }

      case DESCENT:
      {
        if(gc_heel_z.val < 0.002)
        {
          raibertHopperState.set(States.LOADING);
          switchTime.val = t.val;
        }
        break;
      }


    }

//    Do the actions
    switch((States) raibertHopperState.getEnumValue())
        {
          case LOADING:
          {
            // Zero hip torque
            tau_hip.val = 0.0;
            break;
          }

          case COMPRESSION:
          {
            // Servo body attitude with hip
//            tau_hip.val = pitch_gain_pc.val * (0.0- q_pitch.val) - pitch_gain_vc.val * qd_pitch.val;
            // Leg acts as spring during compression.
            tau_knee.val = spring_p.val * (q_d_knee_compress.val - q_knee.val) - spring_v.val * qd_knee.val;
            break;
          }

          case THRUST:
          {
            // Servo body attitude with hip
//            tau_hip.val = pitch_gain_pc.val * (0.0-q_pitch.val) - pitch_gain_vc.val * qd_pitch.val;
            // Pressurize Thrust Leg
            tau_knee.val = spring_p.val * (0.0 - q_knee.val) - spring_v.val * qd_knee.val;
//            tau_knee.val = hopping_gain_p.val * (q_d_knee_thrust.val - q_knee.val) - hopping_gain_v.val * qd_knee.val;
            break;
          }

          case UNLOADING:
          {
            // Zero Hip torque
            tau_hip.val = 0.0;
            tau_knee.val = 0.0;
            break;
          }

          case ASCENT:
          {
            //Position leg for landing
//            tau_hip.val = -hip_gain_p.val * ((q_hip.val + q_pitch.val - Math.asin((qd_x.val * stanceTime.val)/(2.0 * (0.5-q_knee.val)) + speed_gain.val * (qd_x.val - desiredForwardVelocity.val)/(0.5-q_knee.val)))) - hip_gain_v.val * (qd_hip.val);
//            tau_knee.val = spring_p.val * (q_d_knee_ascent.val - q_knee.val) - spring_v.val * qd_knee.val;
//            tau_knee.val = hopping_gain_p.val * (q_d_knee_thrust.val - q_knee.val) - hopping_gain_v.val * qd_knee.val;

            break;
          }


          case DESCENT:
          {
            //Position leg for landing
//            tau_hip.val = -hip_gain_p.val * ((q_hip.val + q_pitch.val - Math.asin((qd_x.val * stanceTime.val)/(2.0 * (0.5-q_knee.val)) + speed_gain.val * (qd_x.val - desiredForwardVelocity.val)/(0.5-q_knee.val)))) - hip_gain_v.val * (qd_hip.val);
//            tau_knee.val = spring_p.val * (q_d_knee_ascent.val - q_knee.val) - spring_v.val * qd_knee.val;
            break;
          }
    }

  }

  public YoVariableRegistry getYoVariableRegistry()
  {
    return yoVariableRegistry;
  }

}
