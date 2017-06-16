package us.ihmc.exampleSimulations.stewartPlatform;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;

public class StewartPlatformController implements RobotController
{

   private StewartPlatformRobot rob;

   private YoDouble t, q_platform_x, q_platform_y, q_platform_z;
   private YoDouble qd_platform_x, qd_platform_y, qd_platform_z;

   private YoDouble qd_platform_wx, qd_platform_wy, qd_platform_wz;

   private YoDouble q_act0, q_act1, q_act2, q_act3, q_act4, q_act5;
   //private YoVariable qd_act0, qd_act1, qd_act2, qd_act3, qd_act4, qd_act5;
   private YoDouble tau_act0, tau_act1, tau_act2, tau_act3, tau_act4, tau_act5;

   private YoDouble ef_p_x[] = new YoDouble[6], ef_p_y[] = new YoDouble[6], ef_p_z[] = new YoDouble[6];
   private YoDouble ef_p_dx[] = new YoDouble[6], ef_p_dy[] = new YoDouble[6], ef_p_dz[] = new YoDouble[6];
   private YoDouble ef_p_fx[] = new YoDouble[6], ef_p_fy[] = new YoDouble[6], ef_p_fz[] = new YoDouble[6];

   private YoDouble ef_a_x[] = new YoDouble[6], ef_a_y[] = new YoDouble[6], ef_a_z[] = new YoDouble[6];
   private YoDouble ef_a_dx[] = new YoDouble[6], ef_a_dy[] = new YoDouble[6], ef_a_dz[] = new YoDouble[6];
   private YoDouble ef_a_fx[] = new YoDouble[6], ef_a_fy[] = new YoDouble[6], ef_a_fz[] = new YoDouble[6];

   private YoDouble ef_platform_x, ef_platform_y, ef_platform_z;

   private final YoVariableRegistry registry = new YoVariableRegistry("StewartPlatformController");

   private YoDouble x_offset = new YoDouble("x_offset", registry), y_offset = new YoDouble("y_offset", registry),
                      z_offset = new YoDouble("z_offset", registry);
   private YoDouble yaw_offset = new YoDouble("yaw_offset", registry), roll_offset = new YoDouble("roll_offset", registry),
                      pitch_offset = new YoDouble("pitch_offset", registry);

   private YoDouble x_amp = new YoDouble("x_amp", registry), y_amp = new YoDouble("y_amp", registry), z_amp = new YoDouble("z_amp", registry);
   private YoDouble yaw_amp = new YoDouble("yaw_amp", registry), roll_amp = new YoDouble("roll_amp", registry),
                      pitch_amp = new YoDouble("pitch_amp", registry);

   private YoDouble x_freq = new YoDouble("x_freq", registry), y_freq = new YoDouble("y_freq", registry), z_freq = new YoDouble("z_freq", registry);
   private YoDouble yaw_freq = new YoDouble("yaw_freq", registry), roll_freq = new YoDouble("roll_freq", registry),
                      pitch_freq = new YoDouble("pitch_freq", registry);

   private YoDouble x_phase = new YoDouble("x_phase", registry), y_phase = new YoDouble("y_phase", registry),
                      z_phase = new YoDouble("z_phase", registry);
   private YoDouble yaw_phase = new YoDouble("yaw_phase", registry), roll_phase = new YoDouble("roll_phase", registry),
                      pitch_phase = new YoDouble("pitch_phase", registry);

   private YoDouble outside_limits = new YoDouble("outside_limits", registry);
   private YoDouble Fx = new YoDouble("Fx", registry), Fy = new YoDouble("Fy", registry), Fz = new YoDouble("Fz", registry);
   private YoDouble Nx = new YoDouble("Nx", registry), Ny = new YoDouble("Ny", registry), Nz = new YoDouble("Nz", registry);

   private YoDouble k_loop = new YoDouble("k_loop", registry), b_loop = new YoDouble("b_loop", registry);

   private YoDouble k_x = new YoDouble("k_x", registry), k_y = new YoDouble("k_y", registry), k_z = new YoDouble("k_z", registry);
   private YoDouble b_x = new YoDouble("b_x", registry), b_y = new YoDouble("b_y", registry), b_z = new YoDouble("b_z", registry);

   private YoDouble k_yaw = new YoDouble("k_yaw", registry), k_pitch = new YoDouble("k_pitch", registry), k_roll = new YoDouble("k_roll", registry);
   private YoDouble b_yaw = new YoDouble("b_yaw", registry), b_pitch = new YoDouble("b_pitch", registry), b_roll = new YoDouble("b_roll", registry);

   private YoDouble q_d_x = new YoDouble("q_d_x", registry), q_d_y = new YoDouble("q_d_y", registry), q_d_z = new YoDouble("q_d_z", registry);
   private YoDouble q_yaw = new YoDouble("q_yaw", registry), q_pitch = new YoDouble("q_pitch", registry), q_roll = new YoDouble("q_roll", registry);
   private YoDouble q_d_yaw = new YoDouble("q_d_yaw", registry), q_d_pitch = new YoDouble("q_d_pitch", registry),
                      q_d_roll = new YoDouble("q_d_roll", registry);

   private YoDouble[] controlVars = new YoDouble[]
   {
      x_offset, y_offset, z_offset, yaw_offset, roll_offset, pitch_offset, x_amp, y_amp, z_amp, yaw_amp, roll_amp, pitch_amp, x_freq, y_freq, z_freq, yaw_freq,
      roll_freq, pitch_freq, x_phase, y_phase, z_phase, yaw_phase, roll_phase, pitch_phase, q_yaw, q_roll, q_pitch, k_x, k_y, k_z, k_yaw, k_pitch, k_roll, b_x,
      b_y, b_z, b_yaw, b_pitch, b_roll, q_d_x, q_d_y, q_d_z, q_d_yaw, q_d_pitch, q_d_roll, outside_limits, Fx, Fy, Fz, Nx, Ny, Nz, k_loop, b_loop
   };

   private String name;

   public StewartPlatformController(StewartPlatformRobot rob, String name)
   {
      this.name = name;
      this.rob = rob;

      t = (YoDouble)rob.getVariable("t");
      q_platform_x = (YoDouble)rob.getVariable("q_platform_x");
      q_platform_y = (YoDouble)rob.getVariable("q_platform_y");
      q_platform_z = (YoDouble)rob.getVariable("q_platform_z");
      qd_platform_x = (YoDouble)rob.getVariable("qd_platform_x");
      qd_platform_y = (YoDouble)rob.getVariable("qd_platform_y");
      qd_platform_z = (YoDouble)rob.getVariable("qd_platform_z");

      qd_platform_wx = (YoDouble)rob.getVariable("qd_platform_wx");
      qd_platform_wy = (YoDouble)rob.getVariable("qd_platform_wy");
      qd_platform_wz = (YoDouble)rob.getVariable("qd_platform_wz");

      q_act0 = (YoDouble)rob.getVariable("q_act0");
      q_act1 = (YoDouble)rob.getVariable("q_act1");
      q_act2 = (YoDouble)rob.getVariable("q_act2");
      q_act3 = (YoDouble)rob.getVariable("q_act3");
      q_act4 = (YoDouble)rob.getVariable("q_act4");
      q_act5 = (YoDouble)rob.getVariable("q_act5");

      tau_act0 = (YoDouble)rob.getVariable("tau_act0");
      tau_act1 = (YoDouble)rob.getVariable("tau_act1");
      tau_act2 = (YoDouble)rob.getVariable("tau_act2");
      tau_act3 = (YoDouble)rob.getVariable("tau_act3");
      tau_act4 = (YoDouble)rob.getVariable("tau_act4");
      tau_act5 = (YoDouble)rob.getVariable("tau_act5");

      for (int i = 0; i < 6; i++)
      {
         ef_p_x[i] = (YoDouble)rob.getVariable("ef_p" + i + "_x");
         ef_p_y[i] = (YoDouble)rob.getVariable("ef_p" + i + "_y");
         ef_p_z[i] = (YoDouble)rob.getVariable("ef_p" + i + "_z");
         ef_p_dx[i] = (YoDouble)rob.getVariable("ef_p" + i + "_dx");
         ef_p_dy[i] = (YoDouble)rob.getVariable("ef_p" + i + "_dy");
         ef_p_dz[i] = (YoDouble)rob.getVariable("ef_p" + i + "_dz");
         ef_p_fx[i] = (YoDouble)rob.getVariable("ef_p" + i + "_fx");
         ef_p_fy[i] = (YoDouble)rob.getVariable("ef_p" + i + "_fy");
         ef_p_fz[i] = (YoDouble)rob.getVariable("ef_p" + i + "_fz");

         ef_a_x[i] = (YoDouble)rob.getVariable("ef_a" + i + "_x");
         ef_a_y[i] = (YoDouble)rob.getVariable("ef_a" + i + "_y");
         ef_a_z[i] = (YoDouble)rob.getVariable("ef_a" + i + "_z");
         ef_a_dx[i] = (YoDouble)rob.getVariable("ef_a" + i + "_dx");
         ef_a_dy[i] = (YoDouble)rob.getVariable("ef_a" + i + "_dy");
         ef_a_dz[i] = (YoDouble)rob.getVariable("ef_a" + i + "_dz");
         ef_a_fx[i] = (YoDouble)rob.getVariable("ef_a" + i + "_fx");
         ef_a_fy[i] = (YoDouble)rob.getVariable("ef_a" + i + "_fy");
         ef_a_fz[i] = (YoDouble)rob.getVariable("ef_a" + i + "_fz");

      }

      ef_platform_x = (YoDouble)rob.getVariable("ef_platform_x");
      ef_platform_y = (YoDouble)rob.getVariable("ef_platform_y");
      ef_platform_z = (YoDouble)rob.getVariable("ef_platform_z");
      initControl();
   }

   public YoDouble[] getControlVars()
   {
      return controlVars;
   }

   private ForceDistribution forceDistribution;
   private Vector3D[] a_hat = new Vector3D[6], b = new Vector3D[6];
   private double[] act_forces = new double[6];

   public void initControl()
   {
      forceDistribution = new ForceDistribution();
      q_act0.set(0.8);
    		  q_act1.set(0.8);
    				  q_act2.set(0.8);
    						  q_act3.set(0.8);
    								  q_act4.set(0.8);
    										  q_act5.set(0.8);
    												  q_platform_z.set(0.8);

      q_d_x.set(0.1);
      q_d_y.set(0.05);
      q_d_z.set(0.5);
      q_d_yaw.set(0.1);
      q_d_roll.set(0.1);
      q_d_pitch.set(0.1);

      x_offset.set(0.0);
      y_offset.set(0.0);
      z_offset.set(0.3);
      x_amp.set(0.15);
      y_amp.set(0.15);
      z_amp.set(0.05);

      // x_amp.val = 0.0; y_amp.val = 0.0; z_amp.val = 0.0;
      x_freq.set(1.0);
      y_freq.set(1.0);
      z_freq.set(0.5);
      x_phase.set(0.0);
      y_phase.set(Math.PI / 2.0);
      z_phase.set(0.0);

      yaw_offset.set(0.0);
      roll_offset.set(0.0);
      pitch_offset.set(0.0);
      yaw_amp.set(0.1);
      roll_amp.set(0.1);
      pitch_amp.set(0.1);

      // yaw_amp.val = 0.0; roll_amp.val = 0.0; pitch_amp.val = 0.0;
      yaw_freq.set(2.0);
      roll_freq.set(1.0);
      pitch_freq.set(1.0);
      yaw_phase.set(0.0);
      roll_phase.set(0.0);
      pitch_phase.set(0.0);


      k_loop.set(100000.0);
      b_loop.set(500.0);

      k_x.set(5000.0);
      k_y.set(5000.0);
      k_z.set(5000.0);
      b_x.set(5000.0);
      b_y.set(5000.0);
      b_z.set(500.0);

      k_yaw.set(60.0);
      k_roll.set(60.0);
      k_pitch.set(60.0);
      b_yaw.set(4.0);
      b_roll.set(4.0);
      b_pitch.set(4.0);


      Fx.set(0.0);
      Fy.set(0.0);
      Fz.set(0.0);


      for (int i = 0; i < 6; i++)
      {
         a_hat[i] = new Vector3D();
         b[i] = new Vector3D();
      }
   }


   public void doControl()
   {
      applyLoopForces();

      rob.platformJoint.getYawPitchRoll(q_yaw, q_pitch, q_roll);

      q_d_x.set(x_offset.getDoubleValue() + x_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * x_freq.getDoubleValue() * t.getDoubleValue() + x_phase.getDoubleValue()));
      q_d_y.set(y_offset.getDoubleValue() + y_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * y_freq.getDoubleValue() * t.getDoubleValue() + y_phase.getDoubleValue()));
      q_d_z.set(z_offset.getDoubleValue() + z_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * z_freq.getDoubleValue() * t.getDoubleValue() + z_phase.getDoubleValue()));
      q_d_yaw.set(yaw_offset.getDoubleValue() + yaw_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * yaw_freq.getDoubleValue() * t.getDoubleValue() + yaw_phase.getDoubleValue()));
      q_d_roll.set(roll_offset.getDoubleValue() + roll_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * roll_freq.getDoubleValue() * t.getDoubleValue() + roll_phase.getDoubleValue()));
      q_d_pitch.set(pitch_offset.getDoubleValue() + pitch_amp.getDoubleValue() * Math.sin(2.0 * Math.PI * pitch_freq.getDoubleValue() * t.getDoubleValue() + pitch_phase.getDoubleValue()));


      Fx.set(k_x.getDoubleValue() * (q_d_x.getDoubleValue() - q_platform_x.getDoubleValue()) - b_x.getDoubleValue() * qd_platform_x.getDoubleValue());
      Fy.set(k_y.getDoubleValue() * (q_d_y.getDoubleValue() - q_platform_y.getDoubleValue()) - b_y.getDoubleValue() * qd_platform_y.getDoubleValue());
      Fz.set(k_z.getDoubleValue() * (q_d_z.getDoubleValue() - q_platform_z.getDoubleValue()) - b_z.getDoubleValue() * qd_platform_z.getDoubleValue());

      Nx.set(k_roll.getDoubleValue() * (q_d_roll.getDoubleValue() - q_roll.getDoubleValue()) - b_roll.getDoubleValue() * qd_platform_wx.getDoubleValue());
      Ny.set(k_pitch.getDoubleValue() * (q_d_pitch.getDoubleValue() - q_pitch.getDoubleValue()) - b_pitch.getDoubleValue() * qd_platform_wy.getDoubleValue());
      Nz.set(k_yaw.getDoubleValue() * (q_d_yaw.getDoubleValue() - q_yaw.getDoubleValue()) - b_yaw.getDoubleValue() * qd_platform_wz.getDoubleValue());

      distributeForces();
   }

   Vector3D tempVec = new Vector3D();

   public void distributeForces()
   {
      for (int i = 0; i < 6; i++)
      {
         a_hat[i].set(ef_a_x[i].getDoubleValue(), ef_a_y[i].getDoubleValue(), ef_a_z[i].getDoubleValue());
         a_hat[i].sub(rob.base_offsets[i]);
         a_hat[i].normalize();

         tempVec.set(ef_platform_x.getDoubleValue() - ef_a_x[i].getDoubleValue(), ef_platform_y.getDoubleValue() - ef_a_y[i].getDoubleValue(), ef_platform_z.getDoubleValue() - ef_a_z[i].getDoubleValue());
         b[i].cross(a_hat[i], tempVec);

         // PointOfInterest.DifferenceVectorHat(pointb[i],pointc[i],a_hat[i]);
         // PointOfInterest.DifferenceVector(pointForceApplication,pointc[i],tempVec); Changed from pointc to pointb since applying forces on the platform now...
         // PointOfInterest.DifferenceVector(pointForceApplication,pointb[i],tempVec); // But doesn't matter either way anyway!
         // b[i].cross(tempVec,a_hat[i]);
      }

      forceDistribution.solveActuatorForcesSingleLeg(act_forces, Fx.getDoubleValue(), Fy.getDoubleValue(), Fz.getDoubleValue(), Nx.getDoubleValue(), Ny.getDoubleValue(), Nz.getDoubleValue(), a_hat, b);

      // System.out.println("a_hat: "+ a_hat);
      // System.out.println("b: "+ b);

      tau_act0.set(act_forces[0]);
      tau_act1.set(act_forces[1]);
      tau_act2.set(act_forces[2]);
      tau_act3.set(act_forces[3]);
      tau_act4.set(act_forces[4]);
      tau_act5.set(act_forces[5]);
   }

   public void applyLoopForces()
   {
      for (int i = 0; i < 6; i++)
      {
         // Stiff springs and dampers connected between actuator (ef_a_*) and platform (ef_p_*)
         ef_p_fx[i].set(k_loop.getDoubleValue() * (ef_a_x[i].getDoubleValue() - ef_p_x[i].getDoubleValue()) + b_loop.getDoubleValue() * (ef_a_dx[i].getDoubleValue() - ef_p_dx[i].getDoubleValue()));
         ef_p_fy[i].set(k_loop.getDoubleValue() * (ef_a_y[i].getDoubleValue() - ef_p_y[i].getDoubleValue()) + b_loop.getDoubleValue() * (ef_a_dy[i].getDoubleValue() - ef_p_dy[i].getDoubleValue()));
         ef_p_fz[i].set(k_loop.getDoubleValue() * (ef_a_z[i].getDoubleValue() - ef_p_z[i].getDoubleValue()) + b_loop.getDoubleValue() * (ef_a_dz[i].getDoubleValue() - ef_p_dz[i].getDoubleValue()));

         // Make sure to have equal and opposite forces (Newtons 3rd law) to ensure valid dynamics
         ef_a_fx[i].set(-ef_p_fx[i].getDoubleValue());
         ef_a_fy[i].set(-ef_p_fy[i].getDoubleValue());
         ef_a_fz[i].set(-ef_p_fz[i].getDoubleValue());
      }

   }

   public YoVariableRegistry getYoVariableRegistry()
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
}
