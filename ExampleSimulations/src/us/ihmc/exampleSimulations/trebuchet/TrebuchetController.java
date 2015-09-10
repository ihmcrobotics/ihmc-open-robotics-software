package us.ihmc.exampleSimulations.trebuchet;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;


public class TrebuchetController extends TrebuchetControllerBase implements RobotController
{

   private final YoVariableRegistry registry = new YoVariableRegistry("TrebuchetController");

   // These are the control variables which need to be created now:

   DoubleYoVariable ballAttached = new DoubleYoVariable("ballAttached", registry);
   DoubleYoVariable poleBall_x = new DoubleYoVariable("poleBall_x", registry), poleBall_dx = new DoubleYoVariable("poleBall_dx", registry);
   DoubleYoVariable poleBall_z = new DoubleYoVariable("poleBall_z", registry), poleBall_dz = new DoubleYoVariable("poleBall_dz", registry);

   DoubleYoVariable poleBall_distance = new DoubleYoVariable("poleBall_distance", registry), poleBall_vel = new DoubleYoVariable("poleBall_vel", registry),
              rope_force = new DoubleYoVariable("rope_force", registry);

   private String name;

   public TrebuchetController(TrebuchetRobot rob, String name)
   {
      super(rob);
      this.name = name;
      initControl();
   }

   private void initControl()
   {
      // Initialize the variables.

      t.set(0.0);
      q_x.set(0.0);
      qd_x.set(0.0);
      q_pivot.set(-13.0 * Math.PI / 16.0);
      qd_pivot.set(0.0);

      ballAttached.set(1.0);
      q_ball_x.set(TrebuchetRobot.ROPE_LENGTH - TrebuchetRobot.BASE_LENGTH * 0.4);
      q_ball_z.set(BALL_MIN_HEIGHT);
   }

   private double BALL_MIN_HEIGHT = TrebuchetRobot.WHEEL_RADIUS + TrebuchetRobot.BASE_HEIGHT / 2.0 + TrebuchetRobot.BALL_RADIUS;
   private Vector3d forceDirection = new Vector3d();

   public void doControl()
   {
      if (q_pivot.getDoubleValue() >= 0.0)
         ballAttached.set(0.0);

      tau_x.set(-1500.0 * qd_x.getDoubleValue());
      tau_pivot.set(-8000.0 * qd_pivot.getDoubleValue());

      // tau_x.val = 0.0;

      // Calculate rope length and force:

      poleBall_x.set(ef_pole_x.getDoubleValue() - ef_ball_x.getDoubleValue());
      poleBall_dx.set(ef_pole_dx.getDoubleValue() - ef_ball_dx.getDoubleValue());

      poleBall_z.set(ef_pole_z.getDoubleValue() - ef_ball_z.getDoubleValue());
      poleBall_dz.set(ef_pole_dz.getDoubleValue() - ef_ball_dz.getDoubleValue());

      poleBall_distance.set(Math.sqrt(poleBall_x.getDoubleValue() * poleBall_x.getDoubleValue() + poleBall_z.getDoubleValue() * poleBall_z.getDoubleValue()));
      poleBall_vel.set(Math.sqrt(poleBall_dx.getDoubleValue() * poleBall_dx.getDoubleValue() + poleBall_dz.getDoubleValue() * poleBall_dz.getDoubleValue()));

      // Pull from the rope:

      if ((ballAttached.getDoubleValue() > 0.5) && (poleBall_distance.getDoubleValue() > TrebuchetRobot.ROPE_LENGTH))
      {
         rope_force.set(-5000.0 * (TrebuchetRobot.ROPE_LENGTH - poleBall_distance.getDoubleValue()) + 50.0 * poleBall_vel.getDoubleValue());
      }
      else
         rope_force.set(0.0);

      // Apply to external force points:

      // Calculate unit vector:
      forceDirection.x = poleBall_x.getDoubleValue();
      forceDirection.y = 0.0;
      forceDirection.z = poleBall_z.getDoubleValue();

      forceDirection.normalize();

      ef_ball_fx.set(forceDirection.x * rope_force.getDoubleValue());
      ef_ball_fz.set(forceDirection.z * rope_force.getDoubleValue());

      ef_pole_fx.set(-ef_ball_fx.getDoubleValue());
      ef_pole_fz.set(-ef_ball_fz.getDoubleValue());

      // Prevent ball from falling through the base:
      if ((ballAttached.getDoubleValue() > 0.5) && (ef_ball_z.getDoubleValue() < BALL_MIN_HEIGHT))
      {
         ef_ball_fz.set(ef_ball_fz.getDoubleValue() + 10000.0 * (BALL_MIN_HEIGHT - ef_ball_z.getDoubleValue()) - 1000.0 * ef_ball_dz.getDoubleValue());
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
