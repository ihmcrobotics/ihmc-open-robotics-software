package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

/* package-private */ class DoublePendulumSinusoidalController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final DoublePendulumRobot robot;

   private final static double amplitude1 = 4.0;
   private final static double amplitude2 = 2.0;
   private final static double omega1 = 1.2;
   private final static double omega2 = 0.9;
   private final static double phi1 = 0.0;
   private final static double phi2 = 1.0;

   public DoublePendulumSinusoidalController(DoublePendulumRobot robot)
   {
      this.robot = robot;
   }

   @Override
   public void doControl()
   {
      double time = robot.getTime();
      robot.getScsJoint1().setTau(amplitude1 * Math.sin(omega1 * time + phi1));
      robot.getScsJoint2().setTau(amplitude2 * Math.sin(omega2 * time + phi2));
      robot.setIDStateFromSCS();
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
