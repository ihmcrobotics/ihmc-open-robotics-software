package us.ihmc.exampleSimulations.doubleMassSpring;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class DoubleMassSpringController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final EnumYoVariable<ControlMode> controlMode = new EnumYoVariable<ControlMode>("controlMode", registry, ControlMode.class);

   private final DoubleYoVariable kSpring = new DoubleYoVariable("kSpring", registry);
   private final DoubleYoVariable x1Des = new DoubleYoVariable("x1Des", registry);
   private final DoubleYoVariable x2Des = new DoubleYoVariable("x2Des", registry);

   private final DoubleYoVariable k1 = new DoubleYoVariable("k1", registry);
   private final DoubleYoVariable k2 = new DoubleYoVariable("k2", registry);
   private final DoubleYoVariable b1 = new DoubleYoVariable("b1", registry);
   private final DoubleYoVariable b2 = new DoubleYoVariable("b2", registry);

   private final DoubleMassSpringRobot robot;

   public DoubleMassSpringController(DoubleMassSpringRobot robot)
   {
      this.robot = robot;

      this.kSpring.set(1.0);

      k1.set(200.0);
      b1.set(20.0);

      k2.set(10.0);
      b2.set(5.0);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      doPassiveSpringsControl();

      switch (controlMode.getEnumValue())
      {
      case PASSIVE:
      {
         break;
      }
      case MASS_ONE:
      {
         controlMassOnePosition();
         break;
      }
      case MASS_TWO:
      {
         controlMassTwoByUsingMassOne();
         break;
      }
      }
   }

   private void controlMassTwoByUsingMassOne()
   {
      // This controller achieves control of the x2 mass through control of x1 mass.
      // Very similar to a series elastic actuator.
      // In order for this to work, k1 and b1 must be much larger than k2 and b2.
      x1Des.set(k2.getDoubleValue() * (x2Des.getDoubleValue() - robot.getX2()) - b2.getDoubleValue() * robot.getX2Dot());

      controlMassOnePosition();
   }

   private void controlMassOnePosition()
   {
      // Standard PD controller on mass one.
      double x1FeedbackForce = k1.getDoubleValue() * (x1Des.getDoubleValue() - robot.getX1()) - b1.getDoubleValue() * robot.getX1Dot();
      robot.addX1Force(x1FeedbackForce);
   }

   private void doPassiveSpringsControl()
   {
      // These are the forces from the passive springs. One connected from x1 to ground, one from x1 to x2, and one from x2 to ground.
      double x1PassiveForce = -2.0 * kSpring.getDoubleValue() * robot.getX1() + kSpring.getDoubleValue() * robot.getX2();
      double x2PassiveForce = -2.0 * kSpring.getDoubleValue() * robot.getX2() + kSpring.getDoubleValue() * robot.getX1();

      robot.setX1Force(x1PassiveForce);
      robot.setX2Force(x2PassiveForce);
   }

   private enum ControlMode
   {
      PASSIVE, MASS_ONE, MASS_TWO;
   }
}
