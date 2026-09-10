package us.ihmc.exampleSimulations.doubleMassSpring;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class DoubleMassSpringController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoEnum<ControlMode> controlMode = new YoEnum<ControlMode>("controlMode", registry, ControlMode.class);

   private final YoDouble kSpring = new YoDouble("kSpring", registry);
   private final YoDouble x1Des = new YoDouble("x1Des", registry);
   private final YoDouble x2Des = new YoDouble("x2Des", registry);

   private final YoDouble k1 = new YoDouble("k1", registry);
   private final YoDouble k2 = new YoDouble("k2", registry);
   private final YoDouble b1 = new YoDouble("b1", registry);
   private final YoDouble b2 = new YoDouble("b2", registry);

   private final OneDoFJointReadOnly x1Joint, x2Joint;
   private final OneDoFJointStateBasics x1Output, x2Output;

   public DoubleMassSpringController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      x1Joint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("x1");
      x2Joint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint("x2");
      x1Output = controllerOutput.getOneDoFJointOutput("x1");
      x2Output = controllerOutput.getOneDoFJointOutput("x2");

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
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
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
      x1Des.set(k2.getDoubleValue() * (x2Des.getDoubleValue() - x2Joint.getQ()) - b2.getDoubleValue() * x2Joint.getQd());

      controlMassOnePosition();
   }

   private void controlMassOnePosition()
   {
      // Standard PD controller on mass one.
      double x1FeedbackForce = k1.getDoubleValue() * (x1Des.getDoubleValue() - x1Joint.getQ()) - b1.getDoubleValue() * x1Joint.getQd();
      x1Output.addEffort(x1FeedbackForce);
   }

   private void doPassiveSpringsControl()
   {
      // These are the forces from the passive springs. One connected from x1 to ground, one from x1 to x2, and one from x2 to ground.
      double x1PassiveForce = -2.0 * kSpring.getDoubleValue() * x1Joint.getQ() + kSpring.getDoubleValue() * x2Joint.getQ();
      double x2PassiveForce = -2.0 * kSpring.getDoubleValue() * x2Joint.getQ() + kSpring.getDoubleValue() * x1Joint.getQ();

      x1Output.setEffort(x1PassiveForce);
      x2Output.setEffort(x2PassiveForce);
   }

   private enum ControlMode
   {
      PASSIVE, MASS_ONE, MASS_TWO;
   }
}
