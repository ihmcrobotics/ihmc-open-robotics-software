package us.ihmc.simulationconstructionset.physics.featherstone;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SliderJoint;

public class CartPoleRobot extends RobotWithClosedFormDynamics
{
   private final double cartMass = 0.8, poleMass = 1.2;
   private final double poleLength = 0.6;
   private final double cartDamping = 0.1, poleDamping = 0.2;

   private final SliderJoint sliderJoint;
   private final PinJoint pinJoint;

   public CartPoleRobot(String name, double initialCartVelocity, double initialPoleAngle, double initialPoleAngularVelocity)
   {
      super(name);

      sliderJoint = new SliderJoint(name + "CartJoint", new Vector3D(), this, Axis.X);
      sliderJoint.setDamping(cartDamping);

      Link cartLink = new Link(name + "CartLink");
      cartLink.setMass(cartMass);
      sliderJoint.setLink(cartLink);

      pinJoint = new PinJoint(name + "PoleJoint", new Vector3D(), this, Axis.Y);
      pinJoint.setDamping(poleDamping);

      Link poleLink = new Link(name + "PoleLink");
      poleLink.setMass(poleMass);
      poleLink.setComOffset(0.0, 0.0, - poleLength);
      pinJoint.setLink(poleLink);

      sliderJoint.setQd(initialCartVelocity);
      pinJoint.setQ(initialPoleAngle);
      pinJoint.setQd(initialPoleAngularVelocity);

      sliderJoint.addJoint(pinJoint);
      addRootJoint(sliderJoint);
   }

   /**
    * Solves the equations of motions outlined here:
    * @see <a href="https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf</a>
    */
   @Override
   public void assertStateIsCloseToLagrangianCalculation(double epsilon)
   {
      double xdd = sliderJoint.getQDD();

      double q = pinJoint.getQ();
      double qd = pinJoint.getQD();
      double qdd = pinJoint.getQDD();

      double g = Math.abs(gravityZ.getDoubleValue());

      double xddLagrangian = - (poleMass * Math.sin(q) * (poleLength * MathTools.square(qd) + g * Math.cos(q))) / (cartMass + poleMass * MathTools.square(Math.sin(q)));
      double qddLagrangian = (- poleMass * poleLength * MathTools.square(qd) * Math.cos(q) * Math.sin(q) - (cartMass + poleMass) * g * Math.sin(q)) / (poleLength * (cartMass + poleMass * MathTools.square(Math.sin(q))));

      if(Math.abs(xdd - xddLagrangian) > epsilon || Math.abs(qdd - qddLagrangian) > epsilon)
      {
         System.out.println("x = " + sliderJoint.getQ());
         System.out.println("q = " + q);

         throw new AssertionError("Joint accelerations from simulation and lagrangian don't match. "
                                        + "\nAt t=" + getTime()
                                        + "\nSimulated joint accelerations: (" + xdd + ", " + qdd + ")"
                                        + "\nLagrangian accelerations: (" + xddLagrangian + ", " + qddLagrangian + ")");
      }
   }
}
