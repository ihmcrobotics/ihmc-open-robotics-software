package us.ihmc.simulationconstructionset.physics.featherstone;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
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

   /**
    * Manipulator equation matrices, for matrix definitions see {@link #assertStateIsCloseToClosedFormCalculation}
    */
   private final DenseMatrix64F H = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F C = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F G = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F qd = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F qdd = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);

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
   public void assertStateIsCloseToClosedFormCalculation(double epsilon)
   {
      double xd = sliderJoint.getQD();
      double xdd = sliderJoint.getQDD();

      double theta = pinJoint.getQ();
      double thetaD = pinJoint.getQD();
      double thetaDD = pinJoint.getQDD();

      double g = Math.abs(gravityZ.getDoubleValue());

      double H00 = cartMass + poleMass;
      double H01 = poleMass * poleLength * Math.cos(theta);
      double H10 = H01;
      double H11 = poleMass * MathTools.square(poleLength);

      double C00 = 0.0;
      double C01 = - poleMass * poleLength * thetaD * Math.sin(theta);
      double C10 = 0.0;
      double C11 = 0.0;

      double G00 = 0.0;
      double G10 = poleMass * g * poleLength * Math.sin(theta);

      double cartForce = - cartDamping * xd;
      double poleTorque = - poleDamping * thetaD;

      H.set(0, 0, H00);
      H.set(0, 1, H01);
      H.set(1, 0, H10);
      H.set(1, 1, H11);

      C.set(0, 0, C00);
      C.set(0, 1, C01);
      C.set(1, 0, C10);
      C.set(1, 1, C11);

      G.set(0, 0, G00);
      G.set(1, 0, G10);

      // negative sign since it's negated below
      rightHandSide.set(0, 0, - cartForce);
      rightHandSide.set(1, 0, - poleTorque);

      qd.set(0, 0, xd);
      qd.set(1, 0, thetaD);

      qdd.set(0, 0, xdd);
      qdd.set(1, 0, thetaDD);

      CommonOps.add(rightHandSide, G, rightHandSide);
      CommonOps.multAdd(C, qd, rightHandSide);
      CommonOps.scale(-1.0, rightHandSide);
      CommonOps.solve(H, rightHandSide, qdd);

      double xddLagrangian = - qdd.get(0, 0);
      double thetaDDLagrangian = qdd.get(1, 0);

      if(Math.abs(xdd - xddLagrangian) > epsilon || Math.abs(thetaDD - thetaDDLagrangian) > epsilon)
      {
         throw new AssertionError("Joint accelerations from simulation and lagrangian don't match. "
                                        + "\nAt t=" + getTime()
                                        + "\nSimulated joint accelerations: (" + xdd + ", " + thetaDD + ")"
                                        + "\nLagrangian accelerations: (" + xddLagrangian + ", " + thetaDDLagrangian + ")");
      }
   }
}
