package us.ihmc.simulationconstructionset.physics.featherstone;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;

public class DoublePendulumRobot extends RobotWithClosedFormDynamics
{
   /**
    * Length of first link
    */
   private final double length1 = 0.7;

   /**
    * Length from joint to CoM
    */
   private final double lengthCoM1 = 0.3, lengthCoM2 = 0.25;

   /**
    * Moment of inertia. The equations below use the inertia about the joints, so this specifies it's in body frame
    */
   private final double Ixx1CoM = 0.4, Ixx2CoM = 0.5;

   private final double mass1 = 1.0, mass2 = 1.5;
   private final double damping1 = 0.2, damping2 = 0.1;
   private final Axis axis = Axis.X;

   private final PinJoint pinJoint1;
   private final PinJoint pinJoint2;

   /**
    * Manipulator equation matrices, for matrix definitions see {@link #assertStateIsCloseToClosedFormCalculation}
    */
   private final DenseMatrix64F H = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F C = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F G = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F qd = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F qdd = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);

   public DoublePendulumRobot(String name, double initialQFirstJoint, double initialQdFirstJoint, double initialQSecondJoint, double initialQdSecondJoint)
   {
      super(name);

      pinJoint1 = new PinJoint(name + "Joint1", new Vector3D(), this, axis);

      Link link1 = new Link(name + "Link1");
      link1.setMass(mass1);
      link1.setMomentOfInertia(Ixx1CoM, 0.0, 0.0);
      link1.setComOffset(0.0, 0.0, - lengthCoM1);

      pinJoint1.setLink(link1);
      pinJoint1.setDamping(damping1);

      pinJoint2 = new PinJoint(name + "Joint2", new Vector3D(0.0, 0.0, -length1), this, axis);

      Link link2 = new Link(name + "Link2");
      link2.setMass(mass2);
      link2.setMomentOfInertia(Ixx2CoM, 0.0, 0.0);
      link2.setComOffset(0.0, 0.0, - lengthCoM2);

      pinJoint2.setLink(link2);
      pinJoint2.setDamping(damping2);

      pinJoint1.addJoint(pinJoint2);

      pinJoint1.setQ(initialQFirstJoint);
      pinJoint1.setQd(initialQdFirstJoint);
      pinJoint2.setQ(initialQSecondJoint);
      pinJoint2.setQd(initialQdSecondJoint);

      addRootJoint(pinJoint1);
   }

   /**
    * Solves the equations of motions outlined here:
    * @see <a href="https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf</a>
    */
   @Override
   public void assertStateIsCloseToClosedFormCalculation(double epsilon)
   {
      double q1 = pinJoint1.getQ();
      double qd1 = pinJoint1.getQD();
      double qdd1 = pinJoint1.getQDD();

      double q2 = pinJoint2.getQ();
      double qd2 = pinJoint2.getQD();
      double qdd2 = pinJoint2.getQDD();

      double g = Math.abs(gravityZ.getDoubleValue());
      double Ixx1 = mass1 * MathTools.square(lengthCoM1) + Ixx1CoM;
      double Ixx2 = mass2 * MathTools.square(lengthCoM2) + Ixx2CoM;

      double H00 = Ixx1 + Ixx2 + mass2 * MathTools.square(length1) + 2.0 * mass2 * length1 * lengthCoM2 * Math.cos(q2);
      double H01 = Ixx2 + mass2 * length1 * lengthCoM2 * Math.cos(q2);
      double H10 = H01;
      double H11 = Ixx2;

      double C00 = -2.0 * mass2 * length1 * lengthCoM2 * Math.sin(q2) * qd2 + damping1;
      double C01 = - mass2 * length1 * lengthCoM2 * Math.sin(q2) * qd2;
      double C10 = mass2 * length1 * lengthCoM2 * Math.sin(q2) * qd1;
      double C11 = damping2;

      double G00 = (mass1 * lengthCoM1 + mass2 * length1) * g * Math.sin(q1) + mass2 * g * lengthCoM2 * Math.sin(q1 + q2);
      double G10 = mass2 * g * lengthCoM2 * Math.sin(q1 + q2);

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

      qd.set(0, 0, qd1);
      qd.set(1, 0, qd2);

      rightHandSide.set(G);
      CommonOps.multAdd(C, qd, rightHandSide);
      CommonOps.scale(-1.0, rightHandSide);
      CommonOps.solve(H, rightHandSide, qdd);

      if(Math.abs(qdd.get(0, 0) - qdd1) > epsilon || Math.abs(qdd.get(1, 0) - qdd2) > epsilon)
      {
         throw new AssertionError("Joint accelerations from simulation and lagrangian don't match. "
                                        + "\nAt t=" + getTime()
                                        + "\nSimulated joint accelerations: (" + qdd1 + ", " + qdd2 + ")"
                                        + "\nLagrangian accelerations: (" + qdd.get(0, 0) + ", " + qdd.get(1, 0) + ")");
      }
   }
}
