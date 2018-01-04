package us.ihmc.simulationconstructionset.physics.featherstone;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;

public class SinglePendulumRobot extends RobotWithClosedFormDynamics
{
   private final double mass = 1.0;
   private final double length = 1.0;
   private final double Ixx = 0.5;
   private final Axis axis = Axis.X;
   private final double damping = 0.1;

   private final PinJoint pinJoint;

   public SinglePendulumRobot(String name, double initialQ, double initialQd)
   {
      super(name);

      pinJoint = new PinJoint(name + "Joint", new Vector3D(), this, axis);

      Link link = new Link(name + "Link");
      link.setMass(mass);
      link.setMomentOfInertia(Ixx, 0.0, 0.0);
      link.setComOffset(0.0, 0.0, -0.5 * length);

      pinJoint.setLink(link);
      pinJoint.setDamping(damping);
      addRootJoint(pinJoint);

      pinJoint.setQ(initialQ);
      pinJoint.setQd(initialQd);
   }

   @Override
   public void assertStateIsCloseToClosedFormCalculation(double epsilon) throws AssertionError
   {
      double q = pinJoint.getQ();
      double qd = pinJoint.getQD();
      double qdd = pinJoint.getQDD();

      double qddLagrangian = (mass * getGravityZ() * (0.5 * length) * Math.sin(q) - damping * qd) / (Ixx + mass * MathTools.square(0.5 * length));

      if(Math.abs(qdd - qddLagrangian) > epsilon)
      {
         throw new AssertionError("Joint accelerations from simulation and lagrangian don't match. At t=" + getTime() + ", simulated joint acceleration = " + qdd + ", lagrangian acceleration = " + qddLagrangian);
      }
   }
}
