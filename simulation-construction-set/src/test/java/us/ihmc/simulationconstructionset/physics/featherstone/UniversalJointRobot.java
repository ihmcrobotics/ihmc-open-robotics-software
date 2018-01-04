package us.ihmc.simulationconstructionset.physics.featherstone;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.UniversalJoint;

public class UniversalJointRobot extends RobotWithClosedFormDynamics
{
   private final double mass = 1.2;
   private final double length = 0.4;

   private final UniversalJoint universalJoint;

   public UniversalJointRobot(String name, double initialQFirstJoint, double initialQdFirstJoint, double initialQSecondJoint, double initialQdSecondJoint)
   {
      super(name);

      universalJoint = new UniversalJoint(name + "Joint1", name + "Joint2", new Vector3D(), this, Axis.X, Axis.Y);

      Link link = new Link(name + "Link");
      link.setMass(mass);
      link.setComOffset(0.0, 0.0, -length);
      universalJoint.setLink(link);

      universalJoint.getFirstJoint().setInitialState(initialQFirstJoint, initialQdFirstJoint);
      universalJoint.getSecondJoint().setInitialState(initialQSecondJoint, initialQdSecondJoint);

      addRootJoint(universalJoint);
   }

   @Override
   public void assertStateIsCloseToClosedFormCalculation(double epsilon)
   {
      double qx = universalJoint.getFirstJoint().getQ();
      double qddx = universalJoint.getFirstJoint().getQDD();

      double qy = universalJoint.getSecondJoint().getQ();
      double qddy = universalJoint.getSecondJoint().getQDD();

      double g = Math.abs(gravityZ.getDoubleValue());

      if(Math.abs(Math.cos(qy)) < 1e-4)
         return;

      double qddxLagrangian = - (g * Math.sin(qx)) / (length * Math.cos(qy));
      double qddyLagrangian = - (g * Math.cos(qx) * Math.sin(qy)) / length;

      if(Math.abs(qddxLagrangian - qddx) > epsilon || Math.abs(qddyLagrangian - qddy) > epsilon)
      {
         throw new AssertionError("Joint accelerations from simulation and lagrangian don't match. "
                                        + "\nAt t=" + getTime()
                                        + "\nSimulated joint accelerations: (" + qddx + ", " + qddy + ")"
                                        + "\nLagrangian accelerations: (" + qddxLagrangian + ", " + qddyLagrangian + ")");
      }
   }
}
