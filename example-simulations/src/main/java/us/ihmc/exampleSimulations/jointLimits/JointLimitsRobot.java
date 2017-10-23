package us.ihmc.exampleSimulations.jointLimits;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class JointLimitsRobot extends Robot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double mass = 1.0;
   private static final double length = 1.0;
   private static final double radius = 0.025;

   private static final double lowerLimit = -0.2;
   private static final double upperLimit = 0.6;

   private final PinJoint joint;
   private final RevoluteJoint idJoint;
   private final RigidBody elevator;

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   public JointLimitsRobot()
   {
      super("JointLimitTestingRobot");

      // --- id robot ---
      elevator = new RigidBody("elevator", worldFrame);
      idJoint = ScrewTools.addRevoluteJoint("idJoint", elevator, new Vector3D(0.0, 0.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      Matrix3D inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);
      RigidBody arm = ScrewTools.addRigidBody("arm", idJoint, inertia, mass, new Vector3D(0.0, 0.0, length/2.0));

      // --- scs robot ---
      joint = new PinJoint("joint", new Vector3D(), this, Axis.Y);
      joint.setLimitStops(lowerLimit, upperLimit, 1000.0, 10.0);
      joint.setDamping(0.5);
      joint.setLink(makeLink());
      this.addRootJoint(joint);

      // --- setup ID calculator ---
      inverseDynamicsCalculator = new InverseDynamicsCalculator(elevator, -this.getGravityZ());
   }

   private Link makeLink()
   {
      Link link = new Link("link");
      link.setMass(mass);
      link.setComOffset(new Vector3D(0.0, 0.0, length/2.0));
      double ixx = mass/12.0 * (3.0 * radius*radius + length*length);
      double iyy = ixx;
      double izz = mass/2.0 * radius*radius;
      link.setMomentOfInertia(ixx, iyy, izz);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(length, radius, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);
      return link;
   }

   public double getQ()
   {
      return joint.getQYoVariable().getDoubleValue();
   }

   public double getQd()
   {
      return joint.getQDYoVariable().getDoubleValue();
   }

   public double getLowerLimit()
   {
      return lowerLimit;
   }

   public double getUpperLimit()
   {
      return upperLimit;
   }

   public void setQdd(double qdd)
   {
      updateInverseDynamicsStructureFromSimulation();
      idJoint.setQddDesired(qdd);
      inverseDynamicsCalculator.compute();
      joint.setTau(idJoint.getTau());
   }

   private void updateInverseDynamicsStructureFromSimulation()
   {
      idJoint.setQ(getQ());
      idJoint.setQd(getQd());
      elevator.updateFramesRecursively();
   }
}
