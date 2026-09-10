package us.ihmc.exampleSimulations.doublePendulum;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

/**
 * This class DoublePendulumRobotDefinition is a public class that extends RobotDefinition. It
 * describes the structure of a DoublePendulum robot for use with Simulation Construction Set 2.
 */
public class DoublePendulumRobotDefinition extends RobotDefinition
{
   /* L1 and L2 are the link lengths, M1 and M2 are the link masses, and R1 and R2 are the radii of the links,
    * Iyy1 and Iyy2 are the moments of inertia of the links. The moments of inertia are defined about the COM
    * for each link.
    */
   public static final double
         L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;

   public DoublePendulumRobotDefinition()
   {
      super("DoublePendulum");

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      setRootBodyDefinition(rootBody);

      // Create joints and assign links. Revolute joints have a single axis of rotation.
      RevoluteJointDefinition joint1 = new RevoluteJointDefinition("joint1", new Vector3D(0.0, 0.0, 0.0), Axis3D.Y);
      joint1.setInitialJointState(0.05, 0.0);
      joint1.setSuccessor(link1());
      rootBody.addChildJoint(joint1);

      /*
       *  The second joint is offset by the vector (0.0,0.0,L1) since
       *  it should be placed a distance of L1 in the Z direction from the previous joint.
       */
      RevoluteJointDefinition joint2 = new RevoluteJointDefinition("joint2", new Vector3D(0.0, 0.0, L1), Axis3D.Y);
      joint2.setSuccessor(link2());
      joint1.getSuccessor().addChildJoint(joint2);

      addControllerDefinition((controllerInput, controllerOutput) -> new DoublePendulumController(controllerInput, controllerOutput, "doublePendulumController"));
   }

   /**
    * Create the first link for the DoublePendulumRobotDefinition.
    */
   private RigidBodyDefinition link1()
   {
      RigidBodyDefinition ret = new RigidBodyDefinition("link1");
      ret.setMass(M1);
      ret.setCenterOfMassOffset(0.0, 0.0, L1 / 2.0);
      ret.setMomentOfInertia(new Matrix3D(0.0, 0.0, 0.0, 0.0, Iyy1, 0.0, 0.0, 0.0, 0.0));

      VisualDefinitionFactory linkGraphics = new VisualDefinitionFactory();
      linkGraphics.addCylinder(L1, R1, ColorDefinitions.Red());
      ret.addVisualDefinitions(linkGraphics.getVisualDefinitions());

      return ret;
   }

   /**
    * Create the second link for the DoublePendulumRobotDefinition.
    */
   private RigidBodyDefinition link2()
   {
      RigidBodyDefinition ret = new RigidBodyDefinition("link2");
      ret.setMass(M2);
      ret.setCenterOfMassOffset(0.0, 0.0, L2 / 2.0);
      ret.setMomentOfInertia(new Matrix3D(0.0, 0.0, 0.0, 0.0, Iyy2, 0.0, 0.0, 0.0, 0.0));

      VisualDefinitionFactory linkGraphics = new VisualDefinitionFactory();
      linkGraphics.addCylinder(L2, R2, ColorDefinitions.Green());
      ret.addVisualDefinitions(linkGraphics.getVisualDefinitions());

      return ret;
   }
}
