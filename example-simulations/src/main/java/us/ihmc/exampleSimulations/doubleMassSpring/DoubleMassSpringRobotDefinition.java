package us.ihmc.exampleSimulations.doubleMassSpring;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class DoubleMassSpringRobotDefinition extends RobotDefinition
{
   private static final double mass = 1.0;

   public DoubleMassSpringRobotDefinition()
   {
      super("DoubleMassSpring");

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      setRootBodyDefinition(rootBody);

      // Two independent sliders, both attached directly to the root - not chained to each other.
      PrismaticJointDefinition x1Joint = new PrismaticJointDefinition("x1", new Vector3D(0.0, 0.0, 0.1), Axis3D.X);
      PrismaticJointDefinition x2Joint = new PrismaticJointDefinition("x2", new Vector3D(0.1, 0.0, 0.1), Axis3D.X);

      RigidBodyDefinition massOneLink = new RigidBodyDefinition("mass1");
      massOneLink.setMass(mass);
      VisualDefinitionFactory linkGraphicsOne = new VisualDefinitionFactory();
      linkGraphicsOne.addBox(0.05, 0.05, 0.05, ColorDefinitions.Green());
      massOneLink.addVisualDefinitions(linkGraphicsOne.getVisualDefinitions());

      RigidBodyDefinition massTwoLink = new RigidBodyDefinition("mass2");
      massTwoLink.setMass(mass);
      VisualDefinitionFactory linkGraphicsTwo = new VisualDefinitionFactory();
      linkGraphicsTwo.addBox(0.05, 0.05, 0.05, ColorDefinitions.Red());
      massTwoLink.addVisualDefinitions(linkGraphicsTwo.getVisualDefinitions());

      x1Joint.setSuccessor(massOneLink);
      x2Joint.setSuccessor(massTwoLink);

      rootBody.addChildJoint(x1Joint);
      rootBody.addChildJoint(x2Joint);

      // Modes are (1 1) and (-1 1)
      x1Joint.setInitialJointState(-0.02);
      x2Joint.setInitialJointState(0.02);

      addControllerDefinition((controllerInput, controllerOutput) -> new DoubleMassSpringController(controllerInput, controllerOutput));
   }
}
