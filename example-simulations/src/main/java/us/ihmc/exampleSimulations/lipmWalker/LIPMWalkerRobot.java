package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.robotdefinition.JointDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.LinkDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;

public class LIPMWalkerRobot
{
   private double bodyRadius = 0.2;
   private double bodyRadiusOfGyrationY = 0.1;
   private double bodyRadiusOfGyrationZ = 0.1;
   private double bodyRadiusOfGyrationX = 0.1;
   private double bodyMass = 1.0;
   private Robot robot;

   public LIPMWalkerRobot()
   {
      RobotDescription description = new RobotDescription("LIPMWalker");
      FloatingJointDescription bodyJoint = new FloatingJointDescription("RootJoint");

      LinkDescription bodyLink = new LinkDescription("LinkJoint");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addSphere(bodyRadius, YoAppearance.AluminumMaterial());
      bodyLink.setLinkGraphics(bodyLinkGraphics);
      bodyJoint.setLink(bodyLink);

      description.addRootJoint(bodyJoint);
      robot = new RobotFromDescription(description);
   }

   public Robot getRobot()
   {
      return robot;
   }
}
