package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.Axis3D;
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
   private double hipWidth = 0.3;
   private double thighMass = 0.2;
   private double thighRadiusOfGyrationX = 0.01;
   private double thighRadiusOfGyrationY = 0.01;
   private double thighRadiusOfGyrationZ = 0.01;
   private double thighLength = 0.2;
   private double thighRadius = 0.05;

   public LIPMWalkerRobot()
   {
      RobotDescription description = new RobotDescription("LIPMWalker");
      FloatingPlanarJointDescription bodyJoint = new FloatingPlanarJointDescription("RootJoint", Plane.XZ);

      LinkDescription bodyLink = new LinkDescription("LinkJoint");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addSphere(bodyRadius, YoAppearance.AluminumMaterial());
      bodyLink.setLinkGraphics(bodyLinkGraphics);
      bodyJoint.setLink(bodyLink);


      PinJointDescription leftHipJoint = new PinJointDescription("leftHip", new Vector3D(0.0, hipWidth/2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));

      LinkDescription leftThigh = new LinkDescription("leftThigh");
      bodyLink.setMassAndRadiiOfGyration(thighMass, thighRadiusOfGyrationX, thighRadiusOfGyrationY, thighRadiusOfGyrationZ);
      LinkGraphicsDescription leftThighGraphics = new LinkGraphicsDescription();

      leftThighGraphics.rotate(Math.PI, Axis3D.Y);
      leftThighGraphics.addCylinder(thighLength, thighRadius);
      leftThigh.setLinkGraphics(leftThighGraphics);
      leftHipJoint.setLink(leftThigh);
      bodyJoint.addJoint(leftHipJoint);

      description.addRootJoint(bodyJoint);
      robot = new RobotFromDescription(description);
   }

   public Robot getRobot()
   {
      return robot;
   }
}
