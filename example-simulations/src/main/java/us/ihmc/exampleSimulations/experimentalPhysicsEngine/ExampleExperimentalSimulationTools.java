package us.ihmc.exampleSimulations.experimentalPhysicsEngine;

import us.ihmc.euclid.Axis3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class ExampleExperimentalSimulationTools
{
   static RobotDescription createASingleBallRobot(String name, double radius, double mass, double radiusOfGyrationPercent, AppearanceDefinition appearance,
                                                  boolean addStripes, AppearanceDefinition stripesAppearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);
      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription ballLink = new LinkDescription(name + "Link");
      double radiusOfGyration = radiusOfGyrationPercent * radius;
      ballLink.setMassAndRadiiOfGyration(mass, radiusOfGyration, radiusOfGyration, radiusOfGyration);
   
      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.addSphere(radius, appearance);
   
      if (addStripes)
      {
         double stripePercent = 0.05;
         linkGraphics.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, stripesAppearance);
         linkGraphics.rotate(Math.PI / 2.0, Axis3D.X);
         linkGraphics.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, stripesAppearance);
      }
   
      ballLink.setLinkGraphics(linkGraphics);
      rootJoint.setLink(ballLink);
      robotDescription.addRootJoint(rootJoint);
   
      return robotDescription;
   }

   static RobotDescription createASingleCylinderRobot(String name, double radius, double height, double mass, double radiusOfGyrationPercent,
                                                              AppearanceDefinition appearance, boolean addStripes, AppearanceDefinition stripesAppearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);
   
      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription ballLink = new LinkDescription(name + "Link");
      ballLink.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * height);
   
      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -height / 2.0);
      linkGraphics.addCylinder(height, radius, appearance);
   
      if (addStripes)
      {
         double stripePercent = 0.05;
         linkGraphics.translate(0.0, 0.0, -height * 0.01);
         linkGraphics.addCube(2.0 * radius * 1.01, radius * stripePercent, height * 1.02, stripesAppearance);
         linkGraphics.rotate(Math.PI / 2.0, Axis3D.Z);
         linkGraphics.addCube(2.0 * radius * 1.01, radius * stripePercent, height * 1.02, stripesAppearance);
      }
   
      ballLink.setLinkGraphics(linkGraphics);
      rootJoint.setLink(ballLink);
      robotDescription.addRootJoint(rootJoint);
   
      return robotDescription;
   }

   static RobotDescription createASingleCapsuleRobot(String name, double radius, double height, double mass, double radiusOfGyrationPercent,
                                                             AppearanceDefinition appearance, boolean addStripes, AppearanceDefinition stripesAppearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);
   
      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription capsuleLink = new LinkDescription(name + "Link");
      capsuleLink.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * height);
   
      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.addCapsule(radius, height + 2.0 * radius, appearance);
   
      if (addStripes)
      {
         double stripePercent = 0.05;
         linkGraphics.translate(0.0, 0.0, -height / 2.0 + radius);
         linkGraphics.addCube(2.0 * radius * 1.01, radius * stripePercent, height - 2.0 * radius, stripesAppearance);
         linkGraphics.rotate(Math.PI / 2.0, Axis3D.Z);
         linkGraphics.addCube(2.0 * radius * 1.01, radius * stripePercent, height - 2.0 * radius, stripesAppearance);
      }
   
      capsuleLink.setLinkGraphics(linkGraphics);
      rootJoint.setLink(capsuleLink);
      robotDescription.addRootJoint(rootJoint);
   
      return robotDescription;
   }

   static RobotDescription createASingleBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent,
                                                         AppearanceDefinition appearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);
   
      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription link = new LinkDescription(name + "Link");
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * sizeX, radiusOfGyrationPercent * sizeY, radiusOfGyrationPercent * sizeZ);
   
      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -sizeZ / 2.0);
      linkGraphics.addCube(sizeX, sizeY, sizeZ, appearance);
      link.setLinkGraphics(linkGraphics);
   
      rootJoint.setLink(link);
      robotDescription.addRootJoint(rootJoint);
   
      return robotDescription;
   }
}
