package us.ihmc.exampleSimulations.collidingArms;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class SingleCylinderRobotDescription extends RobotDescription
{
   public SingleCylinderRobotDescription(String name, double mass, double radius, double height)
   {
      super(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription("cylinder");
      LinkDescription link = new LinkDescription("cylinderLink");
      link.setMassAndRadiiOfGyration(mass, radius, radius, height);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -height/2.0);
      linkGraphics.addCylinder(height, radius, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addCylinderReferencedAtCenter(radius, height);
      link.addCollisionMesh(collisionMesh);

      rootJoint.setLink(link);
      this.addRootJoint(rootJoint);
   }

}
