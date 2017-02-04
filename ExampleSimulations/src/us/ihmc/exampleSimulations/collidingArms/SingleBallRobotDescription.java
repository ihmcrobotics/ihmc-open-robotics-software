package us.ihmc.exampleSimulations.collidingArms;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class SingleBallRobotDescription extends RobotDescription
{
   public SingleBallRobotDescription(String name, double mass, double radius)
   {
      super(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription("ball");
      LinkDescription ballLink = new LinkDescription("ballLink");
      ballLink.setMassAndRadiiOfGyration(mass, radius, radius, radius);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.addSphere(radius, YoAppearance.Red());
      ballLink.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addSphere(radius);
      ballLink.setCollisionMesh(collisionMesh);

      rootJoint.setLink(ballLink);
      this.addRootJoint(rootJoint);
   }

}
