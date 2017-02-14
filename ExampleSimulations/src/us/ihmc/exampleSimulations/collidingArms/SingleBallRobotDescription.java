package us.ihmc.exampleSimulations.collidingArms;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class SingleBallRobotDescription
{
   private String name;
   private double mass = 0.6;
   private double radius = 0.2;
   private int collisionGroup = 0xffff;
   private int collisionMask = 0xffff;
   private AppearanceDefinition appearance = YoAppearance.Red();
   private boolean addStripes = true;
   private AppearanceDefinition stripeAppearance = YoAppearance.Green();

   public SingleBallRobotDescription()
   {
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public void setCollisionGroup(int collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
   }

   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }

   public void setAppearance(AppearanceDefinition appearance)
   {
      this.appearance = appearance;
   }

   public void setAddStripes(boolean addStripes)
   {
      this.addStripes = addStripes;
   }

   public void setStripeAppearance(AppearanceDefinition stripeAppearance)
   {
      this.stripeAppearance = stripeAppearance;
   }

   public RobotDescription createRobotDescription()
   {
      RobotDescription robotDescription = new RobotDescription(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription("ball", name);
      LinkDescription ballLink = new LinkDescription("ballLink");
      ballLink.setMassAndRadiiOfGyration(mass, radius, radius, radius);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.addSphere(radius, appearance);

      if (addStripes)
      {
         double stripePercent = 0.05;
         linkGraphics.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, stripeAppearance);
         linkGraphics.rotate(Math.PI/2.0, Axis.X);
         linkGraphics.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, stripeAppearance);
      }

      ballLink.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addSphere(radius);
      collisionMesh.setCollisionGroup(collisionGroup);
      collisionMesh.setCollisionMask(collisionMask);
      ballLink.addCollisionMesh(collisionMesh);

      rootJoint.setLink(ballLink);
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

}
