package us.ihmc.exampleSimulations.collidingArms;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class SingleCapsuleRobotDescription
{
   private String name;
   private double mass = 0.6;
   private double radius = 0.2;
   private double height = 0.5;
   private int collisionGroup = 0xffff;
   private int collisionMask = 0xffff;
   private AppearanceDefinition appearance = YoAppearance.Aqua();
   private boolean addStripes = true;
   private AppearanceDefinition stripeAppearance = YoAppearance.Green();

   public SingleCapsuleRobotDescription()
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

   public void setHeight(double height)
   {
      this.height = height;
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

      FloatingJointDescription rootJoint = new FloatingJointDescription("capsule", name);
      LinkDescription capsuleLink = new LinkDescription("capsuleLink");
      capsuleLink.setMassAndRadiiOfGyration(mass, radius, radius, height);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
//      linkGraphics.translate(0.0, 0.0, -height / 2.0);
      linkGraphics.addCapsule(radius, height, appearance);

      if (addStripes)
      {
         double stripePercent = 0.05;
         linkGraphics.translate(0.0, 0.0, -height/2.0 + radius);
         linkGraphics.addCube(2.0 * radius * 1.01, radius * stripePercent, height - 2.0 * radius, stripeAppearance);
         linkGraphics.rotate(Math.PI / 2.0, Axis.Z);
         linkGraphics.addCube(2.0 * radius * 1.01, radius * stripePercent, height - 2.0 * radius, stripeAppearance);
      }

      capsuleLink.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addCapsule(radius, height);
      collisionMesh.setCollisionGroup(collisionGroup);
      collisionMesh.setCollisionMask(collisionMask);
      capsuleLink.addCollisionMesh(collisionMesh);

      rootJoint.setLink(capsuleLink);
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

}
