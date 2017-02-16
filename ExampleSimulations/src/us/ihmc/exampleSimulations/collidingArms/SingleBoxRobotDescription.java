package us.ihmc.exampleSimulations.collidingArms;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class SingleBoxRobotDescription
{
   private String name;
   private double mass = 0.6;
   private double xLength = 0.2;
   private double yWidth = 0.5;
   private double zHeight = 0.5;
   private double radiusOfGyrationPercent = 1.0;

   private int collisionGroup = 0xffff;
   private int collisionMask = 0xffff;
   private AppearanceDefinition appearance = YoAppearance.Aqua();

   public SingleBoxRobotDescription()
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

   public void setXLength(double xLength)
   {
      this.xLength = xLength;
   }

   public void setYWidth(double yWidth)
   {
      this.yWidth = yWidth;
   }

   public void setZHeight(double zHeight)
   {
      this.zHeight = zHeight;
   }

   public void setRadiusOfGyrationPercent(double radiusOfGyrationPercent)
   {
      this.radiusOfGyrationPercent = radiusOfGyrationPercent;
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

   public RobotDescription createRobotDescription()
   {
      RobotDescription robotDescription = new RobotDescription(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription("box", name);
      LinkDescription link = new LinkDescription("boxLink");
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * xLength, radiusOfGyrationPercent * yWidth, radiusOfGyrationPercent * zHeight);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -zHeight / 2.0);
      linkGraphics.addCube(xLength, yWidth, zHeight, appearance);
      link.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addCubeReferencedAtCenter(xLength, yWidth, zHeight);
      collisionMesh.setCollisionGroup(collisionGroup);
      collisionMesh.setCollisionMask(collisionMask);
      collisionMesh.setEstimatedNumberOfContactPoints(16);
      link.addCollisionMesh(collisionMesh);

      for (int i = 0; i < 8; i++)
      {
         double xSign = 1.0;
         double ySign = 1.0;
         double zSign = 1.0;

         if (i % 2 == 0)
         {
            xSign = -1.0;
         }

         if ((i / 2) % 2 == 0)
         {
            ySign = -1.0;
         }

         if ((i / 4) % 2 == 0)
         {
            zSign = -1.0;
         }

         GroundContactPointDescription groundContactPoint = new GroundContactPointDescription("gc_"
               + i, new Vector3d(xSign * xLength / 2.0, ySign * yWidth / 2.0, zSign * zHeight / 2.0));
         rootJoint.addGroundContactPoint(groundContactPoint);
      }

      rootJoint.setLink(link);
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

}
