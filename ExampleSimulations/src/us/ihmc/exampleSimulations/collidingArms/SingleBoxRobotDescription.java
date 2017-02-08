package us.ihmc.exampleSimulations.collidingArms;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class SingleBoxRobotDescription extends RobotDescription
{
   public SingleBoxRobotDescription(String name, double mass, double xLength, double yWidth, double zHeight, int collisionGroup, int collisionMask)
   {
      super(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription("box");
      LinkDescription link = new LinkDescription("boxLink");
      link.setMassAndRadiiOfGyration(mass, xLength, yWidth, zHeight);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -zHeight / 2.0);
      linkGraphics.addCube(xLength, yWidth, zHeight, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addCubeReferencedAtCenter(xLength, yWidth, zHeight);
      collisionMesh.setCollisionGroup(collisionGroup);
      collisionMesh.setCollisionMask(collisionMask);
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
      this.addRootJoint(rootJoint);
   }

}
