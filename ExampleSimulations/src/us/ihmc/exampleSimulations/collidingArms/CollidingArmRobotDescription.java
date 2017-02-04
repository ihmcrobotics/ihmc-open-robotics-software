package us.ihmc.exampleSimulations.collidingArms;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class CollidingArmRobotDescription extends RobotDescription
{
   private final double baseMass = 1.0;
   private final double baseRadius = 0.1;
   private final double baseHeight = 0.05;

   private final double baseRadiusOfGyrationX = baseRadius;
   private final double baseRadiusOfGyrationY = baseRadius;
   private final double baseRadiusOfGyrationZ = baseHeight;

   public CollidingArmRobotDescription(String name, Vector3d baseOffset)
   {
      super(name);

      PinJointDescription baseJoint = new PinJointDescription("base", baseOffset, Axis.Z);
      LinkDescription baseLink = new LinkDescription("baseLink");
      baseLink.setMassAndRadiiOfGyration(baseMass, baseRadiusOfGyrationX, baseRadiusOfGyrationY, baseRadiusOfGyrationZ);

      LinkGraphicsDescription baseLinkGraphics = new LinkGraphicsDescription();
      AppearanceDefinition baseAppearance = YoAppearance.AliceBlue();
      baseLinkGraphics.addCylinder(baseHeight, baseRadius, baseAppearance);
      baseLink.setLinkGraphics(baseLinkGraphics);

      CollisionMeshDescription baseCollisionMesh = new CollisionMeshDescription();
      baseCollisionMesh.addCylinderReferencedAtBottomMiddle(baseRadius, baseHeight);
      baseLink.setCollisionMesh(baseCollisionMesh);

      baseJoint.setLink(baseLink);
      this.addRootJoint(baseJoint);

//      PinJointDescription shoulderJoint = new PinJointDescription("shoulder", new Vector3d(), Axis.Y);
//      baseJoint.addJoint(shoulderJoint);
//
//      PinJointDescription elbowJoint = new PinJointDescription("elbow", new Vector3d(), Axis.Y);
//      shoulderJoint.addJoint(elbowJoint);
   }

}
