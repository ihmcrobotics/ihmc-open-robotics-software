package us.ihmc.exampleSimulations.newtonsCradle;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class NewtonsCradleRobot extends Robot
{
   public NewtonsCradleRobot()
   {
      super("NewtonsCradle");

      int numberOfBalls = 6;
      double ballRadius = 0.05;

      double stringLength = 0.6;
      double stringRadius = 0.002;
      double ballMass = 0.2;
      double ballRadiusOfGyration = ballRadius * 0.6;

      double pinJointHeight = 1.1 * stringLength;
      double pinJointSeparation = 2.001 * ballRadius;

      for (int i = 0; i < numberOfBalls; i++)
      {
         Vector3d offset = new Vector3d(i * pinJointSeparation, 1.0, pinJointHeight);
         PinJoint pinJoint = new PinJoint("pin" + i, offset, this, Axis.Y);

         Link link = new Link("ball" + i);
         link.setMassAndRadiiOfGyration(ballMass, ballRadiusOfGyration, ballRadiusOfGyration, ballRadiusOfGyration);
         link.setComOffset(0.0, 0.0, -stringLength);

         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.translate(0.0, 0.0, -stringLength);
         linkGraphics.addCylinder(stringLength, stringRadius, YoAppearance.Yellow());
         AppearanceDefinition aliceBlue = YoAppearance.Red();
         aliceBlue.setTransparency(0.4);
         linkGraphics.addSphere(ballRadius, aliceBlue);
         link.setLinkGraphics(linkGraphics);

         CollisionMeshDescription collisionMeshDescription = new CollisionMeshDescription();
         collisionMeshDescription.translate(0.0, 0.0, -stringLength);
         collisionMeshDescription.addSphere(ballRadius);
         link.setCollisionMesh(collisionMeshDescription);

         pinJoint.setLink(link);
         this.addRootJoint(pinJoint);

         if ((i == 0) || (i == 1))
            pinJoint.setQ(0.3);

//         if ((i == numberOfBalls-1) || (i == numberOfBalls-2))
//            pinJoint.setQ(-0.3);
      }

      Graphics3DObject barAbove = new Graphics3DObject();
      this.addStaticLinkGraphics(barAbove);
   }


}
