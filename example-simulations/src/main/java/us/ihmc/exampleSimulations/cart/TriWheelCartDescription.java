package us.ihmc.exampleSimulations.cart;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class TriWheelCartDescription extends RobotDescription
{
   private final int collisionGroup = 0xffffffff;
   private final int collisionMask = 0xffffffff;

   private final double massBody = 100.0;
   private final double massActiveWheel = 50.0;
   private final double massCasterAxis = 30.0;

   private final double heightBody = 0.1;
   private final double widthBody = 0.9;
   private final double lengthBody = 0.6;
   private final double wheelPlacementRatio = 0.8;

   private final double radiusActiveWheel = 0.15;
   private final double radiusCasterWheel = 0.1;
   private final double offsetZActiveWheel = 0.2;
   private final double thicknessActiveWheel = 0.05;
   private final double thicknessCasterWheel = 0.03;
   private final double offsetXCasterWheel = 0.15;
   private final double offsetZCasterWheel = offsetZActiveWheel + radiusActiveWheel - radiusCasterWheel;
   private final double radiusOffset = 0.035;

   private final double heightPoll = 0.6;
   private final double radiusPoll = 0.02;
   private final double widthFlag = 0.4;
   private final double heightFlag = 0.2;
   private final double thicknessFlag = 0.01;

   public TriWheelCartDescription(String name)
   {
      super(name);

      // Body.
      FloatingJointDescription bodyJoint = new FloatingJointDescription("body", "bodyJointVariableName");

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(massBody, lengthBody, widthBody, heightBody);

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -heightBody / 2.0);
      bodyLinkGraphics.addCube(lengthBody, widthBody, heightBody, YoAppearance.Red());
      bodyLinkGraphics.identity();
      bodyLinkGraphics.translate(lengthBody / 2.0 - radiusPoll, widthBody / 2.0 - radiusPoll, 0.0);
      bodyLinkGraphics.addCylinder(heightPoll, radiusPoll);
      bodyLinkGraphics.translate(0.0, 0.0, heightPoll);
      bodyLinkGraphics.translate(-widthFlag / 2.0, 0.0, -heightFlag);
      bodyLinkGraphics.addCube(widthFlag, thicknessFlag, heightFlag, YoAppearance.AliceBlue());
      bodyLinkGraphics.identity();
      bodyLinkGraphics.translate(lengthBody / 2.0 * wheelPlacementRatio, -widthBody / 2.0 * wheelPlacementRatio, -heightBody / 2.0 - offsetZActiveWheel);
      bodyLinkGraphics.addCylinder(offsetZActiveWheel, radiusOffset, YoAppearance.Beige());
      bodyLinkGraphics.identity();
      bodyLinkGraphics.translate(lengthBody / 2.0 * wheelPlacementRatio, widthBody / 2.0 * wheelPlacementRatio, -heightBody / 2.0 - offsetZActiveWheel);
      bodyLinkGraphics.addCylinder(offsetZActiveWheel, radiusOffset, YoAppearance.Beige());
      bodyLinkGraphics.identity();
      bodyLinkGraphics.translate(-lengthBody / 2.0 * wheelPlacementRatio, 0.0, -heightBody / 2.0 - offsetZCasterWheel);
      bodyLinkGraphics.addCylinder(offsetZCasterWheel, radiusOffset, YoAppearance.Beige());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      CollisionMeshDescription bodyCollisionMesh = new CollisionMeshDescription();
      bodyCollisionMesh.addCubeReferencedAtCenter(lengthBody, widthBody, heightBody);
      bodyCollisionMesh.setCollisionGroup(collisionGroup);
      bodyCollisionMesh.setCollisionMask(collisionMask);
      bodyLink.addCollisionMesh(bodyCollisionMesh);

      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      // Right and left side active wheels.
      PinJointDescription rightWheelJoint = createWheelJoint("rightWheel",
                                                             new Vector3D(lengthBody / 2.0 * wheelPlacementRatio, -widthBody / 2.0 * wheelPlacementRatio,
                                                                          -heightBody / 2.0 - offsetZActiveWheel),
                                                             radiusActiveWheel, thicknessActiveWheel, YoAppearance.Black());
      bodyJoint.addJoint(rightWheelJoint);

      PinJointDescription leftWheelJoint = createWheelJoint("leftWheel",
                                                            new Vector3D(lengthBody / 2.0 * wheelPlacementRatio, widthBody / 2.0 * wheelPlacementRatio,
                                                                         -heightBody / 2.0 - offsetZActiveWheel),
                                                            radiusActiveWheel, thicknessActiveWheel, YoAppearance.Grey());
      bodyJoint.addJoint(leftWheelJoint);

      // Passive joint on caster axis.
      PinJointDescription casterAxisJoint = new PinJointDescription("casterAxis", new Vector3D(-lengthBody / 2.0 * wheelPlacementRatio, 0.0,
                                                                                               -heightBody / 2.0 - offsetZCasterWheel),
                                                                    Axis3D.Z);
      LinkDescription casterAxisLink = new LinkDescription("casterAxisLink");
      casterAxisLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, 0.0));
      casterAxisLink.setMassAndRadiiOfGyration(massCasterAxis, radiusActiveWheel, radiusOffset, radiusOffset);

      LinkGraphicsDescription wheelGraphics = new LinkGraphicsDescription();
      AppearanceDefinition wheelAppearance = YoAppearance.Green();
      wheelGraphics.rotate(-Math.PI / 2.0, Axis3D.Y);
      wheelGraphics.addCylinder(offsetXCasterWheel, radiusOffset, wheelAppearance);
      casterAxisLink.setLinkGraphics(wheelGraphics);

      CollisionMeshDescription casterAxisCollisionMesh = new CollisionMeshDescription();
      casterAxisCollisionMesh.rotate(-Math.PI / 2.0, Axis3D.Y);
      casterAxisCollisionMesh.addCylinderReferencedAtBottomMiddle(radiusOffset, offsetXCasterWheel);
      casterAxisCollisionMesh.setCollisionGroup(collisionGroup);
      casterAxisCollisionMesh.setCollisionMask(collisionMask);
      casterAxisLink.addCollisionMesh(casterAxisCollisionMesh);

      casterAxisJoint.setLink(casterAxisLink);
      bodyJoint.addJoint(casterAxisJoint);

      // Passive joint on caster wheel.
      PinJointDescription casterWheelJoint = createWheelJoint("casterWheel", new Vector3D(-offsetXCasterWheel, 0.0, 0.0), radiusCasterWheel,
                                                              thicknessCasterWheel, YoAppearance.Aqua());

      casterAxisJoint.addJoint(casterWheelJoint);
   }

   private PinJointDescription createWheelJoint(String jointName, Vector3D vectorToParent, double radius, double thickness, AppearanceDefinition yoAppearance)
   {
      PinJointDescription wheelJoint = new PinJointDescription(jointName, vectorToParent, Axis3D.Y);
      LinkDescription wheelLink = new LinkDescription(jointName + "link");
      wheelLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, 0.0));
      wheelLink.setMassAndRadiiOfGyration(massActiveWheel, offsetXCasterWheel, thickness, radius);

      LinkGraphicsDescription wheelGraphics = new LinkGraphicsDescription();
      wheelGraphics.rotate(Math.PI / 2.0, Axis3D.X);
      wheelGraphics.translate(new Vector3D(0.0, 0.0, -thickness / 2.0));
      wheelGraphics.addCylinder(thickness, radius, yoAppearance);
      wheelGraphics.identity();
      wheelGraphics.translate(new Vector3D(radius, thickness / 2.0, 0.0));
      wheelGraphics.rotate(Math.PI / 2.0, Axis3D.X);
      wheelGraphics.addCylinder(thickness, radius * 0.1, YoAppearance.AliceBlue());
      wheelLink.setLinkGraphics(wheelGraphics);

      CollisionMeshDescription wheelCollisionMesh = new CollisionMeshDescription();
      wheelCollisionMesh.rotate(Math.PI / 2.0, Axis3D.X);
      wheelCollisionMesh.translate(new Vector3D(0.0, 0.0, -thickness / 2.0));
      wheelCollisionMesh.addCylinderReferencedAtBottomMiddle(radius, thickness);
      wheelCollisionMesh.setCollisionGroup(collisionGroup);
      wheelCollisionMesh.setCollisionMask(collisionMask);
      wheelLink.addCollisionMesh(wheelCollisionMesh);

      wheelJoint.setLink(wheelLink);

      return wheelJoint;
   }

   public double getInitialHeight()
   {
      return offsetZActiveWheel + radiusActiveWheel + heightBody * 0.5;
   }
}
