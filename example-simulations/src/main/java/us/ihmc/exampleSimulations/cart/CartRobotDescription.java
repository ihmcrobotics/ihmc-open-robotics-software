package us.ihmc.exampleSimulations.cart;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class CartRobotDescription extends RobotDescription
{
   private int collisionGroup = 0xffffffff;
   private int collisionMask = 0xffffffff;

   private double massBody = 100.0;
   private double xLengthBody = 0.8;
   private double yLengthBody = 0.4;
   private double zLengthBody = 0.1;

   private double heightPoll = 0.6;
   private double radiusPoll = 0.02;
   private double widthFlag = 0.4;
   private double heightFlag = 0.2;
   private double thicknessFlag = 0.01;

   private double massWheel = 50;
   private double wheelPlacementRatio = 0.8; // ratio with length of x from center to the end in x direction
   private double radiusWheel = 0.08;
   private double lengthWheel = yLengthBody * 0.8;

   public CartRobotDescription(String name)
   {
      super(name);

      FloatingJointDescription bodyJoint = new FloatingJointDescription("body", "bodyjointvariablename");

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(massBody, xLengthBody, yLengthBody, zLengthBody);

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -zLengthBody / 2.0);
      bodyLinkGraphics.addCube(xLengthBody, yLengthBody, zLengthBody, YoAppearance.Red());
      bodyLinkGraphics.identity();
      bodyLinkGraphics.translate(xLengthBody / 2.0 - radiusPoll, yLengthBody / 2.0 - radiusPoll, 0.0);
      bodyLinkGraphics.addCylinder(heightPoll, radiusPoll);
      bodyLinkGraphics.translate(0.0, 0.0, heightPoll);
      bodyLinkGraphics.translate(-widthFlag / 2.0, 0.0, -heightFlag);
      bodyLinkGraphics.addCube(widthFlag, thicknessFlag, heightFlag, YoAppearance.AliceBlue());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      PinJointDescription frontWheelJoint = new PinJointDescription("frontwheel", new Vector3D(xLengthBody / 2.0 * wheelPlacementRatio, 0.0,
                                                                                               -zLengthBody / 2.0 - radiusWheel),
                                                                    Axis.Y);
      LinkDescription frontWheelLink = new LinkDescription("frontWheelLink");
      frontWheelLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, 0.0));
      frontWheelLink.setMassAndRadiiOfGyration(massWheel, radiusWheel, radiusWheel, lengthWheel);

      LinkGraphicsDescription frontWheelGraphics = new LinkGraphicsDescription();
      AppearanceDefinition frontWheelAppearance = YoAppearance.Black();
      frontWheelGraphics.rotate(Math.PI / 2.0, Axis.X);
      frontWheelGraphics.translate(new Vector3D(0.0, 0.0, -lengthWheel / 2.0));
      frontWheelGraphics.addCylinder(lengthWheel, radiusWheel, frontWheelAppearance);
      frontWheelGraphics.identity();
      frontWheelGraphics.translate(new Vector3D(radiusWheel, lengthWheel / 2.0, 0.0));
      frontWheelGraphics.rotate(Math.PI / 2.0, Axis.X);
      frontWheelGraphics.addCylinder(lengthWheel, radiusWheel * 0.1, YoAppearance.AliceBlue());
      frontWheelLink.setLinkGraphics(frontWheelGraphics);

      frontWheelJoint.setLink(frontWheelLink);
      bodyJoint.addJoint(frontWheelJoint);

      PinJointDescription rearWheelJoint = new PinJointDescription("rearwheel", new Vector3D(-xLengthBody / 2.0 * wheelPlacementRatio, 0.0,
                                                                                             -zLengthBody / 2.0 - radiusWheel),
                                                                   Axis.Y);
      LinkDescription rearWheelLink = new LinkDescription("rearWheelLink");
      rearWheelLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, 0.0));
      rearWheelLink.setMassAndRadiiOfGyration(massWheel, radiusWheel, radiusWheel, lengthWheel);

      LinkGraphicsDescription rearWheelGraphics = new LinkGraphicsDescription();
      AppearanceDefinition rearWheelAppearance = YoAppearance.Grey();
      rearWheelGraphics.rotate(Math.PI / 2.0, Axis.X);
      rearWheelGraphics.translate(new Vector3D(0.0, 0.0, -lengthWheel / 2.0));
      rearWheelGraphics.addCylinder(lengthWheel, radiusWheel, rearWheelAppearance);
      rearWheelGraphics.identity();
      rearWheelGraphics.translate(new Vector3D(radiusWheel, lengthWheel / 2.0, 0.0));
      rearWheelGraphics.rotate(Math.PI / 2.0, Axis.X);
      rearWheelGraphics.addCylinder(lengthWheel, radiusWheel * 0.1, YoAppearance.AliceBlue());
      rearWheelLink.setLinkGraphics(rearWheelGraphics);

      rearWheelJoint.setLink(rearWheelLink);
      bodyJoint.addJoint(rearWheelJoint);
   }

}
