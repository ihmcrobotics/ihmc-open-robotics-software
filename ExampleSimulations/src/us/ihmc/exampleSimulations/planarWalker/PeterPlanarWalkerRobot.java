package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.Plane;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class PeterPlanarWalkerRobot extends Robot
{
   private final double initalHipAngle = 0.0;
   private final double initalBodyVelocity = 0.0;
   private double GRAVITY = -9.81;
   private FloatingPlanarJoint bodyJoint;
   private SideDependentList<PinJoint> hipJoints = new SideDependentList<PinJoint>();
   private SideDependentList<SliderJoint> kneeJoints = new SideDependentList<SliderJoint>();
   private SideDependentList<GroundContactPoint> gCpoints = new SideDependentList<GroundContactPoint>();

   private double bodyMass = 10.0, lowerLinkMass = 0.5, upperLinkMass = 1.0;
   public final double lowerLinkLength = 0.8, upperLinkLength = 0.7;
   private double lowerLinkRadius = 0.04, upperLinkRadius = 0.05;
   private double legHeight = lowerLinkLength + upperLinkLength;
   private double gcOffset = -lowerLinkLength;
   private double bodyLength = 0.3, bodyWidth = 0.1, bodyHeight = 0.1;
   private double hipOffsetY = bodyWidth / 2.0;
   private double maxLegExtension = lowerLinkLength;

   public final double nominalHeight = upperLinkLength + lowerLinkLength / 2.0;

   public PeterPlanarWalkerRobot()
   {
      super("Walker");
      this.setGravity(GRAVITY);

      bodyJoint = new FloatingPlanarJoint("body", this, Plane.XZ);

      Link bodyLink = getBodyLink();

      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      for (RobotSide robotSide : RobotSide.values())
      {

         PinJoint hipJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "Hip", new Vector3D(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0), this,
               Axis.Y);
         hipJoints.put(robotSide, hipJoint);
         hipJoint.setDynamic(true);
         hipJoint.setLimitStops(-Math.PI, Math.PI, 1e6, 1e3);
         Link upperLink = upperLink(robotSide);
         hipJoint.setLink(upperLink);
         bodyJoint.addJoint(hipJoint);

         /************************************************************/

         SliderJoint kneeJoint = new SliderJoint(robotSide.getSideNameFirstLetter() + "Knee", new Vector3D(0.0, 0.0, -upperLinkLength), this, new Vector3D(0.0,
               0.0, -1.0)); //TODO change offset depending on height
         kneeJoints.put(robotSide, kneeJoint);
         kneeJoint.setDynamic(true);
         kneeJoint.setLimitStops(0.0, maxLegExtension, 1e5, 1e4); //TODO change limits depending on initial position. Eg: (0.0, 0.6)
         Link lowerLink = lowerLink(robotSide);
         kneeJoint.setLink(lowerLink);
         hipJoint.addJoint(kneeJoint);

         /*************************************************************/

         GroundContactPoint contactPoint = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "Foot", new Vector3D(0.0, 0.0, 0.0), this);
         gCpoints.set(robotSide, contactPoint);
         kneeJoints.get(robotSide).addGroundContactPoint(contactPoint);
         Graphics3DObject graphics = kneeJoints.get(robotSide).getLink().getLinkGraphics();
         graphics.identity();
         graphics.translate(0.0, 0.0, 0.0);
         double radius = 1.5 * lowerLinkRadius;
         graphics.addSphere(radius, YoAppearance.Orange());
      }

      double groundKxy = 1e4;
      double groundBxy = 1e2;
      double groundKz = 125.0;
      double groundBz = 300.0;
      LinearGroundContactModel ground = new LinearGroundContactModel(this, groundKxy, groundBxy, groundKz, groundBz, this.getRobotsYoVariableRegistry());

      this.setGroundContactModel(ground);

      initialize();
   }

   private void initialize()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         kneeJoints.get(robotSide).getQYoVariable().set(maxLegExtension / 2.0);

         double sign = robotSide.negateIfLeftSide(1.0);
         hipJoints.get(robotSide).getQYoVariable().set(initalHipAngle * sign);
      }

      bodyJoint.getQd_t1().set(initalBodyVelocity);
      this.setRobotXZ(0.0, nominalHeight);

   }

   public void setRobotXZ(double x, double z)
   {
      bodyJoint.getQ_t1().set(x);
      bodyJoint.getQ_t2().set(z);
   }

   private Link upperLink(RobotSide robotSide)
   {

      Link ret = new Link("upperLink");
      ret.setMass(upperLinkMass);

      // Inertia tensor
      double IxxCyl = (upperLinkMass / 3) * (Math.pow(upperLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, 0.0, IzzCyl);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (robotSide == RobotSide.RIGHT)
      {
         linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.Pink());
      }
      else
      {
         linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.CadetBlue());
      }
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, -upperLinkLength / 2.0);
      //ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link lowerLink(RobotSide robotSide)
   {

      Link ret = new Link("lowerLink");
      ret.setMass(lowerLinkMass);

      // Inertia tensor
      double IxxCyl = (lowerLinkMass / 3.0) * (Math.pow(lowerLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, 0.0, IzzCyl);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      //linkGraphics.addCoordinateSystem(0.4);
      if (robotSide == RobotSide.RIGHT)
      {
         linkGraphics.addCylinder(lowerLinkLength, lowerLinkRadius, YoAppearance.Red());
      }
      else
      {
         linkGraphics.addCylinder(lowerLinkLength, lowerLinkRadius, YoAppearance.Blue());
      }

      ret.setLinkGraphics(linkGraphics);
      //ret.addCoordinateSystemToCOM(0.4);
      return ret;
   }

   public double getKneePosition(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQYoVariable().getDoubleValue();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQDYoVariable().getDoubleValue();
   }

   public double getHipPosition(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQYoVariable().getDoubleValue();
   }

   public double getHipVelocity(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQDYoVariable().getDoubleValue();
   }

   public void setKneeTorque(RobotSide robotSide, double torque)
   {
      kneeJoints.get(robotSide).getTauYoVariable().set(torque);
   }

   public void setHipTorque(RobotSide robotSide, double torque)
   {
      hipJoints.get(robotSide).getTauYoVariable().set(torque);
   }

   public double getHipTorque(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getTauYoVariable().getDoubleValue();
   }

   public boolean isFootOnGround(RobotSide robotSide)
   {
      return gCpoints.get(robotSide).isInContact();
   }

   public double getBodyPitch()
   {
      return bodyJoint.getQ_rot().getDoubleValue();
   }

   public double getBodyPitchVelocity()
   {
      return bodyJoint.getQd_rot().getDoubleValue();
   }

   public double getBodyHeight()
   {
      return bodyJoint.getQ_t2().getDoubleValue();
   }

   public double getBodyHeightVelocity()
   {
      return bodyJoint.getQd_t2().getDoubleValue();
   }

   public double getBodyVelocity()
   {
      return bodyJoint.getQd_t1().getDoubleValue();
   }

   private Link getBodyLink()
   {
      Link body = new Link("body");
      body.setMassAndRadiiOfGyration(bodyMass, bodyLength, bodyWidth, bodyHeight);

      body.setComOffset(new Vector3D(0.0, 0.0, 0.0));
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.addCube(0.4, 0.2, 0.1, YoAppearance.AliceBlue());
      graphics.addCoordinateSystem(0.6);
      body.setLinkGraphics(graphics);
      return body;
   }

}
