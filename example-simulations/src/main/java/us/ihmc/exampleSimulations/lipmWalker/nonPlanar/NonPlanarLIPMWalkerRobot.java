package us.ihmc.exampleSimulations.lipmWalker.nonPlanar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotDescription.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.*;

public class NonPlanarLIPMWalkerRobot
{
   private double bodyXRadius = 0.1, bodyYRadius = 0.2, bodyZRadius = 0.4;
   private double bodyRadiusOfGyrationY = 0.2;
   private double bodyRadiusOfGyrationZ = 0.2;
   private double bodyRadiusOfGyrationX = 0.2;
   private double bodyMass = 30.0;
   private double hipWidth = 0.3;
   private double thighMass = 0.2;
   private double thighRadiusOfGyrationX = 0.01;
   private double thighRadiusOfGyrationY = 0.01;
   private double thighRadiusOfGyrationZ = 0.01;
   private double thighLength = 0.6;
   private double thighRadius = 0.05;
   private double shinMass = 0.05;
   private double shinRadiusOfGyrationX = 0.01;
   private double shinRadiusOfGyrationY = 0.01;
   private double shinRadiusOfGyrationZ = 0.01;
   private double shinLength = 0.6;
   private double shinRadius = 0.03;


   private Robot robot;

   private FloatingJoint bodyJoint;
   private PinJoint leftHipPitchJoint, rightHipPitchJoint, leftHipRollJoint, rightHipRollJoint;
   private SliderJoint leftKneeJoint, rightKneeJoint;
   private SideDependentList<PinJoint> hipPitchJoints, hipRollJoints;
   private SideDependentList<SliderJoint> kneeJoints;
   private SideDependentList<GroundContactPoint> heelPoints;

   private final ArrayList<GroundContactPointDescription> gcPoints = new ArrayList<GroundContactPointDescription>(2);

   public NonPlanarLIPMWalkerRobot()
   {
      RobotDescription description = new NonPlanarLIPMWalkerRobotDescription("LIPMWalker");
      robot = new RobotFromDescription(description);

      bodyJoint = (FloatingJoint) robot.getRootJoints().get(0);

      leftHipPitchJoint = (PinJoint) robot.getJoint("leftHipPitch");
      rightHipPitchJoint = (PinJoint) robot.getJoint("rightHipPitch");

      leftHipRollJoint = (PinJoint) robot.getJoint("leftHipRoll");
      rightHipRollJoint = (PinJoint) robot.getJoint("rightHipRoll");

      leftKneeJoint = (SliderJoint) robot.getJoint("leftKnee");
      rightKneeJoint = (SliderJoint) robot.getJoint("rightKnee");

      kneeJoints = new SideDependentList<>(leftKneeJoint, rightKneeJoint);
      hipPitchJoints = new SideDependentList<>(leftHipPitchJoint, rightHipPitchJoint);
      hipRollJoints = new SideDependentList<>(leftHipRollJoint, rightHipRollJoint);

      heelPoints = new SideDependentList<GroundContactPoint>();

      List<GroundContactPoint> contactPoints = robot.getAllGroundContactPoints();
      for (GroundContactPoint point : contactPoints)
      {
         if (point.getName() == "gc_rheel")
         {
            heelPoints.set(RobotSide.RIGHT, point);
         }
         if (point.getName() == "gc_lheel")
         {
            heelPoints.set(RobotSide.LEFT, point);
         }
      }

      setupInitialConditions();

      LogTools.info("Robot: {}", robot.toString());
   }

   public double getMass()
   {
      Point3DBasics comPosition = new Point3D();
      double mass = robot.computeCenterOfMass(comPosition);
      return mass;
   }

   public Point3DReadOnly getCenterOfMassPosition()
   {
      Point3DBasics comPosition = new Point3D();
      robot.computeCenterOfMass(comPosition);
      return comPosition;
   }

   public Vector3DReadOnly getCenterOfMassVelocity()
   {
      Vector3DBasics comVelocity = new Vector3D();
      double mass = robot.computeLinearMomentum(comVelocity);
      comVelocity.scale(1.0 / mass);
      return comVelocity;
   }

   public double getBodyXPosition()
   {
      return bodyJoint.getQx().getDoubleValue();
   }

   public double getBodyZPosition()
   {
      return bodyJoint.getQz().getDoubleValue();
   }

   public double getBodyPitchAngle()
   {
      return bodyJoint.getOrientation().getPitch();
   }

   public double getBodyRollAngle()
   {
      return bodyJoint.getOrientation().getRoll();
   }

   public double getBodyPitchAngularVelocity()
   {
      return bodyJoint.getAngularVelocity().getY();
   }

   public double getBodyRollAngularVelocity()
   {
      return bodyJoint.getAngularVelocity().getX();
   }

   public double getHipAngleRoll(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getQ();
   }

   public double getHipAnglePitch(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getQ();
   }

   public double getHipVelocityPitch(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getQD();
   }

   public double getHipVelocityRoll(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getQD();
   }

   public double getKneeLength(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQD();
   }

   public void setHipTorquePitch(RobotSide robotSide, double torque)
   {
      hipPitchJoints.get(robotSide).setTau(torque);
   }

   public void setHipTorqueRoll(RobotSide robotSide, double torque)
   {
      hipRollJoints.get(robotSide).setTau(torque);
   }

   public void setKneeForce(RobotSide robotSide, double force)
   {
      kneeJoints.get(robotSide).setTau(force);
   }

   public Point3D getFootPosition(RobotSide robotSide)
   {
      return heelPoints.get(robotSide).getPositionCopy();
   }

   public double getFootZForce(RobotSide robotSide)
   {
      return heelPoints.get(robotSide).getYoForce().getZ();
   }

   public boolean getFootSwitch(RobotSide robotSide)
   {
      return (heelPoints.get(robotSide).getYoFootSwitch().getValue() > 0.5);
   }

   public double getKneeForce(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getTau();
   }

   public double getHipTorque(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getTau();
   }

   private void setupInitialConditions()
   {
      bodyJoint.setPosition(0.0, 0.0, 0.8);
      bodyJoint.setVelocity(0.7, 0.1, 0.0);
      bodyJoint.setYawPitchRoll(0.0, 0.0, 0.0);

      leftHipPitchJoint.setQ(0.0);
      rightHipPitchJoint.setQ(0.0);

      leftHipRollJoint.setQ(0.0);
      rightHipRollJoint.setQ(0.0);

      leftKneeJoint.setQ(0.8);
      rightKneeJoint.setQ(0.6);
   }

   private RobotDescription getRobotDescription()
   {
      RobotDescription description = new RobotDescription("LIPMWalker");
      FloatingPlanarJointDescription bodyJoint = new FloatingPlanarJointDescription("RootJoint", Plane.XZ);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      bodyJoint.setLink(bodyLink);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addEllipsoid(bodyXRadius, bodyYRadius, bodyZRadius, YoAppearance.AluminumMaterial());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      PinJointDescription leftHipJoint = new PinJointDescription("leftHip", new Vector3D(0.0, hipWidth / 2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(leftHipJoint);

      PinJointDescription rightHipJoint = new PinJointDescription("rightHip", new Vector3D(0.0, -hipWidth / 2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(rightHipJoint);

      LinkDescription leftThigh = createThighLink("leftThigh");
      leftHipJoint.setLink(leftThigh);

      LinkDescription rightThigh = createThighLink("rightThigh");
      rightHipJoint.setLink(rightThigh);

      SliderJointDescription leftKneeJoint = new SliderJointDescription("leftKnee", new Vector3D(), new Vector3D(0.0, 0.0, -1.0));
      SliderJointDescription rightKneeJoint = new SliderJointDescription("rightKnee", new Vector3D(), new Vector3D(0.0, 0.0, -1.0));

      LinkDescription leftShin = createShinLink("leftShin", YoAppearance.Red());
      leftKneeJoint.setLink(leftShin);
      leftHipJoint.addJoint(leftKneeJoint);

      LinkDescription rightShin = createShinLink("rightShin", YoAppearance.Green());
      rightKneeJoint.setLink(rightShin);
      rightHipJoint.addJoint(rightKneeJoint);

      GroundContactPointDescription gc_rheel = new GroundContactPointDescription("gc_rheel", new Vector3D(0.0, 0.0, 0.0));
      GroundContactPointDescription gc_lheel = new GroundContactPointDescription("gc_lheel", new Vector3D(0.0, 0.0, 0.0));

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_lheel);

      leftKneeJoint.addGroundContactPoint(gc_lheel);
      rightKneeJoint.addGroundContactPoint(gc_rheel);

      description.addRootJoint(bodyJoint);
      return description;
   }

   private LinkDescription createShinLink(String shinLink, AppearanceDefinition appearance)
   {
      LinkDescription shinLinkDescription = new LinkDescription(shinLink);
      shinLinkDescription.setMassAndRadiiOfGyration(shinMass, shinRadiusOfGyrationX, shinRadiusOfGyrationY, shinRadiusOfGyrationZ);
      LinkGraphicsDescription shinGraphics = new LinkGraphicsDescription();
      shinGraphics.addCylinder(shinLength, shinRadius, appearance);
      shinGraphics.addSphere(shinRadius, YoAppearance.AluminumMaterial());
      shinLinkDescription.setLinkGraphics(shinGraphics);
      return shinLinkDescription;
   }

   private LinkDescription createThighLink(String thighName)
   {
      LinkDescription thighLinkDescription = new LinkDescription(thighName);
      thighLinkDescription.setMassAndRadiiOfGyration(thighMass, thighRadiusOfGyrationX, thighRadiusOfGyrationY, thighRadiusOfGyrationZ);
      LinkGraphicsDescription thighGraphics = new LinkGraphicsDescription();
      thighGraphics.rotate(Math.PI, Axis3D.Y);
      thighGraphics.addCylinder(thighLength, thighRadius, YoAppearance.AluminumMaterial());
      thighLinkDescription.setLinkGraphics(thighGraphics);
      return thighLinkDescription;
   }

   public Point3D getCenterOfMassPositionFromSupportFoot()
   {
      Point3D leftFootPosition = getFootPosition(RobotSide.LEFT);
      Point3D rightFootPosition = getFootPosition(RobotSide.RIGHT);

      double leftForce = getFootZForce(RobotSide.LEFT);
      double rightForce = getFootZForce(RobotSide.RIGHT);

      Point3D loadedFootPosition = leftFootPosition;
      if (Math.abs(rightForce) > Math.abs(leftForce))
      {
         loadedFootPosition = rightFootPosition;
      }

      Point3D result = new Point3D(0.0, 0.0, 0.0);
      result.add(getCenterOfMassPosition());
      result.sub(loadedFootPosition);
      return result;
   }

   //   public double getCenterOfMassXDistanceFromSupportFoot(RobotSide stanceSide)
   //   {
   //      Point3D loadedFootPosition = getFootPosition(stanceSide);
   //      return getCenterOfMassPosition().getX() - loadedFootPosition.getX();
   //   }

   public Robot getRobot()
   {
      return robot;
   }

}

