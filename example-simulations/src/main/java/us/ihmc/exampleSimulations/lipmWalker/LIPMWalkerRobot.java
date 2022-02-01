package us.ihmc.exampleSimulations.lipmWalker;

import java.util.ArrayList;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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

public class LIPMWalkerRobot
{
   private double bodyRadius = 0.2;
   private double bodyRadiusOfGyrationY = 0.1;
   private double bodyRadiusOfGyrationZ = 0.1;
   private double bodyRadiusOfGyrationX = 0.1;
   private double bodyMass = 1.0;
   private Robot robot;
   private double hipWidth = 0.3;
   private double thighMass = 0.2;
   private double thighRadiusOfGyrationX = 0.01;
   private double thighRadiusOfGyrationY = 0.01;
   private double thighRadiusOfGyrationZ = 0.01;
   private double thighLength = 0.6;
   private double thighRadius = 0.05;
   private double shinMass = 0.2;
   private double shinRadiusOfGyrationX = 0.01;
   private double shinRadiusOfGyrationY = 0.01;
   private double shinRadiusOfGyrationZ = 0.01;
   private double shinLength = 0.6;
   private double shinRadius = 0.03;

   private final ArrayList<GroundContactPointDescription> gcPoints = new ArrayList<GroundContactPointDescription>(2);

   private final FloatingPlanarJoint bodyJoint;
   private final PinJoint leftHipJoint, rightHipJoint;
   private final SliderJoint leftKneeJoint, rightKneeJoint;
   private final SideDependentList<PinJoint> hipJoints;
   private final SideDependentList<SliderJoint> kneeJoints;

   public LIPMWalkerRobot()
   {
      RobotDescription description = getRobotDescription();
      robot = new RobotFromDescription(description);

      bodyJoint = (FloatingPlanarJoint) robot.getRootJoints().get(0);

      leftHipJoint = (PinJoint) robot.getJoint("leftHip");
      rightHipJoint = (PinJoint) robot.getJoint("rightHip");

      hipJoints = new SideDependentList<>(leftHipJoint, rightHipJoint);

      leftKneeJoint = (SliderJoint) robot.getJoint("leftKnee");
      rightKneeJoint = (SliderJoint) robot.getJoint("rightKnee");

      kneeJoints = new SideDependentList<>(leftKneeJoint, rightKneeJoint);

      setupInitialConditions();

      LogTools.info("Robot: {}", robot.toString());
   }

   public double getHipAngle(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQ();
   }

   public double getKneeLength(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getHipVelocity(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQD();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQD();
   }

   public void setJointTorque(RobotSide robotSide, double torque)
   {
      hipJoints.get(robotSide).setTau(torque);
   }

   public void setKneeForce(RobotSide robotSide, double force)
   {
      hipJoints.get(robotSide).setTau(force);
   }

   private void setupInitialConditions()
   {
      bodyJoint.setCartesianPosition(0.0, 1.0);
      bodyJoint.setCartesianVelocity(0.0, 0.5);
      bodyJoint.setRotation(0.0);

      leftHipJoint.setQ(0.2);
      rightHipJoint.setQ(-0.2);

      leftKneeJoint.setQ(1.0);
      rightKneeJoint.setQ(1.0);

   }

   private RobotDescription getRobotDescription()
   {
      RobotDescription description = new RobotDescription("LIPMWalker");
      FloatingPlanarJointDescription bodyJoint = new FloatingPlanarJointDescription("RootJoint", Plane.XZ);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      bodyJoint.setLink(bodyLink);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addSphere(bodyRadius, YoAppearance.AluminumMaterial());
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
//      shinGraphics.translate(new Vector3D(0.0, 0.0, 2.0 * -thighLength));
//      shinGraphics.rotate(Math.PI, Axis3D.Y);
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

   public Robot getRobot()
   {
      return robot;
   }
}
