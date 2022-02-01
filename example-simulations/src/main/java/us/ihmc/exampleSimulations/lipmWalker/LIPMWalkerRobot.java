package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.robotdefinition.JointDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.LinkDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;

import java.util.ArrayList;

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

   public LIPMWalkerRobot()
   {
      RobotDescription description = new RobotDescription("LIPMWalker");
      FloatingPlanarJointDescription bodyJoint = new FloatingPlanarJointDescription("RootJoint", Plane.XZ);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      bodyJoint.setLink(bodyLink);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addSphere(bodyRadius, YoAppearance.AluminumMaterial());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      PinJointDescription leftHipJoint = new PinJointDescription("leftHip", new Vector3D(0.0, hipWidth/2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(leftHipJoint);

      PinJointDescription rightHipJoint = new PinJointDescription("rightHip", new Vector3D(0.0, -hipWidth/2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(rightHipJoint);

      LinkDescription leftThigh = new LinkDescription("leftThigh");
      leftThigh.setMassAndRadiiOfGyration(thighMass, thighRadiusOfGyrationX, thighRadiusOfGyrationY, thighRadiusOfGyrationZ);
      LinkGraphicsDescription leftThighGraphics = new LinkGraphicsDescription();
      leftThighGraphics.rotate(Math.PI, Axis3D.Y);
      leftThighGraphics.addCylinder(thighLength, thighRadius, YoAppearance.AluminumMaterial());
      leftThigh.setLinkGraphics(leftThighGraphics);
      leftHipJoint.setLink(leftThigh);

      LinkDescription rightThigh = new LinkDescription("rightThigh");
      rightThigh.setMassAndRadiiOfGyration(thighMass, thighRadiusOfGyrationX, thighRadiusOfGyrationY, thighRadiusOfGyrationZ);
      LinkGraphicsDescription rightThighGraphics = new LinkGraphicsDescription();
      rightThighGraphics.rotate(Math.PI, Axis3D.Y);
      rightThighGraphics.addCylinder(thighLength, thighRadius, YoAppearance.AluminumMaterial());
      rightThigh.setLinkGraphics(rightThighGraphics);
      rightHipJoint.setLink(rightThigh);


      SliderJointDescription rightKneeJoint = new SliderJointDescription("leftKnee", new Vector3D(0, 0, thighLength), new Vector3D(0,0,1));
      SliderJointDescription leftKneeJoint = new SliderJointDescription("rightKnee", new Vector3D(0, 0, thighLength), new Vector3D(0,0,1));

      LinkDescription leftShin = new LinkDescription("leftShin");
      leftShin.setMassAndRadiiOfGyration(shinMass, shinRadiusOfGyrationX, shinRadiusOfGyrationY, shinRadiusOfGyrationZ);
      LinkGraphicsDescription leftShinGraphics = new LinkGraphicsDescription();
      leftShinGraphics.translate(new Vector3D(0, 0, 2 * -thighLength));
      leftShinGraphics.rotate(Math.PI, Axis3D.Y);
      leftShinGraphics.addCylinder(shinLength, shinRadius, YoAppearance.BlackMetalMaterial());
      leftShin.setLinkGraphics(leftShinGraphics);
      leftKneeJoint.setLink(leftShin);
      leftHipJoint.addJoint(leftKneeJoint);

      LinkDescription rightShin = new LinkDescription("rightShin");
      rightShin.setMassAndRadiiOfGyration(shinMass, shinRadiusOfGyrationX, shinRadiusOfGyrationY, shinRadiusOfGyrationZ);
      LinkGraphicsDescription rightShinGraphics = new LinkGraphicsDescription();
      rightShinGraphics.translate(new Vector3D(0, 0, 2 * -thighLength));
      rightShinGraphics.rotate(Math.PI, Axis3D.Y);
      rightShinGraphics.addCylinder(shinLength, shinRadius, YoAppearance.BlackMetalMaterial());
      rightShin.setLinkGraphics(rightShinGraphics);
      rightKneeJoint.setLink(rightShin);
      rightHipJoint.addJoint(rightKneeJoint);

      GroundContactPointDescription gc_rheel = new GroundContactPointDescription("gc_rheel", new Vector3D(0, 0.0, shinLength));
      GroundContactPointDescription gc_lheel = new GroundContactPointDescription("gc_lheel", new Vector3D(0, 0.0, shinLength));

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_lheel);

      leftKneeJoint.addGroundContactPoint(gc_lheel);
      rightKneeJoint.addGroundContactPoint(gc_rheel);

      description.addRootJoint(bodyJoint);
      robot = new RobotFromDescription(description);

      LogTools.info("Robot: {}", robot.toString());
   }

   public Robot getRobot()
   {
      return robot;
   }
}
