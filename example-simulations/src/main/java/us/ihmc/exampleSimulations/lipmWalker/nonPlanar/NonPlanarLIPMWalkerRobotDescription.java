package us.ihmc.exampleSimulations.lipmWalker.nonPlanar;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.simulationconstructionset.*;

import java.util.ArrayList;

public class NonPlanarLIPMWalkerRobotDescription extends RobotDescription
{
   private double bodyXRadius = 0.1, bodyYRadius = 0.2, bodyZRadius = 0.3;
   private double bodyRadiusOfGyrationY = 0.2;
   private double bodyRadiusOfGyrationZ = 0.2;
   private double bodyRadiusOfGyrationX = 0.2;
   private double bodyMass = 30.0;
   private Robot robot;
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

   private double hipMass = 0.02;
   private double hipRadiusOfGyrationX = 0.005;
   private double hipRadiusOfGyrationY = 0.005;
   private double hipRadiusOfGyrationZ = 0.005;
   private double hipLength = 0.15;
   private double hipRadius = 0.1;

   private final ArrayList<GroundContactPointDescription> gcPoints = new ArrayList<GroundContactPointDescription>(2);

   private final FloatingJointDescription bodyJoint;
   private final PinJointDescription leftHipPitchJoint, rightHipPitchJoint, leftHipRollJoint, rightHipRollJoint;
   private final SliderJointDescription leftKneeJoint, rightKneeJoint;

   public NonPlanarLIPMWalkerRobotDescription(String name)
   {
      super(name);

      bodyJoint = new FloatingJointDescription("RootJoint");

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      bodyJoint.setLink(bodyLink);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addEllipsoid(bodyXRadius, bodyYRadius, bodyZRadius, YoAppearance.AluminumMaterial());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      leftHipPitchJoint = new PinJointDescription("leftHipPitch", new Vector3D(0.0, hipWidth / 2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      rightHipPitchJoint = new PinJointDescription("rightHipPitch", new Vector3D(0.0, -hipWidth / 2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(leftHipPitchJoint);
      bodyJoint.addJoint(rightHipPitchJoint);

      LinkDescription leftHipLink = createHipLink("leftHipLink", YoAppearance.Brown());
      leftHipPitchJoint.setLink(leftHipLink);

      LinkDescription rightHipLink = createHipLink("rightHipLink", YoAppearance.Brown());
      rightHipPitchJoint.setLink(rightHipLink);

      leftHipRollJoint = new PinJointDescription("leftHipRoll", new Vector3D(0.0, 0.0, 0.0), new Vector3D(1.0, 0.0, 0.0));
      rightHipRollJoint = new PinJointDescription("rightHipRoll", new Vector3D(0.0, 0.0, 0.0), new Vector3D(1.0, 0.0, 0.0));
      leftHipPitchJoint.addJoint(leftHipRollJoint);
      rightHipPitchJoint.addJoint(rightHipRollJoint);

      LinkDescription leftThigh = createThighLink("leftThigh");
      leftHipRollJoint.setLink(leftThigh);

      LinkDescription rightThigh = createThighLink("rightThigh");
      rightHipRollJoint.setLink(rightThigh);

      leftKneeJoint = new SliderJointDescription("leftKnee", new Vector3D(), new Vector3D(0.0, 0.0, -1.0));
      rightKneeJoint = new SliderJointDescription("rightKnee", new Vector3D(), new Vector3D(0.0, 0.0, -1.0));

      LinkDescription leftShin = createShinLink("leftShin", YoAppearance.Red());
      leftKneeJoint.setLink(leftShin);
      leftHipRollJoint.addJoint(leftKneeJoint);

      LinkDescription rightShin = createShinLink("rightShin", YoAppearance.Green());
      rightKneeJoint.setLink(rightShin);
      rightHipRollJoint.addJoint(rightKneeJoint);

      GroundContactPointDescription gc_rheel = new GroundContactPointDescription("gc_rheel", new Vector3D(0.0, 0.0, 0.0));
      GroundContactPointDescription gc_lheel = new GroundContactPointDescription("gc_lheel", new Vector3D(0.0, 0.0, 0.0));

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_lheel);

      leftKneeJoint.addGroundContactPoint(gc_lheel);
      rightKneeJoint.addGroundContactPoint(gc_rheel);

      this.addRootJoint(bodyJoint);

   }

   private LinkDescription createHipLink(String hipLink, AppearanceDefinition appearance)
   {
      LinkDescription hipLinkDescription = new LinkDescription(hipLink);
      hipLinkDescription.setMassAndRadiiOfGyration(hipMass, hipRadiusOfGyrationX, hipRadiusOfGyrationY, hipRadiusOfGyrationZ);
      LinkGraphicsDescription hipGraphics = new LinkGraphicsDescription();
      hipGraphics.addCube(hipRadius * 2.0, hipLength, hipRadius * 2.0, appearance);
      hipLinkDescription.setLinkGraphics(hipGraphics);
      return hipLinkDescription;
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

}
