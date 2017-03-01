package us.ihmc.exampleSimulations.omniWrist;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class OmniWristDescription
{
   public boolean showCoordinateSystems = false;
   
   public final static String omniWristBaseModelFile = "models/omniWrist/OmniWristBase.stl";

   private final static String omniWristLink1ModelFile = "models/omniWrist/OmniWristLink1.stl";
   private final static String omniWristLink2ModelFile = "models/omniWrist/OmniWristLink2.stl";
   private final static String omniWristLink1SecondSetModelFile = "models/omniWrist/OmniWristLink1_SecondSet.stl";

   private double baseLength = 0.05;

   private double linkOneThickness = 0.01;
   private double linkOneLengthOne = baseLength + linkOneThickness / 2.0;
   private double linkOneLengthTwo = 0.03;
   private double linkOneLengthThree = 0.025;

   private double linkTwoLengthOne = 0.0179;
   private double linkTwoLengthTwo = 0.0433;
   private double linkTwoLengthThree = linkOneLengthThree;

   private double totalOmniWristMass = 1.0;
   private double payloadMass = 3.0;
   private double payloadLength = 0.60;

   private double linkOneMass = totalOmniWristMass/4.0 / 4.0;
   private double radiusOfGyrationScaleOne = 0.8;

   private double linkTwoMass = totalOmniWristMass/4.0 / 4.0;
   private double radiusOfGyrationScaleTwo = 0.8;

   private double linkThreeMass = totalOmniWristMass/4.0 / 4.0;
   private double radiusOfGyrationScaleThree = 0.8;

   private double linkFourMass = payloadMass + totalOmniWristMass/4.0;
   private double radiusOfGyrationScaleFour = 0.8;

   
   public RobotDescription createRobotDescription()
   {
      RobotDescription description = new RobotDescription("OmniWrist");

      RigidBodyTransform transformA = new RigidBodyTransform();
      transformA.setIdentity();
      PinJointDescription jointOneA = createALegOfOmniWrist(transformA, "A");
      description.addRootJoint(jointOneA);

      JointDescription jointFourAParent = jointOneA.getChildrenJoints().get(0).getChildrenJoints().get(0);

      RigidBodyTransform fortyFiveDegreeTransform = new RigidBodyTransform();
      fortyFiveDegreeTransform.setRotationEuler(0.0, Math.PI / 4.0, 0.0);

      Vector3D jointFourOffset = new Vector3D(-linkOneLengthOne, 0.0, 0.0);
      fortyFiveDegreeTransform.transform(jointFourOffset);

      Vector3D jointFourAxis = new Vector3D(0.0, 1.0, 0.0);
      fortyFiveDegreeTransform.transform(jointFourAxis);
      PinJointDescription jointFourA = new PinJointDescription("jointFourA", jointFourOffset, jointFourAxis);

      LinkDescription linkFour = new LinkDescription("linkFour");
      
      linkFour.setCenterOfMassOffset(new Vector3D(0.0, 0.0, payloadLength));
      linkFour.setMassAndRadiiOfGyration(linkFourMass, 0.1 * payloadLength/2.0 * radiusOfGyrationScaleFour, 0.1 * payloadLength/2.0 * radiusOfGyrationScaleFour, payloadLength/2.0 * radiusOfGyrationScaleFour);

      LinkGraphicsDescription linkFourGraphics = new LinkGraphicsDescription();
      linkFourGraphics.addModelFile(OmniWristDescription.omniWristBaseModelFile, YoAppearance.Black());
      if (showCoordinateSystems) linkFourGraphics.addCoordinateSystem(0.1);
      linkFourGraphics.addCylinder(payloadLength, payloadLength/100.0, YoAppearance.Red());
      linkFourGraphics.translate(0.0, 0.0, payloadLength);
      linkFourGraphics.addCylinder(payloadLength/30.0, payloadLength/5.0, YoAppearance.Gold());
      linkFour.setLinkGraphics(linkFourGraphics);

      jointFourA.setLink(linkFour);
      jointFourAParent.addJoint(jointFourA);

      double externalForceOffsetDistance = (baseLength + linkOneThickness)/ 2.0;
      ExternalForcePointDescription externalForcePointOnB1 = new ExternalForcePointDescription("ef_matchB1", new Vector3D(externalForceOffsetDistance, 0.0, 0.0));
      ExternalForcePointDescription externalForcePointOnC1 = new ExternalForcePointDescription("ef_matchC1", new Vector3D(0.0, externalForceOffsetDistance, 0.0));
      ExternalForcePointDescription externalForcePointOnD1 = new ExternalForcePointDescription("ef_matchD1", new Vector3D(-externalForceOffsetDistance, 0.0, 0.0));

      jointFourA.addExternalForcePoint(externalForcePointOnB1);
      jointFourA.addExternalForcePoint(externalForcePointOnC1);
      jointFourA.addExternalForcePoint(externalForcePointOnD1);
      
      ExternalForcePointDescription externalForcePointOnB2 = new ExternalForcePointDescription("ef_matchB2", new Vector3D(0.0, 0.0, 0.0));
      ExternalForcePointDescription externalForcePointOnC2 = new ExternalForcePointDescription("ef_matchC2", new Vector3D(0.0, 0.0, 0.0));
      ExternalForcePointDescription externalForcePointOnD2 = new ExternalForcePointDescription("ef_matchD2", new Vector3D(0.0, 0.0, 0.0));

      jointFourA.addExternalForcePoint(externalForcePointOnB2);
      jointFourA.addExternalForcePoint(externalForcePointOnC2);
      jointFourA.addExternalForcePoint(externalForcePointOnD2);

      RigidBodyTransform transformB = new RigidBodyTransform();
      transformB.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);
      PinJointDescription jointOneB = createALegOfOmniWrist(transformB, "B");
      description.addRootJoint(jointOneB);

      RigidBodyTransform transformC = new RigidBodyTransform();
      transformC.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI);
      PinJointDescription jointOneC = createALegOfOmniWrist(transformC, "C");
      description.addRootJoint(jointOneC);

      RigidBodyTransform transformD = new RigidBodyTransform();
      transformD.setRotationEulerAndZeroTranslation(0.0, 0.0, 3.0 * Math.PI / 2.0);
      PinJointDescription jointOneD = createALegOfOmniWrist(transformD, "D");
      description.addRootJoint(jointOneD);

      return description;
   }

   private PinJointDescription createALegOfOmniWrist(RigidBodyTransform transform, String jointSuffix)
   {
      String jointName = "jointOne" + jointSuffix;

      Vector3D jointOneOffset = new Vector3D(0.0, linkOneLengthTwo, 0.0);
      transform.transform(jointOneOffset);

      Vector3D axisOne = new Vector3D(0.0, 1.0, 0.0);
      transform.transform(axisOne);
      PinJointDescription jointOne = new PinJointDescription(jointName, jointOneOffset, axisOne);

      LinkDescription linkOne = new LinkDescription("linkOne");
      Vector3D centerOfMassOffsetOne = new Vector3D(linkOneLengthOne/2.0, -linkOneLengthTwo/2.0, 0.0);
      transform.transform(centerOfMassOffsetOne);
      linkOne.setCenterOfMassOffset(centerOfMassOffsetOne);
      Vector3D radiusOfGyrationVector = new Vector3D(radiusOfGyrationScaleOne * linkOneLengthOne/2.0, radiusOfGyrationScaleOne * linkOneLengthTwo/2.0, radiusOfGyrationScaleOne * linkOneLengthThree/2.0);
      transform.transform(radiusOfGyrationVector);
      linkOne.setMassAndRadiiOfGyration(linkOneMass, radiusOfGyrationVector.getX(), radiusOfGyrationVector.getY(), radiusOfGyrationVector.getZ());

      LinkGraphicsDescription linkOneGraphics = new LinkGraphicsDescription();
      linkOneGraphics.transform(transform);
      if (showCoordinateSystems) linkOneGraphics.addCoordinateSystem(0.05);
      linkOneGraphics.addModelFile(omniWristLink1ModelFile, YoAppearance.Red());

      linkOne.setLinkGraphics(linkOneGraphics);
      jointOne.setLink(linkOne);
      
      ForceSensorDescription forceSensorOne = new ForceSensorDescription("forceSensorOne" + jointSuffix, new RigidBodyTransform());
      forceSensorOne.setUseGroundContactPoints(false);
      jointOne.addForceSensor(forceSensorOne);

      jointName = "jointTwo" + jointSuffix;
      Vector3D jointTwoOffset = new Vector3D(linkOneLengthOne + linkOneThickness, -linkOneLengthTwo, 0.0);
      transform.transform(jointTwoOffset);
      Vector3D jointTwoAxis = new Vector3D(1.0, 0.0, 0.0);
      transform.transform(jointTwoAxis);
      PinJointDescription jointTwo = new PinJointDescription(jointName, jointTwoOffset, jointTwoAxis);

      LinkDescription linkTwo = new LinkDescription("linkTwo");
      Vector3D centerOfMassOffsetTwo = new Vector3D(linkTwoLengthOne/2.0, 0.0, linkTwoLengthTwo/2.0);
      transform.transform(centerOfMassOffsetTwo);
      linkTwo.setCenterOfMassOffset(centerOfMassOffsetTwo);
      radiusOfGyrationVector = new Vector3D(radiusOfGyrationScaleTwo * linkTwoLengthOne/2.0, radiusOfGyrationScaleTwo * linkTwoLengthThree/2.0, radiusOfGyrationScaleTwo * linkTwoLengthTwo/2.0);
      transform.transform(radiusOfGyrationVector);
      linkTwo.setMassAndRadiiOfGyration(linkTwoMass, radiusOfGyrationVector.getX(), radiusOfGyrationVector.getY(), radiusOfGyrationVector.getZ());
      
      LinkGraphicsDescription linkTwoGraphics = new LinkGraphicsDescription();
      linkTwoGraphics.transform(transform);
      if (showCoordinateSystems) linkTwoGraphics.addCoordinateSystem(0.06);
      linkTwoGraphics.addModelFile(omniWristLink2ModelFile, YoAppearance.Blue());
      linkTwo.setLinkGraphics(linkTwoGraphics);
      jointTwo.setLink(linkTwo);

      ForceSensorDescription forceSensorTwo = new ForceSensorDescription("forceSensorTwo" + jointSuffix, new RigidBodyTransform());
      forceSensorTwo.setUseGroundContactPoints(false);
      jointTwo.addForceSensor(forceSensorTwo);
      
      jointOne.addJoint(jointTwo);

      RigidBodyTransform fortyFiveDegreeTransform = new RigidBodyTransform();
      fortyFiveDegreeTransform.setRotationEuler(0.0, Math.PI / 4.0, 0.0);

      jointName = "jointThree" + jointSuffix;
      Vector3D linkTwoBearingToBearingVector = new Vector3D(linkTwoLengthOne, 0.0, linkTwoLengthTwo);
      Vector3D thicknessVector = new Vector3D(-linkOneThickness, 0.0, 0.0);
      fortyFiveDegreeTransform.transform(thicknessVector);

      Vector3D jointThreeOffsetVector = new Vector3D(linkTwoBearingToBearingVector);
      jointThreeOffsetVector.add(thicknessVector);
      transform.transform(jointThreeOffsetVector);

      Vector3D jointThreeAxis = new Vector3D(1.0, 0.0, 0.0);
      fortyFiveDegreeTransform.transform(jointThreeAxis);
      transform.transform(jointThreeAxis);

      PinJointDescription jointThree = new PinJointDescription(jointName, jointThreeOffsetVector, jointThreeAxis);

      LinkDescription linkThree = new LinkDescription("linkThree");
      Vector3D centerOfMassOffsetThree = new Vector3D(-linkOneLengthOne/2.0, -linkOneLengthTwo/2.0, 0.0);
      fortyFiveDegreeTransform.transform(centerOfMassOffsetThree);
      transform.transform(centerOfMassOffsetThree);
      linkThree.setCenterOfMassOffset(centerOfMassOffsetThree);
      linkThree.setMassAndRadiiOfGyration(linkThreeMass, radiusOfGyrationScaleThree * linkOneLengthOne/2.0, radiusOfGyrationScaleThree * linkOneLengthTwo/2.0, radiusOfGyrationScaleThree * linkOneLengthThree/2.0);
      Matrix3D momentOfInertia = linkThree.getMomentOfInertiaCopy();
      fortyFiveDegreeTransform.transform(momentOfInertia);
      transform.transform(momentOfInertia);
      linkThree.setMomentOfInertia(momentOfInertia);

      Vector3D externalForcePointForWeldingOffset1 = new Vector3D(-linkOneLengthOne, -linkOneLengthTwo, 0.0);
      fortyFiveDegreeTransform.transform(externalForcePointForWeldingOffset1);
      transform.transform(externalForcePointForWeldingOffset1);
      
      Vector3D externalForcePointForWeldingOffset2 = new Vector3D(-linkOneLengthOne, 0.0, 0.0);
      fortyFiveDegreeTransform.transform(externalForcePointForWeldingOffset2);
      transform.transform(externalForcePointForWeldingOffset2);

      LinkGraphicsDescription linkThreeGraphics = new LinkGraphicsDescription();
      linkThreeGraphics.transform(transform);
      linkThreeGraphics.transform(fortyFiveDegreeTransform);
      if (showCoordinateSystems) linkThreeGraphics.addCoordinateSystem(0.07);
      linkThreeGraphics.addModelFile(omniWristLink1SecondSetModelFile, YoAppearance.Green());
      linkThreeGraphics.identity();
      linkThreeGraphics.translate(externalForcePointForWeldingOffset1);
      if (showCoordinateSystems) linkThreeGraphics.addCoordinateSystem(0.06);
      linkThree.setLinkGraphics(linkThreeGraphics);

      jointThree.setLink(linkThree);

      ExternalForcePointDescription externalForcePointForWelding1 = new ExternalForcePointDescription("ef_" + jointSuffix + "1", externalForcePointForWeldingOffset1);
      jointThree.addExternalForcePoint(externalForcePointForWelding1);
      ExternalForcePointDescription externalForcePointForWelding2 = new ExternalForcePointDescription("ef_" + jointSuffix + "2", externalForcePointForWeldingOffset2);
      jointThree.addExternalForcePoint(externalForcePointForWelding2);

      ForceSensorDescription forceSensorThree = new ForceSensorDescription("forceSensorThree" + jointSuffix, new RigidBodyTransform());
      forceSensorThree.setUseGroundContactPoints(false);
      jointThree.addForceSensor(forceSensorThree);

      jointTwo.addJoint(jointThree);
      return jointOne;
   }
}
