package us.ihmc.exampleSimulations.simple3DWalker.FullRobotModel;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

import java.util.*;

public class SimpleWalkerRobotModel implements FullRobotModel
{
   private static final double toFootCenterX = 0.05042;
   private static final double toFootCenterY = 0.082;

   private static final double footBack = 0.085;
   private static final double footLength = 0.22;
   private static final double ankleHeight = 0.0875;

   private static final double POUNDS = 1.0 / 2.2;    // Pound to Kg conversion.
   private static final double INCHES = 0.0254;    // Inch to Meter Conversion.

   private static final double PELVIS_HEIGHT = 1.0;
   private static final double PELVIS_RAD = 0.1;

   private static final double HIP_WIDTH = 0.2;

   private static final double HIP_DIFFERENTIAL_HEIGHT = 0.05;
   private static final double HIP_DIFFERENTIAL_WIDTH = 0.075;

   private static final double THIGH_LENGTH = 23.29 * INCHES;
   private static final double THIGH_RAD = 0.05;
   private static final double THIGH_MASS = 6.7 * POUNDS;

   private static final double SHIN_LENGTH = 23.29 * INCHES;
   private static final double SHIN_RAD = 0.03;
   private static final double SHIN_MASS = 6.7 * POUNDS;

   private static final double ANKLE_DIFFERENTIAL_HEIGHT = 0.025;
   private static final double ANKLE_DIFFERENTIAL_WIDTH = 0.0375;

   private static final double FOOT_LENGTH = 0.08;
   private static final double FOOT_COM_OFFSET = 3.0 * INCHES;
   private static final double FOOT_RAD = 0.05;
   private static final double FOOT_MASS = 3.0 * POUNDS;
   private static final double GRAVITY = -9.81;
   private double bodyMass = 10.0, lowerLinkMass = 0.1, upperLinkMass = 0.2;
   public final double lowerLinkLength = 0.8, upperLinkLength = 0.7;
   private double lowerLinkRadius = 0.04, upperLinkRadius = 0.05;
   private double legHeight = lowerLinkLength + upperLinkLength;
   private double gcOffset = -lowerLinkLength;
   private double bodyLength = 0.3, bodyWidth = 0.3, bodyHeight = 0.2;
   private double hipOffsetY = bodyWidth / 2.0;
   private double maxLegExtension = lowerLinkLength;
   private double footZMin = -0.04;
   private double footZmax = -0.01;
   private double footY = 0.25;
   private double footX = 0.25;
   private double footOffsetPercent = 0;
   private double footForward = (footX * footOffsetPercent);
   private final Vector3D bodyOffset = new Vector3D(0.0, 0.0, 0.0);


   public final double nominalHeight = upperLinkLength + lowerLinkLength / 2.0;

   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final MovingReferenceFrame elevatorFrame;
   private final ReferenceFrame centerOfMassFrame;

   private final RigidBody elevator;

   private static final double mass = 1.0;
   private static final double Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;

   private final RigidBody body;

   private final SixDoFJoint floatingJoint;
   private final OneDoFJoint[] oneDoFJoints;

   private final CenterOfMassJacobian centerOfMassJacobian;


   Vector3D bodyAcceleration = new Vector3D();

   private final double totalMass;

   //private final SixDoFJoint rootJoint;
   // private final RigidBody floatingBody;

   private final SideDependentList<RevoluteJoint> hipPitchJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> hipRollJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> hipYawJoints = new SideDependentList<>();
   private final SideDependentList<RigidBody> upperLegs = new SideDependentList<>();

   private final SideDependentList<PrismaticJoint> kneeJoints = new SideDependentList<>();
   private final SideDependentList<RigidBody> lowerLegs = new SideDependentList<>();

   private final SideDependentList<RevoluteJoint> anklePitchJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> ankleRollJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> ankleYawJoints = new SideDependentList<>();
   private final SideDependentList<RigidBody> feet = new SideDependentList<>();
   private final SideDependentList<Joint> scsAnkles = new SideDependentList<>();
   private SideDependentList<List<GroundContactPoint>> gcPoints = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

   private final SideDependentList<Link> upperLegLinks = new SideDependentList<>();
   private final SideDependentList<Link> lowerLegLinks = new SideDependentList<>();
   private final SideDependentList<Link> footLinks = new SideDependentList<>();

   private final SideDependentList<Graphics3DObject> upperLegGraphics = new SideDependentList<>();
   private final SideDependentList<Graphics3DObject> lowerLegGraphics = new SideDependentList<>();
   private final SideDependentList<Graphics3DObject> footGraphics = new SideDependentList<>();

   private static final Random random = new Random(100L);

   private final SCSRobotFromInverseDynamicsRobotModel scsRobot;

   public SimpleWalkerRobotModel()
   {
      elevator = new RigidBody("elevator", worldFrame);
      elevatorFrame = elevator.getBodyFixedFrame();
      Matrix3D inertia = new Matrix3D(Ixx1, 0.0, 0.0, 0.0, Iyy1, 0.0, 0.0, 0.0, Izz1);
      floatingJoint = new SixDoFJoint("floatingJoint", elevator);
      body = ScrewTools.addRigidBody("body", floatingJoint, inertia, mass, new Vector3D());
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);
      centerOfMassJacobian = new CenterOfMassJacobian(elevator);
      oneDoFJoints = ScrewTools.createOneDoFJointPath(elevator, body);
      totalMass = TotalMassCalculator.computeSubTreeMass(body);

      floatingJoint.getLinearAcceleration(bodyAcceleration);


      for (RobotSide robotSide : RobotSide.values())
      {
         RevoluteJoint hipPitchJoint = ScrewTools.addRevoluteJoint(robotSide.getSideNameFirstLetter() + "HipPitch", body, new Vector3D(), Y);
         hipPitchJoints.put(robotSide, hipPitchJoint);
         RigidBody hipPitchDiff = createDifferential(hipPitchJoint);
         RevoluteJoint hipRollJoint = ScrewTools.addRevoluteJoint(robotSide.getSideNameFirstLetter() + "HipRoll", hipPitchDiff, new Vector3D(), X);
         hipRollJoints.put(robotSide, hipRollJoint);
         RigidBody upperLeg = createUpperLeg(hipRollJoint, robotSide);
         upperLegs.put(robotSide, upperLeg);
         PrismaticJoint kneeJoint = ScrewTools.addPrismaticJoint(robotSide.getSideNameFirstLetter() + "Knee", upperLeg, new Vector3D(), Z);
         kneeJoints.put(robotSide, kneeJoint);
         RigidBody lowerLeg = createLowerLeg(kneeJoint, robotSide);
         lowerLegs.put(robotSide, lowerLeg);
         RevoluteJoint anklePitchJoint = ScrewTools.addRevoluteJoint(robotSide.getSideNameFirstLetter() + "AnklePitch", lowerLeg, new Vector3D(), Y);
         anklePitchJoints.put(robotSide, anklePitchJoint);
         RigidBody anklePitchDiff = createDifferential(anklePitchJoint);
         RevoluteJoint ankleRollJoint = ScrewTools.addRevoluteJoint(robotSide.getSideNameFirstLetter() + "AnkleYaw", anklePitchDiff, new Vector3D(), X);
         ankleRollJoints.put(robotSide, ankleRollJoint);
         RigidBody foot = createFoot(ankleRollJoint, robotSide);
         feet.put(robotSide, foot);
      }


      scsRobot = new SCSRobotFromInverseDynamicsRobotModel("robot", elevator.getChildrenJoints().get(0));
      scsRobot.setGravity(GRAVITY);
      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.update();


      for (RobotSide robotSide : RobotSide.values())
      {
         addLinkGraphics(robotSide);
         // SCSRobotFromInverseDynamicsRobotModel scsRobotAnkle = new SCSRobotFromInverseDynamicsRobotModel(robotSide.getSideNameFirstLetter() + "scsFoot", feet.get(robotSide).getParentJoint());
         // Joint scsAnkle = scsRobotAnkle.getRootJoints().get(0);

         Joint scsAnkle = footLinks.get(robotSide).getParentJoint();

         GroundContactPoint gcHeelL = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcHeelL", new Vector3D(0.5 * footX, -0.5 * footY, footZMin), scsRobot);
         GroundContactPoint gcHeelR = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcHeelR", new Vector3D(-0.5 * footX, -0.5 * footY, footZMin), scsRobot);
         GroundContactPoint gcToeL = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcToeL", new Vector3D(-0.5 * footX, 0.5 * footY, footZMin), scsRobot);
         GroundContactPoint gcToeR = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcToeR", new Vector3D(0.5 * footX, 0.5 * footY, footZMin), scsRobot);

         scsAnkle.addGroundContactPoint(gcHeelL);
         scsAnkle.addGroundContactPoint(gcHeelR);
         scsAnkle.addGroundContactPoint(gcToeL);
         scsAnkle.addGroundContactPoint(gcToeR);

         gcPoints.get(robotSide).add(gcHeelL);
         gcPoints.get(robotSide).add(gcHeelR);
         gcPoints.get(robotSide).add(gcToeL);
         gcPoints.get(robotSide).add(gcToeR);

         scsAnkles.put(robotSide, scsAnkle);
      }

      double groundKxy = 1e4;
      double groundBxy = 1e2;
      double groundKz = 125.0;
      double groundBz = 300.0;
      LinearGroundContactModel ground = new LinearGroundContactModel(scsRobot, groundKxy, groundBxy, groundKz, groundBz, scsRobot.getRobotsYoVariableRegistry());



      scsRobot.setGroundContactModel(ground);



   }

   private void addLinkGraphics(RobotSide robotSide)
   {

      AppearanceDefinition upperArmAppearance;
      AppearanceDefinition lowerArmAppearance;
      AppearanceDefinition handAppearance;

      if (robotSide == RobotSide.RIGHT)
      {
         upperArmAppearance = YoAppearance.Red();
         lowerArmAppearance = YoAppearance.Pink();
         handAppearance = YoAppearance.Orange();
      }

      else
      {
         upperArmAppearance = YoAppearance.Blue();
         lowerArmAppearance = YoAppearance.LightBlue();
         handAppearance = YoAppearance.Green();
      }

      Link upperArmLink = scsRobot.getLink(robotSide.getSideNameFirstLetter() + "upperLeg");
      upperLegLinks.put(robotSide, upperArmLink);
      Link lowerArmLink = scsRobot.getLink(robotSide.getSideNameFirstLetter() + "lowerLeg");
      lowerLegLinks.put(robotSide, lowerArmLink);
      Link handLink = scsRobot.getLink(robotSide.getSideNameFirstLetter() + "foot");
      footLinks.put(robotSide, handLink);

      Graphics3DObject upperArmGraphics = new Graphics3DObject();
      upperLegGraphics.put(robotSide, upperArmGraphics);
      Graphics3DObject lowerArmGraphics = new Graphics3DObject();
      lowerLegGraphics.put(robotSide, lowerArmGraphics);
      Graphics3DObject handGraphics = new Graphics3DObject();
      footGraphics.put(robotSide, handGraphics);

      upperLegGraphics.get(robotSide).addCylinder(THIGH_LENGTH, THIGH_RAD, upperArmAppearance);
      lowerLegGraphics.get(robotSide).addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
      footGraphics.get(robotSide).addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);

      upperLegLinks.get(robotSide).setLinkGraphics(upperArmGraphics);
      lowerLegLinks.get(robotSide).setLinkGraphics(lowerArmGraphics);
      footLinks.get(robotSide).setLinkGraphics(handGraphics);


   }

   private RigidBody createDifferential(OneDoFJoint joint)
   {
      Vector3D comOffset = new Vector3D();
      return ScrewTools.addRigidBody(joint.getName() + "Differential", joint, 0.005, 0.005, 0.005, 0.1, comOffset);
   }

   private RigidBody createUpperLeg(OneDoFJoint parentJoint, RobotSide robotSide)
   {
      Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
      return ScrewTools.addRigidBody(robotSide.getSideNameFirstLetter() + "upperLeg", parentJoint, 0.0437, 0.0437, 0.0054, THIGH_MASS, comOffset);
   }

   private RigidBody createLowerLeg(OneDoFJoint parentJoint, RobotSide robotSide)
   {
      Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);
      return ScrewTools.addRigidBody(robotSide.getSideNameFirstLetter() + "lowerLeg", parentJoint, 0.0437, 0.0437, 0.0054, SHIN_MASS, comOffset);
   }

   private RigidBody createFoot(OneDoFJoint parentJoint, RobotSide robotSide)
   {
      Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);
      return ScrewTools.addRigidBody(robotSide.getSideNameFirstLetter() + "foot", parentJoint, 0.0437, 0.0437, 0.0054, FOOT_MASS, comOffset);
   }

   private void addGroundContactPoints(RobotSide robotSide)
   {
      // externalForcePoint = new GroundContactPoint("handForcePoint", scsRobotArm.getLink("hand").getComOffset(), scsRobotArm);
      // scsRobotArm.getLink("hand").getParentJoint().addExternalForcePoint(externalForcePoint);
   }

   public SCSRobotFromInverseDynamicsRobotModel getSCSRobot()
   {
      return scsRobot;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }



   @Override
   public RobotSpecificJointNames getRobotSpecificJointNames()
   {
      return null;
   }

   @Override
   public void updateFrames()
   {
      worldFrame.update();
      elevator.updateFramesRecursively();
   }

   @Override
   public MovingReferenceFrame getElevatorFrame()
   {
      return elevatorFrame;
   }

   @Override
   public FloatingInverseDynamicsJoint getRootJoint()
   {
      return null;
   }

   @Override
   public RigidBody getElevator()
   {
      return elevator;
   }

   @Override
   public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
   {
      return null;
   }

   @Override
   public RigidBody getEndEffector(Enum<?> segmentEnum)
   {
      return null;
   }

   @Override
   public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
   {
      return null;
   }

   @Override
   public InverseDynamicsJoint getLidarJoint(String lidarName)
   {
      return null;
   }

   @Override
   public ReferenceFrame getLidarBaseFrame(String name)
   {
      return null;
   }

   @Override
   public RigidBodyTransform getLidarBaseToSensorTransform(String name)
   {
      return null;
   }

   @Override
   public ReferenceFrame getCameraFrame(String name)
   {
      return null;
   }

   @Override
   public RigidBody getRootBody()
   {
      return getElevator();
   }

   @Override
   public RigidBody getHead()
   {
      return null;
   }

   @Override
   public ReferenceFrame getHeadBaseFrame()
   {
      return null;
   }

   @Override
   public OneDoFJoint[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   @Override
   public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
   {
      return null;
   }

   @Override
   public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, List<OneDoFJoint> oneDoFJointsToPack)
   {

   }

   @Override
   public OneDoFJoint[] getControllableOneDoFJoints()
   {
      return new OneDoFJoint[0];
   }

   @Override
   public void getOneDoFJoints(List<OneDoFJoint> oneDoFJointsToPack)
   {
      List<OneDoFJoint> list = Arrays.asList(oneDoFJoints);

      for (int i = 0; i < list.size(); i++)
         oneDoFJointsToPack.set(i, list.get(i));
   }

   @Override
   public OneDoFJoint getOneDoFJointByName(String name)
   {
      return null;
   }

   @Override
   public void getControllableOneDoFJoints(List<OneDoFJoint> oneDoFJointsToPack)
   {

   }

   @Override
   public IMUDefinition[] getIMUDefinitions()
   {
      return new IMUDefinition[0];
   }

   @Override
   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return new ForceSensorDefinition[0];
   }

   @Override
   public ContactSensorDefinition[] getContactSensorDefinitions()
   {
      return new ContactSensorDefinition[0];
   }

   @Override
   public double getTotalMass()
   {
      return Double.NaN;
   }

   public double getKneePosition(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQd();
   }

   public double getHipPitchPosition(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getQ();
   }

   public double getHipPitchVelocity(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getQd();
   }

   public double getHipRollPosition(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getQ();
   }

   public double getHipRollVelocity(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getQd();
   }

   public double getHipYawPosition(RobotSide robotSide)
   {
      return hipYawJoints.get(robotSide).getQ();
   }

   public double getHipYawVelocity(RobotSide robotSide)
   {
      return hipYawJoints.get(robotSide).getQd();
   }

   public double getAnklePitchPosition(RobotSide robotSide) { return anklePitchJoints.get(robotSide).getQ();}

   public double getAnklePitchVelocity(RobotSide robotSide) { return anklePitchJoints.get(robotSide).getQd();}

   public double getAnkleRollPosition(RobotSide robotSide) { return ankleRollJoints.get(robotSide).getQ();}

   public double getAnkleRollVelocity(RobotSide robotSide) { return ankleRollJoints.get(robotSide).getQd();}

   public double getAnkleYawPosition(RobotSide robotSide) { return ankleYawJoints.get(robotSide).getQ();}

   public double getAnkleYawVelocity(RobotSide robotSide) { return ankleYawJoints.get(robotSide).getQd();}

   public void setKneeTorque(RobotSide robotSide, double torque)
   {
      kneeJoints.get(robotSide).setTau(torque);
   }

   public double getKneeTorque(RobotSide robotside)
   {
      return kneeJoints.get(robotside).getTau();
   }

   public void setHipPitchTorque(RobotSide robotSide, double torque)
   {
      hipPitchJoints.get(robotSide).setTau(torque);
   }

   public double getHipPitchTorque(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getTau();
   }

   public void setHipRollTorque(RobotSide robotSide, double torque)
   {
      hipRollJoints.get(robotSide).setTau(torque);
   }

   public double getHipRollTorque(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getTau();
   }

   public void setHipYawTorque(RobotSide robotSide, double torque)
   {
      hipYawJoints.get(robotSide).setTau(torque);
   }

   public double getHipYawTorque(RobotSide robotSide)
   {
      return hipYawJoints.get(robotSide).getTau();
   }

   public void setAnklePitchTorque(RobotSide robotSide, double torque) { anklePitchJoints.get(robotSide).setTau(torque); }

   public double getAnklePitchTorque(RobotSide robotSide) { return anklePitchJoints.get(robotSide).getTau();}

   public void setAnkleRollTorque(RobotSide robotSide, double torque) { ankleRollJoints.get(robotSide).setTau(torque); }

   public double getAnkleRollTorque(RobotSide robotSide) { return ankleRollJoints.get(robotSide).getTau();}

   public void setAnkleYawTorque(RobotSide robotSide, double torque) { ankleYawJoints.get(robotSide).setTau(torque); }

   public double getAnkleYawTorque(RobotSide robotSide) { return ankleYawJoints.get(robotSide).getTau();}

   public double getFootSizeX() { return footX;}

   public double getFootSizeY() { return footY;}

   public boolean isFootOnGround(RobotSide robotSide)
   {
      for (int i = 0; i < gcPoints.get(robotSide).size(); i++)
      {
         if (gcPoints.get(robotSide).get(i).isInContact())
            return true;
      }
      return false;
   }

   public SideDependentList<List<GroundContactPoint>> getgCpoints()
   {
      return gcPoints;
   }

   public double getBodyYaw()
   {
      return floatingJoint.getRotationForReading().getYaw();
   }

   public double getBodyPitch()
   {
      return floatingJoint.getRotationForReading().getPitch();
   }

   public double getBodyRoll()
   {
      return floatingJoint.getRotationForReading().getRoll();
   }

   public double getBodyYawVelocity()
   {
      return floatingJoint.getAngularVelocityForReading().getZ();
   }

   public double getBodyPitchVelocity()
   {
      return floatingJoint.getAngularVelocityForReading().getY();
   }

   public double getBodyRollVelocity()
   {
      return floatingJoint.getAngularVelocityForReading().getX();
   }

   public double getBodyHeight()
   {
      return floatingJoint.getTranslationForReading().getZ();
   }

   public double getZ0()
   {
      double z0 = 1.12;
      return z0;
   }
   public double getGravity() { return GRAVITY;}

   public double getBodyPositionX(){ return  floatingJoint.getTranslationForReading().getX();}

   public double getBodyPositionY(){ return floatingJoint.getTranslationForReading().getY();}

   public double getBodyHeightVelocity()
   {
      return floatingJoint.getLinearVelocityForReading().getZ();
   }

   public double getBodyVelocityX()
   {
      return floatingJoint.getLinearVelocityForReading().getX();
   }

   public double getBodyVelocityY()
   {
      return floatingJoint.getLinearVelocityForReading().getY();
   }



   public double getBodyAccelerationX() { return bodyAcceleration.getX() ;}

   public double getBodyAccelerationY() { return bodyAcceleration.getY();}

   public double getBodyMass() { return bodyMass;}


}
