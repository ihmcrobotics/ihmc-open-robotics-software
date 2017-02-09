package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;
import java.util.LinkedHashMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters2.JointNames;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters2.LinkNames;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class Step7IDandSCSRobot_pinKnee extends Robot
{

   /**
    * Variables
    */

   // Registry
   private final YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
   private final ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

   // ID
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
   private FramePoint bodyPosition = new FramePoint();
   private FramePoint footPosition = new FramePoint();

   private final Vector3d jointAxesPinJoints = new Vector3d(0.0, 1.0, 0.0); // rotate around Y-axis (for revolute joints)
   private final RigidBody elevator;

   private final SideDependentList<EnumMap<JointNames, OneDoFJoint>> allLegJoints = SideDependentList.createListOfEnumMaps(JointNames.class); // Includes the ankle!
   private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> idToSCSLegJointMap = new LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>(); // makes the SCS joints available without another side dependent enum
   private final SideDependentList<EnumMap<LinkNames, RigidBody>> allLegRigidBodies = SideDependentList.createListOfEnumMaps(LinkNames.class);
   private final SixDoFJoint bodyJointID;

   // SCS
   private final FloatingPlanarJoint bodyJointSCS;
   SideDependentList<GroundContactPoint> GCpointsHeel = new SideDependentList<GroundContactPoint>();
   SideDependentList<GroundContactPoint> GCpointsToe = new SideDependentList<GroundContactPoint>();

   private double cubeX = RobotParameters2.BODY_DIMENSIONS.get(Axis.X);
   private double cubeY = RobotParameters2.BODY_DIMENSIONS.get(Axis.Y);
   private double cubeZ = RobotParameters2.BODY_DIMENSIONS.get(Axis.Z);

   private double footX = RobotParameters2.FOOT_DIMENSIONS.get(Axis.X);
   private double footY = RobotParameters2.FOOT_DIMENSIONS.get(Axis.Y);
   private double footZ = RobotParameters2.FOOT_DIMENSIONS.get(Axis.Z);

   // General
   private double bodyMass = RobotParameters2.MASSES.get(LinkNames.BODY_LINK);
   private double footMass = RobotParameters2.MASSES.get(LinkNames.FOOT_LINK);

   private double hipOffsetY = RobotParameters2.BODY_DIMENSIONS.get(Axis.X) / 2.0;
   private double kneeOffsetZ = -RobotParameters2.LENGTHS.get(LinkNames.UPPER_LINK) + 0.03;
   private double ankleOffsetZ = -RobotParameters2.LENGTHS.get(LinkNames.LOWER_LINK);
   private double footOffsetX = 0.03;
   private double gcOffset = -footZ;
   private double gcRadius = 0.015;

   Vector3d comOffsetBody = new Vector3d(0.0, 0.0, cubeZ / 2.0);
   Vector3d comOffsetFoot = new Vector3d(footX / 2.0 - 0.075, 0.0, -footZ / 2.0);

   private double bodyZ, bodyX; //global so that it is created only once (avoid generating garbage)
   private double initialBodyHeight = RobotParameters2.LENGTHS.get(LinkNames.UPPER_LINK) + RobotParameters2.LENGTHS.get(LinkNames.LOWER_LINK) + footZ - 0.045;  

   // Frames
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ZUpFrame> soleZUpFrames = new SideDependentList<>();
   private final SideDependentList<YoGraphicReferenceFrame> soleFramesViz = new SideDependentList<>();
   private final MidFrameZUpFrame midSoleZUpFrame;
   private YoGraphicReferenceFrame bodyFrameViz;

   // Jacobians
   private final SideDependentList<GeometricJacobian> legJacobians = new SideDependentList<>();
   private RigidBody bodyRigidBody, footRigidBody;

   // CoM, CoP and ICP calculations and plots
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final TwistCalculator twistCalculator;

   private double gravity;
   private final double totalRobotMass;
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", yoVariableRegistry);

   private final YoFramePoint yoCoM = new YoFramePoint("centerOfMass", worldFrame, yoVariableRegistry);
   private final YoFrameVector yoCoMVelocity = new YoFrameVector("centerOfMassVelocity", worldFrame, yoVariableRegistry);
   private final YoFramePoint2d yoICP = new YoFramePoint2d("capturePoint", worldFrame, yoVariableRegistry);
   private final YoFramePoint yoCoP = new YoFramePoint("centerOfPressure", worldFrame, yoVariableRegistry);
   private final SideDependentList<YoFramePoint> yoFootPositions = new SideDependentList<>();
   private final YoFrameVector measuredGroundReactionForce = new YoFrameVector("measuredGroundReactionForce", worldFrame, yoVariableRegistry);

   private final Twist bodyJointTwist = new Twist();
   private final double qd_x, qd_z, qd_wy;
   private final FrameVector bodyJointLinearVelocity = new FrameVector();
   private final FrameVector bodyJointAngularVelocity = new FrameVector();

   private final FramePoint centerOfMass = new FramePoint();
   private final FrameVector centerOfMassVelocity = new FrameVector();
   private final FramePoint2d centerOfMass2d = new FramePoint2d();
   private final FrameVector2d centerOfMassVelocity2d = new FrameVector2d();
   private final FramePoint2d capturePoint = new FramePoint2d();
   private double totalFz;
   


   /**
    * Constructor - ID robot + SCS robot + visualizers
    */

   public Step7IDandSCSRobot_pinKnee()
   {
      super("Robot");

      /****************** ID ROBOT ***********************/
      elevator = new RigidBody("elevator", elevatorFrame);

      bodyJointID = new SixDoFJoint(JointNames.BODY.getName(), elevator, elevatorFrame);
      bodyRigidBody = createBodyRB(LinkNames.BODY_LINK, bodyJointID);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         // HIP ID (location, joint, and rigidBody) 
         Vector3d hipOffset = new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0);
         RevoluteJoint hipJointID = ScrewTools.addRevoluteJoint(JointNames.HIP.getName(), bodyJointID.getSuccessor(), hipOffset, jointAxesPinJoints); //The parent rigid body of the hip joint is: bodyJointID.getSuccessor()
         allLegJoints.get(robotSide).put(JointNames.HIP, hipJointID);
         createAndAttachCylinderRB(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         // KNEE ID
         Vector3d kneeOffset = new Vector3d(0.0, 0.0, kneeOffsetZ);
         RevoluteJoint kneeJointID = ScrewTools.addRevoluteJoint(JointNames.KNEE.getName(), hipJointID.getSuccessor(), kneeOffset, jointAxesPinJoints);
         allLegJoints.get(robotSide).put(JointNames.KNEE, kneeJointID);
         createAndAttachCylinderRB(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);

         // ANKLE ID (location, joint, and rigidBody) 
         Vector3d ankleOffset = new Vector3d(0.0, 0.0, ankleOffsetZ);
         RevoluteJoint ankleJointID = ScrewTools.addRevoluteJoint(JointNames.ANKLE.getName(), kneeJointID.getSuccessor(), ankleOffset, jointAxesPinJoints);
         allLegJoints.get(robotSide).put(JointNames.ANKLE, ankleJointID);
         footRigidBody = createFootRB(LinkNames.FOOT_LINK, JointNames.ANKLE, robotSide);

         // Jacobian
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.setTranslation(0.0, 0.0, -footZ);
		 ReferenceFrame soleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(sidePrefix + "SoleFrame", ankleJointID.getFrameAfterJoint(), transformToParent);
         soleFrames.put(robotSide, soleFrame);

         ZUpFrame soleZUpFrame = new ZUpFrame(worldFrame, soleFrame, sidePrefix + "SoleZUpFrame");
         soleZUpFrames.put(robotSide, soleZUpFrame);

         GeometricJacobian jacobian = new GeometricJacobian(bodyRigidBody, allLegRigidBodies.get(robotSide).get(LinkNames.LOWER_LINK), soleFrame); 
         legJacobians.put(robotSide, jacobian);
      }

      /****************** SCS ROBOT ***********************/
      // BODY SCS
      bodyJointSCS = new FloatingPlanarJoint(JointNames.BODY.getName(), this);
      this.addRootJoint(bodyJointSCS);
      createAndAttachBodyLink(LinkNames.BODY_LINK);
      bodyJointSCS.setCartesianPosition(0.0, initialBodyHeight);

      for (RobotSide robotSide : RobotSide.values)
      {
         // HIP SCS
         PinJoint hipJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.HIP.getName(), new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0), this, jointAxesPinJoints);
         hipJointSCS.setLimitStops(-0.75, 0.75, 1e3, 1e1);

         if (robotSide == RobotSide.LEFT)
         {
            hipJointSCS.setQ(-0.2); 
         }

         else
         {
            hipJointSCS.setQ(0.37); 
         }

         bodyJointSCS.addJoint(hipJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.HIP), hipJointSCS);
         createAndAttachCylinderLink(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         // KNEE SCS
         PinJoint kneeJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.KNEE.getName(), new Vector3d(0.0, 0.0, kneeOffsetZ), this, jointAxesPinJoints);
         kneeJointSCS.setLimitStops(-0.01, 1.8, 1e5, 1e3);
         hipJointSCS.addJoint(kneeJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.KNEE), kneeJointSCS);
         createAndAttachCylinderLink(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);

         // ANKLE SCS
         PinJoint ankleJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.ANKLE.getName(), new Vector3d(0.0, 0.0, ankleOffsetZ), this, jointAxesPinJoints);
         ankleJointSCS.setLimitStops(-0.7, 0.7, 1e3, 1e2);
         
         if (robotSide == RobotSide.LEFT)
         {
            ankleJointSCS.setQ(0.21);
         }

         kneeJointSCS.addJoint(ankleJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.ANKLE), ankleJointSCS);

         // FEET SCS
         createAndAttachFootLink(LinkNames.FOOT_LINK, JointNames.ANKLE, robotSide);
         GroundContactPoint gcHeel = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcHeel", new Vector3d(-0.04, 0.0, gcOffset), this);
         GCpointsHeel.set(robotSide, gcHeel);
         ankleJointSCS.addGroundContactPoint(gcHeel);
         Graphics3DObject graphicsGCheel = ankleJointSCS.getLink().getLinkGraphics();
         graphicsGCheel.identity();
         graphicsGCheel.translate(-0.04, 0.0, gcOffset);
         graphicsGCheel.addSphere(gcRadius, YoAppearance.Orange());

         if (robotSide == RobotSide.RIGHT)
         {
            setFStoTrue(robotSide);
         }

         GroundContactPoint gcToe = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcToe", new Vector3d(0.1, 0.0, gcOffset), this);
         GCpointsToe.set(robotSide, gcToe);
         ankleJointSCS.addGroundContactPoint(gcToe);
         Graphics3DObject graphics = ankleJointSCS.getLink().getLinkGraphics();
         graphics.identity();
         graphics.translate(0.1, 0.0, gcOffset);
         graphics.addSphere(gcRadius, YoAppearance.Orange());
      }

      /**************** SCS Ground Model *************************/
      GroundContactModel groundModel = new LinearGroundContactModel(this, 1e3, 1e3, 5e3, 1e3, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);

      // Some useful parameters 
      qd_x = bodyJointSCS.getQd_t1().getDoubleValue();
      qd_z = bodyJointSCS.getQd_t2().getDoubleValue();
      qd_wy = bodyJointSCS.getQd_rot().getDoubleValue();

      
      /****************** Visualizers ***********************/
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         YoFramePoint yoFootPosition = new YoFramePoint(sidePrefix + "FootPosition", worldFrame, yoVariableRegistry);
         yoFootPositions.put(robotSide, yoFootPosition);
              
         // Visualize feet location on plotter panel
         YoArtifactPosition footPositionArtifact = new YoArtifactPosition(sidePrefix + " Foot", yoFootPosition.getYoX(), yoFootPosition.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Green().getColor().get(), 0.009);
         artifactList.add(footPositionArtifact);
         
         // Visualize feel location on SCS
//         YoGraphicPosition footPositionGraphic = new YoGraphicPosition(sidePrefix + "Foot", yoFootPosition, 0.025, YoAppearance.Yellow());
//         yoGraphicsList.add(footPositionGraphic); 

         // Visualize sole frames on SCS
//         soleFramesViz.put(robotSide, new YoGraphicReferenceFrame(soleFrames.get(robotSide), registry, 0.1));
//         yoGraphicsList.add(soleFramesViz.get(robotSide));
      }

      // Visualize body frame in SCS
//      bodyFrameViz = new YoGraphicReferenceFrame(bodyJointID.getFrameAfterJoint(), registry, 0.3);
//      yoGraphicsList.add(bodyFrameViz);

      // Visualize CoM, CP and CoP on plotter panel
      YoArtifactPosition centerOfMassArtifact = new YoArtifactPosition("Center of Mass", yoCoM.getYoX(), yoCoM.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Black().getColor().get(), 0.009);
      YoArtifactPosition capturePointArtifact = new YoArtifactPosition("Capture Point", yoICP.getYoX(), yoICP.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Blue().getColor().get(), 0.009);
      YoArtifactPosition centerOfPressureArtifact = new YoArtifactPosition("Center of Pressure", yoCoP.getYoX(), yoCoP.getYoY(), GraphicType.SOLID_BALL, YoAppearance.Red().getColor().get(), 0.009);
      artifactList.add(centerOfMassArtifact);
      artifactList.add(capturePointArtifact);
      artifactList.add(centerOfPressureArtifact);

      // Initialize parameters for CoM, CoP and ICP calculations
      midSoleZUpFrame = new MidFrameZUpFrame("midSoleZUpFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      twistCalculator = new TwistCalculator(worldFrame, elevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      centerOfMassJacobian = new CenterOfMassJacobian(elevator);
      totalRobotMass = TotalMassCalculator.computeSubTreeMass(elevator);

      gravity = 9.81;
      omega0.set(Math.sqrt(gravity / bodyPosition.getZ()));
      
      updatePositionsIDrobot();
   }

   /**
    * Inertias
    */

   private Matrix3d createInertiaMatrixBox(LinkNames linkName)
   {
      Matrix3d inertiaBox = new Matrix3d();

      if (linkName == LinkNames.BODY_LINK)
      {
         inertiaBox = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(cubeX, cubeY, cubeZ, bodyMass);
      }

      else
      {
         inertiaBox = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(footX, footY, footZ, footMass);
      }
      return inertiaBox;
   }

   private Matrix3d createInertiaMatrixCylinder(LinkNames linkName)
   {
      Matrix3d inertiaCylinder = new Matrix3d();
      inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(RobotParameters2.MASSES.get(linkName),
            RobotParameters2.RADII.get(linkName), RobotParameters2.LENGTHS.get(linkName), Axis.Z);
      return inertiaCylinder;
   }

   /**
    * Rigid Bodies and Links
    */

   /************************* ID ROBOT - Rigid bodies ********************************/
   private RigidBody createBodyRB(LinkNames linkName, SixDoFJoint bodyJointID)
   {
      Matrix3d inertiaBody = createInertiaMatrixBox(linkName);
      return ScrewTools.addRigidBody(linkName.getName(), bodyJointID, inertiaBody, bodyMass, comOffsetBody);
   }

   private void createAndAttachCylinderRB(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Matrix3d inertiaCylinder = createInertiaMatrixCylinder(linkName);
      Vector3d comOffsetCylinder = new Vector3d(0.0, 0.0, -RobotParameters2.LENGTHS.get(linkName) / 2.0);
      RigidBody cyl = ScrewTools.addRigidBody(linkName.getName(), allLegJoints.get(robotSide).get(jointName), inertiaCylinder, bodyMass, comOffsetCylinder);
      allLegRigidBodies.get(robotSide).put(linkName, cyl);
   }

   private RigidBody createFootRB(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Matrix3d inertiaBody = createInertiaMatrixBox(linkName);
      return ScrewTools.addRigidBody(linkName.getName(), allLegJoints.get(robotSide).get(JointNames.KNEE), inertiaBody, footMass, comOffsetFoot);
   }

   /************************* SCS ROBOT - Links ********************************/
   private void createAndAttachBodyLink(LinkNames linkName)
   {
      Link link = new Link(LinkNames.BODY_LINK.getName());
      Matrix3d inertiaBody = createInertiaMatrixBox(linkName);
      link.setMomentOfInertia(inertiaBody);
      link.setMass(RobotParameters2.MASSES.get(LinkNames.BODY_LINK));
      link.setComOffset(comOffsetBody);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(cubeX, cubeY, cubeZ, RobotParameters2.APPEARANCE.get(LinkNames.BODY_LINK));
      link.setLinkGraphics(linkGraphics);
      link.addCoordinateSystemToCOM(0.3);
      bodyJointSCS.setLink(link);
   }

   private void createAndAttachCylinderLink(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Link link = new Link(linkName.getName());
      Matrix3d inertiaCylinder = createInertiaMatrixCylinder(linkName);
      link.setMomentOfInertia(inertiaCylinder);
      link.setMass(RobotParameters2.MASSES.get(linkName));
      link.setComOffset(new Vector3d(0.0, 0.0, -RobotParameters2.LENGTHS.get(linkName) / 2.0));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(-RobotParameters2.LENGTHS.get(linkName), RobotParameters2.RADII.get(linkName), RobotParameters2.APPEARANCE.get(linkName));
      link.setLinkGraphics(linkGraphics);
      link.addCoordinateSystemToCOM(0.15);
      idToSCSLegJointMap.get(allLegJoints.get(robotSide).get(jointName)).setLink(link);
   }

   private void createAndAttachFootLink(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Link link = new Link(LinkNames.FOOT_LINK.getName());
      Matrix3d inertiaFoot = createInertiaMatrixBox(linkName);
      link.setMomentOfInertia(inertiaFoot);
      link.setMass(RobotParameters2.MASSES.get(LinkNames.FOOT_LINK));
      link.setComOffset(comOffsetFoot);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(footOffsetX, 0.0, 0.0);
      linkGraphics.addCube(footX, footY, -footZ, RobotParameters2.APPEARANCE.get(LinkNames.FOOT_LINK));
      link.setLinkGraphics(linkGraphics);
      link.addCoordinateSystemToCOM(0.10);
      idToSCSLegJointMap.get(allLegJoints.get(robotSide).get(jointName)).setLink(link);
   }

   /**
    * SCS Robot --> ID Robot 
    * Send positions and velocities + CoM, CoP and ICP + Frames
    */
   public void updatePositionsIDrobot()
   {
      // Body positions
      bodyJointID.setPosition(bodyJointSCS.getQ_t1().getDoubleValue(), 0.0, bodyJointSCS.getQ_t2().getDoubleValue());
      bodyJointID.setRotation(0.0, bodyJointSCS.getQ_rot().getDoubleValue(), 0.0);

      // Body twist   
      ReferenceFrame bodyFrame = bodyJointID.getFrameAfterJoint();
      bodyJointLinearVelocity.setIncludingFrame(worldFrame, qd_x, 0.0, qd_z);
      bodyJointLinearVelocity.changeFrame(bodyFrame);
      bodyJointAngularVelocity.setIncludingFrame(bodyFrame, 0.0, qd_wy, 0.0);
      bodyJointTwist.set(bodyFrame, elevatorFrame, bodyFrame, bodyJointLinearVelocity, bodyJointAngularVelocity);
      bodyJointID.setJointTwist(bodyJointTwist);

      // Body velocities
      double[] velocityArray = new double[6];
      velocityArray[0] = 0.0; // yaw
      velocityArray[1] = bodyJointSCS.getQd_rot().getDoubleValue(); // pitch
      velocityArray[2] = 0.0; // roll
      velocityArray[3] = bodyJointSCS.getQd_t1().getDoubleValue(); // x
      velocityArray[4] = 0.0; // y
      velocityArray[5] = bodyJointSCS.getQd_t2().getDoubleValue(); // z
      DenseMatrix64F velocities = new DenseMatrix64F(6, 1, true, velocityArray);
      bodyJointID.setVelocity(velocities, 0);

      // Leg  and foot info (hip, knee and ankle)
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         idJoint.setQ(scsJoint.getQYoVariable().getDoubleValue());
         idJoint.setQd(scsJoint.getQDYoVariable().getDoubleValue());
      }

      // Update frames
      elevator.updateFramesRecursively(); //REALLY important line!!

      for (RobotSide robotSide : RobotSide.values)
      {
    	 soleFrames.get(robotSide).update();
         soleZUpFrames.get(robotSide).update();
         legJacobians.get(robotSide).compute();
      }

      midSoleZUpFrame.update();
      twistCalculator.compute();

      // CoM
      centerOfMassFrame.update();
      centerOfMassJacobian.compute();
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(worldFrame); //to make sure that we are actually in world frame
      centerOfMass2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMass);
      centerOfMassVelocity2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMassVelocity);

      yoCoM.set(centerOfMass);
      yoCoMVelocity.set(centerOfMassVelocity);

      // ICP
      CapturePointCalculator.computeCapturePoint(capturePoint, centerOfMass2d, centerOfMassVelocity2d, omega0.getDoubleValue());
      yoICP.set(capturePoint);

      // CoP  (+ updating  body and sole Frames)  //TODO important update!
      measuredGroundReactionForce.setToZero();
//      bodyFrameViz.update();
      
      for (RobotSide robotside : RobotSide.values)
      {
         footPosition.setToZero(soleFrames.get(robotside));
         footPosition.changeFrame(worldFrame);
         yoFootPositions.get(robotside).setAndMatchFrame(footPosition);
//         soleFramesViz.get(robotside).update();
         measuredGroundReactionForce.add(GCpointsHeel.get(robotside).getYoForce());
         measuredGroundReactionForce.add(GCpointsToe.get(robotside).getYoForce());
      }

      totalFz = measuredGroundReactionForce.getZ();

      if (totalFz < 1.0e-3)
      {
         yoCoP.setToNaN();
      }
      else
      {
         double alpha = (GCpointsHeel.get(RobotSide.RIGHT).getYoForce().getZ() + GCpointsToe.get(RobotSide.RIGHT).getYoForce().getZ()) / totalFz; //fraction of the total Fz represented by the FzRightFoot
         yoCoP.interpolate(yoFootPositions.get(RobotSide.LEFT), yoFootPositions.get(RobotSide.RIGHT), alpha);
      }

      // Get the body coordinates
      bodyPosition.setToZero(bodyJointID.getFrameAfterJoint());
      bodyPosition.changeFrame(worldFrame);

   }

   /**
    * ID Robot --> SCS Robot 
    * Copy the torques from the IDRobot to the SCSRobot.
    */
   public void updateTorquesSCSrobot() //Remember! the body joint is NOT actuated
   {
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         scsJoint.setTau(idJoint.getTau());
      }
   }

   /**
    * Getters and Setters for the controller
    */

   // ID joints and rigid bodies
   public RigidBody getElevator()
   {
      return elevator;
   }

   public OneDoFJoint getLegJoint(JointNames jointName, RobotSide robotSide)
   {
      return allLegJoints.get(robotSide).get(jointName);
   }

   public SixDoFJoint getBodyJoint()
   {
      return bodyJointID;
   }

   public RigidBody getLegRigidBody(LinkNames linkName, RobotSide robotSide)
   {
      return allLegRigidBodies.get(robotSide).get(linkName);
   }

   // Body
   public double getBodyPositionZ()
   {
      bodyZ = bodyPosition.getZ();
      return bodyZ;
   }

   public double getBodyPositionX()
   {
      bodyX = bodyPosition.getX();
      return bodyX;
   }

   public void getBodyPitch(Quat4d rotationToPack)
   {
      bodyJointID.getFrameAfterJoint().getRotation(rotationToPack);
   }

   public void getBodyLinearVel(Vector3d linearVelocityToPack)
   {
      bodyJointID.getLinearVelocity(linearVelocityToPack);
   }

   public void getBodyAngularVel(Vector3d angularVelocityToPack)
   {
      bodyJointID.getAngularVelocity(angularVelocityToPack);
   }

   public double getBodyVelX(Vector3d linearVelocityToPack)
   {
      bodyJointID.getLinearVelocity(linearVelocityToPack);
      double velX = linearVelocityToPack.getX();
      return velX;
   }

   // Hip
   public void setHipTau(RobotSide robotSide, double desiredHipTau)
   {
      allLegJoints.get(robotSide).get(JointNames.HIP).setTau(desiredHipTau);
   }

   public double getHipVelocity(RobotSide robotSide)
   {
      double hipVelocity = allLegJoints.get(robotSide).get(JointNames.HIP).getQd();
      return hipVelocity;
   }

   public double getHipPitch(RobotSide robotSide)
   {
      double hipPitch = allLegJoints.get(robotSide).get(JointNames.HIP).getQ();
      return hipPitch;
   }

   public double getHipTau(RobotSide robotSide)
   {
      double hipTau = allLegJoints.get(robotSide).get(JointNames.HIP).getTau();
      return hipTau;
   }

   public OneDoFJoint getHipJointID(RobotSide robotSide)
   {
      return allLegJoints.get(robotSide).get(JointNames.HIP);
   }

   // Knee
   public double getKneeVelocity(RobotSide robotSide)
   {
      double kneeVelocity = allLegJoints.get(robotSide).get(JointNames.KNEE).getQd();
      return kneeVelocity;
   }

   public void setKneeTau(RobotSide robotSide, double desiredKneeTau)
   {
      allLegJoints.get(robotSide).get(JointNames.KNEE).setTau(desiredKneeTau);
   }

   public double getKneePitch(RobotSide robotSide)
   {
      double kneePitch = allLegJoints.get(robotSide).get(JointNames.KNEE).getQ();
      return kneePitch;
   }

   public double getKneeTau(RobotSide robotSide)
   {
      double kneeTau = allLegJoints.get(robotSide).get(JointNames.KNEE).getTau();
      return kneeTau;
   }

   public OneDoFJoint getKneeJointID(RobotSide robotSide)
   {
      return allLegJoints.get(robotSide).get(JointNames.KNEE);
   }

   //Ankle
   public void setAnkleTau(RobotSide robotSide, double desiredAnkleTau)
   {
      allLegJoints.get(robotSide).get(JointNames.ANKLE).setTau(desiredAnkleTau);
   }

   public double getAnkleVelocity(RobotSide robotSide)
   {
      double ankleVelocity = allLegJoints.get(robotSide).get(JointNames.ANKLE).getQd();
      return ankleVelocity;
   }

   public double getAnklePitch(RobotSide robotSide)
   {
      double anklePitch = allLegJoints.get(robotSide).get(JointNames.ANKLE).getQ();
      return anklePitch;
   }

   public double getAnkleHeight(RobotSide robotSide)
   {
      footPosition.setToZero(allLegJoints.get(robotSide).get(JointNames.ANKLE).getFrameAfterJoint());
      footPosition.changeFrame(worldFrame);
      double ankleZ = footPosition.getZ();
      return ankleZ;
   }

   // Feet
   public boolean heelOnTheFloor(RobotSide robotSide)
   {
      return GCpointsHeel.get(robotSide).isInContact();
   }

   public boolean heelToeOffAhead(RobotSide robotSide)
   {
      return GCpointsHeel.get(robotSide).getX() > GCpointsHeel.get(robotSide.getOppositeSide()).getX();
   }

   public boolean toeOnTheFloor(RobotSide robotSide)
   {
      return GCpointsToe.get(robotSide).isInContact();
   }

   public void setFStoTrue(RobotSide robotSide)
   {
      GCpointsHeel.get(robotSide).getYoFootSwitch().set(1.0);
   }

   public double getToeX(RobotSide robotSide)
   {
      return GCpointsToe.get(robotSide).getX();
   }

   public double getHeelX(RobotSide robotSide)
   {
      return GCpointsHeel.get(robotSide).getX();
   }

   public double getHeelZ(RobotSide robotSide)
   {
      return GCpointsHeel.get(robotSide).getZ();
   }

   public void getFootLinearVelocity(RobotSide robotSide, FrameVector linearVelocityToPack)
   {
      GCpointsHeel.get(robotSide).getYoVelocity().getFrameTupleIncludingFrame(linearVelocityToPack);
   }
   
   // Others
   public double getTotalRobotMass()
   {
      return totalRobotMass;
   }

   public SideDependentList<YoFramePoint> getYoFootPositions()
   {
      return yoFootPositions;
   }

   public ReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   public ArtifactList getArtifactList()
   {
      return artifactList;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }

   public MidFrameZUpFrame getMidSoleZUpFrame()
   {
      return midSoleZUpFrame;
   }

   public GeometricJacobian getLegJacobian(RobotSide robotSide)
   {
      return legJacobians.get(robotSide);
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public double getGravity()
   {
      return gravity;
   }
   
   //CoM and ICP
   public void getCoM(FramePoint2d comToPack)
   {
      yoCoM.getFrameTuple2dIncludingFrame(comToPack);
   }

   public void getCoM(FramePoint centerOfMassToPack)
   {
      yoCoM.getFrameTupleIncludingFrame(centerOfMassToPack);
   }

   public void getCapturePoint(FramePoint2d capturePoinToPack)
   {
      yoICP.getFrameTuple2dIncludingFrame(capturePoinToPack);
   }
   
   // Set initial velocity
   public void setInitialVelocity(double desInitialVelocity)
   {
	   bodyJointSCS.getQd_t1().set(desInitialVelocity);
   }
   
   // Other  
   public double getHipOffsetY()
   {
	   return hipOffsetY;
   }
}