package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.JointNames;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.LinkNames;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PrismaticJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class Step5IDandSCSRobot extends Robot
{

   /**
    * Variables
    */

   // ID
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
   private FramePoint bodyPosition = new FramePoint();

   private final Vector3D jointAxesHip = new Vector3D(0.0, 1.0, 0.0); // rotate around Y-axis (for revolute joints)
   private final Vector3D jointAxesKnee = new Vector3D(0.0, 0.0, 1.0);
   private final Vector3D jointAxesAnkle = new Vector3D(0.0, 1.0, 0.0);
   private final RigidBody elevator;

   private final SideDependentList<EnumMap<JointNames, OneDoFJoint>> allLegJoints = SideDependentList.createListOfEnumMaps(JointNames.class); // Includes the ankle!
   private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> idToSCSLegJointMap = new LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>(); // makes the SCS joints available without another side dependent enum
   private final SideDependentList<EnumMap<LinkNames, RigidBody>> allLegRigidBodies = SideDependentList.createListOfEnumMaps(LinkNames.class);
   private final SixDoFJoint bodyJointID;

   // SCS
   private final FloatingPlanarJoint bodyJointSCS;
   //SideDependentList<GroundContactPoint> GCpoints = new SideDependentList<GroundContactPoint>();
   SideDependentList<GroundContactPoint> GCpointsHeel = new SideDependentList<GroundContactPoint>();
   SideDependentList<GroundContactPoint> GCpointsToe = new SideDependentList<GroundContactPoint>();

   private double cubeX = RobotParameters.BODY_DIMENSIONS.get(Axis.X);
   private double cubeY = RobotParameters.BODY_DIMENSIONS.get(Axis.Y);
   private double cubeZ = RobotParameters.BODY_DIMENSIONS.get(Axis.Z);

   private double footX = RobotParameters.FOOT_DIMENSIONS.get(Axis.X);
   private double footY = RobotParameters.FOOT_DIMENSIONS.get(Axis.Y);
   private double footZ = RobotParameters.FOOT_DIMENSIONS.get(Axis.Z);

   private double gcOffset = -footZ;
   private double initialBodyHeight = RobotParameters.LENGTHS.get(LinkNames.UPPER_LINK) + RobotParameters.LENGTHS.get(LinkNames.LOWER_LINK) - 0.4 + footZ;

   // GENERAL
   private double gcRadius = 0.03;
   //private GroundContactPoint gcHeel, gcToe;

   private double bodyMass = RobotParameters.MASSES.get(LinkNames.BODY_LINK);
   private double footMass = RobotParameters.MASSES.get(LinkNames.FOOT_LINK);

   private double hipOffsetY = RobotParameters.BODY_DIMENSIONS.get(Axis.X) / 2.0;
   private double kneeOffsetZ = -RobotParameters.LENGTHS.get(LinkNames.UPPER_LINK) + 0.4;
   private double ankleOffsetZ = -RobotParameters.LENGTHS.get(LinkNames.LOWER_LINK);
   private double footOffsetX = 0.15;
   Vector3D comOffsetBody = new Vector3D((3.0 * cubeX) / 4.0, 0.0, cubeZ / 2.0);
   Vector3D comOffsetFoot = new Vector3D(footOffsetX, 0.0, -footZ / 2.0); //TODO is it correct to include the footOffsetX (since the foot isn't centered)?

   public DoubleYoVariable qd_x;
   
   /**
    * Joints
    */

   public Step5IDandSCSRobot()
   {
      super("Robot");

      /****************** ID ROBOT ***********************/
      elevator = new RigidBody("elevator", elevatorFrame);

      bodyJointID = new SixDoFJoint(JointNames.BODY.getName(), elevator, elevatorFrame);
      createAndAttachBodyRB(LinkNames.BODY_LINK, bodyJointID);

      for (RobotSide robotSide : RobotSide.values)
      {
         // HIP ID (location, joint, and rigidBody) 
         Vector3D hipOffset = new Vector3D(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0);
         //System.out.println("bodyJointID.getSuccessor() = " + bodyJointID.getSuccessor().getName()); //Check point to see which is the successor and predecessor of the body
         RevoluteJoint hipJointID = ScrewTools.addRevoluteJoint(JointNames.HIP.getName(), bodyJointID.getSuccessor(), hipOffset, jointAxesHip); //The parent rigid body of the hip joint is: bodyJointID.getSuccessor()
         allLegJoints.get(robotSide).put(JointNames.HIP, hipJointID);
         createAndAttachCylinderRB(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         // KNEE ID
         Vector3D kneeOffset = new Vector3D(0.0, 0.0, kneeOffsetZ);
         PrismaticJoint kneeJointID = ScrewTools.addPrismaticJoint(JointNames.KNEE.getName(), hipJointID.getSuccessor(), kneeOffset, jointAxesKnee);
         allLegJoints.get(robotSide).put(JointNames.KNEE, kneeJointID);
         createAndAttachCylinderRB(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);

         // ANKLE ID (location, joint, and rigidBody) 
         Vector3D ankleOffset = new Vector3D(0.0, 0.0, ankleOffsetZ);
         RevoluteJoint ankleJointID = ScrewTools.addRevoluteJoint(JointNames.ANKLE.getName(), kneeJointID.getSuccessor(), ankleOffset, jointAxesAnkle); //The parent rigid body of the hip joint is: bodyJointID.getSuccessor()
         allLegJoints.get(robotSide).put(JointNames.ANKLE, ankleJointID);
         createAndAttachFootRB(LinkNames.FOOT_LINK, JointNames.ANKLE, robotSide);
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
         PinJoint hipJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.HIP.getName(), new Vector3D(0.0,
               robotSide.negateIfRightSide(hipOffsetY), 0.0), this, jointAxesHip);
         hipJointSCS.setLimitStops(-0.6, 0.6, 1e6, 1e3); //It is NOT necessary to set limits in the ID description because if the SCS description doesn't let the robot move passed a point the ID robot won't be able to pass it either

         if (robotSide == RobotSide.LEFT) 
         {
            hipJointSCS.setQ(-0.6); // TODO Remember!! Initial position of the left leg. I put 0.6 like the limits, initially
         }

         bodyJointSCS.addJoint(hipJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.HIP), hipJointSCS);
         createAndAttachCylinderLink(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         // KNEE SCS
//         PinJoint kneeJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.KNEE.getName(), new Vector3d(0.0, 0.0, kneeOffsetZ), this, jointAxesHip);
         SliderJoint kneeJointSCS = new SliderJoint(robotSide.getSideNameFirstLetter() + JointNames.KNEE.getName(), new Vector3D(0.0, 0.0, kneeOffsetZ), this,
               jointAxesKnee);
         kneeJointSCS.setLimitStops(-0.6, 0.6, 1e5, 1e4);
         hipJointSCS.addJoint(kneeJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.KNEE), kneeJointSCS);
         createAndAttachCylinderLink(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);

         // ANKLE SCS
         PinJoint ankleJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.ANKLE.getName(), new Vector3D(0.0, 0.0, ankleOffsetZ), this,
               jointAxesAnkle);
         ankleJointSCS.setLimitStops(-0.3, 0.7, 1e5, 1e3); //TODO tweak as needed
         kneeJointSCS.addJoint(ankleJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.ANKLE), ankleJointSCS);
         createAndAttachFootLink(LinkNames.FOOT_LINK, JointNames.ANKLE, robotSide);

         // FEET SCS
         GroundContactPoint gcHeel = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcHeel", new Vector3D(-0.1, 0.0, gcOffset), this);
         GCpointsHeel.set(robotSide, gcHeel);
         ankleJointSCS.addGroundContactPoint(gcHeel);
         Graphics3DObject graphicsGCheel = ankleJointSCS.getLink().getLinkGraphics();
         graphicsGCheel.identity();
         graphicsGCheel.translate(-0.1, 0.0, gcOffset);
         graphicsGCheel.addSphere(gcRadius, YoAppearance.Orange());
         if (robotSide == RobotSide.RIGHT)
         {
            setFStoTrue(robotSide);
         }

         GroundContactPoint gcToe = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcToe", new Vector3D(0.4, 0.0, gcOffset), this);
         GCpointsToe.set(robotSide, gcToe);
         ankleJointSCS.addGroundContactPoint(gcToe);
         Graphics3DObject graphics = ankleJointSCS.getLink().getLinkGraphics();
         graphics.identity();
         graphics.translate(0.4, 0.0, gcOffset);
         graphics.addSphere(gcRadius, YoAppearance.DarkOliveGreen());
      }

      /**************** SCS Ground Model *************************/
      GroundContactModel groundModel = new LinearGroundContactModel(this, 150000, 150000, 50000.0, 1e5, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);
   }

   /**
    * Initialization for walking
    */
   public void initializeForBallisticWalking()
   {
      qd_x = (DoubleYoVariable)getVariable("qd_x");
      qd_x.set(7.458267603119068);   //initial velocity 
   }
   
   /**
    * Inertias
    */

   private Matrix3D createInertiaMatrixBox(LinkNames linkName)
   {
      Matrix3D inertiaBox = new Matrix3D();

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

   private Matrix3D createInertiaMatrixCylinder(LinkNames linkName)
   {
      Matrix3D inertiaCylinder = new Matrix3D();
      inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(RobotParameters.MASSES.get(linkName),
            RobotParameters.RADII.get(linkName), RobotParameters.LENGTHS.get(linkName), Axis.Z);
      return inertiaCylinder;
   }

   /**
    * Rigid Bodies and Links
    */

   /************************* ID ROBOT - Rigid bodies ********************************/
   private void createAndAttachBodyRB(LinkNames linkName, SixDoFJoint bodyJointID)
   {
      Matrix3D inertiaBody = createInertiaMatrixBox(linkName);
      ScrewTools.addRigidBody(linkName.getName(), bodyJointID, inertiaBody, bodyMass, comOffsetBody);
   }

   private void createAndAttachCylinderRB(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Matrix3D inertiaCylinder = createInertiaMatrixCylinder(linkName);
      Vector3D comOffsetCylinder = new Vector3D(0.0, 0.0, -RobotParameters.LENGTHS.get(linkName) / 2.0);
      allLegRigidBodies.get(robotSide).put(linkName,
            ScrewTools.addRigidBody(linkName.getName(), allLegJoints.get(robotSide).get(jointName), inertiaCylinder, bodyMass, comOffsetCylinder));
   }

   private void createAndAttachFootRB(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Matrix3D inertiaBody = createInertiaMatrixBox(linkName);
      ScrewTools.addRigidBody(linkName.getName(), bodyJointID, inertiaBody, bodyMass, comOffsetBody);
   }

   /************************* SCS ROBOT - Links ********************************/
   private void createAndAttachBodyLink(LinkNames linkName)
   {
      Link link = new Link(LinkNames.BODY_LINK.getName());
      Matrix3D inertiaBody = createInertiaMatrixBox(linkName);
      link.setMomentOfInertia(inertiaBody);
      link.setMass(RobotParameters.MASSES.get(LinkNames.BODY_LINK));
      link.setComOffset(comOffsetBody);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(cubeX, cubeY, cubeZ, RobotParameters.APPEARANCE.get(LinkNames.BODY_LINK));
      link.setLinkGraphics(linkGraphics);
      bodyJointSCS.setLink(link);
   }

   private void createAndAttachCylinderLink(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Link link = new Link(linkName.getName());
      Matrix3D inertiaCylinder = createInertiaMatrixCylinder(linkName);
      link.setMomentOfInertia(inertiaCylinder);
      link.setMass(RobotParameters.MASSES.get(linkName));
      link.setComOffset(new Vector3D(0.0, 0.0, -RobotParameters.LENGTHS.get(linkName) / 2.0));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(-RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
      link.setLinkGraphics(linkGraphics);
      idToSCSLegJointMap.get(allLegJoints.get(robotSide).get(jointName)).setLink(link);   
   }

   private void createAndAttachFootLink(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Link link = new Link(LinkNames.FOOT_LINK.getName());
      Matrix3D inertiaFoot = createInertiaMatrixBox(linkName);
      link.setMomentOfInertia(inertiaFoot);
      link.setMass(RobotParameters.MASSES.get(LinkNames.FOOT_LINK));
      link.setComOffset(comOffsetFoot);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(footOffsetX, 0.0, 0.0);
      if (robotSide == RobotSide.LEFT) 
      {
         linkGraphics.addCube(footX, footY, -footZ, RobotParameters.APPEARANCE.get(LinkNames.FOOT_LINK));  
      }
      if (robotSide == RobotSide.RIGHT) 
      {
         linkGraphics.addCube(footX, footY, -footZ, YoAppearance.Tomato());
      }
      link.setLinkGraphics(linkGraphics);
      idToSCSLegJointMap.get(allLegJoints.get(robotSide).get(jointName)).setLink(link);
   }

   /**
    * SCS Robot --> ID Robot 
    * Send positions and velocities.
    */

   public void updatePositionsIDrobot()
   {
      //Body info
      bodyJointID.setPosition(bodyJointSCS.getQ_t1().getDoubleValue(), 0.0, bodyJointSCS.getQ_t2().getDoubleValue());
      bodyJointID.setRotation(0.0, bodyJointSCS.getQ_rot().getDoubleValue(), 0.0);

      double[] velocityArray = new double[6];
      velocityArray[0] = 0.0; // yaw
      velocityArray[1] = bodyJointSCS.getQd_rot().getDoubleValue(); // pitch
      velocityArray[2] = 0.0; // roll
      velocityArray[3] = bodyJointSCS.getQd_t1().getDoubleValue(); // x
      velocityArray[4] = 0.0; // y
      velocityArray[5] = bodyJointSCS.getQd_t2().getDoubleValue(); // z
      DenseMatrix64F velocities = new DenseMatrix64F(6, 1, true, velocityArray);
      bodyJointID.setVelocity(velocities, 0);

      //Leg  and foot info (hip, knee and ankle)
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         idJoint.setQ(scsJoint.getQYoVariable().getDoubleValue());
         idJoint.setQd(scsJoint.getQDYoVariable().getDoubleValue());
      }

      elevator.updateFramesRecursively();

      // Get the body coordinates
      bodyPosition = new FramePoint();
      bodyPosition.setToZero(bodyJointID.getFrameAfterJoint());
      bodyPosition.changeFrame(worldFrame);

   }

   /**
    * ID Robot --> SCS Robot 
    * Copy the torques from the IDRobot to the SCSRobot.
    */
   public void updateTorquesSCSrobot()
   {
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         scsJoint.setTau(idJoint.getTau());
      }
   }

   public RigidBody getElevator() // TODO In the IDMP this is used in the robot  
   {
      return elevator;
   }

   /**
    * Getters and Setters for the controller
    */

   //ID joints and rigid bodies
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
   //   public void getBodyPoint(Point3D bodyPointToPack) // TODO Why pack it instead of doing this?
   //   {
   //      Point3D bodyPoint = this.bodyPosition.getPoint();
   //      bodyPointToPack.set(bodyPoint);
   //   }

   public double getBodyPositionZ()
   {
      double h = bodyPosition.getZ();
      return h;
   }

   public void getBodyPitch(Quaternion rotationToPack)
   {
      bodyJointID.getFrameAfterJoint().getRotation(rotationToPack);
   }

   public void getBodyLinearVel(Vector3D linearVelocityToPack)
   {
      bodyJointID.getLinearVelocity(linearVelocityToPack);
   }

   public void getBodyAngularVel(Vector3D angularVelocityToPack)
   {
      bodyJointID.getAngularVelocity(angularVelocityToPack);
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

   public double getKneePositionZ(RobotSide robotSide)
   {
      double kneeHeight = allLegJoints.get(robotSide).get(JointNames.KNEE).getQ();
      return kneeHeight;
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

   public boolean heelOnTheFloor(RobotSide robotSide)
   {
//            return GCpointsHeel.get(robotSide).getYoFootSwitch().getDoubleValue() > 0.05;
            return GCpointsHeel.get(robotSide).isInContact();
   }

   public boolean heelToeOffAhead(RobotSide robotSide)
   {
      return GCpointsHeel.get(robotSide).getX() > GCpointsHeel.get(robotSide.getOppositeSide()).getX();
   }
   
   public boolean toeOnTheFloor(RobotSide robotSide)
   {
      //      return GCpointsHeel.get(robotSide).getYoFootSwitch().getDoubleValue() > 0.5;
            return GCpointsToe.get(robotSide).isInContact();
   }
   
   public void setFStoTrue(RobotSide robotSide)
   {
      GCpointsHeel.get(robotSide).getYoFootSwitch().set(1.0);
   }
}
