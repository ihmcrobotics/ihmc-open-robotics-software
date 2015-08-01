package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;
import java.util.LinkedHashMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.JointNames;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.LinkNames;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.utilities.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.PrismaticJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class Step5IDandSCSRobot extends Robot
{

   /**
    * Variables
    */

   // ID
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());

   private final Vector3d jointAxesHip = new Vector3d(0.0, 1.0, 0.0); // rotate around Y-axis (for revolute joints)
   private final Vector3d jointAxesKnee = new Vector3d(0.0, 0.0, 1.0); 
   private final RigidBody elevator;

   SideDependentList<GroundContactPoint> GCpoints = new SideDependentList<GroundContactPoint>();
   private final SideDependentList<EnumMap<JointNames, OneDoFJoint>> allLegJoints = SideDependentList.createListOfEnumMaps(JointNames.class);
   private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> idToSCSLegJointMap = new LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>(); // makes the SCS stuff available
   private final SideDependentList<EnumMap<LinkNames, RigidBody>> allLegRigidBodies = SideDependentList.createListOfEnumMaps(LinkNames.class);
   private FramePoint bodyPosition = new FramePoint();

   // SCS
   private final FloatingPlanarJoint bodyJointSCS;

   // GENERAL
   private double cubeX = RobotParameters.BODY_DIMENSIONS.get(Axis.X);
   private double cubeY = RobotParameters.BODY_DIMENSIONS.get(Axis.Y);
   private double cubeZ = RobotParameters.BODY_DIMENSIONS.get(Axis.Z);

   private double bodyMass = RobotParameters.MASSES.get(LinkNames.BODY_LINK);

   private double hipOffsetY = RobotParameters.BODY_DIMENSIONS.get(Axis.X)/2.0;
   private double kneeOffsetZ = -RobotParameters.LENGTHS.get(LinkNames.UPPER_LINK) + 0.4;
   private final SixDoFJoint bodyJointID;

   Vector3d comOffsetBody = new Vector3d(0.0, 0.0, cubeZ / 2.0);
   private double gcOffset = -RobotParameters.LENGTHS.get(LinkNames.LOWER_LINK);
   private double initialBodyHeight = RobotParameters.LENGTHS.get(LinkNames.UPPER_LINK) + RobotParameters.LENGTHS.get(LinkNames.LOWER_LINK) - 0.4;
   
   /**
    * Joints
    */

   public Step5IDandSCSRobot()
   {
      super("Robot");

      /****************** ID ROBOT ***********************/
      elevator = new RigidBody("elevator", elevatorFrame);

      bodyJointID = new SixDoFJoint(JointNames.BODY.getName(), elevator, elevatorFrame);
      createAndAttachBody(LinkNames.BODY_LINK, bodyJointID);

      for (RobotSide robotSide : RobotSide.values)
      {
         // HIP ID (location, joint, and rigidBody) 
         Vector3d hipOffset = new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0);
         System.out.println("bodyJointID.getSuccessor() = " + bodyJointID.getSuccessor().getName());
         RevoluteJoint hipJointID = ScrewTools.addRevoluteJoint(JointNames.HIP.getName(), bodyJointID.getSuccessor(), hipOffset, jointAxesHip);
         allLegJoints.get(robotSide).put(JointNames.HIP, hipJointID);
         createAndAttachCylinder(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         // KNEE ID
         Vector3d kneeOffset = new Vector3d(0.0, 0.0, kneeOffsetZ);
         PrismaticJoint kneeJointID = ScrewTools.addPrismaticJoint(JointNames.KNEE.getName(), hipJointID.getSuccessor(), kneeOffset, jointAxesKnee);
         allLegJoints.get(robotSide).put(JointNames.KNEE, kneeJointID);
         createAndAttachCylinder(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);
      }

      /****************** SCS ROBOT ***********************/
      // BODY SCS
      bodyJointSCS = new FloatingPlanarJoint(JointNames.BODY.getName(), this);
      this.addRootJoint(bodyJointSCS);
      createAndAttachBodyLink();
      bodyJointSCS.setCartesianPosition(0.0, initialBodyHeight);

      for (RobotSide robotSide : RobotSide.values)
      {
         // HIP SCS
         PinJoint hipJointSCS = new PinJoint(robotSide.getSideNameFirstLetter() + JointNames.HIP.getName(), new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0), this, jointAxesHip);
         hipJointSCS.setLimitStops(-1.0, 1.0, 1e6, 1e3); //TODO also in ID robot?
         bodyJointSCS.addJoint(hipJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.HIP), hipJointSCS);
         createAndAttachCylinderLink(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         // KNEE SCS
         SliderJoint kneeJointSCS = new SliderJoint(robotSide.getSideNameFirstLetter() + JointNames.KNEE.getName(), new Vector3d(0.0, 0.0, kneeOffsetZ), this, jointAxesKnee);
         kneeJointSCS.setLimitStops(-0.3, 0.3, 1e5, 1e4);
         hipJointSCS.addJoint(kneeJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.KNEE), kneeJointSCS);
         createAndAttachCylinderLink(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);
         
         GroundContactPoint contactPoint = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "Foot", new Vector3d(0.0, 0.0, gcOffset), this);
         GCpoints.set(robotSide, contactPoint);
         kneeJointSCS.addGroundContactPoint(contactPoint);
         Graphics3DObject graphics = kneeJointSCS.getLink().getLinkGraphics();
         graphics.identity();
         graphics.translate(0.0, 0.0, gcOffset);
         double radius = 0.03;
         graphics.addSphere(radius, YoAppearance.Orange());
      }
         
      /**************** Ground Model *************************/
      GroundContactModel groundModel = new LinearGroundContactModel(this, 150000, 15000, 50000.0, 1e5, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);
   }

   /**
    * Inertias
    */
   private Matrix3d createInertiaCube()
   {
      Matrix3d inertiaCube = new Matrix3d();
      inertiaCube = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(cubeX, cubeY, cubeZ, bodyMass);
      return inertiaCube;
   }

   private Matrix3d createInertiaCylinder(LinkNames linkName, JointNames jointName)
   {
      Matrix3d inertiaCylinder = new Matrix3d();

      inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(RobotParameters.MASSES.get(linkName),
            RobotParameters.RADII.get(linkName), RobotParameters.LENGTHS.get(linkName), Axis.Z);
      return inertiaCylinder;
   }

   /**
    * Rigid Bodies and Links
    */

   /************************* ID ROBOT - Rigid bodies ********************************/
   private void createAndAttachBody(LinkNames linkName, SixDoFJoint bodyJointID)
   {
      Matrix3d inertiaCube = createInertiaCube();
      // Vector3d comOffset = RobotParameters.COMs.get(linkName);
      // System.out.println("linkName "+ linkName.getName() + "\nbodyJoint " +
      // bodyJointID + "\ninertiaCube " + inertiaCube + "\nbodyMass " +
      // bodyMass +"\ncomOffset "+ comOffset); //Check point
      //allLegRigidBodies.put(linkName, ScrewTools.addRigidBody(linkName.getName(), bodyJointID, inertiaCube, bodyMass, comOffsetBody));
      ScrewTools.addRigidBody(linkName.getName(), bodyJointID, inertiaCube, bodyMass, comOffsetBody);
   }

   private void createAndAttachCylinder(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Matrix3d inertiaCylinder = createInertiaCylinder(linkName, jointName);
      // Vector3d comOffset = RobotParameters.COMs.get(linkName);
//      rigidBodies.put(linkName, ScrewTools.addRigidBody(linkName.getName(), allLegJoints.get(robotSide).get(jointName), inertiaCylinder, bodyMass,
//            new Vector3d(0.0, 0.0, -RobotParameters.LENGTHS.get(linkName) / 2.0)));
      allLegRigidBodies.get(robotSide).put(linkName, ScrewTools.addRigidBody(linkName.getName(), allLegJoints.get(robotSide).get(jointName), inertiaCylinder, bodyMass, new Vector3d(0.0, 0.0, -RobotParameters.LENGTHS.get(linkName) / 2.0)));
   }

   /************************* SCS ROBOT - Links ********************************/
   private void createAndAttachBodyLink()
   {
      Link link = new Link(LinkNames.BODY_LINK.getName());
      Matrix3d inertiaCube = createInertiaCube();
      link.setMomentOfInertia(inertiaCube);
      link.setMass(RobotParameters.MASSES.get(LinkNames.BODY_LINK));
      // link.setComOffset(RobotParameters.COMs.get(linkName));
      link.setComOffset(comOffsetBody);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(cubeX, cubeY, cubeZ, RobotParameters.APPEARANCE.get(LinkNames.BODY_LINK));
      link.setLinkGraphics(linkGraphics);
      bodyJointSCS.setLink(link);
   }

   private void createAndAttachCylinderLink(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Link link = new Link(linkName.getName());
      Matrix3d inertiaCylinder = createInertiaCylinder(linkName, jointName);
      link.setMomentOfInertia(inertiaCylinder);
      link.setMass(RobotParameters.MASSES.get(linkName));
      // link.setComOffset(RobotParameters.COMs.get(linkName));
      link.setComOffset(new Vector3d(0.0, 0.0, -RobotParameters.LENGTHS.get(linkName) / 2.0));

//      Graphics3DObject linkGraphics = new Graphics3DObject();
//      if (RobotParameters.LENGTHS.get(linkName) > 0.0)
//         linkGraphics.addCylinder(RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
//      else
//      {
//         linkGraphics.translate(0.0, 0.0, RobotParameters.LENGTHS.get(linkName));
//         linkGraphics.addCylinder(-RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
//      }
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(-RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
    
      link.setLinkGraphics(linkGraphics);
      idToSCSLegJointMap.get(allLegJoints.get(robotSide).get(jointName)).setLink(link);
   }

   /**
    * SCS Robot --> ID Robot. Send positions and velocities.
    */

   public void updateIDRobot()
   {
      //Body info
      bodyJointID.setPosition(bodyJointSCS.getQ_t1().getDoubleValue(), 0.0, bodyJointSCS.getQ_t2().getDoubleValue());
      bodyJointID.setRotation(0.0, bodyJointSCS.getQ_rot().getDoubleValue(), 0.0);

      double[] velocityArray = new double[6];
      velocityArray[0] = 0.0;                                        // yaw
      velocityArray[1] = bodyJointSCS.getQd_rot().getDoubleValue();  // pitch
      velocityArray[2] = 0.0;                                        // roll
      velocityArray[3] = bodyJointSCS.getQd_t1().getDoubleValue();   // x
      velocityArray[4] = 0.0;                                        // y
      velocityArray[5] = bodyJointSCS.getQd_t2().getDoubleValue();   // z
      DenseMatrix64F velocities = new DenseMatrix64F(6, 1, true, velocityArray);
      bodyJointID.setVelocity(velocities, 0);
      
      //Leg info
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         idJoint.setQ(scsJoint.getQ().getDoubleValue());
         idJoint.setQd(scsJoint.getQD().getDoubleValue());
         System.out.println("I am sending the leg info to ID robot");
      }

      elevator.updateFramesRecursively();

      // Get the body point
      bodyPosition = new FramePoint();
      bodyPosition.setToZero(bodyJointID.getFrameAfterJoint());
      bodyPosition.changeFrame(worldFrame);

   }

   /**
    * Copy the torques from the IDRobot to the SCSRobot so it is taken into
    * account in SCS
    */
   public void applyTorques()
   {
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         idJoint.setTau(scsJoint.getTau().getDoubleValue());
         System.out.println("I am copying the tau to the SCS robot");
      }
   }

   public RigidBody getElevator() // TODO In the IDMP this is used in the robot  
   {
      return elevator;
   }

   /**
    * Getters and Setters for the controller
    */
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
   public void getBodyPoint(Point3d bodyPointToPack) // TODO double h = bodyPosition.getZ(); //Why pack it instead of doing this?
   {
      Point3d bodyPoint = this.bodyPosition.getPoint();
      bodyPointToPack.set(bodyPoint);
   }

   // public Quat4d getBodyPitch(Quat4d rotationToPack) //RotationFunctions
   // {
   // Quat4d bodyPitch =
   // bodyJointID.getFrameAfterJoint().getRotation(rotationToPack);
   // return bodyPitch;
   // }

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
}
