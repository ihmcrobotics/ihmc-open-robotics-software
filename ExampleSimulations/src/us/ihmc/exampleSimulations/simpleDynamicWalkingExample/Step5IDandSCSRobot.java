package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import java.util.EnumMap;
import java.util.LinkedHashMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.JointNames;
import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.LinkNames;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
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

   //ID 
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());

   private final Vector3d jointAxes = new Vector3d(0.0, 1.0, 0.0); //rotate around Y-axis (for revolute joints)
   private final RigidBody elevator;

   private final SideDependentList<EnumMap<JointNames, OneDoFJoint>> allLegJoints = SideDependentList.createListOfEnumMaps(JointNames.class);
   private final LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint> idToSCSLegJointMap = new  LinkedHashMap<OneDoFJoint, OneDegreeOfFreedomJoint>(); //makes the SCS stuff available
   private final EnumMap<LinkNames, RigidBody> rigidBodies = new EnumMap<LinkNames, RigidBody>(LinkNames.class);
   private FramePoint bodyPosition;
   
   //SCS
   private final FloatingPlanarJoint bodyJointSCS ;

   //GENERAL
   private double cubeL = RobotParameters.BODY_DIMENSIONS.get(Axis.X);
   private double cubeW = RobotParameters.BODY_DIMENSIONS.get(Axis.Y);
   private double cubeH = RobotParameters.BODY_DIMENSIONS.get(Axis.Z);

   private double bodyMass = RobotParameters.MASSES.get(LinkNames.BODY_LINK);

   private double hipOffsetY = RobotParameters.BODY_DIMENSIONS.get(Axis.X);
   private double kneeOffsetZ = -RobotParameters.LENGTHS.get(LinkNames.UPPER_LINK) + 0.4;
   private final SixDoFJoint bodyJointID;
   
   
   
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
         //HIP (location, joint, and rigidBody) ID
         Vector3d hipOffset = new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0);
         RevoluteJoint hipJointID = ScrewTools.addRevoluteJoint(JointNames.HIP.getName(), bodyJointID.getSuccessor(), hipOffset, jointAxes);
         allLegJoints.get(robotSide).put(JointNames.HIP, hipJointID);
         createAndAttachCylinder(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         //KNEE ID
         Vector3d kneeOffset = new Vector3d(0.0, 0.0, kneeOffsetZ);
         PrismaticJoint kneeJointID = ScrewTools.addPrismaticJoint(JointNames.KNEE.getName(), hipJointID.getSuccessor(), kneeOffset, jointAxes);
         allLegJoints.get(robotSide).put(JointNames.KNEE, kneeJointID);
         createAndAttachCylinder(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);
      }
      
      /****************** SCS ROBOT ***********************/
      //BODY SCS
      bodyJointSCS = new FloatingPlanarJoint(JointNames.BODY.getName(), this);
      this.addRootJoint(bodyJointSCS);
      createAndAttachBodyLink(LinkNames.BODY_LINK, JointNames.BODY);

      for (RobotSide robotSide : RobotSide.values)
      {
         //HIP SCS
         PinJoint hipJointSCS = new PinJoint(JointNames.HIP.getName(), new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0), this, jointAxes);
         bodyJointSCS.addJoint(hipJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.HIP), hipJointSCS);
         createAndAttachCylinderLink(LinkNames.UPPER_LINK, JointNames.HIP, robotSide);

         //KNEE SCS
         SliderJoint kneeJointSCS = new SliderJoint(JointNames.KNEE.getName(), new Vector3d(0.0, 0.0, kneeOffsetZ), this, jointAxes);
         hipJointSCS.addJoint(kneeJointSCS);
         idToSCSLegJointMap.put(allLegJoints.get(robotSide).get(JointNames.KNEE), kneeJointSCS);
         createAndAttachCylinderLink(LinkNames.LOWER_LINK, JointNames.KNEE, robotSide);
      }   
   }

/**
 * Inertias
 */
   private Matrix3d createInertiaCube()
   {
      Matrix3d inertiaCube = new Matrix3d();   
      inertiaCube = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(cubeL, cubeW, cubeH, bodyMass);
      return inertiaCube;
   }
   
   private Matrix3d createInertiaCylinder(LinkNames linkName, JointNames jointName)
   {
      Matrix3d inertiaCylinder = new Matrix3d();
      inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(RobotParameters.MASSES.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.LENGTHS.get(jointName), Axis.Z);  
      return inertiaCylinder;
   }
 
   /**
    * Rigid Bodies and Links
    */

   /************************* ID ROBOT - Rigid bodies ********************************/
   private void createAndAttachBody(LinkNames linkName, SixDoFJoint bodyJointID)
   {
      Matrix3d inertiaCube = createInertiaCube();
      Vector3d comOffset = RobotParameters.COMs.get(linkName);
      System.out.println("linkName "+ linkName.getName() + "\nbodyJoint " + bodyJointID + "\ninertiaCube " + inertiaCube +  "\nbodyMass " +  bodyMass +"\ncomOffset "+ comOffset); //Check point
      rigidBodies.put(linkName, ScrewTools.addRigidBody(linkName.getName(), bodyJointID, inertiaCube, bodyMass, comOffset));
   }

   private void createAndAttachCylinder(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Matrix3d inertiaCylinder = createInertiaCylinder(linkName, jointName);
      Vector3d comOffset = RobotParameters.COMs.get(linkName);
      rigidBodies.put(linkName, ScrewTools.addRigidBody(linkName.getName(), allLegJoints.get(robotSide).get(jointName), inertiaCylinder, bodyMass, comOffset));
   }

   /************************* SCS ROBOT - Links ********************************/
   private void createAndAttachBodyLink(LinkNames linkName, JointNames jointName)
   {
      Link link = new Link(linkName.getName());
      Matrix3d inertiaCube = createInertiaCube();
      link.setMomentOfInertia(inertiaCube);
      link.setMass(RobotParameters.MASSES.get(linkName));
      link.setComOffset(RobotParameters.COMs.get(linkName));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (RobotParameters.LENGTHS.get(linkName) > 0.0)
         linkGraphics.addCylinder(RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
      else
      {
         linkGraphics.translate(0.0, 0.0, RobotParameters.LENGTHS.get(linkName));
         linkGraphics.addCylinder(-RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
      }

      link.setLinkGraphics(linkGraphics);
      bodyJointSCS.setLink(link);
   }

   private void createAndAttachCylinderLink(LinkNames linkName, JointNames jointName, RobotSide robotSide)
   {
      Link link = new Link(linkName.getName());
      Matrix3d inertiaCylinder = createInertiaCylinder(linkName, jointName);
      link.setMomentOfInertia(inertiaCylinder);
      link.setMass(RobotParameters.MASSES.get(linkName));
      link.setComOffset(RobotParameters.COMs.get(linkName));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (RobotParameters.LENGTHS.get(linkName) > 0.0)
         linkGraphics.addCylinder(RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
      else
      {
         linkGraphics.translate(0.0, 0.0, RobotParameters.LENGTHS.get(linkName));
         linkGraphics.addCylinder(-RobotParameters.LENGTHS.get(linkName), RobotParameters.RADII.get(linkName), RobotParameters.APPEARANCE.get(linkName));
      }

      link.setLinkGraphics(linkGraphics);
      idToSCSLegJointMap.get(allLegJoints.get(robotSide).get(jointName)).setLink(link);
   }

   /**
    * SCS Robot --> ID Robot. Send positions and velocities.
    */

   public void updateIDRobot()
   {
      bodyJointID.setPosition(bodyJointSCS.getQ_t1().getDoubleValue(), 0.0, bodyJointSCS.getQ_t2().getDoubleValue());
      bodyJointID.setRotation(0.0, bodyJointSCS.getQ_rot().getDoubleValue(), 0.0);
      
      double[] velocityArray = new double[6];
      velocityArray[0] = 0.0;                                         //yaw
      velocityArray[1] = bodyJointSCS.getQd_rot().getDoubleValue();   //pitch
      velocityArray[2] = 0.0;                                         //roll
      velocityArray[3] = bodyJointSCS.getQd_t1().getDoubleValue();    //x
      velocityArray[4] = 0.0;                                         //y
      velocityArray[5] = bodyJointSCS.getQd_t2().getDoubleValue();    //z
      DenseMatrix64F velocities = new DenseMatrix64F(6, 1, true, velocityArray);
      bodyJointID.setVelocity(velocities, 0);
  
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         idJoint.setQ(scsJoint.getQ().getDoubleValue());
         idJoint.setQd(scsJoint.getQD().getDoubleValue());
      }

      elevator.updateFramesRecursively();
      
      //Get the body point
      bodyPosition = new FramePoint();
      bodyPosition.setToZero(bodyJointID.getFrameAfterJoint());
      bodyPosition.changeFrame(worldFrame);
      
   }

   /**
    *  Copy the torques from the IDRobot to the SCSRobot so it is taken into account in SCS
    */
   public void applyTorques()
   {
      for (OneDoFJoint idJoint : idToSCSLegJointMap.keySet())
      {
         OneDegreeOfFreedomJoint scsJoint = idToSCSLegJointMap.get(idJoint);
         idJoint.setTau(scsJoint.getTau().getDoubleValue());
      }
   }

   public RigidBody getElevator()  //TODO what does this do?
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
   
   //Body
   public void getBodyPoint(Point3d bodyPointToPack) //TODO  double h = bodyPosition.getZ(); //Why pack it instead of doing this?
   {
      Point3d bodyPoint = this.bodyPosition.getPoint(); 
      bodyPointToPack.set(bodyPoint);
   }
   
//   public Quat4d getBodyPitch(Quat4d rotationToPack) //RotationFunctions
//   {
//      Quat4d bodyPitch = bodyJointID.getFrameAfterJoint().getRotation(rotationToPack);
//      return bodyPitch;
//   }
   
   //Knee
   public double getKneeVelocity(RobotSide robotSide)
   {
      double kneeVelocity = allLegJoints.get(robotSide).get(JointNames.KNEE).getQd();
      return kneeVelocity;
   }
   
   public void setKneeTau(RobotSide robotSide, double desiredKneeTau)
   {
      allLegJoints.get(robotSide).get(JointNames.KNEE).setTau(desiredKneeTau);
   }
   
   //Hip
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
