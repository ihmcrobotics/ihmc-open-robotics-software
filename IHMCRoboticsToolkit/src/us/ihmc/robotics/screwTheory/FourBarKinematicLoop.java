package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorFromFastRunner;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoop
{
   private static final boolean DEBUG = true;
   /*
    * Representation of the four bar with name correspondences:
    *   
    *              masterL
    *     master=A--------B
    *            |\      /|
    *            | \    / |
    *            |  \  /  |
    *            |   \/   |
    *            |   /\   |
    *            |  /  \  |
    *            | /    \ |
    *            |/      \|
    *            D--------C
    */

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private final String name;

   private final DoubleYoVariable masterJointQ;
   private final DoubleYoVariable masterJointQd;

   private final FramePoint masterJointAPosition, jointBPosition, jointCPosition, jointDPosition;
   private double masterLinkAB, BC, CD, DA;

   private FourBarCalculatorFromFastRunner fourBarCalculator;
   private double[] interiorAnglesAtZeroConfiguration = new double[4];

   private double maxValidMasterJointAngle, minValidMasterJointAngle;

   private final FrameVector vectorBC, vectorCD, vectorDA, vectorAB;
   
   private final FrameVector jointAxisInWorld;
   private final FrameVector tempVector = new FrameVector();
   private final FrameVector linkLengthVector = new FrameVector();
   
   private final FrameVector masterAxis, jointBAxis, jointCAxis, jointDAxis;
   
   private final FrameVector closurePointFromLastPassiveJointFrameVect;

   public FourBarKinematicLoop(String name, YoVariableRegistry registry, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d closurePointFromLastPassiveJoint)
   {
      this.name = name;
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;   
      closurePointFromLastPassiveJointFrameVect = new FrameVector(worldFrame, closurePointFromLastPassiveJoint);
      
      jointBPosition = new FramePoint();
      jointCPosition = new FramePoint();
      jointDPosition = new FramePoint();
      masterJointAPosition = new FramePoint();

      masterJointQ = new DoubleYoVariable(name + "MasterJointQ", registry);
      masterJointQ.set(masterJointA.getQ());

      masterJointQd = new DoubleYoVariable(name + "MasterJointQd", registry);
      masterJointQd.set(masterJointA.getQd());

      vectorBC = new FrameVector();
      vectorCD = new FrameVector();
      vectorDA = new FrameVector();
      vectorAB = new FrameVector();
     
      masterAxis = new FrameVector(masterJointA.getJointAxis());
      jointBAxis = new FrameVector(passiveJointB.getJointAxis());
      jointCAxis = new FrameVector(passiveJointC.getJointAxis());
      jointDAxis = new FrameVector(passiveJointD.getJointAxis());

      jointAxisInWorld = new FrameVector();

      checkJointAxesAreParallel();
      checkCorrectJointOrder();     
     
      initializeLinkVectors();     
      masterLinkAB = getLinkLength(vectorAB);
      BC = getLinkLength(vectorBC);
      CD = getLinkLength(vectorCD);
      DA = getLinkLength(closurePointFromLastPassiveJointFrameVect);

      if (DEBUG)
      {
         System.out.println("\nLink length debugging: \n");
         System.out.println("masterL L1 L2 L3 : " + masterLinkAB + ", " + BC + ", " + CD + ", " + DA);
      }

      verifyMasterJointLimits();
      setInteriorAngleOffsets();

      fourBarCalculator = new FourBarCalculatorFromFastRunner(masterLinkAB, BC, CD, DA);
      updateAnglesAndVelocities();
      
      if (DEBUG)
      {
         System.out.println("\nInitial joint angles debugging:\n\n"
               + "MasterQ: " + masterJointA.getQ() + "\njointBQ: " + passiveJointB.getQ() + "\njointCQ: " + passiveJointC.getQ() + "\njointDQ: " + passiveJointD.getQ() + "\n");         
      }
   }
   
   private void checkJointAxesAreParallel()
   {     
      masterAxis.changeFrame(worldFrame);
      jointBAxis.changeFrame(worldFrame);     
      jointCAxis.changeFrame(worldFrame);     
      jointDAxis.changeFrame(worldFrame);     
     
      // Both the exact same axis and a flipped axis are valid (eg: y and -y). So as long as the absolute value of the dot product is 1, the axis are parallel.
      if (Math.abs(masterAxis.dot(jointBAxis)) == 1 && Math.abs(masterAxis.dot(jointCAxis)) == 1 && Math.abs(masterAxis.dot(jointDAxis)) == 1)
      {
         jointAxisInWorld.set(masterAxis);
      }
      else
      {
         throw new RuntimeException("All joints in the four bar must rotate around the same axis!");
      }      
   }
   
   private void checkCorrectJointOrder()
   {
      if (masterJointA.getSuccessor() != passiveJointB.getPredecessor() || passiveJointB.getSuccessor() != passiveJointC.getPredecessor() || passiveJointC.getSuccessor() != passiveJointD.getPredecessor())
      {
         throw new RuntimeException("The joints that form the " + name + " four bar must be passed in clockwise or counterclockwise order");
      }
      
      if (DEBUG)
      {
         System.out.println("\nDebugging  check joint order:\n\nsuccessor \t predecessor\n"
               + masterJointA.getSuccessor() + "\t  " + passiveJointB.getPredecessor() + "\n" 
               + passiveJointB.getSuccessor() + "\t  " + passiveJointC.getPredecessor() + "\n"
               + passiveJointC.getSuccessor() + "\t  " + passiveJointD.getPredecessor() + "\n");
      }
   }
   
   private void initializeLinkVectors()
   {
      jointBPosition.setToZero(passiveJointB.getFrameAfterJoint());
      jointCPosition.setToZero(passiveJointC.getFrameAfterJoint());
      jointDPosition.setToZero(passiveJointD.getFrameAfterJoint());
      masterJointAPosition.setToZero(masterJointA.getFrameAfterJoint());
      
      jointBPosition.changeFrame(worldFrame);
      jointCPosition.changeFrame(worldFrame);
      jointDPosition.changeFrame(worldFrame);
      masterJointAPosition.changeFrame(worldFrame);
      
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDPosition, jointCPosition);
      vectorDA.sub(masterJointAPosition, jointDPosition);
      vectorAB.sub(jointBPosition, masterJointAPosition);
   }
   
   private void setInteriorAngleOffsets()
   {      
      vectorDA.normalize();
      vectorAB.normalize();
      vectorBC.normalize();
      vectorCD.normalize();

      interiorAnglesAtZeroConfiguration[0] = Math.PI - Math.acos(vectorDA.dot(vectorAB));
      interiorAnglesAtZeroConfiguration[1] = Math.PI - Math.acos(vectorAB.dot(vectorBC));
      interiorAnglesAtZeroConfiguration[2] = Math.PI - Math.acos(vectorBC.dot(vectorCD));
      interiorAnglesAtZeroConfiguration[3] = Math.PI - Math.acos(vectorCD.dot(vectorDA));

      if (DEBUG)
      {
         System.out.println("\nOffset angle debugging:\n");
         System.out.println("offset A = " + interiorAnglesAtZeroConfiguration[0]);
         System.out.println("offset B = " + interiorAnglesAtZeroConfiguration[1]);
         System.out.println("offset C = " + interiorAnglesAtZeroConfiguration[2]);
         System.out.println("offset D = " + interiorAnglesAtZeroConfiguration[3]);        
      }
   }
   
   /**
    * Projects the link onto the plane of the four bar
    */
   private double getLinkLength(FrameVector jointToJointVector)
   {
      tempVector.set(jointAxisInWorld);
      tempVector.normalize();
      tempVector.scale(jointAxisInWorld.dot(jointToJointVector));
      linkLengthVector.sub(jointToJointVector, tempVector);
      return linkLengthVector.length();
   }

   private void verifyMasterJointLimits()
   {
      maxValidMasterJointAngle = Math.acos((-(CD + DA) * (CD + DA) + masterLinkAB * masterLinkAB + BC * BC) / (2 * masterLinkAB * BC));
      minValidMasterJointAngle = Math.acos((-DA * DA + masterLinkAB * masterLinkAB + (BC + CD) * (BC + CD)) / (2 * masterLinkAB * (BC + CD)));

      if (masterJointA.getJointLimitLower() == Double.NEGATIVE_INFINITY || masterJointA.getJointLimitUpper() == Double.POSITIVE_INFINITY)
      {
         throw new RuntimeException("Must set the joint limits for the master joint of the " + name + " four bar.\nNote that for the given link lengths max angle is " + maxValidMasterJointAngle + "and min angle is" + minValidMasterJointAngle);
      }

      if (BC + masterLinkAB < CD + DA)
      {
         if (masterJointA.getJointLimitUpper() > maxValidMasterJointAngle)
         {
            throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
         }
      }
      else if (masterJointA.getJointLimitUpper() > Math.PI)
      {
         throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
      }

      if (BC + CD < masterLinkAB + DA)
      {
         if (masterJointA.getJointLimitLower() < minValidMasterJointAngle)
         {
            throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
         }
      }
      else if (masterJointA.getJointLimitLower() < 0.0)
      {
         throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
      }
   }

   public void updateAnglesAndVelocities() //TODO solve problem with this method
   {
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(masterJointA.getQ() + interiorAnglesAtZeroConfiguration[0], masterJointA.getQd());
      passiveJointB.setQ(fourBarCalculator.getAngleABC() + interiorAnglesAtZeroConfiguration[1]);
      passiveJointC.setQ(fourBarCalculator.getAngleBCD() + interiorAnglesAtZeroConfiguration[2]);
      passiveJointD.setQ(fourBarCalculator.getAngleCDA() + interiorAnglesAtZeroConfiguration[3]);
      passiveJointB.setQd(fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(fourBarCalculator.getAngleDtCDA());
   }
}
