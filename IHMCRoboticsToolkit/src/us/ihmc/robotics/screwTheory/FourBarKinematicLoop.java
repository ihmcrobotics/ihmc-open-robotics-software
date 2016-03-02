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
    *      masterJ--------J1
    *            |\      /|
    *            | \    / |
    *          L3|  \  /  |L1
    *            |   \/   |
    *            |   /\   |
    *            |  /  \  |
    *            | /    \ |
    *            |/      \|
    *           J3--------J2
    *                L2
    */

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RevoluteJoint masterJoint;
   private final PassiveRevoluteJoint passiveJoint1, passiveJoint2, passiveJoint3;
   private final String name;

   private final DoubleYoVariable masterJointQ;
   private final DoubleYoVariable masterJointQd;

   private final FramePoint masterJointPosition, joint1Position, joint2Position, joint3Position;
   private double masterL, L1, L2, L3;

   private FourBarCalculatorFromFastRunner fourBarCalculator;
   private double[] interiorAnglesAtZeroConfiguration = new double[4];

   private double maxValidMasterJointAngle, minValidMasterJointAngle;

   private final FrameVector vectorJoint1to2, vectorJoint2to3, vectorJoint3toMaster, vectorJointMasterto1;
   
   private final FrameVector jointAxisInWorld;
   private final FrameVector tempVector = new FrameVector();
   private final FrameVector linkLengthVector = new FrameVector();
   
   private final FrameVector masterAxis, joint1Axis, joint2Axis, joint3Axis;

   public FourBarKinematicLoop(String name, YoVariableRegistry registry, RevoluteJoint masterJoint, PassiveRevoluteJoint passiveJoint1, PassiveRevoluteJoint passiveJoint2, PassiveRevoluteJoint passiveJoint3, Vector3d closurePointFromLastPassiveJoint)
   {
      this.name = name;
      this.masterJoint = masterJoint;
      this.passiveJoint1 = passiveJoint1;
      this.passiveJoint2 = passiveJoint2;
      this.passiveJoint3 = passiveJoint3;
      
      joint1Position = new FramePoint();
      joint2Position = new FramePoint();
      joint3Position = new FramePoint();
      masterJointPosition = new FramePoint();

      masterJointQ = new DoubleYoVariable(name + "MasterJointQ", registry);
      masterJointQ.set(masterJoint.getQ());

      masterJointQd = new DoubleYoVariable(name + "MasterJointQd", registry);
      masterJointQd.set(masterJoint.getQd());

      vectorJoint1to2 = new FrameVector();
      vectorJoint2to3 = new FrameVector();
      vectorJoint3toMaster = new FrameVector();
      vectorJointMasterto1 = new FrameVector();
     
      masterAxis = new FrameVector(masterJoint.getJointAxis());
      joint1Axis = new FrameVector(passiveJoint1.getJointAxis());
      joint2Axis = new FrameVector(passiveJoint2.getJointAxis());
      joint3Axis = new FrameVector(passiveJoint3.getJointAxis());

      jointAxisInWorld = new FrameVector();

      checkJointAxesAreParallel();
      checkCorrectJointOrder();     
      initializeLinkVectors();
      
      masterL = getLinkLength(vectorJointMasterto1);
      L1 = getLinkLength(vectorJoint1to2);
      L2 = getLinkLength(vectorJoint2to3);
      L3 = closurePointFromLastPassiveJoint.length();

      if (DEBUG)
      {
         System.out.println("\nLink length debugging: \n");
         System.out.println("masterL L1 L2 L3 : " + masterL + ", " + L1 + ", " + L2 + ", " + L3);
      }

      verifyMasterJointLimits();
      setInteriorAngleOffsets();

      fourBarCalculator = new FourBarCalculatorFromFastRunner(masterL, L1, L2, L3);
      updateAnglesAndVelocities();
      
      if (DEBUG)
      {
         System.out.println("\nInitial joint angles debugging:\n\n"
               + "MasterQ: " + masterJoint.getQ() + "\njoint1Q: " + passiveJoint1.getQ() + "\njoint2Q: " + passiveJoint2.getQ() + "\njoint3Q: " + passiveJoint3.getQ() + "\n");         
      }
   }
   
   private void checkJointAxesAreParallel()
   {     
      masterAxis.changeFrame(worldFrame);
      joint1Axis.changeFrame(worldFrame);     
      joint2Axis.changeFrame(worldFrame);     
      joint3Axis.changeFrame(worldFrame);     
     
      // Both the exact same axis and a flipped axis are valid (eg: y and -y). So as long as the absolute value of the dot product is 1, the axis are parallel.
      if (Math.abs(masterAxis.dot(joint1Axis)) == 1 && Math.abs(masterAxis.dot(joint2Axis)) == 1 && Math.abs(masterAxis.dot(joint3Axis)) == 1)
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
      if (masterJoint.getSuccessor() != passiveJoint1.getPredecessor() || passiveJoint1.getSuccessor() != passiveJoint2.getPredecessor() || passiveJoint2.getSuccessor() != passiveJoint3.getPredecessor())
      {
         throw new RuntimeException("The joints that form the " + name + " four bar must be passed in clockwise or counterclockwise order");
      }
      
      if (DEBUG)
      {
         System.out.println("\nDebugging  check joint order:\n\nsuccessor \t predecessor\n"
               + masterJoint.getSuccessor() + "\t  " + passiveJoint1.getPredecessor() + "\n" 
               + passiveJoint1.getSuccessor() + "\t  " + passiveJoint2.getPredecessor() + "\n"
               + passiveJoint2.getSuccessor() + "\t  " + passiveJoint3.getPredecessor() + "\n");
      }
   }
   
   private void initializeLinkVectors()
   {
      joint1Position.setToZero(passiveJoint1.getFrameAfterJoint());
      joint2Position.setToZero(passiveJoint2.getFrameAfterJoint());
      joint3Position.setToZero(passiveJoint3.getFrameAfterJoint());
      masterJointPosition.setToZero(masterJoint.getFrameAfterJoint());
      
      joint1Position.changeFrame(worldFrame);
      joint2Position.changeFrame(worldFrame);
      joint3Position.changeFrame(worldFrame);
      masterJointPosition.changeFrame(worldFrame);
      
      vectorJoint1to2.sub(joint2Position, joint1Position);
      vectorJoint2to3.sub(joint3Position, joint2Position);
      vectorJoint3toMaster.sub(masterJointPosition, joint3Position);
      vectorJointMasterto1.sub(joint1Position, masterJointPosition);
   }
   
   private void setInteriorAngleOffsets()
   {      
      vectorJoint3toMaster.normalize();
      vectorJointMasterto1.normalize();
      vectorJoint1to2.normalize();
      vectorJoint2to3.normalize();

      interiorAnglesAtZeroConfiguration[0] = Math.PI - Math.acos(vectorJoint3toMaster.dot(vectorJointMasterto1));
      interiorAnglesAtZeroConfiguration[1] = Math.PI - Math.acos(vectorJointMasterto1.dot(vectorJoint1to2));
      interiorAnglesAtZeroConfiguration[2] = Math.PI - Math.acos(vectorJoint1to2.dot(vectorJoint2to3));
      interiorAnglesAtZeroConfiguration[3] = Math.PI - Math.acos(vectorJoint2to3.dot(vectorJoint3toMaster));

      if (DEBUG)
      {
         System.out.println("\nOffset angle debugging:\n");
         System.out.println("offset 0 = " + interiorAnglesAtZeroConfiguration[0]);
         System.out.println("offset 1 = " + interiorAnglesAtZeroConfiguration[1]);
         System.out.println("offset 2 = " + interiorAnglesAtZeroConfiguration[2]);
         System.out.println("offset 3 = " + interiorAnglesAtZeroConfiguration[3]);        
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
      maxValidMasterJointAngle = Math.acos((-(L2 + L3) * (L2 + L3) + masterL * masterL + L1 * L1) / (2 * masterL * L1));
      minValidMasterJointAngle = Math.acos((-L3 * L3 + masterL * masterL + (L1 + L2) * (L1 + L2)) / (2 * masterL * (L1 + L2)));

      if (masterJoint.getJointLimitLower() == Double.NEGATIVE_INFINITY || masterJoint.getJointLimitUpper() == Double.POSITIVE_INFINITY)
      {
         throw new RuntimeException("Must set the joint limits for the master joint of the " + name + " four bar.\nNote that for the given link lengths max angle is " + maxValidMasterJointAngle + "and min angle is" + minValidMasterJointAngle);
      }

      if (L1 + masterL < L2 + L3)
      {
         if (masterJoint.getJointLimitUpper() > maxValidMasterJointAngle)
         {
            throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle + " to avoid flipping, but was set to " + masterJoint.getJointLimitUpper());
         }
      }
      else if (masterJoint.getJointLimitUpper() > Math.PI)
      {
         throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle + " to avoid flipping, but was set to " + masterJoint.getJointLimitUpper());
      }

      if (L1 + L2 < masterL + L3)
      {
         if (masterJoint.getJointLimitLower() < minValidMasterJointAngle)
         {
            throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle + " to avoid flipping, but was set to " + masterJoint.getJointLimitLower());
         }
      }
      else if (masterJoint.getJointLimitLower() < 0.0)
      {
         throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle + " to avoid flipping, but was set to " + masterJoint.getJointLimitLower());
      }
   }

   public void updateAnglesAndVelocities() //TODO solve problem with this method
   {
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(masterJoint.getQ() + interiorAnglesAtZeroConfiguration[0], masterJoint.getQd());
      passiveJoint1.setQ(fourBarCalculator.getAngleABC() + interiorAnglesAtZeroConfiguration[1]);
      passiveJoint2.setQ(fourBarCalculator.getAngleBCD() + interiorAnglesAtZeroConfiguration[2]);
      passiveJoint3.setQ(fourBarCalculator.getAngleCDA() + interiorAnglesAtZeroConfiguration[3]);
      passiveJoint1.setQd(fourBarCalculator.getAngleDtABC());
      passiveJoint2.setQd(fourBarCalculator.getAngleDtBCD());
      passiveJoint3.setQd(fourBarCalculator.getAngleDtCDA());
   }
}
