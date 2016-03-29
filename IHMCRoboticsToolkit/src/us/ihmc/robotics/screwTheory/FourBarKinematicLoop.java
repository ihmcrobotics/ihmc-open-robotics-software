package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoop
{
   /*
    * Representation of the four bar with name correspondences.
    * This name convention matches the one used in the FourBarCalculator from fastRunner
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
   private static final boolean DEBUG = false;

   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD;
   private final Vector3d closurePointFromLastPassiveJointVect;  
   
   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   
   private final FourBarKinematicLoopJacobianSolver fourBarJacobianSolver;
   private final Vector3d jacobian, outputJointLinearVelocitiesToPack;
   
   private final double[] passiveInteriorAnglesAtZeroConfiguration = new double[3];
   private final double[] passiveJointSigns = new double[3];
   
   private final ReferenceFrame frameBeforeFourBarWithZAlongJointAxis;
   
   public FourBarKinematicLoop(String name, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB,
         PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d closurePointFromLastPassiveJointInJointDFrame, boolean recomputeJointLimits)
   {
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;
      this.closurePointFromLastPassiveJointVect = new Vector3d(closurePointFromLastPassiveJointInJointDFrame);
      
      // Rotation axis
      FrameVector masterJointAxis = masterJointA.getJointAxis();
      masterJointAxis.changeFrame(masterJointA.getFrameBeforeJoint());
      frameBeforeFourBarWithZAlongJointAxis = ReferenceFrame.constructReferenceFrameFromPointAndAxis(name + "FrameWithZAlongJointAxis", new FramePoint(masterJointA.getFrameBeforeJoint()), Axis.Z, masterJointAxis);

      FrameVector masterAxis = masterJointA.getJointAxis();
      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();
      FourBarKinematicLoopTools.checkJointAxesAreParallelAndSetJointAxis(masterAxis, jointBAxis, jointCAxis, jointDAxis);
      
      // Joint order
      FourBarKinematicLoopTools.checkCorrectJointOrder(name, masterJointA, passiveJointB, passiveJointC, passiveJointD);
      
      // Go to zero configuration
      masterJointA.setQ(0.0);
      passiveJointB.setQ(0.0);
      passiveJointC.setQ(0.0);
      passiveJointD.setQ(0.0);

      // Link lengths
      FrameVector2d vectorBCProjected = new FrameVector2d();
      FrameVector2d vectorCDProjected = new FrameVector2d();
      FrameVector2d vectorDAProjected = new FrameVector2d();
      FrameVector2d vectorABProjected = new FrameVector2d();
      computeJointToJointVectorProjectedOntoFourBarPlane(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected);

      // Calculator
      fourBarCalculator = createFourBarCalculator(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected);
      
      // Jacobian solver
      fourBarJacobianSolver = new FourBarKinematicLoopJacobianSolver(fourBarCalculator);
      jacobian = fourBarJacobianSolver.computeJacobian(masterJointA.getQ());
      outputJointLinearVelocitiesToPack = new Vector3d();
      
      // Initialize interior angle offsets and signs
      initializeInteriorAnglesAtZeroConfigurationAndJointSigns(vectorDAProjected, vectorABProjected, vectorBCProjected, vectorCDProjected);
       
      // Find the most conservative limits for the master joint angle (A) and reset them if necessary
      if (recomputeJointLimits) 
      {
         /* - If the limits for B, C, and/or D are given and are more restrictive than those of A crop the value of Amax and/or Amin. 
          * - Else if the limits given for A are the most restrictive of all, keep them.
          * - Else set the limits to the value given by the calculator. 
          */
         double minValidMasterJointAngle = computeMinValidMasterJointAngle(masterJointA, passiveJointB, passiveJointC, passiveJointD);
         double maxValidMasterJointAngle = computeMaxValidMasterJointAngle(masterJointA, passiveJointB, passiveJointC, passiveJointD);         

         masterJointA.setJointLimitLower(minValidMasterJointAngle);
         masterJointA.setJointLimitUpper(maxValidMasterJointAngle);
         
         if(DEBUG)
         {
            System.out.println("NOTE: The master joint limits have been set to " + minValidMasterJointAngle + " and " + maxValidMasterJointAngle);            
         }
      }
      else
      {
         FourBarKinematicLoopTools.verifyMasterJointLimits(name, masterJointA, fourBarCalculator);

         if (DEBUG)
         {
            System.out.println("\nMax master joint angle: " + masterJointA.getJointLimitUpper());
            System.out.println("Min master joint angle: " + masterJointA.getJointLimitLower());
         }
      }
      
      if (DEBUG)
      {
         double qA = masterJointA.getQ();
         double qB = passiveJointB.getQ();
         double qC = passiveJointC.getQ();
         double qD = passiveJointD.getQ();
         System.out.println("\nInitial joint angles debugging:\n\n" + "MasterQ: " + qA + "\njointBQ: " + qB + "\njointCQ: " + qC + "\njointDQ: " + qD + "\n");
      }
   }

   private void computeJointToJointVectorProjectedOntoFourBarPlane(FrameVector2d vectorBCProjectedToPack, FrameVector2d vectorCDProjectedToPack, FrameVector2d vectorDAProjectedToPack, FrameVector2d vectorABProjectedToPack)
   {
      FramePoint masterJointAPosition = new FramePoint(masterJointA.getFrameAfterJoint());
      FramePoint jointBPosition = new FramePoint(passiveJointB.getFrameAfterJoint());
      FramePoint jointCPosition = new FramePoint(passiveJointC.getFrameAfterJoint());
      FramePoint jointDPosition = new FramePoint(passiveJointD.getFrameAfterJoint());

      jointBPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      masterJointAPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      FrameVector vectorAB = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorBC = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorCD = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorDA = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);

      vectorAB.sub(jointBPosition, masterJointAPosition);
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDPosition, jointCPosition);
      vectorDA.setIncludingFrame(passiveJointD.getFrameAfterJoint(), closurePointFromLastPassiveJointVect);
      vectorDA.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      
      vectorBCProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorBC);
      vectorCDProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorCD);
      vectorDAProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorDA);
      vectorABProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorAB);
      
      if (DEBUG)
      {
         System.out.println("\nJoint to joint vectors debugging:\n");
         System.out.println("vector ab = " + vectorAB);
         System.out.println("vector bc = " + vectorBC);
         System.out.println("vector cd = " + vectorCD);
         System.out.println("vector da = " + vectorDA);
      }
   }

   private FourBarCalculatorWithDerivatives createFourBarCalculator(FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected, FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected)
   {
      double masterLinkAB = vectorABProjected.length();
      double BC = vectorBCProjected.length();
      double CD = vectorCDProjected.length();
      double DA = vectorDAProjected.length();
      
      if (DEBUG)
      {
         System.out.println("\nLink length debugging: \n");
         System.out.println("masterLinkAB BC CD DA : " + masterLinkAB + ", " + BC + ", " + CD + ", " + DA);
      }

      FourBarCalculatorWithDerivatives fourBarCalculator = new FourBarCalculatorWithDerivatives(DA, masterLinkAB, BC, CD);

      return fourBarCalculator;
   }

   private void initializeInteriorAnglesAtZeroConfigurationAndJointSigns(FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected,
         FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected)
   {
      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();

      jointBAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDAxis.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      Vector2d tempVectorAB = new Vector2d();
      Vector2d tempVectorBC = new Vector2d();
      Vector2d tempVectorCD = new Vector2d();
      Vector2d tempVectorDA = new Vector2d();
      
      vectorABProjected.get(tempVectorAB);
      vectorBCProjected.get(tempVectorBC);
      vectorCDProjected.get(tempVectorCD);
      vectorDAProjected.get(tempVectorDA);
      
      passiveInteriorAnglesAtZeroConfiguration[0] = Math.PI + jointBAxis.getZ() * AngleTools.angleMinusPiToPi(tempVectorAB, tempVectorBC);
      passiveInteriorAnglesAtZeroConfiguration[1] = Math.PI + jointCAxis.getZ() * AngleTools.angleMinusPiToPi(tempVectorBC, tempVectorCD);
      passiveInteriorAnglesAtZeroConfiguration[2] = Math.PI + jointDAxis.getZ() * AngleTools.angleMinusPiToPi(tempVectorCD, tempVectorDA);
      passiveJointSigns[0] = jointBAxis.getZ();
      passiveJointSigns[1] = jointCAxis.getZ();
      passiveJointSigns[2] = jointDAxis.getZ();
      
      if (DEBUG)
      {  
         System.out.println("\nOffset angle debugging:\n");
         System.out.println("offset B = " + passiveInteriorAnglesAtZeroConfiguration[0]);
         System.out.println("offset C = " + passiveInteriorAnglesAtZeroConfiguration[1]);
         System.out.println("offset D = " + passiveInteriorAnglesAtZeroConfiguration[2]);
         System.out.println("joint sign B = " + passiveJointSigns[0]);
         System.out.println("joint sign C = " + passiveJointSigns[1]);
         System.out.println("joint sign D = " + passiveJointSigns[2]);
      }
   }
   
   /**
    * Clips the min master joint angle if the lower limit for any of the joints that are passed in is more restrictive
    */
   private double computeMinValidMasterJointAngle(RevoluteJoint masterJointA, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC, PassiveRevoluteJoint jointD)
   {
      double minValidMasterJointAngle = fourBarCalculator.getMinDAB();

      if (masterJointA.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double minAngleASetByUser = masterJointA.getJointLimitLower();

         if (MathTools.isInsideBoundsExclusive(minAngleASetByUser, 0.0, Math.PI))
         {
            minValidMasterJointAngle = Math.max(minAngleASetByUser, minValidMasterJointAngle);
         }
      }
      
      if (jointB.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double interiorAngleBLimit0 = convertJointAngleToInteriorAngle(jointB.getJointLimitLower(), 0);
         double interiorAngleBLimit1 = convertJointAngleToInteriorAngle(jointB.getJointLimitUpper(), 0);
         double maxAngleB = Math.max(interiorAngleBLimit0, interiorAngleBLimit1);

         if (MathTools.isInsideBoundsExclusive(maxAngleB, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(maxAngleB);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointC.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double interiorAngleCLimit0 = convertJointAngleToInteriorAngle(jointC.getJointLimitLower(), 0);
         double interiorAngleCLimit1 = convertJointAngleToInteriorAngle(jointC.getJointLimitUpper(), 0);
         double minAngleC = Math.min(interiorAngleCLimit0, interiorAngleCLimit1);
         
         if (MathTools.isInsideBoundsExclusive(minAngleC, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(minAngleC);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointD.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double interiorAngleDLimit0 = convertJointAngleToInteriorAngle(jointD.getJointLimitLower(), 0);
         double interiorAngleDLimit1 = convertJointAngleToInteriorAngle(jointD.getJointLimitUpper(), 0);
         double maxAngleD = Math.max(interiorAngleDLimit0, interiorAngleDLimit1);

         if (MathTools.isInsideBoundsExclusive(maxAngleD, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(maxAngleD);
            minValidMasterJointAngle = Math.max(minValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      return minValidMasterJointAngle;
   }
   
   /**
    * Clips the max master joint angle if the upper limit for any of the joints that are passed in is more restrictive
    */
   private double computeMaxValidMasterJointAngle(RevoluteJoint masterJointA, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC, PassiveRevoluteJoint jointD)
   {
      double maxValidMasterJointAngle = fourBarCalculator.getMaxDAB();

      if (masterJointA.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double maxAngleASetByUser = masterJointA.getJointLimitUpper();

         if (MathTools.isInsideBoundsExclusive(maxAngleASetByUser, 0.0, Math.PI))
         {
            maxValidMasterJointAngle = Math.min(maxAngleASetByUser, maxValidMasterJointAngle);
         }
      }
      
      if (jointB.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double interiorAngleBLimit0 = convertJointAngleToInteriorAngle(jointB.getJointLimitLower(), 0);
         double interiorAngleBLimit1 = convertJointAngleToInteriorAngle(jointB.getJointLimitUpper(), 0);
         double minAngleB = Math.min(interiorAngleBLimit0, interiorAngleBLimit1);

         if (MathTools.isInsideBoundsExclusive(minAngleB, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(minAngleB);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointC.getJointLimitUpper() != Double.POSITIVE_INFINITY)
      {
         double interiorAngleCLimit0 = convertJointAngleToInteriorAngle(jointC.getJointLimitLower(), 0);
         double interiorAngleCLimit1 = convertJointAngleToInteriorAngle(jointC.getJointLimitUpper(), 0);
         double maxAngleC = Math.max(interiorAngleCLimit0, interiorAngleCLimit1);
         
         if (MathTools.isInsideBoundsExclusive(maxAngleC, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(maxAngleC);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      if (jointD.getJointLimitLower() != Double.NEGATIVE_INFINITY)
      {
         double interiorAngleDLimit0 = convertJointAngleToInteriorAngle(jointD.getJointLimitLower(), 0);
         double interiorAngleDLimit1 = convertJointAngleToInteriorAngle(jointD.getJointLimitUpper(), 0);
         double minAngleD = Math.min(interiorAngleDLimit0, interiorAngleDLimit1);
         
         if (MathTools.isInsideBoundsExclusive(minAngleD, 0.0, Math.PI))
         {
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(minAngleD);
            maxValidMasterJointAngle = Math.min(maxValidMasterJointAngle, fourBarCalculator.getAngleDAB());
         }
      }

      return maxValidMasterJointAngle;
   }
   
   public void update()
   {
      double currentMasterJointA = masterJointA.getQ();
      if (currentMasterJointA < masterJointA.getJointLimitLower() || currentMasterJointA > masterJointA.getJointLimitUpper())
      {
         throw new RuntimeException(
               masterJointA.getName() + " is set outside of its bounds [" + masterJointA.getJointLimitLower() + ", " + masterJointA.getJointLimitUpper() + "]");
      }
      
      fourBarCalculator.updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(masterJointA.getQ(), masterJointA.getQd(), masterJointA.getQdd());
      
      fourBarJacobianSolver.computeJacobian(masterJointA.getQ());
      fourBarJacobianSolver.solveLinearVelFromAngularVel(jacobian, fourBarCalculator.getAngleDtDAB(), outputJointLinearVelocitiesToPack);
      
      System.out.println("\nJacobian debugging: \n" + outputJointLinearVelocitiesToPack);
      
      passiveJointB.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleABC(), 0));
      passiveJointC.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleBCD(), 1));
      passiveJointD.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleCDA(), 2));
      
      passiveJointB.setQd(passiveJointSigns[0] * fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(passiveJointSigns[1] * fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(passiveJointSigns[2] * fourBarCalculator.getAngleDtCDA());
      
      passiveJointB.setQdd(passiveJointSigns[0] * fourBarCalculator.getAngleDt2ABC());
      passiveJointC.setQdd(passiveJointSigns[1] * fourBarCalculator.getAngleDt2BCD());
      passiveJointD.setQdd(passiveJointSigns[2] * fourBarCalculator.getAngleDt2CDA());
   }
   
   /**
    * @param interiorAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->B, 1->C, 2->D
    * @return
    */
   private double convertInteriorAngleToJointAngle(double interiorAngle, int passiveJointIndex)
   {
      return passiveJointSigns[passiveJointIndex] * (interiorAngle - passiveInteriorAnglesAtZeroConfiguration[passiveJointIndex]);
   }
   
   /**
    * @param interiorAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->B, 1->C, 2->D
    * @return
    */
   private double convertJointAngleToInteriorAngle(double jointAngle, int passiveJointIndex)
   {
      return passiveInteriorAnglesAtZeroConfiguration[passiveJointIndex] + passiveJointSigns[passiveJointIndex] * jointAngle;
   }

   public PassiveRevoluteJoint getPassiveRevoluteJointB()
   {
      return passiveJointB;
   }
   
   public PassiveRevoluteJoint getPassiveRevoluteJointC()
   {
      return passiveJointC;
   }
   
   public PassiveRevoluteJoint getPassiveRevoluteJointD()
   {
      return passiveJointD;
   }
}
