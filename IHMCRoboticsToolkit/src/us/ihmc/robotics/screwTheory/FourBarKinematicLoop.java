package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.Collections;

public class FourBarKinematicLoop
{
   /*
    * Representation of the four bar with name correspondences. This name
    * convention matches the one used in the FourBarCalculatorWithDerivatives
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
    *            
    * Note: zero configuration corresponds to vertical links. Positive angles clockwise.             
    */
   
   private static final boolean DEBUG = false;
   private static final boolean DEFAULT_OUTPUT_JOINT = true; //temporary variable to test loop while it's not part of a larger kinematic chain
   private static final double eps = 1e-7;

   private final String name;
   private final RevoluteJoint masterJointA;
   private final PassiveRevoluteJoint passiveJointB, passiveJointC, passiveJointD, fourBarOutputJoint;
   private final Vector3d jointDInJointABeforeFrame;

   private final FourBarCalculatorWithDerivatives fourBarCalculator;

   private final FourBarKinematicLoopJacobianSolver fourBarJacobianSolver;
   private DenseMatrix64F jacobian, outputJointVelocitiesToPack;

   private final double[] interiorAnglesAtZeroConfiguration = new double[4];
   private final double[] jointSigns = new double[4];

   private final ReferenceFrame frameBeforeFourBarWithZAlongJointAxis;
   
   private final boolean fourBarIsClockwise;

   /**
    * @param name
    * @param masterJointA
    * @param passiveJointB
    * @param passiveJointC
    * @param passiveJointD
    * @param jointDInJointABeforeFrame The position in A's frame to which D is rigidly attached
    * @param recomputeJointLimits
    *
    * <dt><b>Precondition:</b><dd>
    * The cartesian positions of masterJointA, passiveJointB, passiveJointC, and passiveJointD must form a convex quadrilateral when their joint angles are 0. <br>
    * Additionally, the cartesian positions of masterJointA, passiveJointB, passiveJointC and jointDInJointABeforeFrame must form a convex quadrilateral when their joint angles are 0
    *
    */
   public FourBarKinematicLoop(String name, RevoluteJoint masterJointA, PassiveRevoluteJoint passiveJointB, PassiveRevoluteJoint passiveJointC, PassiveRevoluteJoint passiveJointD, Vector3d jointDInJointABeforeFrame,
         boolean recomputeJointLimits)
   {
      this.name = name;
      this.masterJointA = masterJointA;
      this.passiveJointB = passiveJointB;
      this.passiveJointC = passiveJointC;
      this.passiveJointD = passiveJointD;
      this.jointDInJointABeforeFrame = new Vector3d(jointDInJointABeforeFrame);
      
      // Rotation axis
      FrameVector masterJointAxis = masterJointA.getJointAxis();
      masterJointAxis.changeFrame(masterJointA.getFrameBeforeJoint());
      frameBeforeFourBarWithZAlongJointAxis = ReferenceFrame.constructReferenceFrameFromPointAndAxis(name + "FrameWithZAlongJointAxis", new FramePoint(masterJointA.getFrameBeforeJoint()), Axis.Z, masterJointAxis);
   
      FrameVector masterAxis = masterJointA.getJointAxis();
      FrameVector jointBAxis = passiveJointB.getJointAxis();
      FrameVector jointCAxis = passiveJointC.getJointAxis();
      FrameVector jointDAxis = passiveJointD.getJointAxis();
      FourBarKinematicLoopTools.checkJointAxesAreParallel(masterAxis, jointBAxis, jointCAxis, jointDAxis);

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
      FrameVector2d vectorDAClosurePointProjected = new FrameVector2d();
      FrameVector2d vectorCDClosurePointProjected = new FrameVector2d();
      computeJointToJointVectorProjectedOntoFourBarPlane(vectorBCProjected, vectorCDProjected, vectorDAProjected, vectorABProjected,
            vectorDAClosurePointProjected, vectorCDClosurePointProjected);

      // Check that the fourbar is convex in its zero angle configuration and it's orientation (CW or CCW)
      FourBarKinematicLoopTools
            .checkFourBarConvexityAndOrientation(vectorABProjected, vectorBCProjected, vectorCDClosurePointProjected, vectorDAClosurePointProjected);
      fourBarIsClockwise = FourBarKinematicLoopTools
            .checkFourBarConvexityAndOrientation(vectorABProjected, vectorBCProjected, vectorCDProjected, vectorDAProjected);

      // Calculator
      fourBarCalculator = createFourBarCalculator(vectorBCProjected, vectorCDProjected, vectorDAClosurePointProjected, vectorABProjected);
      
      // Set output joint     
      if(DEFAULT_OUTPUT_JOINT)
      {
         fourBarOutputJoint = passiveJointC;

         if(DEBUG)
         {
            System.out.println("\nNote! Output joint set to default: " + fourBarOutputJoint.name + "\n");
         }
      }
      else
      {
         fourBarOutputJoint = FourBarKinematicLoopTools.setFourBarOutputJoint(passiveJointB, passiveJointC, passiveJointD);
      }   

      // Jacobian        
      fourBarJacobianSolver = new FourBarKinematicLoopJacobianSolver(fourBarCalculator, getJointsForJacobianCalculation(fourBarOutputJoint), fourBarOutputJoint);
      jacobian = fourBarJacobianSolver.computeJacobian(fourBarOutputJoint);      
      
      // Initialize interior angle offsets and signs
      initializeInteriorAnglesAtZeroConfigurationAndJointSigns(vectorDAClosurePointProjected, vectorABProjected, vectorBCProjected, vectorCDProjected);
      
      // Find the most conservative limits for the master joint angle (A) and reset them if necessary
      if (recomputeJointLimits)
      {
         /*
          * - If the limits for B, C, and/or D are given and are more restrictive than those of A crop the value of Amax and/or Amin. 
          * - Else if the limits given for A are the most restrictive of all, keep them. 
          * - Else set the limits to the value given by the calculator.
          */
         double[] masterJointBounds = computeMinAndMaxValidMasterJointAngles(masterJointA, passiveJointB, passiveJointC, passiveJointD);

         masterJointA.setJointLimitLower(masterJointBounds[0]);
         masterJointA.setJointLimitUpper(masterJointBounds[1]);

         if (DEBUG)
         {
            System.out.println("NOTE: The master joint limits have been set to " + masterJointBounds[0] + " and " + masterJointBounds[1]);
         }
      }
      else
      {
         verifyMasterJointLimits();

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

    private void computeJointToJointVectorProjectedOntoFourBarPlane(FrameVector2d vectorBCProjectedToPack, FrameVector2d vectorCDProjectedToPack,
         FrameVector2d vectorDAProjectedToPack, FrameVector2d vectorABProjectedToPack, FrameVector2d vectorDAClosurePointProjectedToPack,
         FrameVector2d vectorCDClosurePointProjectedToPack)
   {
      FramePoint masterJointAPosition = new FramePoint(masterJointA.getFrameAfterJoint());
      FramePoint jointBPosition = new FramePoint(passiveJointB.getFrameAfterJoint());
      FramePoint jointCPosition = new FramePoint(passiveJointC.getFrameAfterJoint());
      FramePoint jointDZeroAnglePosition = new FramePoint(passiveJointD.getFrameAfterJoint());
      FramePoint jointDClosedLoopPosition = new FramePoint(masterJointA.getFrameBeforeJoint(), jointDInJointABeforeFrame);

      if(DEBUG)
         System.out.println("\njointDInJointABeforeFrame = " + jointDInJointABeforeFrame + "\n");

      if(DEBUG)
         System.out.println("\njointDClosedLoopPosition = " + jointDClosedLoopPosition + "\n");

      jointBPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointCPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDZeroAnglePosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      jointDClosedLoopPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);
      masterJointAPosition.changeFrame(frameBeforeFourBarWithZAlongJointAxis);

      FrameVector vectorAB = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorBC = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorCD = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorDA = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorDAClosurePoint = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);
      FrameVector vectorCDClosurePoint = new FrameVector(frameBeforeFourBarWithZAlongJointAxis);

      vectorAB.sub(jointBPosition, masterJointAPosition);
      vectorBC.sub(jointCPosition, jointBPosition);
      vectorCD.sub(jointDZeroAnglePosition, jointCPosition);
      vectorDA.sub(masterJointAPosition, jointDZeroAnglePosition);
      vectorDAClosurePoint.sub(masterJointAPosition, jointDClosedLoopPosition);
      vectorCDClosurePoint.sub(jointDClosedLoopPosition, jointCPosition);

      vectorBCProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorBC);
      vectorCDProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorCD);
      vectorDAProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorDA);
      vectorABProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorAB);
      vectorDAClosurePointProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorDAClosurePoint);
      vectorCDClosurePointProjectedToPack.setByProjectionOntoXYPlaneIncludingFrame(vectorCDClosurePoint);

      if (DEBUG)
      {
         System.out.println("\nJoint to joint vectors debugging:\n");
         System.out.println("vector ab = " + vectorAB);
         System.out.println("vector bc = " + vectorBC);
         System.out.println("vector cd = " + vectorCD);
         System.out.println("vector da = " + vectorDA);
      }
   }

   private FourBarCalculatorWithDerivatives createFourBarCalculator(FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected,
         FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected)
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

   private void initializeInteriorAnglesAtZeroConfigurationAndJointSigns(FrameVector2d vectorDAProjected, FrameVector2d vectorABProjected, FrameVector2d vectorBCProjected, FrameVector2d vectorCDProjected)
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

      if (fourBarIsClockwise)
      {
         interiorAnglesAtZeroConfiguration[0] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorAB, tempVectorDA);
         interiorAnglesAtZeroConfiguration[1] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorBC, tempVectorAB);
         interiorAnglesAtZeroConfiguration[2] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorCD, tempVectorBC);
         interiorAnglesAtZeroConfiguration[3] = Math.PI;

         jointSigns[0] = 1.0;
         jointSigns[1] = jointBAxis.getZ();
         jointSigns[2] = jointCAxis.getZ();
         jointSigns[3] = jointDAxis.getZ();
      }
      else
      {
         interiorAnglesAtZeroConfiguration[0] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorDA, tempVectorAB);
         interiorAnglesAtZeroConfiguration[1] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorAB, tempVectorBC);
         interiorAnglesAtZeroConfiguration[2] = Math.PI - AngleTools.angleMinusPiToPi(tempVectorBC, tempVectorCD);
         interiorAnglesAtZeroConfiguration[3] = Math.PI;

         jointSigns[0] = - 1.0;
         jointSigns[1] = - jointBAxis.getZ();
         jointSigns[2] = - jointCAxis.getZ();
         jointSigns[3] = - jointDAxis.getZ();
      }

      if (DEBUG)
      {
         System.out.println("\nOffset angle debugging:");
         System.out.println("offset A = " + interiorAnglesAtZeroConfiguration[0] / Math.PI);
         System.out.println("offset B = " + interiorAnglesAtZeroConfiguration[1] / Math.PI);
         System.out.println("offset C = " + interiorAnglesAtZeroConfiguration[2] / Math.PI);
         System.out.println("offset D = " + interiorAnglesAtZeroConfiguration[3] / Math.PI);
         System.out.println("joint sign A = " + jointSigns[0]);
         System.out.println("joint sign B = " + jointSigns[1]);
         System.out.println("joint sign C = " + jointSigns[2]);
         System.out.println("joint sign D = " + jointSigns[2]);
      }
   }

   /**
    * Clips the min/max master joint angles if the lower/upper limit for any of the joints that are passed in is more restrictive
    */
   private double[] computeMinAndMaxValidMasterJointAngles(RevoluteJoint masterJointA, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC,
         PassiveRevoluteJoint jointD)
   {
      ArrayList<Double> lowerBounds = new ArrayList<Double>();
      ArrayList<Double> upperBounds = new ArrayList<Double>();

      // bounds from the max and min quadrilateral angles
      double masterLimitFromQuadrilateralMin = convertInteriorAngleToJointAngle(fourBarCalculator.getMinDAB(), 0);
      double masterLimitFromQuadrilateralMax = convertInteriorAngleToJointAngle(fourBarCalculator.getMaxDAB(), 0);

      if(jointSigns[0] > 0)
      {
         lowerBounds.add(masterLimitFromQuadrilateralMin);
         upperBounds.add(masterLimitFromQuadrilateralMax);
      }
      else
      {
         lowerBounds.add(masterLimitFromQuadrilateralMax);
         upperBounds.add(masterLimitFromQuadrilateralMin);
      }

      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, masterJointA.getJointLimitLower(), 0, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, masterJointA.getJointLimitUpper(), 0, true);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointB.getJointLimitLower(), 1, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointB.getJointLimitUpper(), 1, true);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointC.getJointLimitLower(), 2, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointC.getJointLimitUpper(), 2, true);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointD.getJointLimitLower(), 3, false);
      addJointLimitToBoundsIfValid(lowerBounds, upperBounds, jointD.getJointLimitUpper(), 3, true);

      Collections.sort(lowerBounds);
      Collections.reverse(lowerBounds);
      Collections.sort(upperBounds);

      return new double[]{lowerBounds.get(0), upperBounds.get(0)};
   }

   private void addJointLimitToBoundsIfValid(ArrayList<Double> lowerBoundsList, ArrayList<Double> upperBoundsList, double jointAngleLimit, int jointIndex,
         boolean isAnUpperBound)
   {
      if (jointAngleLimit != Double.POSITIVE_INFINITY && jointAngleLimit != Double.NEGATIVE_INFINITY)
      {
         double interiorAngle = convertJointAngleToInteriorAngle(jointAngleLimit, jointIndex);
         double masterJointLimit;

         if (!MathTools.isInsideBoundsExclusive(interiorAngle, 0.0, Math.PI))
            return;

         switch (jointIndex)
         {
         case 0:
            masterJointLimit = convertInteriorAngleToJointAngle(interiorAngle, 0);
            break;
         case 1:
            fourBarCalculator.computeMasterJointAngleGivenAngleABC(interiorAngle);
            masterJointLimit = convertInteriorAngleToJointAngle(fourBarCalculator.getAngleDAB(), 0);
            break;
         case 2:
            fourBarCalculator.computeMasterJointAngleGivenAngleBCD(interiorAngle);
            masterJointLimit = convertInteriorAngleToJointAngle(fourBarCalculator.getAngleDAB(), 0);
            break;
         case 3:
            fourBarCalculator.computeMasterJointAngleGivenAngleCDA(interiorAngle);
            masterJointLimit = convertInteriorAngleToJointAngle(fourBarCalculator.getAngleDAB(), 0);
            break;
         default:
            throw new RuntimeException("Invalid joint index " + jointIndex + " in four bar kinematic loop");
         }

         boolean isAMasterJointUpperBound = isAnUpperBound ^ !jointMaxCorrespondsToMasterJointMax(jointIndex);
         if(isAMasterJointUpperBound)
            upperBoundsList.add(masterJointLimit);
         else
            lowerBoundsList.add(masterJointLimit);
      }
   }

   private boolean jointMaxCorrespondsToMasterJointMax(int jointIndex)
   {
      boolean positiveMasterJointCoeff = jointSigns[0] > 0;
      boolean positivePassiveJointCoeff = jointSigns[jointIndex] > 0;
      boolean overallSignChangeIsNegativeOne = positiveMasterJointCoeff ^ positivePassiveJointCoeff;

      // when b or d have a max interior angle, a has a min interior angle
      if(jointIndex == 1 || jointIndex == 3)
         return overallSignChangeIsNegativeOne;
      else
         return !overallSignChangeIsNegativeOne;
   }

   public void verifyMasterJointLimits()
   {
      double maxValidMasterJointAngle = fourBarIsClockwise ?
            convertInteriorAngleToJointAngle(fourBarCalculator.getMaxDAB(), 0) :
            convertInteriorAngleToJointAngle(fourBarCalculator.getMinDAB(), 0);
      double minValidMasterJointAngle = fourBarIsClockwise ?
            convertInteriorAngleToJointAngle(fourBarCalculator.getMinDAB(), 0) :
            convertInteriorAngleToJointAngle(fourBarCalculator.getMaxDAB(), 0);

      // A) Angle limits not set
      if (masterJointA.getJointLimitLower() == Double.NEGATIVE_INFINITY || masterJointA.getJointLimitUpper() == Double.POSITIVE_INFINITY)
      {
         throw new RuntimeException(
               "Must set the joint limits for the master joint of the " + name + " four bar.\nNote that for the given link lengths max angle is "
                     + maxValidMasterJointAngle + "and min angle is" + minValidMasterJointAngle);
      }

      // B) Max angle limit is too large
      if (masterJointA.getJointLimitUpper() > maxValidMasterJointAngle + eps)
      {
         throw new RuntimeException("The maximum valid joint angle for the master joint of the " + name + " four bar is " + maxValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitUpper());
      }

      // C) Min angle limit is too small
      if (masterJointA.getJointLimitLower() < minValidMasterJointAngle - eps)
      {
         throw new RuntimeException("The minimum valid joint angle for the master joint of the " + name + " four bar is " + minValidMasterJointAngle + " to avoid flipping, but was set to " + masterJointA.getJointLimitLower());
      }
   }

   public void update()
   {
      double currentMasterJointA = masterJointA.getQ();
      double interiorAngleA = convertJointAngleToInteriorAngle(currentMasterJointA, 0);
      double interiorAngleDtA = jointSigns[0] * masterJointA.getQd();
      double interiorAngleDt2A = jointSigns[0] * masterJointA.getQdd();

      if(DEBUG)
         System.out.println("interior angle A = " + interiorAngleA / Math.PI);

      if (currentMasterJointA < masterJointA.getJointLimitLower() || currentMasterJointA > masterJointA.getJointLimitUpper())
      {
         throw new RuntimeException(masterJointA.getName() + " is set outside of its bounds [" + masterJointA.getJointLimitLower() + ", " + masterJointA.getJointLimitUpper() + "]. The current value is: " +  masterJointA.getQ());
      }

      fourBarCalculator.updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(interiorAngleA, interiorAngleDtA, interiorAngleDt2A);
      jacobian = fourBarJacobianSolver.computeJacobian(fourBarOutputJoint);
      
      if(DEBUG)
      {
         System.out.println("\nJacobian: \n" + jacobian);
      }
      
      outputJointVelocitiesToPack = jacobian;
      fourBarJacobianSolver.solveLinearVelFromAngularVel(outputJointVelocitiesToPack, masterJointA.getQd());
     
      if(DEBUG)
      {
         System.out.println("\nVelocity output (J*qd): \n" + outputJointVelocitiesToPack);

         System.out.println("Calculator output:");
         System.out.println("ABC = " + fourBarCalculator.getAngleABC());
         System.out.println("BCD = " + fourBarCalculator.getAngleBCD());
         System.out.println("CDA = " + fourBarCalculator.getAngleCDA());
         System.out.println("DtABC = " + fourBarCalculator.getAngleDtABC());
         System.out.println("DtBCD = " + fourBarCalculator.getAngleDtBCD());
         System.out.println("DtCDA = " + fourBarCalculator.getAngleDtCDA());
      }

      passiveJointB.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleABC(), 1));
      passiveJointC.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleBCD(), 2));
      passiveJointD.setQ(convertInteriorAngleToJointAngle(fourBarCalculator.getAngleCDA(), 3));

      passiveJointB.setQd(jointSigns[1] * fourBarCalculator.getAngleDtABC());
      passiveJointC.setQd(jointSigns[2] * fourBarCalculator.getAngleDtBCD());
      passiveJointD.setQd(jointSigns[3] * fourBarCalculator.getAngleDtCDA());

      passiveJointB.setQdd(jointSigns[1] * fourBarCalculator.getAngleDt2ABC());
      passiveJointC.setQdd(jointSigns[2] * fourBarCalculator.getAngleDt2BCD());
      passiveJointD.setQdd(jointSigns[3] * fourBarCalculator.getAngleDt2CDA());
   }

   /**
    * @param interiorAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->A, 1->B, 2->C, 3->D
    * @return
    */
   private double convertInteriorAngleToJointAngle(double interiorAngle, int passiveJointIndex)
   {
      return jointSigns[passiveJointIndex] * (interiorAngle - interiorAnglesAtZeroConfiguration[passiveJointIndex]);
   }

   /**
    * @param jointAngle interior angle of the joint when viewing the fourbar as a quadrilateral
    * @param passiveJointIndex 0->A, 1->B, 2->C, 3->D
    * @return
    */
   private double convertJointAngleToInteriorAngle(double jointAngle, int passiveJointIndex)
   {
      return interiorAnglesAtZeroConfiguration[passiveJointIndex] + jointSigns[passiveJointIndex] * jointAngle;
   }

   private InverseDynamicsJoint[] getJointsForJacobianCalculation(PassiveRevoluteJoint outputJoint)
   {
      InverseDynamicsJoint[] joints;
      
      if (fourBarOutputJoint == passiveJointD)
      {      
         joints = new InverseDynamicsJoint[3] ;
         joints[0] = masterJointA;
         joints[1] = passiveJointB;
         joints[2] = passiveJointC;
      }      
      else if (fourBarOutputJoint == passiveJointC)
      {      
         joints = new InverseDynamicsJoint[2] ;
         joints[0] = masterJointA;
         joints[1] = passiveJointB;
      }    
      else
      {
         joints = new InverseDynamicsJoint[1] ;
         joints[0] = masterJointA;
      }
      
      return joints;
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
   
   public DenseMatrix64F getOutputJointLinearVelocities()
   {
      return outputJointVelocitiesToPack;
   }
   
   public DenseMatrix64F getFourBarJacobian()
   {
      return jacobian;
   }
}
