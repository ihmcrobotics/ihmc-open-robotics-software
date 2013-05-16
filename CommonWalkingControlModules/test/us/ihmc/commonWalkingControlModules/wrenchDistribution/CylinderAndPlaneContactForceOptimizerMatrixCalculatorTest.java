package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;


import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeOutput;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class CylinderAndPlaneContactForceOptimizerMatrixCalculatorTest
{
   private static final double HIGH_ACCURACY_THRESHOLD = 1.00;    // Newtons
   private static final double ROBOT_WEIGHT = 9.81 * 100;    // Newtons
   private static final boolean DEBUG = true;
   private ReferenceFrame comFrame = ReferenceFrame.constructAWorldFrame("com");

   @Test
   public void test4limbsTogether()
   {
      List<EndEffector> endEffectorsWithDefinedContactModels = new ArrayList<EndEffector>();
      List<EndEffectorOutput> endEffectorsWithAssignedForces = new ArrayList<EndEffectorOutput>();
      double[] desiredCOMForces = new double[]
      {
         0.0, 0.0, 0.0, 0.0, 0.0, ROBOT_WEIGHT
      };
      double leanBack = 0.4;
      addFootAtPose(endEffectorsWithDefinedContactModels, "leftFoot",
                    new FramePose(comFrame, new Point3d(leanBack, 0.3, -1.0), new Quat4d(0.0, 0.0, 0.0, 1.0)));
      endEffectorsWithAssignedForces.add(new EndEffectorOutput(comFrame));
      addFootAtPose(endEffectorsWithDefinedContactModels, "rightFoot",
                    new FramePose(comFrame, new Point3d(leanBack, -0.3, -1.0), new Quat4d(0.0, 0.0, 0.0, 1.0)));
      endEffectorsWithAssignedForces.add(new EndEffectorOutput(comFrame));
      addHandAtPose(endEffectorsWithDefinedContactModels, "leftHand",
                    new FramePose(comFrame, new Point3d(leanBack, 0.4, 0.5), new Quat4d(Math.sqrt(2), 0.0, 0.0, Math.sqrt(2))));
      endEffectorsWithAssignedForces.add(new EndEffectorOutput(comFrame));
      addHandAtPose(endEffectorsWithDefinedContactModels, "rightHand",
                    new FramePose(comFrame, new Point3d(leanBack, -0.4, 0.5), new Quat4d(Math.sqrt(2), 0.0, 0.0, Math.sqrt(2))));
      endEffectorsWithAssignedForces.add(new EndEffectorOutput(comFrame));

      runSolver(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces, desiredCOMForces);


      assertSumOfForcesIsAsExpected(endEffectorsWithAssignedForces, desiredCOMForces, HIGH_ACCURACY_THRESHOLD);
   }

   @Test
   public void testLeftFootAtOrigin()
   {
      List<EndEffector> endEffectorsWithDefinedContactModels = new ArrayList<EndEffector>();
      List<EndEffectorOutput> endEffectorsWithAssignedForces = new ArrayList<EndEffectorOutput>();
      double[] desiredCOMForces = new double[]
      {
         0.0, 0.0, 0.0, 0.0, 0.0, ROBOT_WEIGHT
      };
      addLeftFootAtCOMOrigin(endEffectorsWithDefinedContactModels);

      EndEffectorOutput leftFootResult = new EndEffectorOutput(comFrame);
      endEffectorsWithAssignedForces.add(leftFootResult);


      runSolver(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces, desiredCOMForces);


      assertSumOfForcesIsAsExpected(endEffectorsWithAssignedForces, desiredCOMForces, HIGH_ACCURACY_THRESHOLD);
   }

   @Test
   public void testLeftHandAtOrigin()
   {
      List<EndEffector> endEffectorsWithDefinedContactModels = new ArrayList<EndEffector>();
      List<EndEffectorOutput> endEffectorsWithAssignedForces = new ArrayList<EndEffectorOutput>();
      double[] desiredCOMForces = new double[]
      {
         0.0, 0.0, 0.0, 0.0, 0.0, ROBOT_WEIGHT
      };
      addLeftHandAtCOMOrigin(endEffectorsWithDefinedContactModels);

      EndEffectorOutput leftFootResult = new EndEffectorOutput(comFrame);
      endEffectorsWithAssignedForces.add(leftFootResult);


      runSolver(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces, desiredCOMForces);

      assertSumOfForcesIsAsExpected(endEffectorsWithAssignedForces, desiredCOMForces, HIGH_ACCURACY_THRESHOLD);
   }



   public void assertSumOfForcesIsAsExpected(List<EndEffectorOutput> endEffectorsWithAssignedForces, double[] desiredCOMForces, double eps)
   {
      double[] sumOfForces = new double[]
      {
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0
      };
      double[] forcesToPack = new double[6];
      for (EndEffectorOutput output : endEffectorsWithAssignedForces)
      {
         for (int i = 0; i < 6; i++)
         {
            SpatialForceVector vectorToPack = new SpatialForceVector();
            output.packExternallyActingSpatialForceVector(vectorToPack);
            vectorToPack.packMatrix(forcesToPack);
            sumOfForces[i] += forcesToPack[i];
         }
      }

      if (DEBUG)
      {
         System.out.println(" resulting COM force is [" + sumOfForces[0] + ", " + sumOfForces[1] + ", " + sumOfForces[2] + ", " + sumOfForces[3] + ", "
                            + sumOfForces[4] + ", " + sumOfForces[5] + "]");
      }

      assertEquals(desiredCOMForces[0], sumOfForces[0], eps);
      assertEquals(desiredCOMForces[1], sumOfForces[1], eps);
      assertEquals(desiredCOMForces[2], sumOfForces[2], eps);
      assertEquals(desiredCOMForces[3], sumOfForces[3], eps);
      assertEquals(desiredCOMForces[4], sumOfForces[4], eps);
      assertEquals(desiredCOMForces[5], sumOfForces[5], eps);
   }

   public void addLeftFootAtCOMOrigin(List<EndEffector> endEffectorsWithDefinedContactModels)
   {
      OptimizerPlaneContactModel planeContactModel = new OptimizerPlaneContactModel();
      EndEffector leftFoot = new EndEffector();
      leftFoot.setLoadBearing(true);
      PoseReferenceFrame leftFootFrame = new PoseReferenceFrame("left foot body", comFrame);
      PoseReferenceFrame leftFootPlaneFrame = new PoseReferenceFrame("left foot plane", leftFootFrame);
      leftFoot.setReferenceFrame(leftFootFrame);
      leftFoot.setContactModel(planeContactModel);
      ArrayList<FramePoint> leftFootContactPoints = new ArrayList<FramePoint>();
      double footLengthForward = 0.2;
      double footHalfWidth = 0.1;
      double footLengthBackward = 0.1;
      leftFootContactPoints.add(new FramePoint(leftFootPlaneFrame, footLengthForward, -footHalfWidth, 0));
      leftFootContactPoints.add(new FramePoint(leftFootPlaneFrame, footLengthForward, footHalfWidth, 0));
      leftFootContactPoints.add(new FramePoint(leftFootPlaneFrame, -footLengthBackward, footHalfWidth, 0));
      leftFootContactPoints.add(new FramePoint(leftFootPlaneFrame, -footLengthBackward, -footHalfWidth, 0));
      endEffectorsWithDefinedContactModels.add(leftFoot);

      planeContactModel.setup(0.3, leftFootContactPoints, leftFootFrame);
   }

   public void addFootAtPose(List<EndEffector> endEffectorsWithDefinedContactModels, String name, FramePose pose)
   {
      OptimizerPlaneContactModel planeContactModel = new OptimizerPlaneContactModel();
      EndEffector leftFoot = new EndEffector();
      leftFoot.setLoadBearing(true);
      PoseReferenceFrame footFrame = new PoseReferenceFrame(name + "Body", comFrame);
      PoseReferenceFrame leftFootPlaneFrame = new PoseReferenceFrame(name + "Plane", footFrame);
      leftFoot.setReferenceFrame(footFrame);
      leftFoot.setContactModel(planeContactModel);
      ArrayList<FramePoint> contactPoints = new ArrayList<FramePoint>();
      double footLengthForward = 0.1;
      double footHalfWidth = 0.1;
      double footLengthBackward = 0.1;
      contactPoints.add(new FramePoint(leftFootPlaneFrame, footLengthForward, -footHalfWidth, 0));
      contactPoints.add(new FramePoint(leftFootPlaneFrame, footLengthForward, footHalfWidth, 0));
      contactPoints.add(new FramePoint(leftFootPlaneFrame, -footLengthBackward, footHalfWidth, 0));
      contactPoints.add(new FramePoint(leftFootPlaneFrame, -footLengthBackward, -footHalfWidth, 0));
      endEffectorsWithDefinedContactModels.add(leftFoot);

      planeContactModel.setup(0.3, contactPoints, footFrame);

      footFrame.updatePose(pose);
   }

   public void addLeftHandAtCOMOrigin(List<EndEffector> endEffectorsWithDefinedContactModels)
   {
      OptimizerCylinderContactModel cylinderCon = new OptimizerCylinderContactModel();
      EndEffector leftHand = new EndEffector();
      leftHand.setLoadBearing(true);
      PoseReferenceFrame leftHandFrame = new PoseReferenceFrame("left hand body", comFrame);
      leftHand.setReferenceFrame(leftHandFrame);
      leftHand.setContactModel(cylinderCon);

      double cylinderRadius = 0.2;
      double mu = 0.3;
      double cylinderHalfHandWidth = 0.3;
      double cylinderTensileGripStrength = 450 * 4;    // TuneMe: aggressive!
      double gripWeaknessFactor = 1.0;

      endEffectorsWithDefinedContactModels.add(leftHand);

      cylinderCon.setup(mu, cylinderRadius, cylinderHalfHandWidth, cylinderTensileGripStrength, gripWeaknessFactor, leftHandFrame);
   }

   public void addHandAtPose(List<EndEffector> endEffectorsWithDefinedContactModels, String name, FramePose pose)
   {
      OptimizerCylinderContactModel cylinderCon = new OptimizerCylinderContactModel();
      EndEffector leftHand = new EndEffector();
      leftHand.setLoadBearing(true);
      PoseReferenceFrame handFrame = new PoseReferenceFrame(name + "Body", comFrame);
      leftHand.setReferenceFrame(handFrame);
      leftHand.setContactModel(cylinderCon);

      double cylinderRadius = 0.2;
      double mu = 0.3;
      double cylinderHalfHandWidth = 0.3;
      double cylinderTensileGripStrength = 450 * 4;    // TuneMe: aggressive!
      double gripWeaknessFactor = 1.0;

      endEffectorsWithDefinedContactModels.add(leftHand);

      cylinderCon.setup(mu, cylinderRadius, cylinderHalfHandWidth, cylinderTensileGripStrength, gripWeaknessFactor, handFrame);
      handFrame.updatePose(pose);
   }

   public void runSolver(List<EndEffector> endEffectorsWithDefinedContactModels, List<EndEffectorOutput> endEffectorsWithAssignedForces,
                         double[] desiredCOMForces)
   {
      CylinderAndPlaneContactForceOptimizerMatrixCalculator calc = new CylinderAndPlaneContactForceOptimizerMatrixCalculator(comFrame);
      CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator vectorCalc =
         new CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator(comFrame);
      CylinderAndPlaneContactForceOptimizerNativeInput nativeSolverInput = new CylinderAndPlaneContactForceOptimizerNativeInput();
      CylinderAndPlaneContactForceOptimizerNativeOutput nativeSolverOutput;
      CylinderAndPlaneContactForceOptimizerNative nativeSolver = new CylinderAndPlaneContactForceOptimizerNative();
      double wPhi = 0.000001;
      double wRho = 0.000001;
      DenseMatrix64F Cmatrix = CommonOps.diag(new double[]
      {
         1, 1, 1, 1.0, 1.0, 1.0
      });
      DenseMatrix64F desiredNetEnvironmentReactionWrench = new DenseMatrix64F(new double[][]
      {
         {desiredCOMForces[0]}, {desiredCOMForces[1]}, {desiredCOMForces[2]}, {desiredCOMForces[3]}, {desiredCOMForces[4]}, {desiredCOMForces[5]}
      });

      nativeSolverInput.resetToZeros();
      nativeSolverInput.setCylinderBoundedVectorRegularization(wPhi);
      nativeSolverInput.setGroundReactionForceRegularization(wRho);
      nativeSolverInput.setMomentumDotWeight(Cmatrix);
      nativeSolverInput.setWrenchEquationRightHandSide(desiredNetEnvironmentReactionWrench);
      calc.computeAllMatriciesAndPopulateNativeInput(endEffectorsWithDefinedContactModels, nativeSolverInput);

      try
      {
         nativeSolver.solve(nativeSolverInput);
      }
      catch (NoConvergenceException e)
      {
         e.printStackTrace();
      }

      nativeSolverOutput = nativeSolver.getOutput();

      if (DEBUG)
      {
         System.out.println(nativeSolverInput.toString());
         System.out.println(nativeSolverOutput.toString());
      }

      vectorCalc.computeAllWrenchesBasedOnNativeOutputAndInput(endEffectorsWithDefinedContactModels, nativeSolverInput, nativeSolverOutput);

      for (int i = 0; i < endEffectorsWithDefinedContactModels.size(); i++)
      {
         endEffectorsWithAssignedForces.get(i).setExternallyActingSpatialForceVector(
             vectorCalc.getSpatialForceVector(endEffectorsWithDefinedContactModels.get(i)));
      }
   }

}
