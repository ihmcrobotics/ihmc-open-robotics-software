package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeOutput;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.CylinderAndPlaneContactMatrixCalculator;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.CylinderAndPlaneContactSpatialForceVectorCalculator;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.EndEffector;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.EndEffectorOutput;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.OptimizerCylinderContactModel;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.OptimizerPlaneContactModel;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.PoseReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


public class CylinderAndPlaneContactForceOptimizerMatrixCalculatorTest
{
   private static final double HIGH_ACCURACY_THRESHOLD = 1.00;    // Newtons
   private static final double ROBOT_WEIGHT = 9.81 * 100;    // Newtons
   private static final boolean DEBUG = true;
   private final ReferenceFrame comFrame = ReferenceFrame.constructAWorldFrame("com");

   @Ignore // Cylindrical grasping stuff has been put the grave yard.

	@EstimatedDuration
	@Test(timeout=300000)
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
      addFootAtPose(endEffectorsWithDefinedContactModels, "rightFoot",
                    new FramePose(comFrame, new Point3d(leanBack, -0.3, -1.0), new Quat4d(0.0, 0.0, 0.0, 1.0)));
      addHandAtPose(endEffectorsWithDefinedContactModels, "leftHand",
                    new FramePose(comFrame, new Point3d(leanBack, 0.4, 0.5), new Quat4d(Math.sqrt(2), 0.0, 0.0, Math.sqrt(2))));
      addHandAtPose(endEffectorsWithDefinedContactModels, "rightHand",
                    new FramePose(comFrame, new Point3d(leanBack, -0.4, 0.5), new Quat4d(Math.sqrt(2), 0.0, 0.0, Math.sqrt(2))));

      setupOutputs(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces);

      runSolver(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces, desiredCOMForces);


      assertSumOfForcesIsAsExpected(endEffectorsWithAssignedForces, desiredCOMForces, HIGH_ACCURACY_THRESHOLD);
   }

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void testLeftFootAtOrigin()
   {
      List<EndEffector> endEffectorsWithDefinedContactModels = new ArrayList<EndEffector>();
      List<EndEffectorOutput> endEffectorsWithAssignedForces = new ArrayList<EndEffectorOutput>();
      double[] desiredCOMForces = new double[]
      {
         0.0, 0.0, 0.0, 0.0, 0.0, ROBOT_WEIGHT
      };
      addLeftFootAtCOMOrigin(endEffectorsWithDefinedContactModels);

      setupOutputs(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces);


      runSolver(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces, desiredCOMForces);


      assertSumOfForcesIsAsExpected(endEffectorsWithAssignedForces, desiredCOMForces, HIGH_ACCURACY_THRESHOLD);
   }

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void testLeftHandAtOrigin()
   {
      List<EndEffector> endEffectorsWithDefinedContactModels = new ArrayList<EndEffector>();
      List<EndEffectorOutput> endEffectorsWithAssignedForces = new ArrayList<EndEffectorOutput>();
      double[] desiredCOMForces = new double[]
      {
         0.0, 0.0, 0.0, 0.0, 0.0, ROBOT_WEIGHT
      };
      addLeftHandAtCOMOrigin(endEffectorsWithDefinedContactModels);


      setupOutputs(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces);


      runSolver(endEffectorsWithDefinedContactModels, endEffectorsWithAssignedForces, desiredCOMForces);

      assertSumOfForcesIsAsExpected(endEffectorsWithAssignedForces, desiredCOMForces, HIGH_ACCURACY_THRESHOLD);
   }

   public void setupOutputs(List<EndEffector> endEffectorsWithDefinedContactModels, List<EndEffectorOutput> endEffectorsWithAssignedForces)
   {
      endEffectorsWithAssignedForces.clear();

      for (int i = 0; i < endEffectorsWithDefinedContactModels.size(); i++)
      {
         endEffectorsWithAssignedForces.add(endEffectorsWithDefinedContactModels.get(i).getOutput());
      }
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
      PoseReferenceFrame leftFootFrame = new PoseReferenceFrame("leftFootBody", comFrame);
      EndEffector leftFoot = new EndEffector(comFrame, leftFootFrame, new YoVariableRegistry("fake"));
      leftFoot.setLoadBearing(true);
      PoseReferenceFrame leftFootPlaneFrame = new PoseReferenceFrame("leftFootPlane", leftFootFrame);
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

      FrameVector normalContactVector = new FrameVector(leftFootPlaneFrame, 0.0, 0.0, 1.0);

      planeContactModel.setup(0.3, leftFootContactPoints, normalContactVector, leftFootFrame, Double.NaN, 0.15);    // TODO: don't use NaN. Currently not being used for anything though
   }

   public void addFootAtPose(List<EndEffector> endEffectorsWithDefinedContactModels, String name, FramePose pose)
   {
      OptimizerPlaneContactModel planeContactModel = new OptimizerPlaneContactModel();
      PoseReferenceFrame footFrame = new PoseReferenceFrame(name + "Body", comFrame);
      EndEffector leftFoot = new EndEffector(comFrame, footFrame, new YoVariableRegistry("fake"));
      leftFoot.setLoadBearing(true);
      PoseReferenceFrame leftFootPlaneFrame = new PoseReferenceFrame(name + "Plane", footFrame);
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

      FrameVector normalContactVector = new FrameVector(leftFootPlaneFrame, 0.0, 0.0, 1.0);

      planeContactModel.setup(0.3, contactPoints, normalContactVector, footFrame, Double.NaN, 0.15);    // TODO: don't use NaN. Currently not being used for anything though

      footFrame.setPoseAndUpdate(pose);
   }

   public void addLeftHandAtCOMOrigin(List<EndEffector> endEffectorsWithDefinedContactModels)
   {
      PoseReferenceFrame leftHandFrame = new PoseReferenceFrame("leftHandBody", comFrame);
      EndEffector leftHand = new EndEffector(comFrame, leftHandFrame, new YoVariableRegistry("fake"));
      leftHand.setLoadBearing(true);
      OptimizerCylinderContactModel cylinderCon = new OptimizerCylinderContactModel();
      leftHand.setContactModel(cylinderCon);

      double cylinderRadius = 0.2;
      double mu = 0.3;
      double cylinderHalfHandWidth = 0.3;
      double cylinderTensileGripStrength = 450 * 4;    // TuneMe: aggressive!
      double gripWeaknessFactor = 1.0;

      endEffectorsWithDefinedContactModels.add(leftHand);

      cylinderCon.setup(mu, cylinderRadius, cylinderHalfHandWidth, cylinderTensileGripStrength, gripWeaknessFactor, leftHandFrame, Double.NaN, Double.NaN);    // TODO: don't use NaN. Currently not being used for anything though
   }

   public void addHandAtPose(List<EndEffector> endEffectorsWithDefinedContactModels, String name, FramePose pose)
   {
      OptimizerCylinderContactModel cylinderCon = new OptimizerCylinderContactModel();
      PoseReferenceFrame handFrame = new PoseReferenceFrame(name + "Body", comFrame);
      EndEffector leftHand = new EndEffector(comFrame, handFrame, new YoVariableRegistry("fake"));
      leftHand.setLoadBearing(true);
      leftHand.setContactModel(cylinderCon);

      double cylinderRadius = 0.2;
      double mu = 0.3;
      double cylinderHalfHandWidth = 0.3;
      double cylinderTensileGripStrength = 450 * 4;    // TuneMe: aggressive!
      double gripWeaknessFactor = 1.0;

      endEffectorsWithDefinedContactModels.add(leftHand);

      cylinderCon.setup(mu, cylinderRadius, cylinderHalfHandWidth, cylinderTensileGripStrength, gripWeaknessFactor, handFrame, Double.NaN, Double.NaN);    // TODO: don't use NaN. Currently not being used for anything though
      handFrame.setPoseAndUpdate(pose);
   }

   public void runSolver(List<EndEffector> endEffectorsWithDefinedContactModels, List<EndEffectorOutput> endEffectorsWithAssignedForces,
                         double[] desiredCOMForces)
   {
      int rhoSize = CylinderAndPlaneContactForceOptimizerNative.rhoSize;
      int phiSize = CylinderAndPlaneContactForceOptimizerNative.phiSize;
      CylinderAndPlaneContactMatrixCalculator calc = new CylinderAndPlaneContactMatrixCalculator(comFrame, new YoVariableRegistry("rootRegistry"),
                                                        new YoGraphicsListRegistry(), rhoSize, phiSize);
      CylinderAndPlaneContactSpatialForceVectorCalculator vectorCalc = new CylinderAndPlaneContactSpatialForceVectorCalculator(comFrame,
                                                                          new YoVariableRegistry("rootRegistry"), new YoGraphicsListRegistry(),
                                                                          rhoSize, phiSize);
      CylinderAndPlaneContactForceOptimizerNativeInput nativeSolverInput = new CylinderAndPlaneContactForceOptimizerNativeInput();
      CylinderAndPlaneContactForceOptimizerNativeOutput nativeSolverOutput;
      CylinderAndPlaneContactForceOptimizerNative nativeSolver = new CylinderAndPlaneContactForceOptimizerNative(new YoVariableRegistry("rootRegistry"));
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
      calc.computeMatrices(endEffectorsWithDefinedContactModels);

      nativeSolverInput.setRhoMin(calc.getRhoMin());
      nativeSolverInput.setPhiMin(calc.getPhiMin());
      nativeSolverInput.setPhiMax(calc.getPhiMax());
      nativeSolverInput.setQRho(calc.getQRho());
      nativeSolverInput.setQPhi(calc.getQPhi());

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

      vectorCalc.setQRho(calc.getQRho());
      vectorCalc.setQPhi(calc.getQPhi());
      vectorCalc.computeWrenches(endEffectorsWithDefinedContactModels, nativeSolverOutput.getRho(), nativeSolverOutput.getPhi());

      for (int i = 0; i < endEffectorsWithDefinedContactModels.size(); i++)
      {
         endEffectorsWithAssignedForces.get(i).setExternallyActingSpatialForceVector(
             vectorCalc.getSpatialForceVector(endEffectorsWithDefinedContactModels.get(i)));
      }
   }

}
