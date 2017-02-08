package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateADotVTerm;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public class CentroidalMomentumBenchmarkTest
{
   private static final int iters = 50000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double controlDT = 0.00000005;

   private final DenseMatrix64F a = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aPrevVal = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aDot = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F aDotVNumerical = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F aDotVAnalytical = new DenseMatrix64F(6, 1);

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 150000)
   public void floatingChainTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      int numberOfJoints = 34;
      Vector3d[] jointAxes = new Vector3d[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);

      ScrewTestTools.RandomFloatingChain idRobot = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      RigidBody elevator = idRobot.getElevator();
      joints.addAll(idRobot.getRevoluteJoints());

      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", idRobot.getRootJoint());

      assertADotV(random, joints, elevator, robot, numberOfJoints + 1);
   }

   private void assertADotV(Random random, ArrayList<RevoluteJoint> joints, RigidBody elevator, SCSRobotFromInverseDynamicsRobotModel robot, int numJoints)
         throws UnreasonableAccelerationException
   {
      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(ScrewTools.computeSubtreeJoints(elevator));
      DenseMatrix64F v = new DenseMatrix64F(numberOfDoFs, 1);

      InverseDynamicsJoint[] idJoints = new InverseDynamicsJoint[numJoints];
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);

      CentroidalMomentumMatrix centroidalMomentumMatrixCalculator = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);

      a.reshape(6, numberOfDoFs);
      aPrevVal.reshape(6, numberOfDoFs);
      aDot.reshape(6, numberOfDoFs);

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      CentroidalMomentumRateADotVTerm aDotVAnalyticalCalculator = new CentroidalMomentumRateADotVTerm(elevator, centerOfMassFrame,
            centroidalMomentumMatrixCalculator, totalMass, v);

      CentroidalMomentumRateTermCalculator testTermCalc = new CentroidalMomentumRateTermCalculator(elevator, centerOfMassFrame, v, totalMass);

      ScrewTestTools.setRandomVelocities(joints, random);
      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomTorques(joints, random);

      robot.updateJointPositions_ID_to_SCS();
      robot.updateJointVelocities_ID_to_SCS();
      robot.updateJointTorques_ID_to_SCS();

      centerOfMassFrame.update();

      centroidalMomentumMatrixCalculator.compute();
      aPrevVal.set(centroidalMomentumMatrixCalculator.getMatrix());

      robot.doDynamicsAndIntegrate(controlDT);
      robot.updateVelocities();
      robot.updateJointPositions_SCS_to_ID();
      robot.updateJointVelocities_SCS_to_ID();
      elevator.updateFramesRecursively();
      centerOfMassFrame.update();

      robot.packIdJoints(idJoints);
      ScrewTools.getJointVelocitiesMatrix(idJoints, v);

      long startTime = System.nanoTime();
      for (int i = 0; i < iters; i++)
      {
         testTermCalc.compute();
      }
      long duration = (System.nanoTime() - startTime) / (iters);
      double termCalculatorTime = ((double) duration / 1000000000);

      // Compute aDotV analytically
      aDotVAnalyticalCalculator.compute();
      aDotVAnalytical.set(aDotVAnalyticalCalculator.getMatrix());

      // Compute aDotV numerically
      startTime = System.nanoTime();
      for (int i = 0; i < iters; i++)
      {
         centroidalMomentumMatrixCalculator.compute();
         a.set(centroidalMomentumMatrixCalculator.getMatrix());
         MatrixTools.numericallyDifferentiate(aDot, aPrevVal, a, controlDT);
         CommonOps.mult(aDot, v, aDotVNumerical);
      }
      duration = (System.nanoTime() - startTime) / (iters);
      double numericallyDifferentiatedTime = ((double) duration / 1000000000);

      System.out.println("solution time using analytical solution: " + new DecimalFormat("#.##########").format(termCalculatorTime) + " Seconds");
      System.out.println(
            "solution time using numerically differentiated solution: " + new DecimalFormat("#.##########").format(numericallyDifferentiatedTime) + " Seconds");

   }
}
