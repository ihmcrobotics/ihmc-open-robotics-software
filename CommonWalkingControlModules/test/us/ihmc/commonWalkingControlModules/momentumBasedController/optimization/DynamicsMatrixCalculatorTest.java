package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Map;
import java.util.Random;

public class DynamicsMatrixCalculatorTest
{
   // // TODO: 12/30/16  get the tolerance a lot smaller
   private final static double tolerance = 5.0;

   private final Random random = new Random(5641654L);

   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private WholeBodyControlCoreToolbox toolbox;

   private WrenchMatrixCalculator wrenchMatrixCalculator;
   private JointIndexHandler jointIndexHandler;

   private InverseDynamicsCalculator inverseDynamicsCalculator;
   private DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private double gravityZ;

   int degreesOfFreedom;
   int floatingBaseDoFs;
   int bodyDoFs;

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testEquivalence() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnly() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);

      int rhoSize = wrenchMatrixCalculator.getRhoSize();
      DenseMatrix64F rhoSolution = new DenseMatrix64F(rhoSize, 1);
      DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);
      DenseMatrix64F qddotSolutionRevised = new DenseMatrix64F(degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolutionRevised, rhoSolution);
      computeRequiredRhoSolution(qddotSolutionRevised, rhoSolution, true);

      solveAndCompare(qddotSolutionRevised, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnlyUnitAccel() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);
      CommonOps.fill(qddotSolution, 1.0);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testNoLoad() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testCoriolisAndGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixAndGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixAndCoriolisOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);

      DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
      DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testForceAndGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      ScrewTestTools.setRandomPositions(joints, random);

      DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, 1000.0);
      DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

      computeRequiredRhoSolution(qddotSolution, rhoSolution);
      computeAchievableQddotSolution(qddotSolution, rhoSolution);

      solveAndCompare(qddotSolution, rhoSolution);
   }

   private void setupTest()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double controlDT = 0.005;

      fullHumanoidRobotModel = new FullRobotModelTestTools.RandomFullHumanoidRobotModel(random);
      fullHumanoidRobotModel.updateFrames();
      CommonHumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullHumanoidRobotModel.getElevator());
      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();

      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();
      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullHumanoidRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullHumanoidRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }

      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullHumanoidRobotModel, new InverseDynamicsJoint[0]);
      toolbox = new WholeBodyControlCoreToolbox(fullHumanoidRobotModel, null, jointsToOptimizeFor, momentumOptimizationSettings,
            jointPrivilegedConfigurationParameters, referenceFrames, controlDT, gravityZ, geometricJacobianHolder, twistCalculator, contactablePlaneBodies,
            yoGraphicsListRegistry, registry);

      wrenchMatrixCalculator = new WrenchMatrixCalculator(toolbox, registry);
      jointIndexHandler = toolbox.getJointIndexHandler();

      inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox, wrenchMatrixCalculator);

      degreesOfFreedom = jointIndexHandler.getNumberOfDoFs();
      floatingBaseDoFs = fullHumanoidRobotModel.getRootJoint().getDegreesOfFreedom();
      bodyDoFs = degreesOfFreedom - floatingBaseDoFs;
   }

   private void computeRequiredRhoSolution(DenseMatrix64F qddotSolution, DenseMatrix64F rhoSolutionToPack)
   {
      computeRequiredRhoSolution(qddotSolution, rhoSolutionToPack, false);
   }

   private void computeRequiredRhoSolution(DenseMatrix64F qddotSolution, DenseMatrix64F rhoSolutionToPack, boolean check)
   {
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      fullHumanoidRobotModel.updateFrames();
      toolbox.getTwistCalculator().compute();
      toolbox.getGeometricJacobianHolder().compute();

      wrenchMatrixCalculator.computeMatrices();
      dynamicsMatrixCalculator.compute();

      DenseMatrix64F floatingBaseMassMatrix = new DenseMatrix64F(floatingBaseDoFs, degreesOfFreedom);
      DenseMatrix64F floatingBaseContactJacobian = new DenseMatrix64F(rhoSize, floatingBaseDoFs);
      DenseMatrix64F floatingBaseContactJacobianTranspose = new DenseMatrix64F(floatingBaseDoFs, rhoSize);
      DenseMatrix64F floatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseDoFs, 1);
      DenseMatrix64F B = new DenseMatrix64F(floatingBaseDoFs, 1);
      DenseMatrix64F B_check = new DenseMatrix64F(floatingBaseDoFs, 1);

      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(floatingBaseMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(floatingBaseCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(floatingBaseContactJacobian);

      CommonOps.transpose(floatingBaseContactJacobian, floatingBaseContactJacobianTranspose);

      CommonOps.mult(floatingBaseMassMatrix, qddotSolution, B);
      CommonOps.add(floatingBaseCoriolisMatrix, B, B);

      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      pseudoInverseSolver.setA(floatingBaseContactJacobianTranspose);
      pseudoInverseSolver.solve(B, rhoSolutionToPack);

      CommonOps.mult(floatingBaseContactJacobianTranspose, rhoSolutionToPack, B_check);

      if (check)
         JUnitTools.assertMatrixEquals(B_check, B, 0.001);
   }

   private void computeAchievableQddotSolution(DenseMatrix64F qddotSolutionToPack, DenseMatrix64F rhoSolution)
   {
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      DenseMatrix64F floatingBaseMassMatrix = new DenseMatrix64F(floatingBaseDoFs, degreesOfFreedom);
      DenseMatrix64F floatingBaseContactJacobian = new DenseMatrix64F(rhoSize, floatingBaseDoFs);
      DenseMatrix64F floatingBaseContactJacobianTranspose = new DenseMatrix64F(floatingBaseDoFs, rhoSize);
      DenseMatrix64F floatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseDoFs, 1);
      DenseMatrix64F B = new DenseMatrix64F(floatingBaseDoFs, 1);
      DenseMatrix64F B_check = new DenseMatrix64F(floatingBaseDoFs, 1);
      DenseMatrix64F zero = new DenseMatrix64F(floatingBaseDoFs, 1);

      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(floatingBaseMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(floatingBaseCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(floatingBaseContactJacobian);

      CommonOps.transpose(floatingBaseContactJacobian, floatingBaseContactJacobianTranspose);

      CommonOps.mult(floatingBaseContactJacobianTranspose, rhoSolution, B);
      CommonOps.subtract(B, floatingBaseCoriolisMatrix, B);

      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(B, qddotSolutionToPack);

      CommonOps.mult(floatingBaseMassMatrix, qddotSolutionToPack, B_check);
      JUnitTools.assertMatrixEquals(B, B_check, tolerance);

      CommonOps.add(B_check, floatingBaseCoriolisMatrix, B_check);
      CommonOps.multAdd(-1.0, floatingBaseContactJacobianTranspose, rhoSolution, B_check);

      JUnitTools.assertMatrixEquals(B_check, zero, tolerance);
   }

   private void solveAndCompare(DenseMatrix64F qddotSolution, DenseMatrix64F rhoSolution)
   {
      fullHumanoidRobotModel.updateFrames();
      toolbox.getTwistCalculator().compute();
      toolbox.getGeometricJacobianHolder().compute();

      wrenchMatrixCalculator.computeMatrices();
      Map<RigidBody, Wrench> contactWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      for (int i = 0; i < toolbox.getContactablePlaneBodies().size(); i++)
      {
         RigidBody rigidBody = toolbox.getContactablePlaneBodies().get(i).getRigidBody();
         inverseDynamicsCalculator.setExternalWrench(rigidBody, contactWrenches.get(rigidBody));
      }

      DenseMatrix64F inverseDynamicsTauSolution = new DenseMatrix64F(bodyDoFs, 1);
      DenseMatrix64F dynamicsMatrixTauSolution = new DenseMatrix64F(bodyDoFs, 1);

      // compute torques using dynamics matrix calculator
      dynamicsMatrixCalculator.compute();
      dynamicsMatrixCalculator.computeJointTorques(dynamicsMatrixTauSolution, qddotSolution, rhoSolution);

      // compute torques using inverse dynamics calculator
      ScrewTools.setDesiredAccelerations(fullHumanoidRobotModel.getOneDoFJoints(), qddotSolution);
      inverseDynamicsCalculator.compute();

      DenseMatrix64F tmpTauMatrix = new DenseMatrix64F(1, 1);
      for (int i = 0; i < bodyDoFs; i++)
      {
         InverseDynamicsJoint joint = jointIndexHandler.getIndexedOneDoFJoints()[i];
         int[] jointIndices = jointIndexHandler.getJointIndices(joint);
         joint.getTauMatrix(tmpTauMatrix);
         for (int dof = 0; dof < jointIndices.length; dof++)
            inverseDynamicsTauSolution.set(jointIndices[dof] - floatingBaseDoFs, tmpTauMatrix.get(dof, 0));
      }

      JUnitTools.assertMatrixEquals(inverseDynamicsTauSolution, dynamicsMatrixTauSolution, tolerance);
   }
}
