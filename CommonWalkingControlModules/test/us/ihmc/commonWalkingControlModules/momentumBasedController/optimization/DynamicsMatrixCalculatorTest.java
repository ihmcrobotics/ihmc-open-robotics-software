package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
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
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Map;
import java.util.Random;

public class DynamicsMatrixCalculatorTest
{
   private final static double tolerance = 0.0001;
   private final static int iters = 1000;
   private final static double maxRho = 1.0e2;

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

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();

         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnly() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();

         ScrewTestTools.setRandomPositions(joints, random);

         int rhoSize = wrenchMatrixCalculator.getRhoSize();
         DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, rhoSize, 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);
         DenseMatrix64F alternativeRho = new DenseMatrix64F(rhoSize, 1);

         update();
         DynamicsMatrixCalculatorTools.computeQddotGivenRho(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, alternativeRho);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnlyNoMotion() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();

         ScrewTestTools.setRandomPositions(joints, random);
         //ScrewTestTools.setRandomVelocities(joints, random);

         int rhoSize = wrenchMatrixCalculator.getRhoSize();
         DenseMatrix64F rhoSolution = new DenseMatrix64F(rhoSize, 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         computeZeroForceQddot(dynamicsMatrixCalculator, qddotSolution);

         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnlyNoMotionWithGravity() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();

         ScrewTestTools.setRandomPositions(joints, random);
         //ScrewTestTools.setRandomVelocities(joints, random);

         int rhoSize = wrenchMatrixCalculator.getRhoSize();
         DenseMatrix64F rhoSolution = new DenseMatrix64F(rhoSize, 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         computeZeroForceQddot(dynamicsMatrixCalculator, qddotSolution);

         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnlyNoMotionAndCoriolis() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();

         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         int rhoSize = wrenchMatrixCalculator.getRhoSize();
         DenseMatrix64F rhoSolution = new DenseMatrix64F(rhoSize, 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         computeZeroForceQddot(dynamicsMatrixCalculator, qddotSolution);

         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixOnlyNoMotionGravityAndCoriolis() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();

         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         int rhoSize = wrenchMatrixCalculator.getRhoSize();
         DenseMatrix64F rhoSolution = new DenseMatrix64F(rhoSize, 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         computeZeroForceQddot(dynamicsMatrixCalculator, qddotSolution);

         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testNoLoad() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);

         DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         solveAndCompare(qddotSolution, rhoSolution, false);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);

         DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         solveAndCompare(qddotSolution, rhoSolution, false);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testGravityAndCoriolisOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         solveAndCompare(qddotSolution, rhoSolution, false);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixAndGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);

         DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixAndCoriolisOnly() throws Exception
   {
      setupTest();

      gravityZ = 0.0;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testMassMatrixGravityAndCoriolisOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         DenseMatrix64F rhoSolution = new DenseMatrix64F(wrenchMatrixCalculator.getRhoSize(), 1);
         DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

         solveAndCompare(qddotSolution, rhoSolution, true);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testForceAndGravityOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);

         DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         solveAndCompare(qddotSolution, rhoSolution, false);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testForceGravityAndCoriolisOnly() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);

      for (int i = 0; i < iters; i++)
      {
         inverseDynamicsCalculator.reset();
         dynamicsMatrixCalculator.reset();
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         solveAndCompare(qddotSolution, rhoSolution, false);
      }
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

   private void update()
   {
      fullHumanoidRobotModel.updateFrames();
      toolbox.getTwistCalculator().compute();
      toolbox.getGeometricJacobianHolder().compute();

      wrenchMatrixCalculator.computeMatrices();
      dynamicsMatrixCalculator.compute();
   }

   private void solveAndCompare(DenseMatrix64F qddotSolution, DenseMatrix64F rhoSolution, boolean checkRigidBodyDynamics)
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
      ScrewTools.setDesiredAccelerations(jointIndexHandler.getIndexedJoints(), qddotSolution);
      inverseDynamicsCalculator.compute();

      DynamicsMatrixCalculatorTools.extractTorqueMatrix(jointIndexHandler.getIndexedJoints(), inverseDynamicsTauSolution);

      if (checkRigidBodyDynamics)
         Assert.assertTrue(DynamicsMatrixCalculatorTools.checkFloatingBaseRigidBodyDynamicsSatisfied(dynamicsMatrixCalculator, qddotSolution, dynamicsMatrixTauSolution, rhoSolution));

      for(int i = 0; i < inverseDynamicsTauSolution.getNumRows(); i++)
      {
         if (Math.abs(inverseDynamicsTauSolution.get(i, 0) - dynamicsMatrixTauSolution.get(i, 0)) > tolerance)
         {
            PrintTools.warn("Joint " + jointIndexHandler.getIndexedOneDoFJoints()[i].getName() + " did not result in an equivalent torque");
         }
      }

      Assert.assertTrue(!MatrixTools.isEmptyMatrix(inverseDynamicsTauSolution));
      Assert.assertTrue(!MatrixTools.isEmptyMatrix(dynamicsMatrixTauSolution));
      JUnitTools.assertMatrixEquals(inverseDynamicsTauSolution, dynamicsMatrixTauSolution, tolerance);
   }

   private static void computeZeroForceQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotSolution)
   {
      DenseMatrix64F floatingBaseMassMatrix = dynamicsMatrixCalculator.getFloatingBaseMassMatrix();
      DenseMatrix64F floatingBaseCoriolisMatrix = dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix();

      DenseMatrix64F localFloatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseCoriolisMatrix.getNumRows(), floatingBaseCoriolisMatrix.getNumCols());

      localFloatingBaseCoriolisMatrix.set(floatingBaseCoriolisMatrix);
      CommonOps.scale(-1.0, localFloatingBaseCoriolisMatrix);

      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(localFloatingBaseCoriolisMatrix, qddotSolution);
   }
}
