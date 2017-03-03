package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.ArrayList;
import java.util.Map;
import java.util.Random;

public class DynamicsMatrixCalculatorTest
{
   private final static double tolerance = 0.0001;
   private final static int iters = 1;
   private final static double maxRho = 1.0e2;
   private final static double maxQddot = 1.0e2;

   private final Random random = new Random(5641654L);

   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private WholeBodyControlCoreToolbox toolbox;

   private WrenchMatrixCalculator wrenchMatrixCalculator;
   private JointIndexHandler jointIndexHandler;

   private InverseDynamicsCalculator inverseDynamicsCalculator;
   private DynamicsMatrixCalculator dynamicsMatrixCalculator;
   private CentroidalMomentumHandler centroidalMomentumHandler;

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

         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

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
         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, rhoSize, 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeQddotGivenRho(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

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

         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

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

         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
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

         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         update();
         DynamicsMatrixCalculatorTools.computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotSolution, rhoSolution);

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

         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
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

         DenseMatrix64F rhoSolution = RandomGeometry.nextDenseMatrix64F(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, maxRho);
         DenseMatrix64F qddotSolution = new DenseMatrix64F(degreesOfFreedom, 1);

         solveAndCompare(qddotSolution, rhoSolution, false);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testOther() throws Exception
   {
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

      MomentumOptimizationSettings momentumOptimizationSettings = new GeneralMomentumOptimizationSettings();
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

      centroidalMomentumHandler = new CentroidalMomentumHandler(twistCalculator.getRootBody(), toolbox.getCenterOfMassFrame(), registry);

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
      centroidalMomentumHandler.compute();
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

      checkTestQuality(qddotSolution, rhoSolution, inverseDynamicsTauSolution, dynamicsMatrixTauSolution);
      JUnitTools.assertMatrixEquals(inverseDynamicsTauSolution, dynamicsMatrixTauSolution, tolerance);
   }

   private void checkTestQuality(DenseMatrix64F qddotSolution, DenseMatrix64F rhoSolution, DenseMatrix64F inverseDynamicsSolution, DenseMatrix64F matrixSolution)
   {
      Assert.assertTrue(CommonOps.elementMin(rhoSolution) > -0.000001);

      Assert.assertTrue(CommonOps.elementMax(qddotSolution) < maxQddot);
      Assert.assertTrue(CommonOps.elementMin(qddotSolution) > -maxQddot);

      Assert.assertTrue(!MatrixTools.isEmptyMatrix(inverseDynamicsSolution));
      Assert.assertTrue(!MatrixTools.isEmptyMatrix(matrixSolution));
   }

   private class GeneralMomentumOptimizationSettings extends MomentumOptimizationSettings
   {

      // defaults for unscaled model:
      private static final double defaultRhoWeight = 0.00001;
      private static final double defaultRhoMin = 4.0;
      private static final double defaultRhoRateDefaultWeight = 0.002;
      private static final double defaultRhoRateHighWeight = 0.05;

      private final Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.01);
      private final Vector3D highLinearMomentumWeightForRecovery = new Vector3D(0.5, 0.5, 0.05);
      private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.0);

      private final Vector3D defaultAngularFootWeight = new Vector3D(0.5, 0.5, 0.5);
      private final Vector3D defaultLinearFootWeight = new Vector3D(30.0, 30.0, 30.0);
      private final Vector3D highAngularFootWeight = new Vector3D(5.0, 5.0, 5.0);
      private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

      private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
      private final double spineJointspaceWeight = 1.0;
      private final Vector3D pelvisAngularWeight = new Vector3D(5.0, 5.0, 5.0);

      private final int nBasisVectorsPerContactPoint = 4;
      private final int nContactPointsPerContactableBody = 4;
      private final int nContactableBodies = 2;

      private final double jointAccelerationWeight = 0.005;
      private final double jointJerkWeight = 0.1;
      private final double jointTorqueWeight = 0.005;
      private final Vector2D copWeight = new Vector2D(100.0, 200.0);
      private final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0);
      private final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);
      private final double headJointspaceWeight = 1.0;
      private final double headTaskspaceWeight = 1.0;
      private final double headUserModeWeight = 1.0;
      private final double handUserModeWeight = 50.0;
      private final double handJointspaceWeight = 1.0;
      private final Vector3D handAngularTaskspaceWeight = new Vector3D(1.0, 1.0, 1.0);
      private final Vector3D handLinearTaskspaceWeight = new Vector3D(1.0, 1.0, 1.0);

      private final double rhoWeight;
      private final double rhoMin;
      private final double rhoRateDefaultWeight;
      private final double rhoRateHighWeight;

      public GeneralMomentumOptimizationSettings()
      {
         double scale = 1.0;
         rhoWeight = defaultRhoWeight / scale;
         rhoMin = defaultRhoMin * scale;
         rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
         rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

         linearMomentumWeight.scale(1.0 / scale);
         highLinearMomentumWeightForRecovery.scale(1.0 / scale);
         angularMomentumWeight.scale(1.0 / scale);
      }

      /** @inheritDoc */
      @Override
      public Vector3D getLinearMomentumWeight()
      {
         return linearMomentumWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getHighLinearMomentumWeightForRecovery()
      {
         return highLinearMomentumWeightForRecovery;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getAngularMomentumWeight()
      {
         return angularMomentumWeight;
      }

      /** @inheritDoc */
      @Override
      public double getJointAccelerationWeight()
      {
         return jointAccelerationWeight;
      }

      /** @inheritDoc */
      @Override
      public double getJointJerkWeight()
      {
         return jointJerkWeight;
      }

      /** @inheritDoc */
      @Override
      public double getRhoWeight()
      {
         return rhoWeight;
      }

      /** @inheritDoc */
      @Override
      public double getRhoMin()
      {
         return rhoMin;
      }

      /** @inheritDoc */
      @Override
      public double getRhoRateDefaultWeight()
      {
         return rhoRateDefaultWeight;
      }

      /** @inheritDoc */
      @Override
      public double getRhoRateHighWeight()
      {
         return rhoRateHighWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector2D getCoPWeight()
      {
         return copWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector2D getCoPRateDefaultWeight()
      {
         return copRateDefaultWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector2D getCoPRateHighWeight()
      {
         return copRateHighWeight;
      }

      /** @inheritDoc */
      @Override
      public double getHeadUserModeWeight()
      {
         return headUserModeWeight;
      }

      /** @inheritDoc */
      @Override
      public double getHeadJointspaceWeight()
      {
         return headJointspaceWeight;
      }

      /** @inheritDoc */
      @Override
      public double getHeadTaskspaceWeight()
      {
         return headTaskspaceWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getChestAngularWeight()
      {
         return chestAngularWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getPelvisAngularWeight()
      {
         return pelvisAngularWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getDefaultLinearFootWeight()
      {
         return defaultLinearFootWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getDefaultAngularFootWeight()
      {
         return defaultAngularFootWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getHighLinearFootWeight()
      {
         return highLinearFootWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getHighAngularFootWeight()
      {
         return highAngularFootWeight;
      }

      /** @inheritDoc */
      @Override
      public double getHandUserModeWeight()
      {
         return handUserModeWeight;
      }

      /** @inheritDoc */
      @Override
      public double getHandJointspaceWeight()
      {
         return handJointspaceWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getHandAngularTaskspaceWeight()
      {
         return handAngularTaskspaceWeight;
      }

      /** @inheritDoc */
      @Override
      public Vector3D getHandLinearTaskspaceWeight()
      {
         return handLinearTaskspaceWeight;
      }

      /** @inheritDoc */
      @Override
      public int getNumberOfBasisVectorsPerContactPoint()
      {
         return nBasisVectorsPerContactPoint;
      }

      /** @inheritDoc */
      @Override
      public int getNumberOfContactPointsPerContactableBody()
      {
         return nContactPointsPerContactableBody;
      }

      /** @inheritDoc */
      @Override
      public int getNumberOfContactableBodies()
      {
         return nContactableBodies;
      }

      /** @inheritDoc */
      @Override
      public int getRhoSize()
      {
         return nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
      }

      /** @inheritDoc */
      @Override
      public double getSpineJointspaceWeight()
      {
         return spineJointspaceWeight;
      }
   }
}
