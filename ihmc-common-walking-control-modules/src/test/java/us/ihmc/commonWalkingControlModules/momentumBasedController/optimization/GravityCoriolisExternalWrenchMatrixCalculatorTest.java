package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GravityCoriolisExternalWrenchMatrixCalculatorTest
{
   private final static double tolerance = 0.00001;
   private final static int iters = 1000;

   private final Random random = new Random(5641654L);

   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private WholeBodyControlCoreToolbox toolbox;

   private JointIndexHandler jointIndexHandler;

   private GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;

   private double gravityZ;

   int degreesOfFreedom;
   int floatingBaseDoFs;
   int bodyDoFs;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   /**
    * This should return equivalence, as acceleration should not play a factor in the coriolis, centrifugal, and gravity force matrix.
    * @throws Exception
    */
   @Test
   public void testWithAndWithoutAcceleration() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJointBasics> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);
      JointBasics[] allJoints = jointIndexHandler.getIndexedJoints();

      HashMap<JointBasics, DenseMatrix64F> noAccelCoriolisMatrices = new HashMap<>();
      HashMap<JointBasics, DenseMatrix64F> coriolisMatrices = new HashMap<>();

      for (int i = 0; i < iters; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         update();

         for (int j = 0; j < allJoints.length; j++)
         {
            JointBasics joint = allJoints[j];

            DenseMatrix64F noAccelCoriolisMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
            coriolisMatrixCalculator.getJointCoriolisMatrix(joint, noAccelCoriolisMatrix);
            noAccelCoriolisMatrices.put(joint, noAccelCoriolisMatrix);
         }

         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         coriolisMatrixCalculator.reset();
         update();

         for (int j = 0; j < allJoints.length; j++)
         {
            JointBasics joint = allJoints[j];

            DenseMatrix64F coriolisMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
            coriolisMatrixCalculator.getJointCoriolisMatrix(joint, coriolisMatrix);
            coriolisMatrices.put(joint, coriolisMatrix);
         }

         for (int j = 0; j < allJoints.length; j++)
         {
            JointBasics joint = allJoints[j];
            JUnitTools.assertMatrixEquals(noAccelCoriolisMatrices.get(joint), coriolisMatrices.get(joint), tolerance);
         }
      }
   }

   private void update()
   {
      fullHumanoidRobotModel.updateFrames();

      coriolisMatrixCalculator.compute();
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

      ControllerCoreOptimizationSettings momentumOptimizationSettings = new GeneralMomentumOptimizationSettings();
      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullHumanoidRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullHumanoidRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }

      JointBasics[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullHumanoidRobotModel, new JointBasics[0]);
      FloatingJointBasics rootJoint = fullHumanoidRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, jointsToOptimizeFor, centerOfMassFrame, momentumOptimizationSettings,
                                                yoGraphicsListRegistry, registry);
      toolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      jointIndexHandler = toolbox.getJointIndexHandler();

      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(toolbox.getRootBody(), new ArrayList<>(), toolbox.getGravityZ());

      degreesOfFreedom = jointIndexHandler.getNumberOfDoFs();
      floatingBaseDoFs = fullHumanoidRobotModel.getRootJoint().getDegreesOfFreedom();
      bodyDoFs = degreesOfFreedom - floatingBaseDoFs;
   }

   private class GeneralMomentumOptimizationSettings implements ControllerCoreOptimizationSettings
   {

      // defaults for unscaled model:
      private static final double defaultRhoWeight = 0.00001;
      private static final double defaultRhoMin = 4.0;
      private static final double defaultRhoRateDefaultWeight = 0.002;
      private static final double defaultRhoRateHighWeight = 0.05;

      private final Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.01);
      private final Vector3D highLinearMomentumWeightForRecovery = new Vector3D(0.5, 0.5, 0.05);
      private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.0);

      private final int nBasisVectorsPerContactPoint = 4;
      private final int nContactPointsPerContactableBody = 4;
      private final int nContactableBodies = 2;

      private final double jointAccelerationWeight = 0.005;
      private final double jointJerkWeight = 0.1;
      private final Vector2D copWeight = new Vector2D(100.0, 200.0);
      private final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0);
      private final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);

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

      @Override
      public boolean getDeactivateRhoWhenNotInContact()
      {
         return false;
      }
   }
}
