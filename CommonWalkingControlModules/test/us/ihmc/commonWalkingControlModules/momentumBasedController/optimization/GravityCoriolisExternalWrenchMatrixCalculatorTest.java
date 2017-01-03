package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

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

   /**
    * This should return equivalence, as acceleration should not play a factor in the coriolis, centrifugal, and gravity force matrix.
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testWithAndWithoutAcceleration() throws Exception
   {
      setupTest();

      gravityZ = 9.81;

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);
      InverseDynamicsJoint[] allJoints = jointIndexHandler.getIndexedJoints();

      HashMap<InverseDynamicsJoint, DenseMatrix64F> noAccelCoriolisMatrices = new HashMap<>();
      HashMap<InverseDynamicsJoint, DenseMatrix64F> coriolisMatrices = new HashMap<>();

      for (int i = 0; i < iters; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);

         update();

         for (int j = 0; j < allJoints.length; j++)
         {
            InverseDynamicsJoint joint = allJoints[j];

            DenseMatrix64F noAccelCoriolisMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
            coriolisMatrixCalculator.getJointCoriolisMatrix(joint, noAccelCoriolisMatrix);
            noAccelCoriolisMatrices.put(joint, noAccelCoriolisMatrix);
         }

         ScrewTestTools.setRandomAccelerations(joints, random);

         coriolisMatrixCalculator.reset();
         update();

         for (int j = 0; j < allJoints.length; j++)
         {
            InverseDynamicsJoint joint = allJoints[j];

            DenseMatrix64F coriolisMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
            coriolisMatrixCalculator.getJointCoriolisMatrix(joint, coriolisMatrix);
            coriolisMatrices.put(joint, coriolisMatrix);
         }

         for (int j = 0; j < allJoints.length; j++)
         {
            InverseDynamicsJoint joint = allJoints[j];
            JUnitTools.assertMatrixEquals(noAccelCoriolisMatrices.get(joint), coriolisMatrices.get(joint), tolerance);
         }
      }
   }

   private void update()
   {
      fullHumanoidRobotModel.updateFrames();
      toolbox.getTwistCalculator().compute();
      toolbox.getGeometricJacobianHolder().compute();

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

      jointIndexHandler = toolbox.getJointIndexHandler();

      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(toolbox.getTwistCalculator(), new ArrayList<>(), toolbox.getGravityZ());

      degreesOfFreedom = jointIndexHandler.getNumberOfDoFs();
      floatingBaseDoFs = fullHumanoidRobotModel.getRootJoint().getDegreesOfFreedom();
      bodyDoFs = degreesOfFreedom - floatingBaseDoFs;
   }
}
