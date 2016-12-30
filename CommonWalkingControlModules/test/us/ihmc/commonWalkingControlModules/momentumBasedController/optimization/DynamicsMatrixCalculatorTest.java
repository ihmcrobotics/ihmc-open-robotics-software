package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
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
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class DynamicsMatrixCalculatorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testDynamicEquivalenceFullHumanoidRobot() throws Exception
   {
      Random random = new Random(5641654L);
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      int numberOfJoints = 10;
      double gravityZ = 9.81;
      double controlDT = 0.005;

      Vector3d[] jointAxes = new Vector3d[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);

      FullRobotModelTestTools.RandomFullHumanoidRobotModel fullHumanoidRobotModel = new FullRobotModelTestTools.RandomFullHumanoidRobotModel(random);
      fullHumanoidRobotModel.updateFrames();
      CommonHumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);

      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);
      InverseDynamicsJoint[] jointArray = fullHumanoidRobotModel.getOneDoFJoints();
      RigidBody elevator = fullHumanoidRobotModel.getElevator();

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      fullHumanoidRobotModel.updateFrames();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
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
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(fullHumanoidRobotModel, null, jointsToOptimizeFor, momentumOptimizationSettings,
            jointPrivilegedConfigurationParameters, referenceFrames, controlDT, gravityZ, geometricJacobianHolder, twistCalculator, contactablePlaneBodies,
            yoGraphicsListRegistry, registry);

      WrenchMatrixCalculator wrenchMatrixCalculator = new WrenchMatrixCalculator(toolbox, registry);
      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();

      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      DynamicsMatrixCalculator dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox, wrenchMatrixCalculator);

      int degreesOfFreedom = jointIndexHandler.getNumberOfDoFs();
      int floatingBaseDoFs = fullHumanoidRobotModel.getRootJoint().getDegreesOfFreedom();
      int bodyDoFs = degreesOfFreedom - floatingBaseDoFs;
      DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, 1.0e5);
      DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, degreesOfFreedom, 1);

      wrenchMatrixCalculator.computeMatrices();
      Map<RigidBody, Wrench> contactWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBody rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         inverseDynamicsCalculator.setExternalWrench(rigidBody, contactWrenches.get(rigidBody));
      }

      DenseMatrix64F inverseDynamicsTauSolution = new DenseMatrix64F(bodyDoFs, 1);
      DenseMatrix64F dynamicsMatrixTauSolution = new DenseMatrix64F(bodyDoFs, 1);

      // compute torques using inverse dynamics calculator
      ScrewTools.setDesiredAccelerations(jointArray, qddotSolution);
      inverseDynamicsCalculator.compute();

      DenseMatrix64F tmpTauMatrix = new DenseMatrix64F(1, 1);
      for (int i = 0; i < jointArray.length; i++)
      {
         jointArray[i].getTauMatrix(tmpTauMatrix);
         inverseDynamicsTauSolution.set(i, tmpTauMatrix.get(0, 0));
      }

      // compute torques using dynamics matrix calculator
      dynamicsMatrixCalculator.compute();
      dynamicsMatrixCalculator.computeJointTorques(dynamicsMatrixTauSolution, qddotSolution, rhoSolution);

      JUnitTools.assertMatrixEquals(inverseDynamicsTauSolution, dynamicsMatrixTauSolution, 1.0);
   }
}
