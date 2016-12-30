package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import javax.vecmath.Vector3d;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class DynamicsMatrixCalculatorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testDynamicEquivalentFloatingChainRobot() throws Exception
   {
      Random random = new Random(5641654L);
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      int numberOfJoints = 10;
      double gravityZ = 9.81;
      double controlDT = 0.005;

      Vector3d[] jointAxes = new Vector3d[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);

      FullRobotModelTestTools.RandomFullHumanoidRobotModel fullHumanoidRobotModel = new FullRobotModelTestTools.RandomFullHumanoidRobotModel(random);
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      InverseDynamicsJoint[] jointArray = new InverseDynamicsJoint[joints.size()];
      joints.toArray(jointArray);
      RigidBody elevator = randomFloatingChain.getElevator();

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      joints.get(0).getPredecessor().updateFramesRecursively();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();

      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(null, null, jointArray, momentumOptimizationSettings,
            jointPrivilegedConfigurationParameters, null, controlDT, gravityZ, geometricJacobianHolder, twistCalculator, null, null, registry);
      WrenchMatrixCalculator wrenchMatrixCalculator = new WrenchMatrixCalculator(toolbox, registry);
      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();

      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      DynamicsMatrixCalculator dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox, wrenchMatrixCalculator);

      DenseMatrix64F rhoSolution = RandomTools.generateRandomMatrix(random, wrenchMatrixCalculator.getRhoSize(), 1, 0.0, Double.POSITIVE_INFINITY);
      DenseMatrix64F qddotSolution = RandomTools.generateRandomMatrix(random, jointIndexHandler.getNumberOfDoFs(), 1);

      Map<RigidBody, Wrench> contactWrenches = wrenchMatrixCalculator.computeWrenchesFromRho(rhoSolution);
      RigidBody[] contactsBodies = new RigidBody[contactWrenches.keySet().size()];
      contactWrenches.keySet().toArray(contactsBodies);
      for (int i = 0; i < contactsBodies.length; i++)
         inverseDynamicsCalculator.setExternalWrench(contactsBodies[i], contactWrenches.get(contactsBodies[i]));

      inverseDynamicsCalculator.compute();
   }
}
