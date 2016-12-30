package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
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

import javax.vecmath.Vector3d;
import java.util.ArrayList;
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
      joints.get(0).getPredecessor().updateFramesRecursively();

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

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(fullHumanoidRobotModel, null, jointArray, momentumOptimizationSettings,
            jointPrivilegedConfigurationParameters, referenceFrames, controlDT, gravityZ, geometricJacobianHolder, twistCalculator, contactablePlaneBodies,
            yoGraphicsListRegistry, registry);

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
