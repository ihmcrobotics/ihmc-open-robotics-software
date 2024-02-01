package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculatorTest;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class CoPObjectiveCalculatorTest
{
   private FullHumanoidRobotModel fullHumanoidRobotModel;
   private WholeBodyControlCoreToolbox toolbox;
   private static final double gravityZ = 9.81;
   private final Random random = new Random(1738L);
   private WrenchMatrixCalculator wrenchMatrixCalculator;
   private final static int iters = 1000;
   private final static double maxRho = 1.0e2;

   @Test
   public void testFormulationWithSimpleJacobian()
   {
      setupTest();


      int degreesOfFreedom = toolbox.getJointIndexHandler().getNumberOfDoFs();

      ArrayList<OneDoFJointBasics> joints = new ArrayList<>();
      fullHumanoidRobotModel.getOneDoFJoints(joints);
      CoPObjectiveCalculator copObjectiveCalculator = new CoPObjectiveCalculator();


      for (int i = 0; i < iters; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         WrenchMatrixCalculator wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
         for (ContactablePlaneBody contactablePlaneBody : toolbox.getContactablePlaneBodies())
            wrenchMatrixCalculator.submitPlaneContactStateCommand(nextPlaneContactStateCommand(random, contactablePlaneBody));

         update();

         for (ContactablePlaneBody contactablePlaneBody : toolbox.getContactablePlaneBodies())
         {
            FramePoint2D desiredCoP = EuclidFrameRandomTools.nextFramePoint2D(random, contactablePlaneBody.getSoleFrame());

            PlaneContactStateToWrenchMatrixHelper helper = wrenchMatrixCalculator.getPlaneContactStateToWrenchMatrixHelper(contactablePlaneBody.getRigidBody());
            DMatrixRMaj jacobian = new DMatrixRMaj(2, helper.getRhoSize());
            DMatrixRMaj objective = new DMatrixRMaj(2, helper.getRhoSize());

            assertEquals(6, helper.getWrenchJacobianMatrix().getNumRows());
            assertEquals(helper.getRhoSize(), helper.getWrenchJacobianMatrix().getNumCols());


            DMatrixRMaj randomRhoVector = RandomMatrices_DDRM.rectangle(helper.getRhoSize(), 1, random);
            DMatrixRMaj randomWrenchVector = new DMatrixRMaj(6, 1);
            CommonOps_DDRM.mult(helper.getWrenchJacobianMatrix(), randomRhoVector, randomWrenchVector);

            Wrench wrench = new Wrench(contactablePlaneBody.getRigidBody().getBodyFixedFrame(), contactablePlaneBody.getSoleFrame());
            wrench.set(randomWrenchVector);

            FramePoint2DReadOnly achievedCoP = computeCoPFromWrench(wrench);

            copObjectiveCalculator.computeTask(helper.getWrenchJacobianMatrix(),
                                               achievedCoP,
                                               helper.getRhoSize(),
                                               jacobian,
                                               objective);

            assertEquals(2, jacobian.getNumRows());
            assertEquals(helper.getRhoSize(), jacobian.getNumCols());

            DMatrixRMaj achievedObjective = new DMatrixRMaj(2, 1);
            CommonOps_DDRM.mult(jacobian, randomRhoVector, achievedObjective);

            assertEquals(0.0, achievedObjective.get(0), 1e-5);
            assertEquals(0.0, achievedObjective.get(1), 1e-5);
         }
      }
   }


   private PlaneContactStateCommand nextPlaneContactStateCommand(Random random, ContactablePlaneBody contactablePlaneBody)
   {
      PlaneContactStateCommand next = new PlaneContactStateCommand();
      next.setContactingRigidBody(contactablePlaneBody.getRigidBody());
      next.setCoefficientOfFriction(random.nextDouble());
      next.setContactNormal(EuclidFrameRandomTools.nextFrameVector3DWithFixedLength(random, contactablePlaneBody.getSoleFrame(), 1.0));
      next.setHasContactStateChanged(true);

      for (FramePoint3D contactPoint : contactablePlaneBody.getContactPointsCopy())
      {
         next.addPointInContact(contactPoint);
      }
      return next;
   }

   private FramePoint2DReadOnly computeCoPFromWrench(WrenchReadOnly wrench)
   {
      FramePoint2D cop = new FramePoint2D(wrench.getReferenceFrame());
      cop.setX(-wrench.getAngularPartY() / wrench.getLinearPartZ());
      cop.setY(wrench.getAngularPartX() / wrench.getLinearPartZ());

      return cop;
   }


   private void setupTest()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      double controlDT = 0.005;

      fullHumanoidRobotModel = new FullRobotModelTestTools.RandomFullHumanoidRobotModel(random);
      fullHumanoidRobotModel.updateFrames();
      CommonHumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);

      ControllerCoreOptimizationSettings momentumOptimizationSettings = new DynamicsMatrixCalculatorTest.GeneralMomentumOptimizationSettings();
      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullHumanoidRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullHumanoidRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }

      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      JointBasics[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullHumanoidRobotModel, new JointBasics[0]);

      FloatingJointBasics rootJoint = fullHumanoidRobotModel.getRootJoint();
      toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, jointsToOptimizeFor, centerOfMassFrame, momentumOptimizationSettings,
                                                yoGraphicsListRegistry, registry);
      toolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
   }


   private void update()
   {
      fullHumanoidRobotModel.updateFrames();

      wrenchMatrixCalculator.computeMatrices(null);
   }



}
