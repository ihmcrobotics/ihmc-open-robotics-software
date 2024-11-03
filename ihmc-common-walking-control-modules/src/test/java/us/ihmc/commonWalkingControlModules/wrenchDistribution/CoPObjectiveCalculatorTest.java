package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculatorTest;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.*;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelTestTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

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

            copObjectiveCalculator.computeTask(helper.getWrenchJacobianMatrix(), achievedCoP, helper.getRhoSize(), jacobian, objective);

            assertEquals(2, jacobian.getNumRows());
            assertEquals(helper.getRhoSize(), jacobian.getNumCols());

            DMatrixRMaj achievedObjective = new DMatrixRMaj(2, 1);
            CommonOps_DDRM.mult(jacobian, randomRhoVector, achievedObjective);

            assertEquals(0.0, achievedObjective.get(0), 1e-5);
            assertEquals(0.0, achievedObjective.get(1), 1e-5);
         }
      }
   }

   @Test
   public void testSimpleCoPObjectiveSatisfaction()
   {
      Random random = new Random(1738L);

      ReferenceFrame soleFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      RigidBodyBasics elevator = new RigidBody("elevator", soleFrame);
      CenterOfPressureResolver copResolver = new CenterOfPressureResolver();

      QPInputTypeA worldFrameCoPInput = new QPInputTypeA(4);
      worldFrameCoPInput.reshape(2);
      QPInputTypeA verticalForceInput = new QPInputTypeA(4);
      verticalForceInput.reshape(1);

      double footWidth = 0.1;
      double footLength = 0.2;

      // we're manually setting up a jacobian in the sole frame, assuming the contacts are clockwise starting from the front left, and pointing straight up.
      DMatrixRMaj fullWrenchJacobian = new DMatrixRMaj(6, 4);
      // do tau_x
      fullWrenchJacobian.set(0, 0, footWidth / 2.0);
      fullWrenchJacobian.set(0, 1, -footWidth / 2.0);
      fullWrenchJacobian.set(0, 2, -footWidth / 2.0);
      fullWrenchJacobian.set(0, 3, footWidth / 2.0);

      // do tau_y
      fullWrenchJacobian.set(1, 0, -footLength / 2.0);
      fullWrenchJacobian.set(1, 1, -footLength / 2.0);
      fullWrenchJacobian.set(1, 2, footLength / 2.0);
      fullWrenchJacobian.set(1, 3, footLength / 2.0);

      // tau_z is zero
      // f_x is zero
      // f_y is zero
      // f_z is easy
      fullWrenchJacobian.set(5, 0, 1.0);
      fullWrenchJacobian.set(5, 1, 1.0);
      fullWrenchJacobian.set(5, 2, 1.0);
      fullWrenchJacobian.set(5, 3, 1.0);

      double desiredVerticalForce = 10.0;

      // check if the desired CoP is in the front left, which should mean only force on the first of the four rhos
      FramePoint2D desiredCoP = new FramePoint2D(soleFrame, 0.5 * footLength, 0.5 * footWidth);
      worldFrameCoPInput.setTaskWeightMatrix(CommonOps_DDRM.identity(2));

      CoPObjectiveCalculator copObjectiveCalculator = new CoPObjectiveCalculator();
      copObjectiveCalculator.computeTask(fullWrenchJacobian, desiredCoP, 4, worldFrameCoPInput.getTaskJacobian(), worldFrameCoPInput.getTaskObjective());

      // add a task for the vertical force
      CommonOps_DDRM.extract(fullWrenchJacobian, 5, 6, 0, 4, verticalForceInput.getTaskJacobian(), 0, 0);
      verticalForceInput.getTaskObjective().set(0, 0, desiredVerticalForce);
      verticalForceInput.getTaskWeightMatrix().set(0, 0, 100.0);

      // Get the optimal solution to this problem
      QuadraticProblem worldFrameProblem = new QuadraticProblem(worldFrameCoPInput, verticalForceInput);
      DMatrixRMaj optimalSolution = worldFrameProblem.getOptimalSolution();

      // check that the vertical force is maintained, and only on the front left point
      assertEquals(desiredVerticalForce, optimalSolution.get(0, 0), 1e-6);
      assertEquals(0.0, optimalSolution.get(1, 0), 1e-6);
      assertEquals(0.0, optimalSolution.get(2, 0), 1e-6);
      assertEquals(0.0, optimalSolution.get(3, 0), 1e-6);

      // Check that the optimal solution is the desired solution
      DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.mult(fullWrenchJacobian, optimalSolution, wrenchVector);
      Wrench wrenchFromRho = new Wrench(elevator.getBodyFixedFrame(), soleFrame);
      wrenchFromRho.set(wrenchVector);


      FramePoint2D resultingCoP = new FramePoint2D();
      copResolver.resolveCenterOfPressureAndNormalTorque(resultingCoP, wrenchFromRho, soleFrame);

      EuclidFrameTestTools.assertEquals(desiredCoP, resultingCoP, 1e-5);
   }

   private static class QuadraticProblem
   {
      private final DMatrixRMaj gradient = new DMatrixRMaj(0, 0);
      private final DMatrixRMaj hessian = new DMatrixRMaj(0, 0);

      private DMatrixRMaj rhoSolution;

      private boolean validSolution = false;

      public QuadraticProblem(QPInputTypeA... inputs)
      {
         boolean first = true;
         for (QPInputTypeA input : inputs)
         {
            if (first)
            {
               int size = input.getTaskJacobian().getNumCols();
               gradient.reshape(size, 1);
               hessian.reshape(size, size);
               gradient.zero();
               hessian.zero();
               first = false;
            }
            addCostTerm(input);
         }
      }

      public void addCostTerm(DMatrixRMaj otherHessian, DMatrixRMaj otherGradient)
      {
         CommonOps_DDRM.addEquals(hessian, otherHessian);
         CommonOps_DDRM.addEquals(gradient, otherGradient);

         validSolution = false;
      }

      public void addCostTerm(QPInputTypeA input)
      {
         addCostTerm(input.getTaskWeightMatrix(), input.getTaskJacobian(), input.getTaskObjective());
      }

      public void addCostTerm(DMatrixRMaj weight, DMatrixRMaj jacobian, DMatrixRMaj objective)
      {
         int size = jacobian.getNumRows();
         DMatrixRMaj WJ = new DMatrixRMaj(size, size);
         DMatrixRMaj JTWJ = new DMatrixRMaj(size, size);
         CommonOps_DDRM.mult(weight, jacobian, WJ);
         CommonOps_DDRM.multTransA(jacobian, WJ, JTWJ);

         DMatrixRMaj JTWb = new DMatrixRMaj(size, 1);
         CommonOps_DDRM.multTransA(-1.0, WJ, objective, JTWb);

         addCostTerm(JTWJ, JTWb);
      }

      public double evaluateCost(DMatrixRMaj x)
      {
         DMatrixRMaj Wx = new DMatrixRMaj(hessian.getNumRows(), 1);
         CommonOps_DDRM.mult(hessian, x, Wx);
         DMatrixRMaj xTWx = new DMatrixRMaj(1, 1);
         CommonOps_DDRM.multTransA(x, Wx, xTWx);

         DMatrixRMaj wx = new DMatrixRMaj(1, 1);
         CommonOps_DDRM.multTransA(gradient, x, wx);

         return 0.5 * xTWx.get(0, 0) + wx.get(0, 0);
      }

      public DMatrixRMaj computeOptimalSolution()
      {
         LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(hessian.getNumRows());
         DMatrixRMaj objective = new DMatrixRMaj(hessian.getNumRows(), 1);
         CommonOps_DDRM.scale(-1.0, gradient, objective);
         solver.setA(hessian);
         rhoSolution = new DMatrixRMaj(hessian.getNumRows(), 1);
         solver.solve(objective, rhoSolution);

         LogTools.info("Hessian: " + hessian);
         LogTools.info("gradient: " + gradient);
         LogTools.info("solution: " + rhoSolution);
         validSolution = true;
         return rhoSolution;
      }

      public DMatrixRMaj getOptimalSolution()
      {
         if (!validSolution)
            computeOptimalSolution();

         return rhoSolution;
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
      toolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                gravityZ,
                                                rootJoint,
                                                jointsToOptimizeFor,
                                                centerOfMassFrame,
                                                momentumOptimizationSettings,
                                                yoGraphicsListRegistry,
                                                registry);
      toolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
   }

   private void update()
   {
      fullHumanoidRobotModel.updateFrames();

      wrenchMatrixCalculator.computeMatrices(null);
   }
}
