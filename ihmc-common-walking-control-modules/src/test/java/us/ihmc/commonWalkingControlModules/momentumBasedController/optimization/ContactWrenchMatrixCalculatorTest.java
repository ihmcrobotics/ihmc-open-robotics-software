package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactWrenchMatrixCalculatorTest
{
   private static final double EPSILON = 1.0e-17;
   private static final int ITERATIONS = 100;

   @Test
   public void testAgainstInverseDynamicsCalculator()
   {
      Random random = new Random(45647);
      int numberOfJoints = 40;

      for (int i = 0; i < ITERATIONS; i++)
      {
         RandomFloatingRevoluteJointChain robot = new RandomFloatingRevoluteJointChain(random, numberOfJoints);
         robot.nextState(random, JointStateType.CONFIGURATION);

         ContactablePlaneBody contactablePlaneBody = nextSimpleContactPointPlaneBody(random,
                                                                                     robot.getRevoluteJoints().get(random.nextInt(numberOfJoints))
                                                                                          .getSuccessor());
         WholeBodyControlCoreToolbox toolbox = createToolbox(robot.getRootJoint());
         toolbox.setupForInverseDynamicsSolver(Collections.singletonList(contactablePlaneBody));
         JointIndexHandler jointIndexHandler = new JointIndexHandler(robot.getJoints());

         WrenchMatrixCalculator wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
         wrenchMatrixCalculator.submitPlaneContactStateCommand(nextPlaneContactStateCommand(random, contactablePlaneBody));
         ContactWrenchMatrixCalculator contactWrenchMatrixCalculator = new ContactWrenchMatrixCalculator(toolbox);
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(robot.getElevator());

         wrenchMatrixCalculator.computeMatrices(null);
         DMatrixRMaj rho = RandomMatrices_DDRM.rectangle(wrenchMatrixCalculator.getRhoSize(), 1, 0.0, 10000.0, random);
         Wrench externalWrench = wrenchMatrixCalculator.computeWrenchesFromRho(rho).get(contactablePlaneBody.getRigidBody());
         inverseDynamicsCalculator.getExternalWrench(contactablePlaneBody.getRigidBody()).setMatchingFrame(externalWrench);
         inverseDynamicsCalculator.compute();
         DMatrixRMaj tau_expected = inverseDynamicsCalculator.getJointTauMatrix();

         int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
         DMatrixRMaj contactForceJacobian = new DMatrixRMaj(wrenchMatrixCalculator.getRhoSize(), numberOfDoFs);
         contactWrenchMatrixCalculator.computeContactForceJacobian(contactForceJacobian);
         DMatrixRMaj tau_actual = new DMatrixRMaj(numberOfDoFs, 1);
         CommonOps_DDRM.multTransA(contactForceJacobian, rho, tau_actual);

         boolean areEqual = true;
         for (int row = 0; row < numberOfDoFs; row++)
            areEqual &= MathTools.epsilonEquals(tau_expected.get(row), tau_actual.get(row), Math.max(1.0, Math.abs(tau_expected.get(row)) * EPSILON));

         if (!areEqual)
         {
            System.out.println("iteration: " + i);
            double maxError = 0.0;
            DMatrixRMaj output = new DMatrixRMaj(numberOfDoFs, 3);
            for (int row = 0; row < numberOfDoFs; row++)
            {
               output.set(row, 0, tau_expected.get(row, 0));
               output.set(row, 1, tau_actual.get(row, 0));
               double error = tau_expected.get(row, 0) - tau_actual.get(row, 0);
               output.set(row, 2, error);
               maxError = Math.max(maxError, Math.abs(error));
            }
            output.print(EuclidCoreIOTools.getStringFormat(9, 6));
            System.out.println("Max error: " + maxError);
         }
         assertTrue(areEqual);
      }
   }

   private ContactablePlaneBody nextSimpleContactPointPlaneBody(Random random, RigidBodyBasics body)
   {
      return new SimpleContactPointPlaneBody(body.getName() + "Contact", body, EuclidCoreRandomTools.nextRigidBodyTransform(random));
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

   private WholeBodyControlCoreToolbox createToolbox(FloatingJointBasics rootJoint)
   {
      double controlDT = 1.0e-3;
      double gravityZ = 9.81;
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(rootJoint.getPredecessor());
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), rootBody);
      YoRegistry registry = new YoRegistry("Dummy");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      return new WholeBodyControlCoreToolbox(controlDT,
                                             gravityZ,
                                             rootJoint,
                                             rootJoint.subtreeArray(),
                                             centerOfMassFrame,
                                             new GeneralOptimizationSettings(),
                                             yoGraphicsListRegistry,
                                             registry);
   }

   private class GeneralOptimizationSettings implements ControllerCoreOptimizationSettings
   {

      // defaults for unscaled model:
      private static final double defaultRhoWeight = 0.00001;
      private static final double defaultRhoMin = 4.0;
      private static final double defaultRhoRateDefaultWeight = 0.002;
      private static final double defaultRhoRateHighWeight = 0.05;

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

      public GeneralOptimizationSettings()
      {
         double scale = 1.0;
         rhoWeight = defaultRhoWeight / scale;
         rhoMin = defaultRhoMin * scale;
         rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
         rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);
      }

      @Override
      public double getJointAccelerationWeight()
      {
         return jointAccelerationWeight;
      }

      @Override
      public double getJointJerkWeight()
      {
         return jointJerkWeight;
      }

      @Override
      public double getRhoWeight()
      {
         return rhoWeight;
      }

      @Override
      public double getRhoMin()
      {
         return rhoMin;
      }

      @Override
      public double getRhoRateDefaultWeight()
      {
         return rhoRateDefaultWeight;
      }

      @Override
      public double getRhoRateHighWeight()
      {
         return rhoRateHighWeight;
      }

      @Override
      public Vector2D getCoPWeight()
      {
         return copWeight;
      }

      @Override
      public Vector2D getCoPRateDefaultWeight()
      {
         return copRateDefaultWeight;
      }

      @Override
      public Vector2D getCoPRateHighWeight()
      {
         return copRateHighWeight;
      }

      @Override
      public int getNumberOfBasisVectorsPerContactPoint()
      {
         return nBasisVectorsPerContactPoint;
      }

      @Override
      public int getNumberOfContactPointsPerContactableBody()
      {
         return nContactPointsPerContactableBody;
      }

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
