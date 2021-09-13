package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.apache.batik.svggen.font.table.CmapFormat;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DirectOrientationValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OrientationTrajectoryInputCalculatorTest
{
   private static final double gravityZ = -9.81;
   private static final double mu = 1.0;

   @Test
   public void testEasyContinuityAndInitialValue()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.0, 1.3);

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();

      segment0.set(contact0);
      segment1.set(contact1);
      segment2.set(contact2);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      OrientationTrajectoryInputCalculator inputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);
      SE3MPCQPSolver qpSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));
      qpSolver.setFirstOrientationVariableRegularization(1.0);
      qpSolver.setSecondOrientationVariableRegularization(1.0);

      NativeQPInputTypeA qpInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         DMatrixRMaj rotation0Value = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation1Value = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation1Offset = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation2Value = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation2Offset = new DMatrixRMaj(6, 1);
         rotation0Value.setData(RandomNumbers.nextDoubleArray(random, 6, 1.0));
         rotation1Offset.setData(RandomNumbers.nextDoubleArray(random, 6, 1.0));
         rotation2Offset.setData(RandomNumbers.nextDoubleArray(random, 6, 1.0));
         CommonOps_DDRM.add(rotation0Value, rotation1Offset, rotation1Value);
         CommonOps_DDRM.add(rotation1Value, rotation2Offset, rotation2Value);

         OrientationValueCommand initialValueCommand = new OrientationValueCommand();
         CommonOps_DDRM.setIdentity(initialValueCommand.getAMatrix());
         initialValueCommand.getBMatrix().reshape(6, 12 + indexHandler.getRhoCoefficientsInSegment(0));
         initialValueCommand.getBMatrix().zero();
         initialValueCommand.getCMatrix().zero();
         initialValueCommand.getObjectiveValue().set(rotation0Value);
         initialValueCommand.setSegmentNumber(0);
         initialValueCommand.setConstraintType(ConstraintType.EQUALITY);

         OrientationContinuityCommand segment1Continuity = new OrientationContinuityCommand();
         segment1Continuity.setSegmentNumber(0);
         segment1Continuity.setConstraintType(ConstraintType.EQUALITY);
         CommonOps_DDRM.setIdentity(segment1Continuity.getAMatrix());
         segment1Continuity.getBMatrix().reshape(6, 12 + indexHandler.getRhoCoefficientsInSegment(1));
         segment1Continuity.getBMatrix().zero();
         segment1Continuity.getCMatrix().set(rotation1Offset);

         OrientationContinuityCommand segment2Continuity = new OrientationContinuityCommand();
         segment2Continuity.setSegmentNumber(1);
         segment2Continuity.setConstraintType(ConstraintType.EQUALITY);
         CommonOps_DDRM.setIdentity(segment2Continuity.getAMatrix());
         segment2Continuity.getBMatrix().reshape(6, 12 + indexHandler.getRhoCoefficientsInSegment(2));
         segment2Continuity.getBMatrix().zero();
         segment2Continuity.getCMatrix().set(rotation2Offset);

         qpSolver.initialize();

         inputCalculator.compute(qpInput, initialValueCommand);

         MatrixTestTools.assertMatrixEquals(rotation0Value, qpInput.getTaskObjective(), 1e-5);
         for (int row = 0; row < 6; row++)
            assertEquals(1.0, rowSum(row, qpInput.getTaskJacobian()));

         qpSolver.addInput(qpInput);

         inputCalculator.compute(qpInput, segment1Continuity);
         qpSolver.addInput(qpInput);

         inputCalculator.compute(qpInput, segment2Continuity);
         qpSolver.addInput(qpInput);

         qpSolver.solve();

         DMatrixRMaj rotation0Result = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation1Result = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation2Result = new DMatrixRMaj(6, 1);

         MatrixTools.setMatrixBlock(rotation0Result, 0, 0, qpSolver.getSolution(), indexHandler.getOrientationStartIndex(0), 0, 6, 1, 1.0);
         MatrixTools.setMatrixBlock(rotation1Result, 0, 0, qpSolver.getSolution(), indexHandler.getOrientationStartIndex(1), 0, 6, 1, 1.0);
         MatrixTools.setMatrixBlock(rotation2Result, 0, 0, qpSolver.getSolution(), indexHandler.getOrientationStartIndex(2), 0, 6, 1, 1.0);

         MatrixTestTools.assertMatrixEquals(rotation0Value, rotation0Result, 1e-5);
         MatrixTestTools.assertMatrixEquals(rotation1Value, rotation1Result, 1e-5);
         MatrixTestTools.assertMatrixEquals(rotation2Value, rotation2Result, 1e-5);
      }
   }

   @Test
   public void testEasyContinuityInitialValueAndFinalValue()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.0, 1.3);

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();

      segment0.set(contact0);
      segment1.set(contact1);
      segment2.set(contact2);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      OrientationTrajectoryInputCalculator inputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);
      SE3MPCQPSolver qpSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));
      qpSolver.setFirstOrientationVariableRegularization(1.0);
      qpSolver.setSecondOrientationVariableRegularization(1.0);

      NativeQPInputTypeA qpInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         DMatrixRMaj rotation0Value = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation1Value = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation1Offset = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation2Value = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation2Offset = new DMatrixRMaj(6, 1);
         rotation0Value.setData(RandomNumbers.nextDoubleArray(random, 6, 1.0));
         rotation1Offset.setData(RandomNumbers.nextDoubleArray(random, 6, 1.0));
         rotation2Offset.setData(RandomNumbers.nextDoubleArray(random, 6, 1.0));
         CommonOps_DDRM.add(rotation0Value, rotation1Offset, rotation1Value);
         CommonOps_DDRM.add(rotation1Value, rotation2Offset, rotation2Value);

         DirectOrientationValueCommand initialValueCommand = new DirectOrientationValueCommand();
         initialValueCommand.getObjectiveValue().set(rotation0Value);
         initialValueCommand.setSegmentNumber(0);
         initialValueCommand.setConstraintType(ConstraintType.EQUALITY);

         OrientationContinuityCommand segment1Continuity = new OrientationContinuityCommand();
         segment1Continuity.setSegmentNumber(0);
         segment1Continuity.setConstraintType(ConstraintType.EQUALITY);
         CommonOps_DDRM.setIdentity(segment1Continuity.getAMatrix());
         segment1Continuity.getBMatrix().reshape(6, 12 + indexHandler.getRhoCoefficientsInSegment(1));
         segment1Continuity.getBMatrix().zero();
         segment1Continuity.getCMatrix().set(rotation1Offset);

         OrientationContinuityCommand segment2Continuity = new OrientationContinuityCommand();
         segment2Continuity.setSegmentNumber(1);
         segment2Continuity.setConstraintType(ConstraintType.EQUALITY);
         CommonOps_DDRM.setIdentity(segment2Continuity.getAMatrix());
         segment2Continuity.getBMatrix().reshape(6, 12 + indexHandler.getRhoCoefficientsInSegment(2));
         segment2Continuity.getBMatrix().zero();
         segment2Continuity.getCMatrix().set(rotation2Offset);

         OrientationValueCommand finalValueCommand = new OrientationValueCommand();
         CommonOps_DDRM.setIdentity(finalValueCommand.getAMatrix());
         finalValueCommand.getBMatrix().reshape(6, 12 + indexHandler.getRhoCoefficientsInSegment(0));
         finalValueCommand.getBMatrix().zero();
         finalValueCommand.getCMatrix().zero();
         finalValueCommand.getObjectiveValue().set(rotation0Value);
         finalValueCommand.setSegmentNumber(2);
         finalValueCommand.setConstraintType(ConstraintType.EQUALITY);

         qpSolver.initialize();

         inputCalculator.compute(qpInput, initialValueCommand);

         MatrixTestTools.assertMatrixEquals(rotation0Value, qpInput.getTaskObjective(), 1e-5);
         for (int row = 0; row < 6; row++)
            assertEquals(1.0, rowSum(row, qpInput.getTaskJacobian()));

         qpSolver.addInput(qpInput);

         inputCalculator.compute(qpInput, segment1Continuity);
         qpSolver.addInput(qpInput);

         inputCalculator.compute(qpInput, segment2Continuity);
         qpSolver.addInput(qpInput);

         qpSolver.solve();

         DMatrixRMaj rotation0Result = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation1Result = new DMatrixRMaj(6, 1);
         DMatrixRMaj rotation2Result = new DMatrixRMaj(6, 1);

         MatrixTools.setMatrixBlock(rotation0Result, 0, 0, qpSolver.getSolution(), indexHandler.getOrientationStartIndex(0), 0, 6, 1, 1.0);
         MatrixTools.setMatrixBlock(rotation1Result, 0, 0, qpSolver.getSolution(), indexHandler.getOrientationStartIndex(1), 0, 6, 1, 1.0);
         MatrixTools.setMatrixBlock(rotation2Result, 0, 0, qpSolver.getSolution(), indexHandler.getOrientationStartIndex(2), 0, 6, 1, 1.0);

         MatrixTestTools.assertMatrixEquals(rotation0Value, rotation0Result, 1e-5);
         MatrixTestTools.assertMatrixEquals(rotation1Value, rotation1Result, 1e-5);
         MatrixTestTools.assertMatrixEquals(rotation2Value, rotation2Result, 1e-5);
      }
   }

   @Test
   public void testComputeDirectOrientationValueCommandCompactIsTheSame()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.0, 1.3);

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();
      segment0.set(contact0);
      segment2.set(contact2);
      segment1.set(contact1);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      OrientationTrajectoryInputCalculator inputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);
      SE3MPCQPSolver qpSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));
      SE3MPCQPSolver compactSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));

      NativeQPInputTypeA regularInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());
      NativeQPInputTypeA compactInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         qpSolver.initialize();
         compactSolver.initialize();

         DMatrixRMaj Objective = new DMatrixRMaj(6, 1);
         Objective.setData(RandomNumbers.nextDoubleArray(random, Objective.getNumElements(), 1.0));

         DirectOrientationValueCommand trajectoryCommand = new DirectOrientationValueCommand();
         trajectoryCommand.setSegmentNumber(1);
         trajectoryCommand.setObjectiveValue(Objective);
         trajectoryCommand.setObjectiveWeight(1.0);

         int regularOffset = inputCalculator.compute(regularInput, trajectoryCommand);
         int compactOffset = inputCalculator.computeCompact(compactInput, trajectoryCommand);

         qpSolver.addInput(regularInput, regularOffset);
         compactSolver.addInput(compactInput, compactOffset);

         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.costQuadraticMatrix, compactSolver.qpSolver.costQuadraticMatrix, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.quadraticCostQVector, compactSolver.qpSolver.quadraticCostQVector, 1e-5);
      }

   }

   @Test
   public void testComputeOrientationValueCommandCompactIsTheSame()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.0, 1.3);


      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();
      segment0.set(contact0);
      segment2.set(contact2);
      segment1.set(contact1);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      OrientationTrajectoryInputCalculator inputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);
      SE3MPCQPSolver qpSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));
      SE3MPCQPSolver compactSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));

      NativeQPInputTypeA regularInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());
      NativeQPInputTypeA compactInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         qpSolver.initialize();
         compactSolver.initialize();

         DMatrixRMaj AMatrix = new DMatrixRMaj(6, 6);
         AMatrix.setData(RandomNumbers.nextDoubleArray(random, AMatrix.getNumElements(), 1.0));

         DMatrixRMaj BMatrix = new DMatrixRMaj(6, indexHandler.getRhoCoefficientsInSegment(1) + LinearMPCIndexHandler.comCoefficientsPerSegment);
         BMatrix.setData(RandomNumbers.nextDoubleArray(random, BMatrix.getNumElements(), 1.0));

         DMatrixRMaj CMatrix = new DMatrixRMaj(6, 1);
         CMatrix.setData(RandomNumbers.nextDoubleArray(random, CMatrix.getNumElements(), 1.0));

         DMatrixRMaj Objective = new DMatrixRMaj(6, 1);
         Objective.setData(RandomNumbers.nextDoubleArray(random, Objective.getNumElements(), 1.0));

         OrientationValueCommand trajectoryCommand = new OrientationValueCommand();
         trajectoryCommand.setSegmentNumber(1);
         trajectoryCommand.setAMatrix(AMatrix);
         trajectoryCommand.setBMatrix(BMatrix);
         trajectoryCommand.setCMatrix(CMatrix);
         trajectoryCommand.setObjectiveValue(Objective);
         trajectoryCommand.setObjectiveWeight(1.0);

         int regularOffset = inputCalculator.compute(regularInput, trajectoryCommand);
         int compactOffset = inputCalculator.computeCompact(compactInput, trajectoryCommand);

         qpSolver.addInput(regularInput, regularOffset);
         compactSolver.addInput(compactInput, compactOffset);

         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.costQuadraticMatrix, compactSolver.qpSolver.costQuadraticMatrix, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.quadraticCostQVector, compactSolver.qpSolver.quadraticCostQVector, 1e-5);
      }

   }

   @Test
   public void testComputeOrientationContinuityCommandCompactIsTheSame()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.0, 1.3);

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();
      segment0.set(contact0);
      segment2.set(contact2);
      segment1.set(contact1);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      OrientationTrajectoryInputCalculator inputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);
      SE3MPCQPSolver qpSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));
      SE3MPCQPSolver compactSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));

      NativeQPInputTypeA regularInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());
      NativeQPInputTypeA compactInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         qpSolver.initialize();
         compactSolver.initialize();

         DMatrixRMaj AMatrix = new DMatrixRMaj(6, 6);
         AMatrix.setData(RandomNumbers.nextDoubleArray(random, AMatrix.getNumElements(), 1.0));

         DMatrixRMaj BMatrix = new DMatrixRMaj(6, indexHandler.getRhoCoefficientsInSegment(1) + LinearMPCIndexHandler.comCoefficientsPerSegment);
         BMatrix.setData(RandomNumbers.nextDoubleArray(random, BMatrix.getNumElements(), 1.0));

         DMatrixRMaj CMatrix = new DMatrixRMaj(6, 1);
         CMatrix.setData(RandomNumbers.nextDoubleArray(random, CMatrix.getNumElements(), 1.0));

         OrientationContinuityCommand trajectoryCommand = new OrientationContinuityCommand();
         trajectoryCommand.setSegmentNumber(1);
         trajectoryCommand.setAMatrix(AMatrix);
         trajectoryCommand.setBMatrix(BMatrix);
         trajectoryCommand.setCMatrix(CMatrix);
         trajectoryCommand.setObjectiveWeight(1.0);

         int regularOffset = inputCalculator.compute(regularInput, trajectoryCommand);
         int compactOffset = inputCalculator.computeCompact(compactInput, trajectoryCommand);

         qpSolver.addInput(regularInput, regularOffset);
         compactSolver.addInput(compactInput, compactOffset);

         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.costQuadraticMatrix, compactSolver.qpSolver.costQuadraticMatrix, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.quadraticCostQVector, compactSolver.qpSolver.quadraticCostQVector, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.linearEqualityConstraintsAMatrix, compactSolver.qpSolver.linearEqualityConstraintsAMatrix, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.linearEqualityConstraintsBVector, compactSolver.qpSolver.linearEqualityConstraintsBVector, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.linearInequalityConstraintsDVectorO, compactSolver.qpSolver.linearInequalityConstraintsDVectorO, 1e-5);
      }

   }

   @Test
   public void testComputeOrientationTrajectoryCommandCompactIsTheSame()
   {
      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);

      MPCContactPlane leftContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());
      MPCContactPlane rightContactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<PreviewWindowSegment> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D leftContactPose = new FramePose3D();
      FramePose3D rightContactPose = new FramePose3D();
      leftContactPose.getPosition().setY(0.2);
      rightContactPose.getPosition().setY(-0.2);

      leftContactPlane.computeBasisVectors(contactPolygon, leftContactPose, mu);
      rightContactPlane.computeBasisVectors(contactPolygon, rightContactPose, mu);

      ContactPlaneProvider contact0 = new ContactPlaneProvider();
      contact0.getTimeInterval().setInterval(0.0, 0.5);
      contact0.addContact(leftContactPose, contactPolygon);
      contact0.addContact(rightContactPose, contactPolygon);
      contact0.setStartECMPPosition(new FramePoint3D());
      contact0.setEndECMPPosition(new FramePoint3D());

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.set(contact0);
      contact1.getTimeInterval().setInterval(0.5, 1.0);
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.set(contact1);
      contact2.getTimeInterval().setInterval(1.0, 1.3);

      PreviewWindowSegment segment0 = new PreviewWindowSegment();
      PreviewWindowSegment segment1 = new PreviewWindowSegment();
      PreviewWindowSegment segment2 = new PreviewWindowSegment();
      segment0.set(contact0);
      segment2.set(contact2);
      segment1.set(contact1);

      contactProviders.add(segment0);
      contactProviders.add(segment1);
      contactProviders.add(segment2);

      indexHandler.initialize(contactProviders);

      OrientationTrajectoryInputCalculator inputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);
      SE3MPCQPSolver qpSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));
      SE3MPCQPSolver compactSolver = new SE3MPCQPSolver(indexHandler, 0.001, gravityZ, new YoRegistry("test"));

      NativeQPInputTypeA regularInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());
      NativeQPInputTypeA compactInput = new NativeQPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         qpSolver.initialize();
         compactSolver.initialize();

         DMatrixRMaj AMatrix = new DMatrixRMaj(6, 6);
         AMatrix.setData(RandomNumbers.nextDoubleArray(random, AMatrix.getNumElements(), 1.0));

         DMatrixRMaj BMatrix = new DMatrixRMaj(6, indexHandler.getRhoCoefficientsInSegment(1) + LinearMPCIndexHandler.comCoefficientsPerSegment);
         BMatrix.setData(RandomNumbers.nextDoubleArray(random, BMatrix.getNumElements(), 1.0));

         DMatrixRMaj CMatrix = new DMatrixRMaj(6, 1);
         CMatrix.setData(RandomNumbers.nextDoubleArray(random, CMatrix.getNumElements(), 1.0));

         OrientationTrajectoryCommand trajectoryCommand = new OrientationTrajectoryCommand();
         trajectoryCommand.setSegmentNumber(1);
         trajectoryCommand.addAMatrix().set(AMatrix);
         trajectoryCommand.addBMatrix().set(BMatrix);
         trajectoryCommand.addCMatrix().set(CMatrix);
         trajectoryCommand.setAngleErrorMinimizationWeight(1.0);
         trajectoryCommand.setVelocityErrorMinimizationWeight(1.0);

         int regularOffset = inputCalculator.compute(0, regularInput, trajectoryCommand);
         int compactOffset = inputCalculator.computeCompact(0, compactInput, trajectoryCommand);

         qpSolver.addInput(regularInput, regularOffset);
         compactSolver.addInput(compactInput, compactOffset);

         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.costQuadraticMatrix, compactSolver.qpSolver.costQuadraticMatrix, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.quadraticCostQVector, compactSolver.qpSolver.quadraticCostQVector, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.linearEqualityConstraintsAMatrix, compactSolver.qpSolver.linearEqualityConstraintsAMatrix, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.linearEqualityConstraintsBVector, compactSolver.qpSolver.linearEqualityConstraintsBVector, 1e-5);
         MatrixTestTools.assertMatrixEquals(qpSolver.qpSolver.linearInequalityConstraintsDVectorO, compactSolver.qpSolver.linearInequalityConstraintsDVectorO, 1e-5);
      }

   }



   private static double rowSum(int row, DMatrix matrix)
   {
      double sum = 0.0;
      for (int i = 0; i < matrix.getNumCols(); i++)
         sum += matrix.get(row, i);

      return sum;
   }

   private static double colSum(int col, DMatrix matrix)
   {
      double sum = 0.0;
      for (int i = 0; i < matrix.getNumRows(); i++)
         sum += matrix.get(i, col);

      return sum;
   }
}
