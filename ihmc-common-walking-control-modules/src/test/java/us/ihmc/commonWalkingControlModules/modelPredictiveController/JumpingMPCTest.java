package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class JumpingMPCTest
{
   private static final double epsilon = 1e-3;

   @Test
   public void testJump()
   {
      double duration = 1.5;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      FrameQuaternion initialBodyOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), -0.0014, -0.0351, -0.0, 0.9994);
      FrameVector3D initialBodyAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), -0.0226, 0.0132, 0.0015);
      FramePoint3D initialCoMPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.0075, 0.0001, 0.8254);
      FrameVector3D initialCoMVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), -0.0272, -0.0004, -0.3081);

      FrameConvexPolygon2D defaultFootPolygon = new FrameConvexPolygon2D();
      defaultFootPolygon.addVertex(-0.11, -0.055);
      defaultFootPolygon.addVertex(-0.11, 0.055);
      defaultFootPolygon.addVertex(0.11, 0.0425);
      defaultFootPolygon.addVertex(0.11, -0.0425);
      defaultFootPolygon.update();

      CoPTrajectoryParameters parameters = new CoPTrajectoryParameters();
      JumpingCoPTrajectoryGeneratorState state = new JumpingCoPTrajectoryGeneratorState(testRegistry);
      state.registerStateToSave(parameters);
      testRegistry.addChild(parameters.getRegistry());
      JumpingCoPTrajectoryGenerator copTrajectoryGenerator = new JumpingCoPTrajectoryGenerator(parameters,
                                                                                               defaultFootPolygon,
                                                                                               new JumpingCoPTrajectoryParameters(testRegistry),
                                                                                               new JumpingParameters(testRegistry),
                                                                                               testRegistry);
      copTrajectoryGenerator.registerState(state);

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(1.5);
      momentOfInertia.setM02(0.1);
      momentOfInertia.setM11(1.5);
      momentOfInertia.setM20(0.1);
      momentOfInertia.setM22(0.5);
      SE3ModelPredictiveController mpc = new SE3ModelPredictiveController(momentOfInertia,
                                                                          new MPCParameters(testRegistry),
                                                                          9.81,
                                                                          1.05,
                                                                          155.94417399999995,
                                                                          0.004,
                                                                          testRegistry);

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(testRegistry);

      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      FrameConvexPolygon2D leftFootPolygon = new FrameConvexPolygon2D();
      FrameConvexPolygon2D rightFootPolygon = new FrameConvexPolygon2D();

      leftFootPose.getPosition().set(-0.0061, 0.1639, -0.0017);
      leftFootPose.getOrientation().set(0.0, -0.0002, 0.0011, 1.0);
      rightFootPose.getPosition().set(-0.0061, -0.1639, -0.0017);
      rightFootPose.getOrientation().set(-0.0002, -0.0001, -0.001, 1.0);

      leftFootPolygon.set(defaultFootPolygon);
      rightFootPolygon.set(defaultFootPolygon);

      // TODO update the state
      JumpingGoal jumpingGoal = new JumpingGoal();
      jumpingGoal.setGoalLength(0.75);
      jumpingGoal.setSupportDuration(0.25);
      jumpingGoal.setFlightDuration(0.25);
      state.setJumpingGoal(jumpingGoal);
      state.setCurrentTimeInState(0.0);
      state.setInitialCoP(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.0196, -0.0001, 0.8163));
      state.setTimeAtStartOfState(0.0);
      state.setFinalTransferDuration(0.25);

      state.initializeStance(RobotSide.LEFT, leftFootPolygon, leftFootPose);
      state.initializeStance(RobotSide.RIGHT, rightFootPolygon, rightFootPose);

      copTrajectoryGenerator.compute(state);

      mpc.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      mpc.setInitialBodyOrientationState(initialBodyOrientation, initialBodyAngularVelocity);
      mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialBodyOrientation, initialBodyAngularVelocity, 0.0);
      mpc.solveForTrajectory(copTrajectoryGenerator.getContactStateProviders());
//      mpc.compute(0.0);

      List<CoMPositionCommand> comPositionCommands = new ArrayList<>();
      List<CoMPositionContinuityCommand> comPositionContinuityCommands = new ArrayList<>();
      List<CoMVelocityContinuityCommand> comVelocityContinuityCommands = new ArrayList<>();
      List<VRPPositionContinuityCommand> vrpContinuityCommands = new ArrayList<>();
      List<CoMVelocityCommand> comVelocityCommands = new ArrayList<>();
      List<DCMPositionCommand> dcmPositionCommands = new ArrayList<>();
      List<OrientationContinuityCommand> orientationContinuityCommands = new ArrayList<>();
      List<VRPPositionCommand> vrpPositionCommands = new ArrayList<>();
      List<VRPTrackingCommand> vrpTrackingCommands = new ArrayList<>();
      for (int i = 0; i < mpc.mpcCommands.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = mpc.mpcCommands.getCommand(i);
         if (command instanceof CoMPositionCommand)
            comPositionCommands.add((CoMPositionCommand) command);
         else if (command instanceof CoMVelocityCommand)
            comVelocityCommands.add((CoMVelocityCommand) command);
         else if (command instanceof DCMPositionCommand)
            dcmPositionCommands.add((DCMPositionCommand) command);
         else if (command instanceof CoMPositionContinuityCommand)
            comPositionContinuityCommands.add((CoMPositionContinuityCommand) command);
         else if (command instanceof CoMVelocityContinuityCommand)
            comVelocityContinuityCommands.add((CoMVelocityContinuityCommand) command);
         else if (command instanceof VRPPositionContinuityCommand)
            vrpContinuityCommands.add((VRPPositionContinuityCommand) command);
         else if (command instanceof OrientationContinuityCommand)
            orientationContinuityCommands.add((OrientationContinuityCommand) command);
      }

      int expectedCoMPositionCommands = 1;
      int expectedCoMVelocityCommands = 1;
      int expectedDCMPositionCommands = 0;
      if (MPCParameters.includeFinalCoMPositionObjective)
         expectedCoMPositionCommands++;
      if (MPCParameters.includeFinalCoMVelocityObjective)
         expectedCoMVelocityCommands++;
      if (MPCParameters.includeFinalDCMPositionObjective)
         expectedDCMPositionCommands++;

      assertEquals(3, comPositionContinuityCommands.size());
      assertEquals(3, comVelocityContinuityCommands.size());
      assertEquals(1, vrpContinuityCommands.size());
      assertEquals(expectedCoMPositionCommands, comPositionCommands.size());
      assertEquals(expectedCoMVelocityCommands, comVelocityCommands.size());
      assertEquals(expectedDCMPositionCommands, dcmPositionCommands.size());

      int numberOfEqualityConstraints = 3 * (comPositionContinuityCommands.size() + comVelocityContinuityCommands.size() + vrpContinuityCommands.size());
      for (CoMPositionCommand command : comPositionCommands)
      {
         if (command.getConstraintType() == ConstraintType.EQUALITY)
            numberOfEqualityConstraints += 3;
      }
      for (CoMVelocityCommand command : comVelocityCommands)
      {
         if (command.getConstraintType() == ConstraintType.EQUALITY)
            numberOfEqualityConstraints += 3;
      }
      for (DCMPositionCommand command : dcmPositionCommands)
      {
         if (command.getConstraintType() == ConstraintType.EQUALITY)
            numberOfEqualityConstraints += 3;
      }
      numberOfEqualityConstraints += 6 * orientationContinuityCommands.size();

      assertEquals(numberOfEqualityConstraints, mpc.qpSolver.qpSolver.linearEqualityConstraintsBVector.getNumRows());
      assertEquals(numberOfEqualityConstraints, mpc.qpSolver.qpSolver.linearEqualityConstraintsAMatrix.getNumRows());

      SE3MPCIndexHandler indexHandler = new SE3MPCIndexHandler(4);
      indexHandler.initialize(mpc.previewWindowCalculator.getPlanningWindow());

      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, 9.81);
//      inputCalculator.calculateCoMContinuityObjectiveInternal()

      NativeMatrix solutionCoefficients = mpc.qpSolver.getSolution();

      DMatrixRMaj linearConstraintMatrix = new DMatrixRMaj(numberOfEqualityConstraints, indexHandler.getTotalProblemSize());
      DMatrixRMaj linearConstraintVector = new DMatrixRMaj(numberOfEqualityConstraints, 1);

      int row = 0;
      if (MPCParameters.initialCoMPositionConstraintType == ConstraintType.EQUALITY)
      {
         DMatrixRMaj tempJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         List<MPCContactPlane> contactPlanes = mpc.contactHandler.getContactPlanesForSegment(0);
         CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(0), 0.0, tempJacobian, 0, 1.0);
         int rhoStart = indexHandler.getRhoCoefficientStartIndex(0);
         for (int i = 0; i < contactPlanes.size(); i++)
         {
            ContactPlaneJacobianCalculator.computeLinearJacobian(0, 0.0, mpc.omega.getValue(), rhoStart, contactPlanes.get(i), tempJacobian);
            rhoStart += contactPlanes.get(i).getCoefficientSize();
         }

         MatrixTools.setMatrixBlock(linearConstraintMatrix, row, 0, tempJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

         row += 3;
      }
      if (MPCParameters.initialCoMVelocityConstraintType == ConstraintType.EQUALITY)
      {
         DMatrixRMaj tempJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

         List<MPCContactPlane> contactPlanes = mpc.contactHandler.getContactPlanesForSegment(0);
         CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(0), 0.0, tempJacobian, 1, 1.0);
         int rhoStart = indexHandler.getRhoCoefficientStartIndex(0);
         for (int i = 0; i < contactPlanes.size(); i++)
         {
            ContactPlaneJacobianCalculator.computeLinearJacobian(1, 0.0, mpc.omega.getValue(), rhoStart, contactPlanes.get(i), tempJacobian);
            rhoStart += contactPlanes.get(i).getCoefficientSize();
         }

         MatrixTools.setMatrixBlock(linearConstraintMatrix, row, 0, tempJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

         row += 3;
      }


      for (int transition = 0; transition < mpc.previewWindowCalculator.getPlanningWindow().size() - 1; transition++)
      {
         int currentSegmentNumber = transition;
         int nextSegmentNumber = transition + 1;
         List<MPCContactPlane> contactPlanesInCurrentSegment = mpc.contactHandler.getContactPlanesForSegment(currentSegmentNumber);
         List<MPCContactPlane> contactPlanesInNextSegment = mpc.contactHandler.getContactPlanesForSegment(nextSegmentNumber);

         double currentSegmentDuration = mpc.previewWindowCalculator.getPlanningWindow().get(currentSegmentNumber).getTimeInterval().getDuration();

         // Position continuity constraint
         {
            DMatrixRMaj tempJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(currentSegmentNumber),
                                                                  currentSegmentDuration,
                                                                  tempJacobian,
                                                                  0,
                                                                  1.0);
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(nextSegmentNumber), 0.0, tempJacobian, 0, -1.0);

            int rhoStart = indexHandler.getRhoCoefficientStartIndex(currentSegmentNumber);
            for (int i = 0; i < contactPlanesInCurrentSegment.size(); i++)
            {
               ContactPlaneJacobianCalculator.computeLinearJacobian(0, currentSegmentDuration, mpc.omega.getValue(), rhoStart, contactPlanesInCurrentSegment.get(i), tempJacobian);
               rhoStart += contactPlanesInCurrentSegment.get(i).getCoefficientSize();
            }

            rhoStart = indexHandler.getRhoCoefficientStartIndex(nextSegmentNumber);
            for (int i = 0; i < contactPlanesInNextSegment.size(); i++)
            {
               ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0, 0, 0.0, mpc.omega.getValue(), rhoStart, contactPlanesInNextSegment.get(i), tempJacobian);
               rhoStart += contactPlanesInNextSegment.get(i).getCoefficientSize();
            }

            linearConstraintVector.set(row + 2, 0, -0.5 * currentSegmentDuration * currentSegmentDuration * -9.81);

            MatrixTools.setMatrixBlock(linearConstraintMatrix, row, 0, tempJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

            row += 3;
         }

         // Velocity continuity constraint
         {
            DMatrixRMaj tempJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(currentSegmentNumber),
                                                                  currentSegmentDuration,
                                                                  tempJacobian,
                                                                  1,
                                                                  1.0);
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(nextSegmentNumber), 0.0, tempJacobian, 1, -1.0);

            int rhoStart = indexHandler.getRhoCoefficientStartIndex(currentSegmentNumber);
            for (int i = 0; i < contactPlanesInCurrentSegment.size(); i++)
            {
               ContactPlaneJacobianCalculator.computeLinearJacobian(1, currentSegmentDuration, mpc.omega.getValue(), rhoStart, contactPlanesInCurrentSegment.get(i), tempJacobian);
               rhoStart += contactPlanesInCurrentSegment.get(i).getCoefficientSize();
            }

            rhoStart = indexHandler.getRhoCoefficientStartIndex(nextSegmentNumber);
            for (int i = 0; i < contactPlanesInNextSegment.size(); i++)
            {
               ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0, 1, 0.0, mpc.omega.getValue(), rhoStart, contactPlanesInNextSegment.get(i), tempJacobian);
               rhoStart += contactPlanesInNextSegment.get(i).getCoefficientSize();
            }

            MatrixTools.setMatrixBlock(linearConstraintMatrix, row, 0, tempJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

            linearConstraintVector.set(row + 2, 0, -currentSegmentDuration * -9.81);

            row += 3;
         }

         // VRP Continuity constraint
         if (contactPlanesInCurrentSegment.size() > 0 && contactPlanesInNextSegment.size() > 0)
         {
            DMatrixRMaj tempJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(currentSegmentNumber),
                                                                  currentSegmentDuration,
                                                                  tempJacobian,
                                                                  0,
                                                                  1.0);
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(currentSegmentNumber),
                                                                  currentSegmentDuration,
                                                                  tempJacobian,
                                                                  2,
                                                                  -1.0 / MathTools.square(mpc.omega.getValue()));
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(nextSegmentNumber), 0.0, tempJacobian, 0, -1.0);
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(indexHandler.getComCoefficientStartIndex(nextSegmentNumber), 0.0, tempJacobian, 2, 1.0 / MathTools.square(mpc.omega.getValue()));

            int rhoStart = indexHandler.getRhoCoefficientStartIndex(currentSegmentNumber);
            for (int i = 0; i < contactPlanesInCurrentSegment.size(); i++)
            {
               ContactPlaneJacobianCalculator.computeLinearJacobian(0, currentSegmentDuration, mpc.omega.getValue(), rhoStart, contactPlanesInCurrentSegment.get(i), tempJacobian);
               ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0 / MathTools.square(mpc.omega.getValue()), 2, currentSegmentDuration, mpc.omega.getValue(), rhoStart, contactPlanesInCurrentSegment.get(i), tempJacobian);
               rhoStart += contactPlanesInCurrentSegment.get(i).getCoefficientSize();
            }

            rhoStart = indexHandler.getRhoCoefficientStartIndex(nextSegmentNumber);
            for (int i = 0; i < contactPlanesInNextSegment.size(); i++)
            {
               ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0, 0, 0.0, mpc.omega.getValue(), rhoStart, contactPlanesInNextSegment.get(i), tempJacobian);
               ContactPlaneJacobianCalculator.computeLinearJacobian(1.0 / MathTools.square(mpc.omega.getValue()), 2, 0.0, mpc.omega.getValue(), rhoStart, contactPlanesInNextSegment.get(i), tempJacobian);
               rhoStart += contactPlanesInNextSegment.get(i).getCoefficientSize();
            }

            MatrixTools.setMatrixBlock(linearConstraintMatrix, row, 0, tempJacobian, 0, 0, 3, indexHandler.getTotalProblemSize(), 1.0);

            linearConstraintVector.set(row + 2, 0, -0.5 * currentSegmentDuration * currentSegmentDuration * -9.81 );

            row += 3;
         }

         // Orientation Continuity constraint
         {
            DMatrixRMaj tempJacobian = new DMatrixRMaj(6, indexHandler.getTotalProblemSize());

            OrientationTrajectoryCommand  trajectoryCommand = mpc.orientationTrajectoryConstructor.getOrientationTrajectoryCommands().get(currentSegmentNumber);
            MatrixTools.setMatrixBlock(tempJacobian,
                                       0,
                                       indexHandler.getOrientationStartIndex(currentSegmentNumber),
                                       trajectoryCommand.getAMatrix(indexHandler.getTicksInSegment(currentSegmentNumber)  - 1),
                                       0,
                                       0,
                                       6,
                                       6,
                                       -1.0);
            MatrixTools.setMatrixBlock(tempJacobian,
                                       0,
                                       indexHandler.getComCoefficientStartIndex(currentSegmentNumber),
                                       trajectoryCommand.getBMatrix(indexHandler.getTicksInSegment(currentSegmentNumber) - 1),
                                       0,
                                       0,
                                       6,
                                       indexHandler.getRhoCoefficientsInSegment(currentSegmentNumber) + LinearMPCIndexHandler.comCoefficientsPerSegment,
                                       -1.0);
            MatrixTools.setMatrixBlock(tempJacobian, 0, indexHandler.getOrientationStartIndex(nextSegmentNumber), CommonOps_DDRM.identity(6), 0, 0, 6, 6, 1.0);

            MatrixTools.setMatrixBlock(linearConstraintMatrix, row, 0, tempJacobian, 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);
            MatrixTools.setMatrixBlock(linearConstraintVector, row, 0, trajectoryCommand.getCMatrix(indexHandler.getTicksInSegment(currentSegmentNumber) - 1), 0, 0, 6, 1, 1.0);

            row += 6;
         }
      }


      MatrixTestTools.assertMatrixEquals(linearConstraintMatrix, mpc.qpSolver.qpSolver.linearEqualityConstraintsAMatrix, 1e-5);
      MatrixTestTools.assertMatrixEquals(linearConstraintVector, mpc.qpSolver.qpSolver.linearEqualityConstraintsBVector, 1e-5);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      mpc.compute(duration - 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), 0.02);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredDCMVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredVRPVelocity(), epsilon);
   }
}
