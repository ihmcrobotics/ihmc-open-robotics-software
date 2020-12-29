package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNotEquals;

public class CoMTrajectoryModelPredictiveControllerTest
{
   private static final double epsilon = 1e-3;
   private static final boolean visualize = false;

   @Test
   public void testSimpleStanding()
   {
      double gravityZ = -9.81;
      double dt = 0.001;
      double nominalHeight = 1.0;
      double duration = 1.5;
      double omega = Math.sqrt(Math.abs(gravityZ) / nominalHeight);
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      CoMTrajectoryModelPredictiveController mpc = new CoMTrajectoryModelPredictiveController(gravityZ, nominalHeight, dt, testRegistry);

      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();


      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.5, 0.3, 0.0);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, duration);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(contactPose.getPosition());
      contact.setEndCopPosition(contactPose.getPosition());

      contactProviders.add(contact);

      FramePoint3D initialCoM = new FramePoint3D(contactPose.getPosition());
      initialCoM.setZ(nominalHeight);

      mpc.setCurrentCenterOfMassState(initialCoM, new FrameVector3D(), initialCoM, 0.0);
      mpc.solveForTrajectory(contactProviders);
      mpc.compute(0.0);

      List<CoMPositionCommand> comPositionCommands = new ArrayList<>();
      List<CoMVelocityCommand> comVelocityCommands = new ArrayList<>();
      List<DCMPositionCommand> dcmPositionCommands = new ArrayList<>();
      List<VRPPositionCommand> vrpPositionCommands = new ArrayList<>();
      List<VRPTrackingCommand> vrpTrackingCommands = new ArrayList<>();
      for (int i = 0; i < mpc.mpcCommands.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = mpc.mpcCommands.getCommand(i);
         if (command.getCommandType() == MPCCommandType.VALUE)
         {
            MPCValueType valueType = ((MPCValueCommand) command).getValueType();
            int derivativeOrder = ((MPCValueCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionCommands.add((CoMPositionCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityCommands.add((CoMVelocityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.DCM)
            {
               if (derivativeOrder == 0)
                  dcmPositionCommands.add((DCMPositionCommand) mpc.mpcCommands.getCommand(i));
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionCommands.add((VRPPositionCommand) mpc.mpcCommands.getCommand(i));
            }
         }
         else if (command.getCommandType() == MPCCommandType.VRP_TRACKING)
         {
            vrpTrackingCommands.add((VRPTrackingCommand) mpc.mpcCommands.getCommand(i));
         }
      }

      assertEquals(1, comPositionCommands.size());
      assertEquals(1, comVelocityCommands.size());
      assertEquals(1, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(1, vrpTrackingCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue(), dcmPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, dcmPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, dcmPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue(), vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, vrpPositionCommands.get(0).getObjective(), epsilon);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, 0.8);

      DMatrixRMaj solutionCoefficients = mpc.qpSolver.getSolution();

      DMatrixRMaj expectedSolutionMatrix = MPCTestHelper.getVectorOfCoefficients(gravityZ, rhoHelper, solutionCoefficients);
      DMatrixRMaj expectedContactForceMatrix = MPCTestHelper.getContactForceCoefficients(rhoHelper, solutionCoefficients);
      DMatrixRMaj contactForceMatrix = mpc.contactPlaneHelperPool.get(0).get(0).getContactWrenchCoefficientMatrix();

      EjmlUnitTests.assertEquals(expectedContactForceMatrix, contactForceMatrix, epsilon);

      assertCoefficientsEqual(expectedSolutionMatrix, mpc.trajectoryHandler);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      mpc.compute(duration - 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), 0.02);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredDCMVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredVRPVelocity(), epsilon);

      visualize(mpc, contactProviders, duration);
   }

   @Test
   public void testStandingTwoSegments()
   {
      double gravityZ = -9.81;
      double dt = 0.001;
      double nominalHeight = 1.0;
      double duration = 0.5;
      double omega = Math.sqrt(Math.abs(gravityZ) / nominalHeight);
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      CoMTrajectoryModelPredictiveController mpc = new CoMTrajectoryModelPredictiveController(gravityZ, nominalHeight, dt, testRegistry);

      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));


      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();


      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.5, 0.3, 0.0);

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.0, duration);
      contact1.addContact(contactPose, contactPolygon);
      contact1.setStartCopPosition(contactPose.getPosition());
      contact1.setEndCopPosition(contactPose.getPosition());
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.getTimeInterval().setInterval(duration, 2.0 * duration);
      contact2.addContact(contactPose, contactPolygon);
      contact2.setStartCopPosition(contactPose.getPosition());
      contact2.setEndCopPosition(contactPose.getPosition());

      contactProviders.add(contact1);
      contactProviders.add(contact2);

      FramePoint3D initialCoM = new FramePoint3D(contactPose.getPosition());
      initialCoM.setZ(nominalHeight);

      mpc.setCurrentCenterOfMassState(initialCoM, new FrameVector3D(), initialCoM, 0.0);
      mpc.solveForTrajectory(contactProviders);
      mpc.compute(0.0);

      List<CoMPositionCommand> comPositionCommands = new ArrayList<>();
      List<CoMVelocityCommand> comVelocityCommands = new ArrayList<>();
      List<DCMPositionCommand> dcmPositionCommands = new ArrayList<>();
      List<VRPPositionCommand> vrpPositionCommands = new ArrayList<>();
      List<VRPTrackingCommand> vrpTrackingCommands = new ArrayList<>();
      List<CoMPositionContinuityCommand> comPositionContinuityCommands = new ArrayList<>();
      List<CoMVelocityContinuityCommand> comVelocityContinuityCommands = new ArrayList<>();
      List<VRPPositionContinuityCommand> vrpPositionContinuityCommands = new ArrayList<>();
      for (int i = 0; i < mpc.mpcCommands.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = mpc.mpcCommands.getCommand(i);
         if (command.getCommandType() == MPCCommandType.VALUE)
         {
            MPCValueType valueType = ((MPCValueCommand) command).getValueType();
            int derivativeOrder = ((MPCValueCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionCommands.add((CoMPositionCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityCommands.add((CoMVelocityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.DCM)
            {
               if (derivativeOrder == 0)
                  dcmPositionCommands.add((DCMPositionCommand) mpc.mpcCommands.getCommand(i));
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionCommands.add((VRPPositionCommand) mpc.mpcCommands.getCommand(i));
            }
         }
         else if (command.getCommandType() == MPCCommandType.VRP_TRACKING)
         {
            vrpTrackingCommands.add((VRPTrackingCommand) command);
         }
         else if (command.getCommandType() == MPCCommandType.CONTINUITY)
         {
            MPCValueType valueType = ((MPCContinuityCommand) command).getValueType();
            int derivativeOrder = ((MPCContinuityCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionContinuityCommands.add((CoMPositionContinuityCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityContinuityCommands.add((CoMVelocityContinuityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionContinuityCommands.add((VRPPositionContinuityCommand) mpc.mpcCommands.getCommand(i));
            }
         }
      }

      assertEquals(1, comPositionCommands.size());
      assertEquals(1, comVelocityCommands.size());
      assertEquals(1, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(2, vrpTrackingCommands.size());
      assertEquals(1, comPositionContinuityCommands.size());
      assertEquals(1, comVelocityContinuityCommands.size());
      assertEquals(1, vrpPositionContinuityCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue() - duration, dcmPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, dcmPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, dcmPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue() - duration, vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, vrpPositionCommands.get(0).getObjective(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredVRPVelocity(), 2e-3);

      // end of first segment
      mpc.compute(duration - 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredVRPVelocity(), epsilon);

      // beginning of next segment
      mpc.compute(duration + 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      // right before final duration
      mpc.compute(previewWindowLength.getDoubleValue() - 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);


      // right after final duration
      mpc.compute(previewWindowLength.getDoubleValue() + 0.01);

//      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
//      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      visualize(mpc, contactProviders, duration);
   }

   @Test
   public void testStandingTwoSegmentsTwoFeet()
   {
      double gravityZ = -9.81;
      double dt = 0.001;
      double nominalHeight = 1.0;
      double duration = 0.5;
      double omega = Math.sqrt(Math.abs(gravityZ) / nominalHeight);
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      CoMTrajectoryModelPredictiveController mpc = new CoMTrajectoryModelPredictiveController(gravityZ, nominalHeight, dt, testRegistry);

      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();


      FramePose3D leftContactPose = new FramePose3D();
      leftContactPose.getPosition().set(0.5, 0.1, 0.0);

      FramePose3D rightContactPose = new FramePose3D();
      rightContactPose.getPosition().set(0.5, 0.4, 0.0);

      FramePoint3D vrp = new FramePoint3D();
      vrp.interpolate(leftContactPose.getPosition(), rightContactPose.getPosition(), 0.5);



      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.0, duration);
      contact1.addContact(leftContactPose, contactPolygon);
      contact1.addContact(rightContactPose, contactPolygon);
      contact1.setStartCopPosition(vrp);
      contact1.setEndCopPosition(vrp);

      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.getTimeInterval().setInterval(duration, 2.0 * duration);
      contact2.addContact(leftContactPose, contactPolygon);
      contact2.addContact(rightContactPose, contactPolygon);
      contact2.setStartCopPosition(vrp);
      contact2.setEndCopPosition(vrp);

      contactProviders.add(contact1);
      contactProviders.add(contact2);

      FramePoint3D initialCoM = new FramePoint3D(vrp);
      initialCoM.setZ(nominalHeight);

      mpc.setCurrentCenterOfMassState(initialCoM, new FrameVector3D(), initialCoM, 0.0);
      mpc.solveForTrajectory(contactProviders);
      mpc.compute(0.0);

      List<CoMPositionCommand> comPositionCommands = new ArrayList<>();
      List<CoMVelocityCommand> comVelocityCommands = new ArrayList<>();
      List<DCMPositionCommand> dcmPositionCommands = new ArrayList<>();
      List<VRPPositionCommand> vrpPositionCommands = new ArrayList<>();
      List<VRPTrackingCommand> vrpTrackingCommands = new ArrayList<>();
      List<CoMPositionContinuityCommand> comPositionContinuityCommands = new ArrayList<>();
      List<CoMVelocityContinuityCommand> comVelocityContinuityCommands = new ArrayList<>();
      List<VRPPositionContinuityCommand> vrpPositionContinuityCommands = new ArrayList<>();
      for (int i = 0; i < mpc.mpcCommands.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = mpc.mpcCommands.getCommand(i);
         if (command.getCommandType() == MPCCommandType.VALUE)
         {
            MPCValueType valueType = ((MPCValueCommand) command).getValueType();
            int derivativeOrder = ((MPCValueCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionCommands.add((CoMPositionCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityCommands.add((CoMVelocityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.DCM)
            {
               if (derivativeOrder == 0)
                  dcmPositionCommands.add((DCMPositionCommand) mpc.mpcCommands.getCommand(i));
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionCommands.add((VRPPositionCommand) mpc.mpcCommands.getCommand(i));
            }
         }
         else if (command.getCommandType() == MPCCommandType.VRP_TRACKING)
         {
            vrpTrackingCommands.add((VRPTrackingCommand) command);
         }
         else if (command.getCommandType() == MPCCommandType.CONTINUITY)
         {
            MPCValueType valueType = ((MPCContinuityCommand) command).getValueType();
            int derivativeOrder = ((MPCContinuityCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionContinuityCommands.add((CoMPositionContinuityCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityContinuityCommands.add((CoMVelocityContinuityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionContinuityCommands.add((VRPPositionContinuityCommand) mpc.mpcCommands.getCommand(i));
            }
         }
      }

      assertEquals(2, comPositionCommands.size());
      assertEquals(2, comVelocityCommands.size());
      assertEquals(0, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(2, vrpTrackingCommands.size());
      assertEquals(1, comPositionContinuityCommands.size());
      assertEquals(1, comVelocityContinuityCommands.size());
      assertEquals(1, vrpPositionContinuityCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue() - duration, comPositionCommands.get(1).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(1).getOmega(), epsilon);
//      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(1).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue() - duration, comVelocityCommands.get(1).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(1).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(1).getObjective(), epsilon);


      assertEquals(previewWindowLength.getDoubleValue() - duration, vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, vrpPositionCommands.get(0).getObjective(), epsilon);




      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      for (double time = 0.0; time < duration; time += 0.01)
      {
         mpc.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredVRPPosition(), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredDCMPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      }

      Random random = new Random(1738L);
      FramePoint3D modifiedCoM = new FramePoint3D();

      for (double time = 0.0; time < duration; time += 0.01)
      {
         modifiedCoM.set(initialCoM);
         modifiedCoM.add(EuclidCoreRandomTools.nextVector3D(random, -0.05, 0.05));

         mpc.setCurrentCenterOfMassState(modifiedCoM, new FrameVector3D(), initialCoM, time);
         mpc.solveForTrajectory(contactProviders);
         mpc.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(modifiedCoM, mpc.getDesiredCoMPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      }


      visualize(mpc, contactProviders, duration);
   }


   @Test
   public void testSimpleStep()
   {
      double gravityZ = -9.81;
      double dt = 0.001;
      double nominalHeight = 1.0;
      double duration = 0.5;
      double omega = Math.sqrt(Math.abs(gravityZ) / nominalHeight);
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      CoMTrajectoryModelPredictiveController mpc = new CoMTrajectoryModelPredictiveController(gravityZ, nominalHeight, dt, testRegistry);
      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();


      FramePose3D contactPose1 = new FramePose3D();
      FramePose3D contactPose2 = new FramePose3D();
      contactPose1.getPosition().set(0.5, 0.3, 0.0);
      contactPose2.getPosition().set(0.7, 0.4, 0.0);

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.0, duration);
      contact1.addContact(contactPose1, contactPolygon);
      contact1.setStartCopPosition(contactPose1.getPosition());
      contact1.setEndCopPosition(contactPose1.getPosition());

      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.getTimeInterval().setInterval(duration, 2 * duration);
      contact2.addContact(contactPose2, contactPolygon);
      contact2.setStartCopPosition(contactPose2.getPosition());
      contact2.setEndCopPosition(contactPose2.getPosition());

      contactProviders.add(contact1);
      contactProviders.add(contact2);

      FramePoint3D initialCoM = new FramePoint3D(contactPose1.getPosition());
      initialCoM.setZ(nominalHeight);
      FrameVector3D initialCoMVelocity = new FrameVector3D();

      FramePoint3D finalDCM = new FramePoint3D(contactPose2.getPosition());
      finalDCM.setZ(nominalHeight);

      mpc.setCurrentCenterOfMassState(initialCoM, initialCoMVelocity, initialCoM, 0.0);
      mpc.solveForTrajectory(contactProviders);
      mpc.compute(0.0);

      List<CoMPositionContinuityCommand> comPositionContinuityCommands = new ArrayList<>();
      List<VRPPositionContinuityCommand> vrpPositionContinuityCommands = new ArrayList<>();
      List<CoMVelocityContinuityCommand> comVelocityContinuityCommands = new ArrayList<>();
      List<CoMPositionCommand> comPositionCommands = new ArrayList<>();
      List<CoMVelocityCommand> comVelocityCommands = new ArrayList<>();
      List<DCMPositionCommand> dcmPositionCommands = new ArrayList<>();
      List<VRPPositionCommand> vrpPositionCommands = new ArrayList<>();

      for (int i = 0; i < mpc.mpcCommands.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = mpc.mpcCommands.getCommand(i);
         if (command.getCommandType() == MPCCommandType.VALUE)
         {
            MPCValueType valueType = ((MPCValueCommand) command).getValueType();
            int derivativeOrder = ((MPCValueCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionCommands.add((CoMPositionCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityCommands.add((CoMVelocityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.DCM)
            {
               if (derivativeOrder == 0)
                  dcmPositionCommands.add((DCMPositionCommand) mpc.mpcCommands.getCommand(i));
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionCommands.add((VRPPositionCommand) mpc.mpcCommands.getCommand(i));
            }
         }
         else if (command.getCommandType() == MPCCommandType.CONTINUITY)
         {
            MPCValueType valueType = ((MPCContinuityCommand) command).getValueType();
            int derivativeOrder = ((MPCContinuityCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionContinuityCommands.add((CoMPositionContinuityCommand) command);
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityContinuityCommands.add((CoMVelocityContinuityCommand) command);
               }
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionContinuityCommands.add((VRPPositionContinuityCommand) mpc.mpcCommands.getCommand(i));
            }
         }
      }

      assertEquals(2, comPositionCommands.size());
      assertEquals(2, comVelocityCommands.size());
//      assertEquals(1, dcmPositionCommands.size());
      assertEquals(1, comPositionContinuityCommands.size());
      assertEquals(1, comVelocityContinuityCommands.size());
      assertEquals(1, vrpPositionContinuityCommands.size());
      assertEquals(1, vrpPositionCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

//      assertEquals(previewWindowLength.getDoubleValue() - duration, dcmPositionCommands.get(0).getTimeOfObjective(), epsilon);
//      assertEquals(omega, dcmPositionCommands.get(0).getOmega(), epsilon);
//      EuclidCoreTestTools.assertTuple3DEquals(finalDCM, dcmPositionCommands.get(0).getObjective(), epsilon);


      assertEquals(previewWindowLength.getDoubleValue() - duration, vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(finalDCM, vrpPositionCommands.get(0).getObjective(), epsilon);


      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), 5-3);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(initialCoMVelocity, mpc.getDesiredCoMVelocity(), 5e-3);

      double windowDelta = 0.0005;
      mpc.compute(duration - windowDelta);
      FramePoint3D vrpPositionRightBefore = new FramePoint3D(mpc.getDesiredVRPPosition());
      FramePoint3D comPositionRightBefore = new FramePoint3D(mpc.getDesiredCoMPosition());
      FramePoint3D comVelocityRightBefore = new FramePoint3D(mpc.getDesiredCoMVelocity());
      FramePoint3D dcmPositionRightBefore = new FramePoint3D(mpc.getDesiredDCMPosition());

      mpc.compute(duration + windowDelta);
      FramePoint3D vrpPositionRightAfter = new FramePoint3D(mpc.getDesiredVRPPosition());
      FramePoint3D comPositionRightAfter = new FramePoint3D(mpc.getDesiredCoMPosition());
      FramePoint3D comVelocityRightAfter = new FramePoint3D(mpc.getDesiredCoMVelocity());
      FramePoint3D dcmPositionRightAfter = new FramePoint3D(mpc.getDesiredDCMPosition());

      mpc.compute(previewWindowLength.getDoubleValue() - windowDelta);
      FramePoint3D vrpPositionEndOfPreview = new FramePoint3D(mpc.getDesiredVRPPosition());
      FramePoint3D comPositionEndOfPreview = new FramePoint3D(mpc.getDesiredCoMPosition());
      FramePoint3D comVelocityEndOfPreview = new FramePoint3D(mpc.getDesiredCoMVelocity());
      FramePoint3D dcmPositionEndOfPreview = new FramePoint3D(mpc.getDesiredDCMPosition());

      mpc.compute(previewWindowLength.getDoubleValue() + windowDelta);
      FramePoint3D vrpPositionEndOfPreviewAfter = new FramePoint3D(mpc.getDesiredVRPPosition());
      FramePoint3D comPositionEndOfPreviewAfter = new FramePoint3D(mpc.getDesiredCoMPosition());
      FramePoint3D comVelocityEndOfPreviewAfter = new FramePoint3D(mpc.getDesiredCoMVelocity());
      FramePoint3D dcmPositionEndOfPreviewAfter = new FramePoint3D(mpc.getDesiredDCMPosition());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPositionRightBefore, comPositionRightAfter, epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comVelocityRightBefore, comVelocityRightAfter, epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(dcmPositionRightBefore, dcmPositionRightAfter, epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(vrpPositionRightBefore, vrpPositionRightAfter, 3e-3);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(dcmPositionEndOfPreview, dcmPositionEndOfPreviewAfter, epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(vrpPositionEndOfPreview, vrpPositionEndOfPreviewAfter, 3e-3);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPositionEndOfPreview, comPositionEndOfPreviewAfter, epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comVelocityEndOfPreview, comVelocityEndOfPreviewAfter, epsilon);

      visualize(mpc, contactProviders, duration);
   }

   @Test
   public void testSimpleStandingFewRhos()
   {
      double gravityZ = -9.81;
      double dt = 0.001;
      double nominalHeight = 1.0;
      double duration = 1.5;
      double omega = Math.sqrt(Math.abs(gravityZ) / nominalHeight);
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      CoMTrajectoryModelPredictiveController mpc = new CoMTrajectoryModelPredictiveController(gravityZ, nominalHeight, dt, testRegistry);
      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(0, 0);
      contactPolygon.update();

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.5, 0.3, 0.0);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, duration);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(contactPose.getPosition());
      contact.setEndCopPosition(contactPose.getPosition());

      contactProviders.add(contact);

      FramePoint3D initialCoM = new FramePoint3D(contactPose.getPosition());
      initialCoM.setZ(nominalHeight);

      mpc.setCurrentCenterOfMassState(initialCoM, new FrameVector3D(), initialCoM, 0.0);
      mpc.solveForTrajectory(contactProviders);
      mpc.compute(0.0);

      List<CoMPositionCommand> comPositionCommands = new ArrayList<>();
      List<CoMVelocityCommand> comVelocityCommands = new ArrayList<>();
      List<DCMPositionCommand> dcmPositionCommands = new ArrayList<>();
      List<VRPPositionCommand> vrpPositionCommands = new ArrayList<>();
      List<VRPTrackingCommand> vrpTrackingCommands = new ArrayList<>();
      for (int i = 0; i < mpc.mpcCommands.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = mpc.mpcCommands.getCommand(i);
         if (command.getCommandType() == MPCCommandType.VALUE)
         {
            MPCValueType valueType = ((MPCValueCommand) command).getValueType();
            int derivativeOrder = ((MPCValueCommand) command).getDerivativeOrder();
            if (valueType == MPCValueType.COM)
            {
               if (derivativeOrder == 0)
               {
                  comPositionCommands.add((CoMPositionCommand) mpc.mpcCommands.getCommand(i));
               }
               else if (derivativeOrder == 1)
               {
                  comVelocityCommands.add((CoMVelocityCommand) mpc.mpcCommands.getCommand(i));
               }
            }
            else if (valueType == MPCValueType.DCM)
            {
               if (derivativeOrder == 0)
                  dcmPositionCommands.add((DCMPositionCommand) mpc.mpcCommands.getCommand(i));
            }
            else if (valueType == MPCValueType.VRP)
            {
               if (derivativeOrder == 0)
                  vrpPositionCommands.add((VRPPositionCommand) mpc.mpcCommands.getCommand(i));
            }
         }
         else if (command.getCommandType() == MPCCommandType.VRP_TRACKING)
         {
            vrpTrackingCommands.add((VRPTrackingCommand) command);
         }
      }

      assertEquals(1, comPositionCommands.size());
      assertEquals(1, comVelocityCommands.size());
      assertEquals(1, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(1, vrpTrackingCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue(), dcmPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, dcmPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, dcmPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(previewWindowLength.getDoubleValue(), vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, vrpPositionCommands.get(0).getObjective(), epsilon);


      for (int i = 0 ; i < 22; i++)
      {
         assertNotEquals(0.0, mpc.qpSolver.solverInput_H.get(i, i), epsilon);
      }
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
   }

   private static void visualize(CoMTrajectoryModelPredictiveController mpc, List<ContactPlaneProvider> contacts, double duration)
   {
      if (!visualize || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));
      YoRegistry registry = scs.getRootRegistry();
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      CoMMPCVisualizer mpcVisualizer = new CoMMPCVisualizer(mpc, scs, registry, graphicsListRegistry);

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();

      mpc.solveForTrajectory(contacts);
      mpcVisualizer.visualize(duration);

      ThreadTools.sleepForever();
   }

   private static void assertCoefficientsEqual(DMatrixRMaj solution, TrajectoryHandler trajectoryHandler)
   {
      for (int i = 0; i < 6; i++)
      {
         assertEquals("Col " + i + " wrong", solution.get(0, i), trajectoryHandler.xCoefficientVector.get(i), epsilon);
         assertEquals("Col " + i + " wrong", solution.get(1, i), trajectoryHandler.yCoefficientVector.get(i), epsilon);
         assertEquals("Col " + i + " wrong", solution.get(2, i), trajectoryHandler.zCoefficientVector.get(i), epsilon);
      }
   }
}
