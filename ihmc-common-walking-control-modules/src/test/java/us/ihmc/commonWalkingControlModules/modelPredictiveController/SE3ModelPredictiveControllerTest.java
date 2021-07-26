package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCTestHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.SE3MPCVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNotEquals;

public class SE3ModelPredictiveControllerTest
{
   private static final double epsilon = 5e-2;
   private static final boolean visualize = false;

   private static final double gravityZ = -9.81;
   private static final double mass = 10.0;
   private static final double dt = 0.001;
   private static final double nominalHeight = 1.0;
   private static final double omega = Math.sqrt(Math.abs(gravityZ) / nominalHeight);

   private static final double Ixx = 1.0;
   private static final double Iyy = 1.0;
   private static final double Izz = 1.0;
   private static final Matrix3D momentOfInertia = new Matrix3D();

   static
   {
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);
   }

   @Test
   public void testSimpleStanding()
   {
      double duration = 1.5;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      SE3ModelPredictiveController mpc = new SE3ModelPredictiveController(momentOfInertia,
                                                                                                        new MPCParameters(testRegistry),
                                                                                                        gravityZ,
                                                                                                        nominalHeight,
                                                                                                        mass,
                                                                                                        dt,
                                                                                                        testRegistry);

      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.5, 0.3, 0.0);

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, duration);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(contactPose.getPosition());
      contact.setEndECMPPosition(contactPose.getPosition());

      contactProviders.add(contact);

      FramePoint3D initialCoMPosition = new FramePoint3D(contactPose.getPosition());
      FrameVector3D initialCoMVelocity = new FrameVector3D();
      FrameQuaternion initialOrientation = new FrameQuaternion();
      FrameVector3D initialAngularVelocity = new FrameVector3D();
      initialCoMPosition.setZ(nominalHeight);

      mpc.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      mpc.setInitialBodyOrientationState(initialOrientation, initialAngularVelocity);
      mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialOrientation, initialAngularVelocity, 0.0);
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

      int expectedCoMPositionCommands = 1;
      int expectedCoMVelocityCommands = 1;
      int expectedDCMPositionCommands = 0;
      if (MPCParameters.includeFinalCoMPositionObjective)
         expectedCoMPositionCommands++;
      if (MPCParameters.includeFinalCoMVelocityObjective)
         expectedCoMVelocityCommands++;
      if (MPCParameters.includeFinalDCMPositionObjective)
         expectedDCMPositionCommands++;
      assertEquals(expectedCoMPositionCommands, comPositionCommands.size());
      assertEquals(expectedCoMVelocityCommands, comVelocityCommands.size());
      assertEquals(expectedDCMPositionCommands, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(1, vrpTrackingCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      if (MPCParameters.includeFinalCoMPositionObjective)
      {
         assertEquals(previewWindowLength.getDoubleValue(), comPositionCommands.get(1).getTimeOfObjective(), epsilon);
         assertEquals(omega, comPositionCommands.get(1).getOmega(), epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, comPositionCommands.get(1).getObjective(), epsilon);
      }

      assertEquals(previewWindowLength.getDoubleValue(), vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, vrpPositionCommands.get(0).getObjective(), epsilon);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(initialCoMPosition, vrpTrackingCommands.get(0).getStartVRP(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(initialCoMPosition, vrpTrackingCommands.get(0).getEndVRP(), epsilon);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, 0.8);

      NativeMatrix solutionCoefficients = mpc.qpSolver.getSolution();

      visualize(mpc,
                contactProviders,
                duration,
                initialCoMPosition,
                initialCoMVelocity,
                initialOrientation,
                initialAngularVelocity,
                testRegistry);

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

   @Test
   public void testStandingTwoSegments()
   {
      double duration = 0.5;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      SE3ModelPredictiveController mpc = new SE3ModelPredictiveController(momentOfInertia,
                                                                          new MPCParameters(testRegistry),
                                                                          gravityZ,
                                                                          nominalHeight,
                                                                          mass,
                                                                          dt,
                                                                          testRegistry);

      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.5, 0.3, 0.0);

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.0, duration);
      contact1.addContact(contactPose, contactPolygon);
      contact1.setStartECMPPosition(contactPose.getPosition());
      contact1.setEndECMPPosition(contactPose.getPosition());
      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.getTimeInterval().setInterval(duration, 2.0 * duration);
      contact2.addContact(contactPose, contactPolygon);
      contact2.setStartECMPPosition(contactPose.getPosition());
      contact2.setEndECMPPosition(contactPose.getPosition());

      contactProviders.add(contact1);
      contactProviders.add(contact2);

      FramePoint3D initialCoMPosition = new FramePoint3D(contactPose.getPosition());
      FrameVector3D initialCoMVelocity = new FrameVector3D();
      FrameQuaternion initialOrientation = new FrameQuaternion();
      FrameVector3D initialAngularVelocity = new FrameVector3D();
      initialCoMPosition.setZ(nominalHeight);

      mpc.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      mpc.setInitialBodyOrientationState(initialOrientation, initialAngularVelocity);
      mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialOrientation, initialAngularVelocity, 0.0);
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

      int expectedCoMPositionCommands = 1;
      int expectedCoMVelocityCommands = 1;
      int expectedDCMPositionCommands = 0;
      if (MPCParameters.includeFinalCoMPositionObjective)
         expectedCoMPositionCommands++;
      if (MPCParameters.includeFinalCoMVelocityObjective)
         expectedCoMVelocityCommands++;
      if (MPCParameters.includeFinalDCMPositionObjective)
         expectedDCMPositionCommands++;
      assertEquals(expectedCoMPositionCommands, comPositionCommands.size());
      assertEquals(expectedCoMVelocityCommands, comVelocityCommands.size());
      assertEquals(expectedDCMPositionCommands, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(2, vrpTrackingCommands.size());
      assertEquals(1, comPositionContinuityCommands.size());
      assertEquals(1, comVelocityContinuityCommands.size());
      assertEquals(1, vrpPositionContinuityCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      if (MPCParameters.includeFinalCoMPositionObjective)
      {
         assertEquals(previewWindowLength.getDoubleValue() - duration, comPositionCommands.get(1).getTimeOfObjective(), epsilon);
         assertEquals(omega, comPositionCommands.get(1).getOmega(), epsilon);
         //      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(1).getObjective(), epsilon);
      }

      assertEquals(previewWindowLength.getDoubleValue() - duration, vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, vrpPositionCommands.get(0).getObjective(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredVRPVelocity(), 0.05);

      // end of first segment
      mpc.compute(duration - 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredVRPVelocity(), 0.05);

      // beginning of next segment
      mpc.compute(duration + 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      // right before final duration
      mpc.compute(previewWindowLength.getDoubleValue() - 0.01);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      // right after final duration
      mpc.compute(previewWindowLength.getDoubleValue() + 0.01);

      //      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
      //      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      visualize(mpc,
                contactProviders,
                duration,
                initialCoMPosition,
                initialCoMVelocity,
                initialOrientation,
                initialAngularVelocity,
                testRegistry);
   }

   @Test
   public void testStandingTwoSegmentsTwoFeet()
   {
      double duration = 0.5;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      SE3ModelPredictiveController mpc = new SE3ModelPredictiveController(momentOfInertia,
                                                                          new MPCParameters(testRegistry),
                                                                          gravityZ,
                                                                                                        nominalHeight,
                                                                                                        mass,
                                                                                                        dt,
                                                                                                        testRegistry);

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
      contact1.setStartECMPPosition(vrp);
      contact1.setEndECMPPosition(vrp);

      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.getTimeInterval().setInterval(duration, 2.0 * duration);
      contact2.addContact(leftContactPose, contactPolygon);
      contact2.addContact(rightContactPose, contactPolygon);
      contact2.setStartECMPPosition(vrp);
      contact2.setEndECMPPosition(vrp);

      contactProviders.add(contact1);
      contactProviders.add(contact2);

      FramePoint3D initialCoMPosition = new FramePoint3D(vrp);
      FrameVector3D initialCoMVelocity = new FrameVector3D();
      FrameQuaternion initialOrientation = new FrameQuaternion();
      FrameVector3D initialAngularVelocity = new FrameVector3D();
      initialCoMPosition.setZ(nominalHeight);

      mpc.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      mpc.setInitialBodyOrientationState(initialOrientation, initialAngularVelocity);
      mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialOrientation, initialAngularVelocity, 0.0);
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

      int expectedCoMPositionCommands = 1;
      int expectedCoMVelocityCommands = 1;
      int expectedDCMPositionCommands = 0;
      if (MPCParameters.includeFinalCoMPositionObjective)
         expectedCoMPositionCommands++;
      if (MPCParameters.includeFinalCoMVelocityObjective)
         expectedCoMVelocityCommands++;
      if (MPCParameters.includeFinalDCMPositionObjective)
         expectedDCMPositionCommands++;
      assertEquals(expectedCoMPositionCommands, comPositionCommands.size());
      assertEquals(expectedCoMVelocityCommands, comVelocityCommands.size());
      assertEquals(expectedDCMPositionCommands, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(2, vrpTrackingCommands.size());
      assertEquals(1, comPositionContinuityCommands.size());
      assertEquals(1, comVelocityContinuityCommands.size());
      assertEquals(1, vrpPositionContinuityCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      if (MPCParameters.includeFinalCoMPositionObjective)
      {
         assertEquals(previewWindowLength.getDoubleValue() - duration, comPositionCommands.get(1).getTimeOfObjective(), epsilon);
         assertEquals(omega, comPositionCommands.get(1).getOmega(), epsilon);
         //      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(1).getObjective(), epsilon);
      }

      if (MPCParameters.includeFinalCoMVelocityObjective)
      {
         assertEquals(previewWindowLength.getDoubleValue() - duration, comVelocityCommands.get(1).getTimeOfObjective(), epsilon);
         assertEquals(omega, comVelocityCommands.get(1).getOmega(), epsilon);
         //      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(1).getObjective(), epsilon);
      }

      assertEquals(previewWindowLength.getDoubleValue() - duration, vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, vrpPositionCommands.get(0).getObjective(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);

      for (double time = 0.0; time < duration; time += 0.01)
      {
         mpc.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredDCMPosition(), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredVRPPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      }

      visualize(mpc,
                contactProviders,
                duration,
                initialCoMPosition,
                initialCoMVelocity,
                initialOrientation,
                initialAngularVelocity,
                testRegistry);

      Random random = new Random(1738L);
      FramePoint3D modifiedCoM = new FramePoint3D();
      FrameQuaternion modifiedOrientation = new FrameQuaternion();

      double time = 0.0;
      for (; time < 0.49; time += 0.01)
      {
         modifiedCoM.set(initialCoMPosition);
         modifiedCoM.add(EuclidCoreRandomTools.nextVector3D(random, -0.05, 0.05));

         Vector3D axisAngleErrorVector = EuclidCoreRandomTools.nextVector3D(random, -0.05, 0.05);
         AxisAngle axisAngleError = new AxisAngle();
         axisAngleError.setRotationVector(axisAngleErrorVector);
         modifiedOrientation.set(initialOrientation);
         modifiedOrientation.append(axisAngleError);

         mpc.setCurrentState(modifiedCoM, initialCoMVelocity, modifiedOrientation, initialAngularVelocity, time);
         mpc.solveForTrajectory(contactProviders);
         mpc.compute(time);

         String errorMessage = "failed at time " + time;

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, modifiedCoM, mpc.getDesiredCoMPosition(), 1e-1);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      }

      for (; time < duration; time += 0.01)
      {
         modifiedCoM.set(initialCoMPosition);
         modifiedCoM.add(EuclidCoreRandomTools.nextVector3D(random, -0.05, 0.05));

         modifiedOrientation.set(initialOrientation);
         modifiedOrientation.append(EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame()));

         mpc.setCurrentState(modifiedCoM, initialCoMVelocity, modifiedOrientation, initialAngularVelocity, time);
         mpc.solveForTrajectory(contactProviders);
         mpc.compute(time);

         String errorMessage = "failed at time " + time;

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, modifiedCoM, mpc.getDesiredCoMPosition(), 0.06);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
      }
   }

   @Test
   public void testSimpleStep()
   {
      double contactDuration = 0.5;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      SE3ModelPredictiveController mpc = new SE3ModelPredictiveController(momentOfInertia,
                                                                                                        new MPCParameters(testRegistry),
                                                                                                        gravityZ,
                                                                                                        nominalHeight,
                                                                                                        mass,
                                                                                                        dt,
                                                                                                        testRegistry);
      YoDouble previewWindowLength = ((YoDouble) testRegistry.findVariable("previewWindowDuration"));

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose1 = new FramePose3D();
      FramePose3D contactPose2 = new FramePose3D();
      contactPose1.getPosition().set(0.5, 0.3, 0.0);
      contactPose2.getPosition().set(0.7, 0.4, 0.0);

      ContactPlaneProvider contact1 = new ContactPlaneProvider();
      contact1.getTimeInterval().setInterval(0.0, contactDuration);
      contact1.addContact(contactPose1, contactPolygon);
      contact1.setStartECMPPosition(contactPose1.getPosition());
      contact1.setEndECMPPosition(contactPose1.getPosition());

      ContactPlaneProvider contact2 = new ContactPlaneProvider();
      contact2.getTimeInterval().setInterval(contactDuration, 2 * contactDuration);
      contact2.addContact(contactPose2, contactPolygon);
      contact2.setStartECMPPosition(contactPose2.getPosition());
      contact2.setEndECMPPosition(contactPose2.getPosition());

      contactProviders.add(contact1);
      contactProviders.add(contact2);

      FramePoint3D initialCoMPosition = new FramePoint3D(contactPose1.getPosition());
      FrameVector3D initialCoMVelocity = new FrameVector3D();
      FrameQuaternion initialOrientation = new FrameQuaternion();
      FrameVector3D initialAngularVelocity = new FrameVector3D();

      initialCoMPosition.setZ(nominalHeight);

      mpc.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      mpc.setInitialBodyOrientationState(initialOrientation, initialAngularVelocity);
      mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialOrientation, initialAngularVelocity, 0.0);

      FramePoint3D finalDCM = new FramePoint3D(contactPose2.getPosition());
      finalDCM.setZ(nominalHeight);
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

      int expectedCoMPositionCommands = 1;
      int expectedCoMVelocityCommands = 1;
      int expectedDCMPositionCommands = 0;
      if (MPCParameters.includeFinalCoMPositionObjective)
         expectedCoMPositionCommands++;
      if (MPCParameters.includeFinalCoMVelocityObjective)
         expectedCoMVelocityCommands++;
      if (MPCParameters.includeFinalDCMPositionObjective)
         expectedDCMPositionCommands++;
      assertEquals(expectedCoMPositionCommands, comPositionCommands.size());
      assertEquals(expectedCoMVelocityCommands, comVelocityCommands.size());
      //      assertEquals(1, dcmPositionCommands.size());
      assertEquals(1, comPositionContinuityCommands.size());
      assertEquals(1, comVelocityContinuityCommands.size());
      assertEquals(1, vrpPositionContinuityCommands.size());
      assertEquals(1, vrpPositionCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoMPosition, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      //      assertEquals(previewWindowLength.getDoubleValue() - duration, dcmPositionCommands.get(0).getTimeOfObjective(), epsilon);
      //      assertEquals(omega, dcmPositionCommands.get(0).getOmega(), epsilon);
      //      EuclidCoreTestTools.assertTuple3DEquals(finalDCM, dcmPositionCommands.get(0).getObjective(), epsilon);

      visualize(mpc,
                contactProviders,
                2 * contactDuration,
                initialCoMPosition,
                initialCoMVelocity,
                initialOrientation,
                initialAngularVelocity,
                testRegistry);

      assertEquals(previewWindowLength.getDoubleValue() - contactDuration, vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(finalDCM, vrpPositionCommands.get(0).getObjective(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, mpc.getDesiredCoMPosition(), 5e-3);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(initialCoMVelocity, mpc.getDesiredCoMVelocity(), 5e-2);

      double windowDelta = 0.0005;
      mpc.compute(contactDuration - windowDelta);
      FramePoint3D vrpPositionRightBefore = new FramePoint3D(mpc.getDesiredVRPPosition());
      FramePoint3D comPositionRightBefore = new FramePoint3D(mpc.getDesiredCoMPosition());
      FramePoint3D comVelocityRightBefore = new FramePoint3D(mpc.getDesiredCoMVelocity());
      FramePoint3D dcmPositionRightBefore = new FramePoint3D(mpc.getDesiredDCMPosition());

      mpc.compute(contactDuration + windowDelta);
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
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPositionEndOfPreview, comPositionEndOfPreviewAfter, epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comVelocityEndOfPreview, comVelocityEndOfPreviewAfter, 3e-2);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(vrpPositionEndOfPreview, vrpPositionEndOfPreviewAfter, 6e-3);
   }

   @Test
   public void testSimpleStandingFewRhos()
   {
      double duration = 1.5;
      YoRegistry testRegistry = new YoRegistry("testRegistry");

      SE3ModelPredictiveController mpc = new SE3ModelPredictiveController(momentOfInertia,
                                                                          new MPCParameters(testRegistry),
                                                                          gravityZ,
                                                                                                        nominalHeight,
                                                                                                        mass,
                                                                                                        dt,
                                                                                                        testRegistry);
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
      contact.setStartECMPPosition(contactPose.getPosition());
      contact.setEndECMPPosition(contactPose.getPosition());

      contactProviders.add(contact);

      FramePoint3D initialCoM = new FramePoint3D(contactPose.getPosition());
      initialCoM.setZ(nominalHeight);

      FramePoint3D initialCoMPosition = new FramePoint3D(contactPose.getPosition());
      FrameVector3D initialCoMVelocity = new FrameVector3D();
      FrameQuaternion initialOrientation = new FrameQuaternion();
      FrameVector3D initialAngularVelocity = new FrameVector3D();

      initialCoMPosition.setZ(nominalHeight);

      mpc.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      mpc.setInitialBodyOrientationState(initialOrientation, initialAngularVelocity);
      mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialOrientation, initialAngularVelocity, 0.0);

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

      int expectedCoMPositionCommands = 1;
      int expectedCoMVelocityCommands = 1;
      int expectedDCMPositionCommands = 0;
      if (MPCParameters.includeFinalCoMPositionObjective)
         expectedCoMPositionCommands++;
      if (MPCParameters.includeFinalCoMVelocityObjective)
         expectedCoMVelocityCommands++;
      if (MPCParameters.includeFinalDCMPositionObjective)
         expectedDCMPositionCommands++;
      assertEquals(expectedCoMPositionCommands, comPositionCommands.size());
      assertEquals(expectedCoMVelocityCommands, comVelocityCommands.size());
      assertEquals(expectedDCMPositionCommands, dcmPositionCommands.size());
      assertEquals(1, vrpPositionCommands.size());
      assertEquals(1, vrpTrackingCommands.size());

      assertEquals(0.0, comPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(0).getObjective(), epsilon);

      assertEquals(0.0, comVelocityCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, comVelocityCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comVelocityCommands.get(0).getObjective(), epsilon);

      if (MPCParameters.includeFinalCoMPositionObjective)
      {
         assertEquals(previewWindowLength.getDoubleValue(), comPositionCommands.get(1).getTimeOfObjective(), epsilon);
         assertEquals(omega, comPositionCommands.get(1).getOmega(), epsilon);
         //      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, comPositionCommands.get(1).getObjective(), epsilon);
      }

      assertEquals(previewWindowLength.getDoubleValue(), vrpPositionCommands.get(0).getTimeOfObjective(), epsilon);
      assertEquals(omega, vrpPositionCommands.get(0).getOmega(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(initialCoM, vrpPositionCommands.get(0).getObjective(), epsilon);

      for (int i = 0; i < 22; i++)
      {
         assertNotEquals(0.0, mpc.qpSolver.qpSolver.costQuadraticMatrix.get(i, i), epsilon);
      }
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoM, mpc.getDesiredCoMPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), mpc.getDesiredCoMVelocity(), epsilon);
   }

   private static void visualize(SE3ModelPredictiveController mpc,
                                 List<ContactPlaneProvider> contacts,
                                 double visualizationDuration,
                                 FramePoint3DReadOnly seedCoMPosition,
                                 FrameVector3DReadOnly seedCoMVelocity,
                                 FrameQuaternionReadOnly seedOrientation,
                                 FrameVector3DReadOnly seedAngularVelocity,
                                 YoRegistry controllerRegistry)
   {
      if (!visualize || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      FramePoint3D initialCoMPosition = new FramePoint3D(seedCoMPosition);
      FrameVector3D initialCoMVelocity = new FrameVector3D(seedCoMVelocity);
      FrameQuaternion initialOrientation = new FrameQuaternion(seedOrientation);
      FrameVector3D initialAngularVelocity = new FrameVector3D(seedAngularVelocity);

      String advanceName = "GoToNextTick";
      YoBoolean goToNextTick = new YoBoolean(advanceName, controllerRegistry);
      YoDouble timeToStart = new YoDouble("timeToStart", controllerRegistry);

      double stepSize = 0.05;

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));
      YoRegistry registry = scs.getRootRegistry();
      registry.addChild(controllerRegistry);
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      addButton(advanceName, 1.0, scs);

      SE3MPCVisualizer mpcVisualizer = new SE3MPCVisualizer(mpc, scs, mass, gravityZ, registry, graphicsListRegistry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      goToNextTick.addListener((v) ->
                               {
                                  goToNextTick.set(false, false);

                                  mpc.setCurrentState(initialCoMPosition, initialCoMVelocity, initialOrientation, initialAngularVelocity, timeToStart.getDoubleValue());
                                  mpc.solveForTrajectory(contacts);

                                  timeToStart.add(stepSize);

                                  mpc.compute(timeToStart.getDoubleValue());

                                  initialCoMPosition.set(mpc.getDesiredCoMPosition());
                                  initialCoMVelocity.set(mpc.getDesiredCoMVelocity());
                                  initialOrientation.set(mpc.getDesiredBodyOrientationSolution());
                                  initialAngularVelocity.set(mpc.getDesiredBodyAngularVelocitySolution());

                                  mpcVisualizer.visualize(timeToStart.getDoubleValue(), visualizationDuration);
                               });

      scs.startOnAThread();



      ThreadTools.sleepForever();
   }

   private static void addButton(String yoVariableName, double newValue, SimulationConstructionSet scs)
   {
      YoRegistry registry = scs.getRootRegistry();

      final YoVariable var = registry.findVariable(yoVariableName);
      final JButton button = new JButton(yoVariableName);
      scs.addButton(button);
      button.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            var.setValueFromDouble(newValue);
         }
      });
   }

   private static void assertCoefficientsEqual(DMatrixRMaj solution, LinearMPCTrajectoryHandler trajectoryHandler)
   {
      for (int i = 0; i < 6; i++)
      {
         assertEquals("Col " + i + " wrong", solution.get(0, i), trajectoryHandler.getXCoefficientVector().get(i), epsilon);
         assertEquals("Col " + i + " wrong", solution.get(1, i), trajectoryHandler.getYCoefficientVector().get(i), epsilon);
         assertEquals("Col " + i + " wrong", solution.get(2, i), trajectoryHandler.getZCoefficientVector().get(i), epsilon);
      }
   }
}
