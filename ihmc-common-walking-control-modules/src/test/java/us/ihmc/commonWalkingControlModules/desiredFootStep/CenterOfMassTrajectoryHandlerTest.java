package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.messageHandlers.CenterOfMassTrajectoryHandler;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassTrajectoryHandlerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleExample()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      CenterOfMassTrajectoryCommand command = new CenterOfMassTrajectoryCommand();
      double omega0 = 1.0;
      double epsilon = 1.0e-10;

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 0.0));
      command.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 1.0));
      command.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 2.0));

      CenterOfMassTrajectoryHandler handler = new CenterOfMassTrajectoryHandler(yoTime);
      handler.handleComTrajectory(command);

      FramePoint3D desiredICPPosition = new FramePoint3D();
      FrameVector3D desiredICPVelocity = new FrameVector3D();

      yoTime.set(-0.1);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());

      yoTime.set(0.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(3.0, 3.0, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.5);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.8125, 2.8125, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(-0.375, -0.375, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(0.1);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.2995, 0.2995, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(2.985, 2.985, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.5, 2.5, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(-0.1);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNonZeroStartTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      CenterOfMassTrajectoryCommand command = new CenterOfMassTrajectoryCommand();
      double omega0 = 1.0;
      double epsilon = 1.0e-10;
      double offset = -0.56;

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 0.0));
      command.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 1.0));
      command.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 2.0));

      CenterOfMassTrajectoryHandler handler = new CenterOfMassTrajectoryHandler(yoTime);
      yoTime.set(offset);
      handler.handleComTrajectory(command);

      FramePoint3D desiredICPPosition = new FramePoint3D();
      FrameVector3D desiredICPVelocity = new FrameVector3D();

      yoTime.set(-0.1 + offset);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());

      yoTime.set(0.0 + offset);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(3.0, 3.0, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.5 + offset);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.8125, 2.8125, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(-0.375, -0.375, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(0.1 + offset);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.2995, 0.2995, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(2.985, 2.985, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.0 + offset);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.5, 2.5, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(-0.1 + offset);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testQueuing()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      double omega0 = 1.0;
      double epsilon = 1.0e-10;

      CenterOfMassTrajectoryHandler handler = new CenterOfMassTrajectoryHandler(yoTime);

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      CenterOfMassTrajectoryCommand command1 = new CenterOfMassTrajectoryCommand();
      command1.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 0.0));
      command1.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 1.0));
      command1.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 2.0));
      handler.handleComTrajectory(command1);

      // assume method x(t) = -1/2 * (t - 3.0)^3 + 3/2 * (t - 3.0)^2 for x and y and 0.0 for z
      CenterOfMassTrajectoryCommand command2 = new CenterOfMassTrajectoryCommand();
      command2.setExecutionMode(ExecutionMode.QUEUE);
      command2.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 1.0));
      command2.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 2.0));
      command2.addComTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 3.0));

      yoTime.set(-20.9);
      handler.handleComTrajectory(command2);

      FramePoint3D desiredICPPosition = new FramePoint3D();
      FrameVector3D desiredICPVelocity = new FrameVector3D();

      yoTime.set(-0.1);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());

      yoTime.set(0.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(3.0, 3.0, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.5);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.8125, 2.8125, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(-0.375, -0.375, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(0.1);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.2995, 0.2995, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(2.985, 2.985, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.5, 2.5, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(-0.1);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());

      yoTime.set(1.5 + 3.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.8125, 2.8125, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(-0.375, -0.375, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(0.1 + 3.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.2995, 0.2995, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(2.985, 2.985, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(1.0 + 3.0);
      assertTrue(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.5, 2.5, 0.0), desiredICPPosition.getPoint(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), desiredICPVelocity.getVector(), epsilon);

      yoTime.set(6.0);
      assertFalse(handler.packCurrentDesiredICP(omega0, desiredICPPosition, desiredICPVelocity));
      assertTrue(desiredICPPosition.containsNaN());
      assertTrue(desiredICPVelocity.containsNaN());
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(CenterOfMassTrajectoryHandler.class, CenterOfMassTrajectoryHandlerTest.class);
   }
}
