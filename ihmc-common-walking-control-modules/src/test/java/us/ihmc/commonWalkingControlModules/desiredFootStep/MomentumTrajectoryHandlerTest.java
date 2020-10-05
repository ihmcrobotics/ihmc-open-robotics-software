package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.ExecutionMode;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

public class MomentumTrajectoryHandlerTest
{
   @Test
   public void testSimpleExample()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      MomentumTrajectoryCommand command = new MomentumTrajectoryCommand();

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.getAngularMomentumTrajectory().addTrajectoryPoint(0.0, new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      command.getAngularMomentumTrajectory().addTrajectoryPoint(1.0, new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0));
      command.getAngularMomentumTrajectory().addTrajectoryPoint(2.0, new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0));

      // offset time by 5.0s
      yoTime.set(5.0);
      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime, registry);
      handler.handleMomentumTrajectory(command);

      // get trajectory for time 6.0 to 7.0 which should be equivalent to 1.0 to 2.0 before time offset
      int samples = 3;
      RecyclingArrayList<EuclideanTrajectoryPoint> momentumTrajectory = new RecyclingArrayList<>(EuclideanTrajectoryPoint.class);
      handler.getAngularMomentumTrajectory(6.0, 7.0, samples, momentumTrajectory);

      // for three samples we expect the following result
      assertEquals(samples, momentumTrajectory.size());

      assertEquals(0.0, momentumTrajectory.get(0).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 1.0, 0.0), momentumTrajectory.get(0).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), momentumTrajectory.get(0).getLinearVelocity(), Double.MIN_VALUE);

      assertEquals(0.5, momentumTrajectory.get(1).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(27.0 / 16.0, 27.0 / 16.0, 0.0), momentumTrajectory.get(1).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(9.0 / 8.0, 9.0 / 8.0, 0.0), momentumTrajectory.get(1).getLinearVelocity(), Double.MIN_VALUE);

      assertEquals(1.0, momentumTrajectory.get(2).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.0, 2.0, 0.0), momentumTrajectory.get(2).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 0.0), momentumTrajectory.get(2).getLinearVelocity(), Double.MIN_VALUE);
   }

   @Test
   public void testSamplingDurations()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      MomentumTrajectoryCommand command = new MomentumTrajectoryCommand();

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.getAngularMomentumTrajectory().addTrajectoryPoint(0.0, new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      command.getAngularMomentumTrajectory().addTrajectoryPoint(1.0, new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0));
      command.getAngularMomentumTrajectory().addTrajectoryPoint(2.0, new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0));

      // offset time by 5.0s
      double clockTime = 5.0;
      yoTime.set(clockTime);
      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime, registry);
      handler.handleMomentumTrajectory(command);

      // get trajectory for time 6.0 to 7.0 which should be equivalent to 1.0 to 2.0 before time offset
      RecyclingArrayList<EuclideanTrajectoryPoint> momentumTrajectory = new RecyclingArrayList<>(EuclideanTrajectoryPoint.class);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double startTime = RandomNumbers.nextDouble(random, clockTime, clockTime + 2.0);
         double endTime = RandomNumbers.nextDouble(random, startTime, clockTime + 2.0);

         double duration = endTime - startTime;

         int samples = 11;


         handler.getAngularMomentumTrajectory(startTime, endTime, samples, momentumTrajectory);

         // for three samples we expect the following result
         assertEquals(samples, momentumTrajectory.size());

         assertEquals(0.0, momentumTrajectory.get(0).getTime(), 1e-5);
         assertEquals(duration, momentumTrajectory.get(samples - 1).getTime(), 1e-5);

         for (int sampleIndex = 1; sampleIndex < samples; sampleIndex++)
         {
            double time = 0.1 * sampleIndex * duration;
            assertEquals("index " + sampleIndex + " of " + samples + " failed.", time, momentumTrajectory.get(sampleIndex).getTime(), 1e-5);
         }
      }


      for (int i = 0; i < 100; i++)
      {
         double startTime = RandomNumbers.nextDouble(random, clockTime, clockTime + 2.0);
         double endTime = RandomNumbers.nextDouble(random, startTime, clockTime + 2.0);

         double duration = endTime - startTime;

         int samples = RandomNumbers.nextInt(random, 2, 12);


         handler.getAngularMomentumTrajectory(startTime, endTime, samples, momentumTrajectory);

         // for three samples we expect the following result
         assertEquals(samples, momentumTrajectory.size());

         assertEquals(0.0, momentumTrajectory.get(0).getTime(), 1e-5);
         assertEquals(duration, momentumTrajectory.get(samples - 1).getTime(), 1e-5);

         for (int sampleIndex = 0; sampleIndex < samples; sampleIndex++)
         {
            double timeFraction = sampleIndex / (samples - 1.0);
            double time = timeFraction * duration;
            assertEquals("index " + sampleIndex + " of " + samples + " failed. time fraction was " + timeFraction, time, momentumTrajectory.get(sampleIndex).getTime(), 1e-5);
         }
      }
   }

   @Test
   public void testOutOfBounds()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      MomentumTrajectoryCommand command = new MomentumTrajectoryCommand();

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.getAngularMomentumTrajectory().addTrajectoryPoint(0.0, new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      command.getAngularMomentumTrajectory().addTrajectoryPoint(1.0, new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0));
      command.getAngularMomentumTrajectory().addTrajectoryPoint(2.0, new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0));

      // offset time by 5.0s
      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime, registry);
      handler.handleMomentumTrajectory(command);

      int samples = 3;
      RecyclingArrayList<EuclideanTrajectoryPoint> momentumTrajectory = new RecyclingArrayList<>(EuclideanTrajectoryPoint.class);

      // try some examples that are invalid
      handler.getAngularMomentumTrajectory(-1.0, 0.0, samples, momentumTrajectory);
      assertEquals(0, momentumTrajectory.size());
      handler.getAngularMomentumTrajectory(2.0, 3.0, samples, momentumTrajectory);
      assertEquals(0, momentumTrajectory.size());

      // try valid example
      handler.getAngularMomentumTrajectory(1.0e-10, 1.0, samples, momentumTrajectory);
      assertEquals(3, momentumTrajectory.size());
   }

   @Test
   public void testQueuing()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);

      MomentumTrajectoryCommand command1 = new MomentumTrajectoryCommand();
      command1.getAngularMomentumTrajectory().addTrajectoryPoint(0.0, new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      command1.getAngularMomentumTrajectory().addTrajectoryPoint(1.0, new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0));
      command1.getAngularMomentumTrajectory().setCommandId(0L);

      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime, registry);
      handler.handleMomentumTrajectory(command1);

      MomentumTrajectoryCommand command2 = new MomentumTrajectoryCommand();
      command2.getAngularMomentumTrajectory().setExecutionMode(ExecutionMode.QUEUE);
      command2.getAngularMomentumTrajectory().addTrajectoryPoint(1.0, new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
      command2.getAngularMomentumTrajectory().setPreviousCommandId(0L);
      handler.handleMomentumTrajectory(command2);

      // get trajectory for time 0.5 to 1.5
      int samples = 3;
      RecyclingArrayList<EuclideanTrajectoryPoint> momentumTrajectory = new RecyclingArrayList<>(EuclideanTrajectoryPoint.class);
      handler.getAngularMomentumTrajectory(0.5, 1.5, samples, momentumTrajectory);

      // for three samples we expect the following result
      assertEquals(samples, momentumTrajectory.size());

      assertEquals(0.0, momentumTrajectory.get(0).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(5.0 / 16.0, 5.0 / 16.0, 0.0), momentumTrajectory.get(0).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(9.0 / 8.0, 9.0 / 8.0, 0.0), momentumTrajectory.get(0).getLinearVelocity(), Double.MIN_VALUE);

      assertEquals(0.5, momentumTrajectory.get(1).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 1.0, 0.0), momentumTrajectory.get(1).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), momentumTrajectory.get(1).getLinearVelocity(), Double.MIN_VALUE);

      assertEquals(1.0, momentumTrajectory.get(2).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(27.0 / 16.0, 27.0 / 16.0, 0.0), momentumTrajectory.get(2).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(9.0 / 8.0, 9.0 / 8.0, 0.0), momentumTrajectory.get(2).getLinearVelocity(), Double.MIN_VALUE);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(MomentumTrajectoryHandler.class, MomentumTrajectoryHandlerTest.class);
   }
}
