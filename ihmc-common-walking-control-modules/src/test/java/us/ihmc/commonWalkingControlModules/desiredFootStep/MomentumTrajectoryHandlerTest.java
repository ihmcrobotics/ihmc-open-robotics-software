package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class MomentumTrajectoryHandlerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleExample()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      MomentumTrajectoryCommand command = new MomentumTrajectoryCommand();

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 0.0));
      command.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 1.0));
      command.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 2.0));

      // offset time by 5.0s
      yoTime.set(5.0);
      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime);
      handler.handleMomentumTrajectory(command);

      // get trajectory for time 6.0 to 7.0 which should be equivalent to 1.0 to 2.0 before time offset
      int samples = 3;
      RecyclingArrayList<TrajectoryPoint3D> momentumTrajectory = new RecyclingArrayList<>(TrajectoryPoint3D.class);
      handler.getAngularMomentumTrajectory(6.0, 7.0, samples, momentumTrajectory);

      // for three samples we expect the following result
      assertEquals(samples, momentumTrajectory.size());

      assertEquals(0.0, momentumTrajectory.get(0).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 1.0, 0.0), momentumTrajectory.get(0).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), momentumTrajectory.get(0).getVelocity(), Double.MIN_VALUE);

      assertEquals(0.5, momentumTrajectory.get(1).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(27.0 / 16.0, 27.0 / 16.0, 0.0), momentumTrajectory.get(1).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(9.0 / 8.0, 9.0 / 8.0, 0.0), momentumTrajectory.get(1).getVelocity(), Double.MIN_VALUE);

      assertEquals(1.0, momentumTrajectory.get(2).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(2.0, 2.0, 0.0), momentumTrajectory.get(2).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 0.0), momentumTrajectory.get(2).getVelocity(), Double.MIN_VALUE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOutOfBounds()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);
      MomentumTrajectoryCommand command = new MomentumTrajectoryCommand();

      // assume method x(t) = -1/2 * t^3 + 3/2 * t^2 for x and y and 0.0 for z
      command.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 0.0));
      command.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 1.0));
      command.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 2.0));

      // offset time by 5.0s
      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime);
      handler.handleMomentumTrajectory(command);

      int samples = 3;
      RecyclingArrayList<TrajectoryPoint3D> momentumTrajectory = new RecyclingArrayList<>(TrajectoryPoint3D.class);

      // try some examples that are invalid
      handler.getAngularMomentumTrajectory(-1.0, 0.0, samples, momentumTrajectory);
      assertEquals(0, momentumTrajectory.size());
      handler.getAngularMomentumTrajectory(2.0, 3.0, samples, momentumTrajectory);
      assertEquals(0, momentumTrajectory.size());
      handler.getAngularMomentumTrajectory(0.0, 1.0, samples, momentumTrajectory);
      assertEquals(0, momentumTrajectory.size());

      // try valid example
      handler.getAngularMomentumTrajectory(1.0e-10, 1.0, samples, momentumTrajectory);
      assertEquals(3, momentumTrajectory.size());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testQueuing()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble yoTime = new YoDouble("time", registry);

      MomentumTrajectoryCommand command1 = new MomentumTrajectoryCommand();
      command1.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 0.0));
      command1.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(1.0, 1.0, 0.0), new Vector3D(1.5, 1.5, 0.0), 1.0));

      MomentumTrajectoryHandler handler = new MomentumTrajectoryHandler(yoTime);
      handler.handleMomentumTrajectory(command1);

      MomentumTrajectoryCommand command2 = new MomentumTrajectoryCommand();
      command2.setExecutionMode(ExecutionMode.QUEUE);
      command2.addAngularMomentumTrajectoryPoint(new TrajectoryPoint3D(new Point3D(2.0, 2.0, 0.0), new Vector3D(0.0, 0.0, 0.0), 1.0));
      handler.handleMomentumTrajectory(command2);

      // get trajectory for time 0.5 to 1.5
      int samples = 3;
      RecyclingArrayList<TrajectoryPoint3D> momentumTrajectory = new RecyclingArrayList<>(TrajectoryPoint3D.class);
      handler.getAngularMomentumTrajectory(0.5, 1.5, samples, momentumTrajectory);

      // for three samples we expect the following result
      assertEquals(samples, momentumTrajectory.size());

      assertEquals(0.0, momentumTrajectory.get(0).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(5.0 / 16.0, 5.0 / 16.0, 0.0), momentumTrajectory.get(0).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(9.0 / 8.0, 9.0 / 8.0, 0.0), momentumTrajectory.get(0).getVelocity(), Double.MIN_VALUE);

      assertEquals(0.5, momentumTrajectory.get(1).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 1.0, 0.0), momentumTrajectory.get(1).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.5, 1.5, 0.0), momentumTrajectory.get(1).getVelocity(), Double.MIN_VALUE);

      assertEquals(1.0, momentumTrajectory.get(2).getTime(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(27.0 / 16.0, 27.0 / 16.0, 0.0), momentumTrajectory.get(2).getPosition(), Double.MIN_VALUE);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(9.0 / 8.0, 9.0 / 8.0, 0.0), momentumTrajectory.get(2).getVelocity(), Double.MIN_VALUE);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(MomentumTrajectoryHandler.class, MomentumTrajectoryHandlerTest.class);
   }
}
