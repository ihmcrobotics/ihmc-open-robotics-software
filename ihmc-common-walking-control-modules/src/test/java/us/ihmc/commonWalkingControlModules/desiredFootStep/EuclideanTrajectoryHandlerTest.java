package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotics.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.messageHandlers.EuclideanTrajectoryHandler;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class EuclideanTrajectoryHandlerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClearPointsInPast()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoDouble time = new YoDouble("time", registry);
      TestTrajectoryHandler trajectoryHandler = new TestTrajectoryHandler("TestHandler", time, registry);

      EuclideanTrajectoryControllerCommand trajectory = new EuclideanTrajectoryControllerCommand();
      trajectory.addTrajectoryPoint(1.0, new Point3D(), new Vector3D());
      trajectory.addTrajectoryPoint(2.0, new Point3D(), new Vector3D());
      trajectory.addTrajectoryPoint(3.0, new Point3D(), new Vector3D());
      trajectory.addTrajectoryPoint(4.0, new Point3D(), new Vector3D());
      trajectory.addTrajectoryPoint(5.0, new Point3D(), new Vector3D());

      trajectoryHandler.handleTrajectory(trajectory);

      double epsilon = 1e-7;
      Assert.assertTrue(trajectoryHandler.isWithinInterval(1.0 + epsilon));
      Assert.assertTrue(trajectoryHandler.isWithinInterval(2.5));
      Assert.assertTrue(trajectoryHandler.isWithinInterval(5.0 - epsilon));

      time.set(4.5);

      trajectoryHandler.clearPointsInPast();
      Assert.assertTrue(!trajectoryHandler.isEmpty());
      Assert.assertTrue(trajectoryHandler.isWithinInterval(4.0 + epsilon));
      Assert.assertTrue(trajectoryHandler.isWithinInterval(5.0 - epsilon));
   }

   private class TestTrajectoryHandler extends EuclideanTrajectoryHandler
   {
      public TestTrajectoryHandler(String name, DoubleProvider yoTime, YoVariableRegistry parentRegistry)
      {
         super(name, yoTime, parentRegistry);
      }

      @Override
      public void handleTrajectory(EuclideanTrajectoryControllerCommand controllerCommand)
      {
         super.handleTrajectory(controllerCommand);
      }
   }
}
