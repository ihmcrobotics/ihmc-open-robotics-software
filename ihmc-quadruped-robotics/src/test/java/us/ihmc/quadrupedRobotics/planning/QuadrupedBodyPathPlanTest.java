package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import org.junit.After;
import org.junit.Before;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.environments.SimpleMazeEnvironment;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedBodyPathPlanTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   private void setUpSimulation(TerrainObject3D terrainObject3D)
   {
      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         if(terrainObject3D != null)
         {
            quadrupedTestFactory.setTerrainObject3D(terrainObject3D);
         }

         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testSimpleBodyPathPlan()
   {
      setUpSimulation(null);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);

      EuclideanTrajectoryPointMessage point1 = new EuclideanTrajectoryPointMessage();
      point1.setTime(3.0);
      point1.position_.set(0.5, 0.0, 0.0);

      EuclideanTrajectoryPointMessage point2 = new EuclideanTrajectoryPointMessage();
      point2.setTime(6.0);
      point2.position_.set(0.5, 0.5, 0.0);

      EuclideanTrajectoryPointMessage point3 = new EuclideanTrajectoryPointMessage();
      point3.setTime(9.0);
      point3.position_.set(0.0, 0.5, 0.0);

      EuclideanTrajectoryPointMessage point4 = new EuclideanTrajectoryPointMessage();
      point4.setTime(12.0);
      point4.position_.set(0.0, 0.0, 0.0);

      QuadrupedTestBehaviors.executeBodyPathPlan(conductor, variables, stepTeleopManager, 0.1, 0.2, point1, point2, point3, point4);
   }

   /**
    * This test will need to be updated as footstep tracking improves, i.e. if this test breaks it could be because step tracking has improved.
    * The last few points have been adjusted pretty heavily to compensate
    */
   public void testBodyPathAroundASimpleMaze()
   {
      setUpSimulation(new SimpleMazeEnvironment());

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);

      double time = 0.0;
      time += 3.0;
      EuclideanTrajectoryPointMessage point1 = new EuclideanTrajectoryPointMessage();
      point1.setTime(time);
      point1.position_.set(0.75, -0.5, -0.5);

      time += 3.0;
      EuclideanTrajectoryPointMessage point2 = new EuclideanTrajectoryPointMessage();
      point2.setTime(time);
      point2.position_.set(1.5, - 0.9, 0.0);

      time += 3.0;
      EuclideanTrajectoryPointMessage point3 = new EuclideanTrajectoryPointMessage();
      point3.setTime(time);
      point3.position_.set(2.25, -0.9, 0.1);

      time += 3.0;
      EuclideanTrajectoryPointMessage point4 = new EuclideanTrajectoryPointMessage();
      point4.setTime(time);
      point4.position_.set(2.25, -0.8, 0.4 * Math.PI);

      time += 7.0;
      EuclideanTrajectoryPointMessage point5 = new EuclideanTrajectoryPointMessage();
      point5.setTime(time);
      point5.position_.set(2.3, 0.7, 0.4 * Math.PI);

      time += 4.0;
      EuclideanTrajectoryPointMessage point6 = new EuclideanTrajectoryPointMessage();
      point6.setTime(time);
      point6.position_.set(2.4, 0.85, 0.1);

      time += 3.0;
      EuclideanTrajectoryPointMessage point7 = new EuclideanTrajectoryPointMessage();
      point7.setTime(time);
      point7.position_.set(2.7, 0.85, 0.0);

      time += 4.0;
      EuclideanTrajectoryPointMessage point8 = new EuclideanTrajectoryPointMessage();
      point8.setTime(time);
      point8.position_.set(4.0, 1.05, 0.0);

      EuclideanTrajectoryPointMessage[] points = new EuclideanTrajectoryPointMessage[8];
      points[0] = point1;
      points[1] = point2;
      points[2] = point3;
      points[3] = point4;
      points[4] = point5;
      points[5] = point6;
      points[6] = point7;
      points[7] = point8;

      point1.linear_velocity_.set(point1.getPosition());
      point1.linear_velocity_.scale(1.0 / (point1.getTime()));

      for (int i = 1; i < points.length - 1; i++)
      {
         EuclideanTrajectoryPointMessage prevPoint = points[i - 1];
         EuclideanTrajectoryPointMessage point = points[i];
         EuclideanTrajectoryPointMessage nextPoint = points[i];

         Vector3D velocity = new Vector3D();
         velocity.set(nextPoint.position_);
         velocity.sub(prevPoint.position_);
         velocity.scale(1.0 / (nextPoint.time_ - prevPoint.time_));
         point.linear_velocity_.set(velocity);
      }

      // huge error bound until step tracking gets better
      QuadrupedTestBehaviors.executeBodyPathPlan(conductor, variables, stepTeleopManager, 0.5, 0.5, points);
   }
}
