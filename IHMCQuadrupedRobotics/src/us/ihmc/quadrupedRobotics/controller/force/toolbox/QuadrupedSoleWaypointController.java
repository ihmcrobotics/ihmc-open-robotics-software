package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

public class QuadrupedSoleWaypointController
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable robotTime;

   // SoleWaypoint variables
   QuadrantDependentList<MultipleWaypointsPositionTrajectoryGenerator> quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;

   // Task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;

   private QuadrupedSoleWaypointList quadrupedSoleWaypointList;
   private double taskStartTime;

   public QuadrupedSoleWaypointController(QuadrupedRuntimeEnvironment environment, QuadrupedReferenceFrames referenceFrames,
         QuadrupedSolePositionController solePositionController, QuadrupedTaskSpaceEstimator taskSpaceEstimator)
   {
      this.quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
      robotTime = environment.getRobotTimestamp();
      quadrupedWaypointsPositionTrajectoryGenerator = new QuadrantDependentList<>();

      // Feedback controller
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();

      // Task space estimates
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      this.taskSpaceEstimator = taskSpaceEstimator;

      // Create waypoint trajectory for each quadrant
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator tempWaypointGenerator = new MultipleWaypointsPositionTrajectoryGenerator(
               quadrant.getCamelCaseName() + "SoleTrajectory", referenceFrames.getBodyFrame(), registry);
         quadrupedWaypointsPositionTrajectoryGenerator.set(quadrant, tempWaypointGenerator);
      }
      environment.getParentRegistry().addChild(registry);
   }

   public void initialize(QuadrupedSoleWaypointList quadrupedSoleWaypointList, YoEuclideanPositionGains positionControllerGains)
   {
      this.quadrupedSoleWaypointList = quadrupedSoleWaypointList;
      updateEstimates();
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      solePositionController.reset();
      updateGains(positionControllerGains);
      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   public boolean compute(QuadrantDependentList<FrameVector> soleForce)
   {
      if (robotTime.getDoubleValue() - taskStartTime > quadrupedSoleWaypointList.getFinalTime())
      {
         return false;
      }
      else
      {
         updateSetpoints(soleForce);
         return true;
      }
   }

   public void createSoleWaypointTrajectory()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).clear();
         for (int i = 0; i < quadrupedSoleWaypointList.size(quadrant); ++i)
         {
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant)
                  .appendWaypoint(quadrupedSoleWaypointList.get(quadrant).get(i).getTime(), quadrupedSoleWaypointList.get(quadrant).get(i).getPosition(),
                        quadrupedSoleWaypointList.get(quadrant).get(i).getVelocity());
         }
         if (quadrupedSoleWaypointList.size(quadrant) > 0)
         {
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).initialize();
         }
      }
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints(QuadrantDependentList<FrameVector> soleForce)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).compute(currentTrajectoryTime);
         //query motion generator at the current time stamp within the trajectory
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).getPosition(solePositionControllerSetpoints.getSolePosition(quadrant));
         quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).getVelocity(solePositionControllerSetpoints.getSoleLinearVelocity(quadrant));

      }
      solePositionController.compute(soleForce, solePositionControllerSetpoints, taskSpaceEstimates);
   }

   private void updateGains(YoEuclideanPositionGains positionControllerGains)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.getGains(quadrant).set(positionControllerGains);
      }
   }

}
