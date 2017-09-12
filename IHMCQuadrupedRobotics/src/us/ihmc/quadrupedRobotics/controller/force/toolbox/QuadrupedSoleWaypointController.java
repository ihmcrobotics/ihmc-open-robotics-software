package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSoleWaypointController
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble robotTime;

   // SoleWaypoint variables
   QuadrantDependentList<MultipleWaypointsPositionTrajectoryGenerator> quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final QuadrantDependentList<QuadrupedSolePositionController> solePositionController;
   private final QuadrantDependentList<QuadrupedSolePositionController.Setpoints> solePositionControllerSetpoints;
   private final QuadrantDependentList<FrameVector3D> initialSoleForces;

   private ReferenceFrame bodyFrame;
   private QuadrupedSoleWaypointList quadrupedSoleWaypointList;
   private double taskStartTime;

   public QuadrupedSoleWaypointController(ReferenceFrame bodyFrame, QuadrantDependentList<QuadrupedSolePositionController> solePositionController,
         YoDouble robotTimeStamp, YoVariableRegistry parentRegistry)
   {
      this.quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
      this.bodyFrame = bodyFrame;
      robotTime = robotTimeStamp;
      quadrupedWaypointsPositionTrajectoryGenerator = new QuadrantDependentList<>();

      // Feedback controller
      this.solePositionController = solePositionController;
      this.solePositionControllerSetpoints = new QuadrantDependentList<>();
      initialSoleForces = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionControllerSetpoints.set(robotQuadrant, new QuadrupedSolePositionController.Setpoints(robotQuadrant));
         initialSoleForces.set(robotQuadrant, new FrameVector3D());
      }
      // Create waypoint trajectory for each quadrant
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator tempWaypointGenerator = new MultipleWaypointsPositionTrajectoryGenerator(
               quadrant.getCamelCaseName() + "SoleTrajectory", bodyFrame, registry);
         quadrupedWaypointsPositionTrajectoryGenerator.set(quadrant, tempWaypointGenerator);
      }
      parentRegistry.addChild(registry);
   }


   public void initialize(QuadrupedSoleWaypointList quadrupedSoleWaypointList, YoPID3DGains positionControllerGains,
         QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      this.quadrupedSoleWaypointList = quadrupedSoleWaypointList;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionControllerSetpoints.get(robotQuadrant).initialize(taskSpaceEstimates);
         solePositionController.get(robotQuadrant).reset();
         if (useInitialSoleForceAsFeedforwardTerm)
         {
            this.initialSoleForces.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSoleVirtualForce(robotQuadrant));
            this.initialSoleForces.get(robotQuadrant).changeFrame(bodyFrame);
         }
         else
         {
            this.initialSoleForces.get(robotQuadrant).setToZero(bodyFrame);
         }
      }
      updateGains(positionControllerGains);
      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   public boolean compute(QuadrantDependentList<FrameVector3D> soleForceCommand, QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;
      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
      {
         return false;
      }
      else
      {
         for (RobotQuadrant quadrant : RobotQuadrant.values)
         {
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).compute(currentTrajectoryTime);
            quadrupedWaypointsPositionTrajectoryGenerator.get(quadrant).getPosition(solePositionControllerSetpoints.get(quadrant).getSolePosition());
            solePositionControllerSetpoints.get(quadrant).getSoleLinearVelocity().setToZero();
            solePositionControllerSetpoints.get(quadrant).getSoleForceFeedforward().setIncludingFrame(initialSoleForces.get(quadrant));
            solePositionController.get(quadrant).compute(soleForceCommand.get(quadrant), solePositionControllerSetpoints.get(quadrant), taskSpaceEstimates);
         }
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

   private void updateGains(YoPID3DGains positionControllerGains)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.get(quadrant).getGains().set(positionControllerGains);
      }
   }

}
