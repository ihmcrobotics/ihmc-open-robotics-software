package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState
{
   // Yo variables
   private final YoVariableRegistry registry;
   private final YoDouble robotTime;

   // SoleWaypoint variables
   private final MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;
   private final FrameVector3D initialSoleForces = new FrameVector3D();

   private final ReferenceFrame bodyFrame;
   private final QuadrupedSoleWaypointList quadrupedSoleWaypointLists = new QuadrupedSoleWaypointList();

   private final RobotQuadrant robotQuadrant;

   private double taskStartTime;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, ReferenceFrame bodyFrame, QuadrupedSolePositionController solePositionController,
                                         YoDouble robotTimeStamp, YoVariableRegistry parentRegistry)
   {
      this.robotQuadrant = robotQuadrant;
      this.bodyFrame = bodyFrame;
      robotTime = robotTimeStamp;

      // Feedback controller
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      registry = new YoVariableRegistry(robotQuadrant.getShortName() + getClass().getSimpleName());

      // Create waypoint trajectory for each quadrant
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       bodyFrame, registry);
      parentRegistry.addChild(registry);
   }


   public void initialize(QuadrupedSoleWaypointList quadrupedSoleWaypointLists, YoPID3DGains positionControllerGains,
                          QuadrupedTaskSpaceEstimates taskSpaceEstimates, boolean useInitialSoleForceAsFeedforwardTerm)
   {
         this.quadrupedSoleWaypointLists.set(quadrupedSoleWaypointLists);
         solePositionControllerSetpoints.initialize(taskSpaceEstimates);
         solePositionController.reset();

         if (useInitialSoleForceAsFeedforwardTerm)
         {
            this.initialSoleForces.setIncludingFrame(taskSpaceEstimates.getSoleVirtualForce(robotQuadrant));
            this.initialSoleForces.changeFrame(bodyFrame);
         }
         else
         {
            this.initialSoleForces.setToZero(bodyFrame);
         }

      updateGains(positionControllerGains);
      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   public boolean compute(QuadrantDependentList<FrameVector3D> soleForceCommand, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;
      double finalTime = quadrupedSoleWaypointLists.getFinalTime();

      if (currentTrajectoryTime > finalTime)
      {
         return false;
      }
      else
      {
         quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
         quadrupedWaypointsPositionTrajectoryGenerator.getPosition(solePositionControllerSetpoints.getSolePosition());
         solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
         solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
         solePositionController.compute(soleForceCommand.get(robotQuadrant), solePositionControllerSetpoints, taskSpaceEstimates);

         return true;
      }
   }

   public void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      for (int i = 0; i < quadrupedSoleWaypointLists.size(); ++i)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoint(quadrupedSoleWaypointLists.get(i).getTime(),
                                                                      quadrupedSoleWaypointLists.get(i).getPosition(),
                                                                      quadrupedSoleWaypointLists.get(i).getVelocity());
      }
      if (quadrupedSoleWaypointLists.size() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }

   private void updateGains(YoPID3DGains positionControllerGains)
   {
      solePositionController.getGains().set(positionControllerGains);
   }

}
