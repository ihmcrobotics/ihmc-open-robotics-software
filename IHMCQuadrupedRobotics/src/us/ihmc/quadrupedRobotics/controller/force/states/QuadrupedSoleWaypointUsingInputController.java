package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSoleWaypointController;

public class QuadrupedSoleWaypointUsingInputController implements QuadrupedController
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable robotTime;

   private QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;
   private final QuadrupedSoleWaypointController quadrupedSoleWaypointController;
   private double taskStartTime;

   public QuadrupedSoleWaypointUsingInputController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedSoleWaypointInputProvider inputProvider)
   {
      soleWaypointInputProvider = inputProvider;
      quadrupedSoleWaypointController = controllerToolbox.getQuadrupedSoleWaypointController();
      robotTime = environment.getRobotTimestamp();
      environment.getParentRegistry().addChild(registry);

   }

   @Override
   public void onEntry()
   {
      quadrupedSoleWaypointController.initialize(soleWaypointInputProvider.get());
   }

   @Override
   public ControllerEvent process()
   {
      taskStartTime = robotTime.getDoubleValue();
      int numberOfWaypoints = soleWaypointInputProvider.get().size(RobotQuadrant.FRONT_LEFT);
      if(numberOfWaypoints >0){
         quadrupedSoleWaypointController.compute();
         return (robotTime.getDoubleValue() - taskStartTime > soleWaypointInputProvider.get().get(RobotQuadrant.FRONT_LEFT).get(numberOfWaypoints - 1).getTime()) ?
               ControllerEvent.DONE :
               null;
      }else{
         return ControllerEvent.FAIL;
      }

   }

   @Override
   public void onExit()
   {
   }
}
