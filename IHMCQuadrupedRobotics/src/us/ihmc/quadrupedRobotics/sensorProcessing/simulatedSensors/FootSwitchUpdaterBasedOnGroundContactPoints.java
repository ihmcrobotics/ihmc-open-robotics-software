package us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors;

import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchUpdater;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootSwitchUpdaterBasedOnGroundContactPoints implements FootSwitchUpdater
{

   private final FootSwitchOutputReadOnly footSwitchOutput;
   
   public FootSwitchUpdaterBasedOnGroundContactPoints(FootSwitchOutputReadOnly footSwitchOutputReadOnly, YoVariableRegistry parentRegistry)
   {
      footSwitchOutput = footSwitchOutputReadOnly;
   }

   @Override
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked)
   {
      return footSwitchOutput.isFootInContact(footToBeChecked);
   }
   
}
