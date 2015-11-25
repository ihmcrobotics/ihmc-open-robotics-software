package us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors;

import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootContactStateInterface;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootSwitchUpdaterBasedOnGroundContactPoints implements FootContactStateInterface
{

   private final FootSwitchOutputReadOnly footSwitchOutput;
   private final QuadrantDependentList<BooleanYoVariable> footContactSwitches;
   
   public FootSwitchUpdaterBasedOnGroundContactPoints(QuadrantDependentList<BooleanYoVariable> footContactSwitchesToBeUpdated, FootSwitchOutputReadOnly footSwitchOutputReadOnly, YoVariableRegistry parentRegistry)
   {
      footSwitchOutput = footSwitchOutputReadOnly;
      footContactSwitches = footContactSwitchesToBeUpdated;
   }

   
   public void initialize()
   {
      updateFootSwitchState();
   }
   
   public void updateFootSwitchState()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         boolean footInContact = footSwitchOutput.isFootInContact(quadrant);
         footContactSwitches.get(quadrant).set(footInContact);
      }
   }

   @Override
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked)
   {
      return footContactSwitches.get(footToBeChecked).getBooleanValue();
   }
   
}
