package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootSwitchUpdater
{

   private final FootSwitchOutputReadOnly footSwitchOutput;
   private final QuadrantDependentList<BooleanYoVariable> footContactSwitches;
   
   public FootSwitchUpdater(QuadrantDependentList<BooleanYoVariable> footContactSwitchesToBeUpdated, FootSwitchOutputReadOnly footSwitchOutputReadOnly, YoVariableRegistry parentRegistry)
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
   
   
}
