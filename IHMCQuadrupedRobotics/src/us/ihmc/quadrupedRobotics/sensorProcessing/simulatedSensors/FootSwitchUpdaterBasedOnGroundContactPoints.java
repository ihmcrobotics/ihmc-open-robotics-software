package us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors;

import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchUpdater;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootSwitchUpdaterBasedOnGroundContactPoints implements FootSwitchUpdater
{
   private static final int FILTER_WINDOW_SIZE = 10;
   
   private final FootSwitchOutputReadOnly footSwitchOutput;

   private final QuadrantDependentList<BooleanYoVariable> rawFootContacts = new QuadrantDependentList<>();
   private final QuadrantDependentList<GlitchFilteredBooleanYoVariable> glitchFilteredFootContacts = new QuadrantDependentList<>();
   
   public FootSwitchUpdaterBasedOnGroundContactPoints(FootSwitchOutputReadOnly footSwitchOutputReadOnly, YoVariableRegistry parentRegistry)
   {
      footSwitchOutput = footSwitchOutputReadOnly;
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         GlitchFilteredBooleanYoVariable filteredBoolean = new GlitchFilteredBooleanYoVariable(prefix + "FilteredContact", parentRegistry, FILTER_WINDOW_SIZE);
         glitchFilteredFootContacts.set(quadrant, filteredBoolean);
         
         BooleanYoVariable rawBoolean = new BooleanYoVariable(prefix + "rawContact", parentRegistry);
         rawFootContacts.set(quadrant, rawBoolean);
         
      }
   }

   @Override
   public boolean isFootInContactWithGround(RobotQuadrant footToBeChecked)
   {
      boolean rawFootInContact = footSwitchOutput.isFootInContact(footToBeChecked);
      rawFootContacts.get(footToBeChecked).set(rawFootInContact);
      
      glitchFilteredFootContacts.get(footToBeChecked).update(rawFootInContact);
      boolean filteredFootInContact = glitchFilteredFootContacts.get(footToBeChecked).getBooleanValue();
      return filteredFootInContact;
   }
   
}
