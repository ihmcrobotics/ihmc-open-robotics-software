package us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors;

import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.sensorProcessors.FootSwitchUpdater;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootSwitchUpdaterBasedOnGroundContactPoints implements FootSwitchUpdater
{
   private static final int FILTER_WINDOW_SIZE = 10;
   
   private final FootSwitchOutputReadOnly footSwitchOutput;

   private final QuadrantDependentList<YoBoolean> rawFootContacts = new QuadrantDependentList<>();
   private final QuadrantDependentList<GlitchFilteredYoBoolean> glitchFilteredFootContacts = new QuadrantDependentList<>();
   
   public FootSwitchUpdaterBasedOnGroundContactPoints(FootSwitchOutputReadOnly footSwitchOutputReadOnly, YoVariableRegistry parentRegistry)
   {
      footSwitchOutput = footSwitchOutputReadOnly;
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         GlitchFilteredYoBoolean filteredBoolean = new GlitchFilteredYoBoolean(prefix + "FilteredContact", parentRegistry, FILTER_WINDOW_SIZE);
         glitchFilteredFootContacts.set(quadrant, filteredBoolean);
         
         YoBoolean rawBoolean = new YoBoolean(prefix + "rawContact", parentRegistry);
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
