package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class FootstepPlannerParametersProperty extends ParametersProperty<SettableFootstepPlannerParameters>
{
   private final DoubleField maximumStepReach = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepReach, SettableFootstepPlannerParameters::setMaximumStepReach);
   private final DoubleField maximumStepWidth = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepWidth, SettableFootstepPlannerParameters::setMaximumStepWidth);
   private final DoubleField minimumStepLength = new DoubleField(SettableFootstepPlannerParameters::getMinimumStepLength, SettableFootstepPlannerParameters::setMinimumStepLength);
   private final DoubleField minimumStepWidth = new DoubleField(SettableFootstepPlannerParameters::getMinimumStepWidth, SettableFootstepPlannerParameters::setMinimumStepWidth);
   private final DoubleField minimumStepYaw = new DoubleField(SettableFootstepPlannerParameters::getMinimumStepYaw, SettableFootstepPlannerParameters::setMinimumStepYaw);
   private final DoubleField maximumStepYaw = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepYaw, SettableFootstepPlannerParameters::setMaximumStepYaw);

   private final DoubleField maximumStepChangeZ = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepChangeZ, SettableFootstepPlannerParameters::setMaximumStepChangeZ);
   private final DoubleField bodyGroundClearance = new DoubleField(SettableFootstepPlannerParameters::getBodyGroundClearance, SettableFootstepPlannerParameters::setBodyGroundClearance);

   private final DoubleField yawWeight = new DoubleField(SettableFootstepPlannerParameters::getYawWeight, SettableFootstepPlannerParameters::setYawWeight);
   private final DoubleField costPerStep = new DoubleField(SettableFootstepPlannerParameters::getCostPerStep, SettableFootstepPlannerParameters::setCostPerStep);
   private final DoubleField stepUpWeight  = new DoubleField(SettableFootstepPlannerParameters::getStepUpWeight, SettableFootstepPlannerParameters::setStepUpWeight);
   private final DoubleField stepDownWeight  = new DoubleField(SettableFootstepPlannerParameters::getStepDownWeight, SettableFootstepPlannerParameters::setStepDownWeight);
   private final DoubleField heuristicsWeight = new DoubleField(SettableFootstepPlannerParameters::getHeuristicsInflationWeight, SettableFootstepPlannerParameters::setHeuristicsWeight);

   private final DoubleField minXClearanceFromFoot = new DoubleField(SettableFootstepPlannerParameters::getMinXClearanceFromFoot, SettableFootstepPlannerParameters::setMinXClearanceFromFoot);
   private final DoubleField minYClearanceFromFoot = new DoubleField(SettableFootstepPlannerParameters::getMinYClearanceFromFoot, SettableFootstepPlannerParameters::setMinYClearanceFromFoot);
   private final DoubleField minimumSurfaceInclineRadians = new DoubleField(SettableFootstepPlannerParameters::getMinimumSurfaceInclineRadians, SettableFootstepPlannerParameters::setMinimumSurfaceInclineRadians);

   public FootstepPlannerParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultFootstepPlannerParameters());
   }

   public FootstepPlannerParametersProperty(Object bean, String name, FootstepPlannerParameters footstepPlannerParameters)
   {
      super(bean, name, new SettableFootstepPlannerParameters(footstepPlannerParameters));
   }

   public void setPlannerParameters(FootstepPlannerParameters parameters)
   {
      setValue(new SettableFootstepPlannerParameters(parameters));
   }

   @Override
   protected SettableFootstepPlannerParameters getValueCopy(SettableFootstepPlannerParameters valueToCopy)
   {
      return new SettableFootstepPlannerParameters(valueToCopy);
   }

   public void bidirectionalBindMaximumStepReach(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumStepReach);
   }

   public void bidirectionalBindMinimumStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumStepLength);
   }

   public void bidirectionalBindMaximumStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumStepWidth);
   }

   public void bidirectionalBindMinimumStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumStepWidth);
   }

   public void bidirectionalBindMaximumStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumStepYaw);
   }

   public void bidirectionalBindMinimumStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumStepYaw);
   }

   public void bidirectionalBindMaximumStepChangeZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumStepChangeZ);
   }

   public void bidirectionalBindBodyGroundClearance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyGroundClearance);
   }

   public void bidirectionalBindYawWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yawWeight);
   }

   public void bidirectionalBindCostPerStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, costPerStep);
   }

   public void bidirectionalBindStepUpWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepUpWeight);
   }

   public void bidirectionalBindStepDownWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepDownWeight);
   }

   public void bidirectionalBindHeuristicsWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, heuristicsWeight);
   }

   public void bidirectionalBindMinXClearanceFromFoot(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minXClearanceFromFoot);
   }

   public void bidirectionalBindMinYClearanceFromFoot(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minYClearanceFromFoot);
   }

   public void bidirectionalBindMinimumSurfaceInclineRadians(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumSurfaceInclineRadians);
   }


}
