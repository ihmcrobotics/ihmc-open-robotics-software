package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerCostParametersProperty extends ParametersProperty<SettableFootstepPlannerCostParameters>
{
   private BooleanField useQuadraticDistanceCost = new BooleanField(SettableFootstepPlannerCostParameters::useQuadraticDistanceCost, (p, v) -> p.setUseQuadraticDistanceCost(v));
   private BooleanField useQuadraticHeightCost = new BooleanField(SettableFootstepPlannerCostParameters::useQuadraticHeightCost, (p, v) -> p.setUseQuadraticHeightCost(v));
   private DoubleField yawWeight = new DoubleField(SettableFootstepPlannerCostParameters::getYawWeight, (p, v) -> p.setYawWeight(v));
   private DoubleField pitchWeight = new DoubleField(SettableFootstepPlannerCostParameters::getPitchWeight, (p, v) -> p.setPitchWeight(v));
   private DoubleField rollWeight = new DoubleField(SettableFootstepPlannerCostParameters::getRollWeight, (p, v) -> p.setRollWeight(v));
   private DoubleField forwardWeight = new DoubleField(SettableFootstepPlannerCostParameters::getForwardWeight, (p, v) -> p.setForwardWeight(v));
   private DoubleField lateralWeight = new DoubleField(SettableFootstepPlannerCostParameters::getLateralWeight, (p, v) -> p.setLateralWeight(v));
   private DoubleField stepUpWeight = new DoubleField(SettableFootstepPlannerCostParameters::getStepUpWeight, (p, v) -> p.setStepUpWeight(v));
   private DoubleField stepDownWeight = new DoubleField(SettableFootstepPlannerCostParameters::getStepDownWeight, (p, v) -> p.setStepDownWeight(v));
   private DoubleField costPerStep = new DoubleField(SettableFootstepPlannerCostParameters::getCostPerStep, (p, v) -> p.setCostPerStep(v));
   private DoubleField aStarHeuristicsWeight = new DoubleField(p -> p.getAStarHeuristicsWeight().getValue(), (p, v) -> p.setAStarHeuristicsWeight(v));
   private DoubleField visGraphWithAStarHeuristicsWeight = new DoubleField(p -> p.getVisGraphWithAStarHeuristicsWeight().getValue(), (p, v) -> p.setVisGraphWithAStarHeuristicsWeight(v));
   private DoubleField depthFirstHeuristicsWeight = new DoubleField(p -> p.getDepthFirstHeuristicsWeight().getValue(), (p, v) -> p.setDepthFirstHeuristicsWeight(v));
   private DoubleField bodyPathBasedHeuristicsWeight = new DoubleField(p -> p.getBodyPathBasedHeuristicsWeight().getValue(), (p, v) -> p.setBodyPathBasedHeuristicsWeight(v));

   public FootstepPlannerCostParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultFootstepPlannerCostParameters());
   }

   public FootstepPlannerCostParametersProperty(Object bean, String name, FootstepPlannerCostParameters footstepPlannerParameters)
   {
      super(bean, name, new SettableFootstepPlannerCostParameters(footstepPlannerParameters));
   }

   public void setPlannerParameters(FootstepPlannerCostParameters parameters)
   {
      setValue(new SettableFootstepPlannerCostParameters(parameters));
   }

   @Override
   protected SettableFootstepPlannerCostParameters getValueCopy(SettableFootstepPlannerCostParameters valueToCopy)
   {
      return new SettableFootstepPlannerCostParameters(valueToCopy);
   }
   
   public void bidirectionalBindYawWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yawWeight);
   }

   public void bidirectionalBindPitchWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, pitchWeight);
   }

   public void bidirectionalBindRollWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, rollWeight);
   }

   public void bidirectionalBindForwardWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, forwardWeight);
   }

   public void bidirectionalBindLateralWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, lateralWeight);
   }

   public void bidirectionalBindStepUpWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepUpWeight);
   }

   public void bidirectionalBindStepDownWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepDownWeight);
   }

   public void bidirectionalBindCostPerStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, costPerStep);
   }

   public void bidirectionalBindHeuristicsWeight(AtomicReference<FootstepPlannerType> plannerTypeReference, Property<? extends Number> property)
   {
      if (plannerTypeReference.get() == null)
         return;

      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.A_STAR), property, aStarHeuristicsWeight);
      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR), property, visGraphWithAStarHeuristicsWeight);
      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.PLANAR_REGION_BIPEDAL), property, depthFirstHeuristicsWeight);
      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.SIMPLE_BODY_PATH), property, bodyPathBasedHeuristicsWeight);
   }

   public void bidirectionalBindUseQuadraticHeightCost(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useQuadraticHeightCost);
   }

   public void bidirectionalBindUseQuadraticDistanceCost(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useQuadraticDistanceCost);
   }
}
