package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.InvalidationListener;
import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PropertySetToParameterPropertyConverter;

import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.*;

public class FootstepPlannerParametersProperty extends PropertySetToParameterPropertyConverter
{
   private FootstepPlannerParametersBasics parameters;

   public FootstepPlannerParametersProperty()
   {
   }

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.parameters = parameters;
   }


   public void bidirectionalBindIdealFootstepWidth(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, idealFootstepWidth, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindIdealFootstepLength(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, idealFootstepLength, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindReturnBestEffortPlan(Property<Boolean> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToBooleanProperty(property, returnBestEffortPlan, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxStepReach(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maxStepReach, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxStepYaw(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maxStepYaw, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinStepWidth(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minStepWidth, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinStepLength(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minStepLength, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinStepYaw(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minStepYaw, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxStepZ(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maxStepZ, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinFootholdPercent(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minFootholdPercent, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinSurfaceIncline(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minSurfaceIncline, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxStepWidth(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maxStepWidth, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinXClearanceFromStance(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minXClearanceFromStance, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinYClearanceFromStance(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minYClearanceFromStance, parameters.getStoredPropertySet(), propertyChangedListener);
   }


   public void bidirectionalBindCheckBodyBoxCollisions(Property<Boolean> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToBooleanProperty(property, checkForBodyBoxCollisions, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindPerformHeuristicSearchPolicies(Property<Boolean> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToBooleanProperty(property, performHeuristicSearchPolicies, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxWiggleXY(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximumXYWiggleDistance, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxWiggleYaw(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximumYawWiggle, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindWiggleInsideDelta(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, wiggleInsideDelta, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBodyBoxWidth(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, bodyBoxWidth, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBodyBoxDepth(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, bodyBoxDepth, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBodyBoxHeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, bodyBoxHeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBodyBoxBaseX(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, bodyBoxBaseX, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBodyBoxBaseY(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, bodyBoxBaseY, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBodyBoxBaseZ(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, bodyBoxBaseZ, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindYawWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, yawWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindPitchWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, pitchWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindRollWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, rollWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindForwardWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, forwardWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindLateralWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, lateralWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindStepUpWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, stepUpWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindStepDownWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, stepDownWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindCostPerStep(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, costPerStep, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindCliffHeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, cliffHeightToAvoid, parameters.getStoredPropertySet(), propertyChangedListener);
   }
   
   public void bidirectionalBindCliffClearance(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, minimumDistanceFromCliffBottoms, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaximum2dDistanceFromBoundingBoxToPenalize(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximum2dDistanceFromBoundingBoxToPenalize, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindBoundingBoxCost(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, boundingBoxCost, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindAStarHeuristicsWeight(Property<? extends Number> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, aStarHeuristicsWeight, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindUseQuadraticHeightCost(Property<Boolean> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToBooleanProperty(property, useQuadraticHeightCost, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindUseQuadraticDistanceCost(Property<Boolean> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToBooleanProperty(property, useQuadraticDistanceCost, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxXForStepUp(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximumStepReachWhenSteppingUp, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinZToConsiderStepUp(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximumStepZWhenSteppingUp, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMaxXForStepDown(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximumStepXWhenForwardAndDown, parameters.getStoredPropertySet(), propertyChangedListener);
   }

   public void bidirectionalBindMinZToConsiderStepDown(Property<Double> property, InvalidationListener propertyChangedListener)
   {
      bidirectionalBindToDoubleProperty(property, maximumStepZWhenForwardAndDown, parameters.getStoredPropertySet(), propertyChangedListener);
   }
}
