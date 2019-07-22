package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class FootstepPlannerParametersProperty extends ParametersProperty<SettableFootstepPlannerParameters>
{
   private final DoubleField maximumFrontStepReach = new DoubleField(SettableFootstepPlannerParameters::getMaximumFrontStepReach, SettableFootstepPlannerParameters::setMaximumFrontStepReach);
   private final DoubleField maximumFrontStepLength= new DoubleField(SettableFootstepPlannerParameters::getMaximumFrontStepLength, SettableFootstepPlannerParameters::setMaximumFrontStepLength);
   private final DoubleField minimumFrontStepLength = new DoubleField(SettableFootstepPlannerParameters::getMinimumFrontStepLength, SettableFootstepPlannerParameters::setMinimumFrontStepLength);
   private final DoubleField maximumHindStepReach = new DoubleField(SettableFootstepPlannerParameters::getMaximumHindStepReach, SettableFootstepPlannerParameters::setMaximumHindStepReach);
   private final DoubleField maximumHindStepLength = new DoubleField(SettableFootstepPlannerParameters::getMaximumHindStepLength, SettableFootstepPlannerParameters::setMaximumHindStepLength);
   private final DoubleField minimumHindStepLength = new DoubleField(SettableFootstepPlannerParameters::getMinimumHindStepLength, SettableFootstepPlannerParameters::setMinimumHindStepLength);
   private final DoubleField maximumStepWidth = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepOutward, SettableFootstepPlannerParameters::setMaximumStepOutward);
   private final DoubleField minimumStepWidth = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepInward, SettableFootstepPlannerParameters::setMaximumStepInward);
   private final DoubleField minimumStepYaw = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepYawInward, SettableFootstepPlannerParameters::setMaximumStepYawInward);
   private final DoubleField maximumStepYaw = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepYawOutward, SettableFootstepPlannerParameters::setMaximumStepYawOutward);

   private final DoubleField maximumFrontStepLengthWhenSteppingUp = new DoubleField(SettableFootstepPlannerParameters::getMaximumFrontStepLengthWhenSteppingUp, SettableFootstepPlannerParameters::setMaximumFrontStepLengthWhenSteppingUp);
   private final DoubleField minimumFrontStepLengthWhenSteppingUp = new DoubleField(SettableFootstepPlannerParameters::getMinimumFrontStepLengthWhenSteppingUp, SettableFootstepPlannerParameters::setMinimumFrontStepLengthWhenSteppingUp);
   private final DoubleField maximumHindStepLengthWhenSteppingUp = new DoubleField(SettableFootstepPlannerParameters::getMaximumHindStepLengthWhenSteppingUp, SettableFootstepPlannerParameters::setMaximumHindStepLengthWhenSteppingUp);
   private final DoubleField minimumHindStepLengthWhenSteppingUp = new DoubleField(SettableFootstepPlannerParameters::getMinimumHindStepLengthWhenSteppingUp, SettableFootstepPlannerParameters::setMinimumHindStepLengthWhenSteppingUp);
   private final DoubleField stepZForSteppingUp = new DoubleField(SettableFootstepPlannerParameters::getStepZForSteppingUp, SettableFootstepPlannerParameters::setStepZForSteppingUp);

   private final DoubleField maximumFrontStepLengthWhenSteppingDown = new DoubleField(SettableFootstepPlannerParameters::getMaximumFrontStepLengthWhenSteppingDown, SettableFootstepPlannerParameters::setMaximumFrontStepLengthWhenSteppingDown);
   private final DoubleField minimumFrontStepLengthWhenSteppingDown = new DoubleField(SettableFootstepPlannerParameters::getMinimumFrontStepLengthWhenSteppingDown, SettableFootstepPlannerParameters::setMinimumFrontStepLengthWhenSteppingDown);
   private final DoubleField maximumHindStepLengthWhenSteppingDown = new DoubleField(SettableFootstepPlannerParameters::getMaximumHindStepLengthWhenSteppingDown, SettableFootstepPlannerParameters::setMaximumHindStepLengthWhenSteppingDown);
   private final DoubleField minimumHindStepLengthWhenSteppingDown = new DoubleField(SettableFootstepPlannerParameters::getMinimumHindStepLengthWhenSteppingDown, SettableFootstepPlannerParameters::setMinimumHindStepLengthWhenSteppingDown);
   private final DoubleField stepZForSteppingDown = new DoubleField(SettableFootstepPlannerParameters::getStepZForSteppingDown, SettableFootstepPlannerParameters::setStepZForSteppingDown);

   private final DoubleField maximumStepChangeZ = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepChangeZ, SettableFootstepPlannerParameters::setMaximumStepChangeZ);
   private final DoubleField bodyGroundClearance = new DoubleField(SettableFootstepPlannerParameters::getBodyGroundClearance, SettableFootstepPlannerParameters::setBodyGroundClearance);

   private final DoubleField maxWalkingSpeedMultiplier = new DoubleField(SettableFootstepPlannerParameters::getMaxWalkingSpeedMultiplier, SettableFootstepPlannerParameters::setMaxWalkingSpeedMultiplier);

   private final DoubleField projectInsideDistance = new DoubleField(SettableFootstepPlannerParameters::getProjectInsideDistance, SettableFootstepPlannerParameters::setProjectInsideDistance);
   private final DoubleField maximumXYWiggleDistance = new DoubleField(SettableFootstepPlannerParameters::getMaximumXYWiggleDistance, SettableFootstepPlannerParameters::setMaximumXYWiggleDistance);
   private final DoubleField cliffHeightToAvoid = new DoubleField(SettableFootstepPlannerParameters::getCliffHeightToAvoid, SettableFootstepPlannerParameters::setCliffHeightToAvoid);
   private final DoubleField minFrontEndForwardDistanceFromCliffBottoms = new DoubleField(SettableFootstepPlannerParameters::getMinimumFrontEndForwardDistanceFromCliffBottoms, SettableFootstepPlannerParameters::setMinimumFrontEndForwardDistanceFromCliffBottoms);
   private final DoubleField minFrontEndBackwardDistanceFromCliffBottoms = new DoubleField(SettableFootstepPlannerParameters::getMinimumFrontEndBackwardDistanceFromCliffBottoms, SettableFootstepPlannerParameters::setMinimumFrontEndBackwardDistanceFromCliffBottoms);
   private final DoubleField minHindEndForwardDistanceFromCliffBottoms = new DoubleField(SettableFootstepPlannerParameters::getMinimumHindEndForwardDistanceFromCliffBottoms, SettableFootstepPlannerParameters::setMinimumHindEndForwardDistanceFromCliffBottoms);
   private final DoubleField minHindEndBackwardDistanceFromCliffBottoms = new DoubleField(SettableFootstepPlannerParameters::getMinimumHindEndBackwardDistanceFromCliffBottoms, SettableFootstepPlannerParameters::setMinimumHindEndBackwardDistanceFromCliffBottoms);
   private final DoubleField minLateralDistanceFromCliffBottoms = new DoubleField(SettableFootstepPlannerParameters::getMinimumLateralDistanceFromCliffBottoms, SettableFootstepPlannerParameters::setMinimumLateralDistanceFromCliffBottoms);

   private final DoubleField distanceWeight = new DoubleField(SettableFootstepPlannerParameters::getDistanceWeight, SettableFootstepPlannerParameters::setDistanceWeight);
   private final DoubleField xGaitWeight = new DoubleField(SettableFootstepPlannerParameters::getXGaitWeight, SettableFootstepPlannerParameters::setXGaitWeight);
   private final DoubleField desiredVelocityWeight = new DoubleField(SettableFootstepPlannerParameters::getDesiredVelocityWeight, SettableFootstepPlannerParameters::setDesiredVelocityWeight);
   private final DoubleField yawWeight = new DoubleField(SettableFootstepPlannerParameters::getYawWeight, SettableFootstepPlannerParameters::setYawWeight);
   private final DoubleField costPerStep = new DoubleField(SettableFootstepPlannerParameters::getCostPerStep, SettableFootstepPlannerParameters::setCostPerStep);
   private final DoubleField stepUpWeight  = new DoubleField(SettableFootstepPlannerParameters::getStepUpWeight, SettableFootstepPlannerParameters::setStepUpWeight);
   private final DoubleField stepDownWeight  = new DoubleField(SettableFootstepPlannerParameters::getStepDownWeight, SettableFootstepPlannerParameters::setStepDownWeight);
   private final DoubleField heuristicsWeight = new DoubleField(SettableFootstepPlannerParameters::getHeuristicsInflationWeight, SettableFootstepPlannerParameters::setHeuristicsInflationWeight);

   private final DoubleField minXClearanceFromFoot = new DoubleField(SettableFootstepPlannerParameters::getMinXClearanceFromFoot, SettableFootstepPlannerParameters::setMinXClearanceFromFoot);
   private final DoubleField minYClearanceFromFoot = new DoubleField(SettableFootstepPlannerParameters::getMinYClearanceFromFoot, SettableFootstepPlannerParameters::setMinYClearanceFromFoot);
   private final DoubleField minimumSurfaceInclineRadians = new DoubleField(SettableFootstepPlannerParameters::getMinimumSurfaceInclineRadians, SettableFootstepPlannerParameters::setMinimumSurfaceInclineRadians);

   private final DoubleField finalTurnProximity = new DoubleField(SettableFootstepPlannerParameters::getFinalTurnProximity, SettableFootstepPlannerParameters::setFinalTurnProximity);
   private final DoubleField finalSlowDownProximity = new DoubleField(SettableFootstepPlannerParameters::getFinalSlowDownProximity, SettableFootstepPlannerParameters::setFinalSlowDownProximity);

   private final BooleanField projectIntoConvexHull = new BooleanField(SettableFootstepPlannerParameters::getProjectInsideUsingConvexHull, SettableFootstepPlannerParameters::setProjectInsideUsingConvexHull);

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

   public void bidirectionalBindMaximumFrontStepReach(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumFrontStepReach);
   }

   public void bidirectionalBindMaximumFrontStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumFrontStepLength);
   }

   public void bidirectionalBindMinimumFrontStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumFrontStepLength);
   }

   public void bidirectionalBindMaximumHindStepReach(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumHindStepReach);
   }

   public void bidirectionalBindMaximumHindStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumHindStepLength);
   }

   public void bidirectionalBindMinimumHindStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumHindStepLength);
   }

   public void bidirectionalBindMaximumFrontStepLengthWhenSteppingUp(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumFrontStepLengthWhenSteppingUp);
   }

   public void bidirectionalBindMinimumFrontStepLengthWhenSteppingUp(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumFrontStepLengthWhenSteppingUp);
   }

   public void bidirectionalBindMaximumHindStepLengthWhenSteppingUp(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumHindStepLengthWhenSteppingUp);
   }

   public void bidirectionalBindMinimumHindStepLengthWhenSteppingUp(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumHindStepLengthWhenSteppingUp);
   }

   public void bidirectionalBindStepZForSteppingUp(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepZForSteppingUp);
   }

   public void bidirectionalBindMaximumFrontStepLengthWhenSteppingDown(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumFrontStepLengthWhenSteppingDown);
   }

   public void bidirectionalBindMinimumFrontStepLengthWhenSteppingDown(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumFrontStepLengthWhenSteppingDown);
   }

   public void bidirectionalBindMaximumHindStepLengthWhenSteppingDown(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumHindStepLengthWhenSteppingDown);
   }

   public void bidirectionalBindMinimumHindStepLengthWhenSteppingDown(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumHindStepLengthWhenSteppingDown);
   }

   public void bidirectionalBindStepZForSteppingDown(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepZForSteppingDown);
   }

   public void bidirectionalBindMaxWalkingSpeedMultiplier(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxWalkingSpeedMultiplier);
   }

   public void bidirectionalBindCliffHeightToAvoid(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, cliffHeightToAvoid);
   }

   public void bidirectionalBindProjectInsideDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, projectInsideDistance);
   }

   public void bidirectionalBindMaximumXYWiggleDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumXYWiggleDistance);
   }

   public void bidirectionalBindMinFrontEndForwardDistanceFromCliffBottoms(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minFrontEndForwardDistanceFromCliffBottoms);
   }

   public void bidirectionalBindMinFrontEndBackwardDistanceFromCliffBottoms(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minFrontEndBackwardDistanceFromCliffBottoms);
   }

   public void bidirectionalBindMinHindEndForwardDistanceFromCliffBottoms(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minHindEndForwardDistanceFromCliffBottoms);
   }

   public void bidirectionalBindMinHindEndBackwardDistanceFromCliffBottoms(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minHindEndBackwardDistanceFromCliffBottoms);
   }

   public void bidirectionalBindMinLateralDistanceFromCliffBottoms(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minLateralDistanceFromCliffBottoms);
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

   public void bidirectionalBindDistanceWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, distanceWeight);
   }

   public void bidirectionalBindYawWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yawWeight);
   }

   public void bidirectionalBindXGaitWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, xGaitWeight);
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

   public void bidirectionalBindProjectIntoConvexHull(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, projectIntoConvexHull);
   }
}
