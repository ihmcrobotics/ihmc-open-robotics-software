package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class FootstepPlannerParametersProperty extends ParametersProperty<SettablePawPlannerParameters>
{
   private final DoubleField maximumFrontStepReach = new DoubleField(SettablePawPlannerParameters::getMaximumFrontStepReach, SettablePawPlannerParameters::setMaximumFrontStepReach);
   private final DoubleField maximumFrontStepLength= new DoubleField(SettablePawPlannerParameters::getMaximumFrontStepLength, SettablePawPlannerParameters::setMaximumFrontStepLength);
   private final DoubleField minimumFrontStepLength = new DoubleField(SettablePawPlannerParameters::getMinimumFrontStepLength, SettablePawPlannerParameters::setMinimumFrontStepLength);
   private final DoubleField maximumHindStepReach = new DoubleField(SettablePawPlannerParameters::getMaximumHindStepReach, SettablePawPlannerParameters::setMaximumHindStepReach);
   private final DoubleField maximumHindStepLength = new DoubleField(SettablePawPlannerParameters::getMaximumHindStepLength, SettablePawPlannerParameters::setMaximumHindStepLength);
   private final DoubleField minimumHindStepLength = new DoubleField(SettablePawPlannerParameters::getMinimumHindStepLength, SettablePawPlannerParameters::setMinimumHindStepLength);
   private final DoubleField maximumStepWidth = new DoubleField(SettablePawPlannerParameters::getMaximumStepOutward, SettablePawPlannerParameters::setMaximumStepOutward);
   private final DoubleField minimumStepWidth = new DoubleField(SettablePawPlannerParameters::getMaximumStepInward, SettablePawPlannerParameters::setMaximumStepInward);
   private final DoubleField minimumStepYaw = new DoubleField(SettablePawPlannerParameters::getMaximumStepYawInward, SettablePawPlannerParameters::setMaximumStepYawInward);
   private final DoubleField maximumStepYaw = new DoubleField(SettablePawPlannerParameters::getMaximumStepYawOutward, SettablePawPlannerParameters::setMaximumStepYawOutward);

   private final DoubleField maximumFrontStepLengthWhenSteppingUp = new DoubleField(SettablePawPlannerParameters::getMaximumFrontStepLengthWhenSteppingUp, SettablePawPlannerParameters::setMaximumFrontStepLengthWhenSteppingUp);
   private final DoubleField minimumFrontStepLengthWhenSteppingUp = new DoubleField(SettablePawPlannerParameters::getMinimumFrontStepLengthWhenSteppingUp, SettablePawPlannerParameters::setMinimumFrontStepLengthWhenSteppingUp);
   private final DoubleField maximumHindStepLengthWhenSteppingUp = new DoubleField(SettablePawPlannerParameters::getMaximumHindStepLengthWhenSteppingUp, SettablePawPlannerParameters::setMaximumHindStepLengthWhenSteppingUp);
   private final DoubleField minimumHindStepLengthWhenSteppingUp = new DoubleField(SettablePawPlannerParameters::getMinimumHindStepLengthWhenSteppingUp, SettablePawPlannerParameters::setMinimumHindStepLengthWhenSteppingUp);
   private final DoubleField stepZForSteppingUp = new DoubleField(SettablePawPlannerParameters::getStepZForSteppingUp, SettablePawPlannerParameters::setStepZForSteppingUp);

   private final DoubleField maximumFrontStepLengthWhenSteppingDown = new DoubleField(SettablePawPlannerParameters::getMaximumFrontStepLengthWhenSteppingDown, SettablePawPlannerParameters::setMaximumFrontStepLengthWhenSteppingDown);
   private final DoubleField minimumFrontStepLengthWhenSteppingDown = new DoubleField(SettablePawPlannerParameters::getMinimumFrontStepLengthWhenSteppingDown, SettablePawPlannerParameters::setMinimumFrontStepLengthWhenSteppingDown);
   private final DoubleField maximumHindStepLengthWhenSteppingDown = new DoubleField(SettablePawPlannerParameters::getMaximumHindStepLengthWhenSteppingDown, SettablePawPlannerParameters::setMaximumHindStepLengthWhenSteppingDown);
   private final DoubleField minimumHindStepLengthWhenSteppingDown = new DoubleField(SettablePawPlannerParameters::getMinimumHindStepLengthWhenSteppingDown, SettablePawPlannerParameters::setMinimumHindStepLengthWhenSteppingDown);
   private final DoubleField stepZForSteppingDown = new DoubleField(SettablePawPlannerParameters::getStepZForSteppingDown, SettablePawPlannerParameters::setStepZForSteppingDown);

   private final DoubleField maximumStepChangeZ = new DoubleField(SettablePawPlannerParameters::getMaximumStepChangeZ, SettablePawPlannerParameters::setMaximumStepChangeZ);
   private final DoubleField bodyGroundClearance = new DoubleField(SettablePawPlannerParameters::getBodyGroundClearance, SettablePawPlannerParameters::setBodyGroundClearance);

   private final DoubleField maxWalkingSpeedMultiplier = new DoubleField(SettablePawPlannerParameters::getMaxWalkingSpeedMultiplier, SettablePawPlannerParameters::setMaxWalkingSpeedMultiplier);

   private final DoubleField projectInsideDistance = new DoubleField(SettablePawPlannerParameters::getProjectInsideDistance, SettablePawPlannerParameters::setProjectInsideDistance);
   private final DoubleField maximumXYWiggleDistance = new DoubleField(SettablePawPlannerParameters::getMaximumXYWiggleDistance, SettablePawPlannerParameters::setMaximumXYWiggleDistance);
   private final DoubleField cliffHeightToAvoid = new DoubleField(SettablePawPlannerParameters::getCliffHeightToAvoid, SettablePawPlannerParameters::setCliffHeightToAvoid);
   private final DoubleField minFrontEndForwardDistanceFromCliffBottoms = new DoubleField(SettablePawPlannerParameters::getMinimumFrontEndForwardDistanceFromCliffBottoms, SettablePawPlannerParameters::setMinimumFrontEndForwardDistanceFromCliffBottoms);
   private final DoubleField minFrontEndBackwardDistanceFromCliffBottoms = new DoubleField(SettablePawPlannerParameters::getMinimumFrontEndBackwardDistanceFromCliffBottoms, SettablePawPlannerParameters::setMinimumFrontEndBackwardDistanceFromCliffBottoms);
   private final DoubleField minHindEndForwardDistanceFromCliffBottoms = new DoubleField(SettablePawPlannerParameters::getMinimumHindEndForwardDistanceFromCliffBottoms, SettablePawPlannerParameters::setMinimumHindEndForwardDistanceFromCliffBottoms);
   private final DoubleField minHindEndBackwardDistanceFromCliffBottoms = new DoubleField(SettablePawPlannerParameters::getMinimumHindEndBackwardDistanceFromCliffBottoms, SettablePawPlannerParameters::setMinimumHindEndBackwardDistanceFromCliffBottoms);
   private final DoubleField minLateralDistanceFromCliffBottoms = new DoubleField(SettablePawPlannerParameters::getMinimumLateralDistanceFromCliffBottoms, SettablePawPlannerParameters::setMinimumLateralDistanceFromCliffBottoms);

   private final DoubleField distanceWeight = new DoubleField(SettablePawPlannerParameters::getDistanceWeight, SettablePawPlannerParameters::setDistanceWeight);
   private final DoubleField xGaitWeight = new DoubleField(SettablePawPlannerParameters::getXGaitWeight, SettablePawPlannerParameters::setXGaitWeight);
   private final DoubleField desiredVelocityWeight = new DoubleField(SettablePawPlannerParameters::getDesiredVelocityWeight, SettablePawPlannerParameters::setDesiredVelocityWeight);
   private final DoubleField yawWeight = new DoubleField(SettablePawPlannerParameters::getYawWeight, SettablePawPlannerParameters::setYawWeight);
   private final DoubleField costPerStep = new DoubleField(SettablePawPlannerParameters::getCostPerStep, SettablePawPlannerParameters::setCostPerStep);
   private final DoubleField stepUpWeight  = new DoubleField(SettablePawPlannerParameters::getStepUpWeight, SettablePawPlannerParameters::setStepUpWeight);
   private final DoubleField stepDownWeight  = new DoubleField(SettablePawPlannerParameters::getStepDownWeight, SettablePawPlannerParameters::setStepDownWeight);
   private final DoubleField heuristicsWeight = new DoubleField(SettablePawPlannerParameters::getHeuristicsInflationWeight, SettablePawPlannerParameters::setHeuristicsInflationWeight);

   private final DoubleField minXClearanceFromFoot = new DoubleField(SettablePawPlannerParameters::getMinXClearanceFromPaw, SettablePawPlannerParameters::setMinXClearanceFromFoot);
   private final DoubleField minYClearanceFromFoot = new DoubleField(SettablePawPlannerParameters::getMinYClearanceFromPaw, SettablePawPlannerParameters::setMinYClearanceFromFoot);
   private final DoubleField minimumSurfaceInclineRadians = new DoubleField(SettablePawPlannerParameters::getMinimumSurfaceInclineRadians, SettablePawPlannerParameters::setMinimumSurfaceInclineRadians);

   private final DoubleField finalTurnProximity = new DoubleField(SettablePawPlannerParameters::getFinalTurnProximity, SettablePawPlannerParameters::setFinalTurnProximity);
   private final DoubleField finalSlowDownProximity = new DoubleField(SettablePawPlannerParameters::getFinalSlowDownProximity, SettablePawPlannerParameters::setFinalSlowDownProximity);

   private final BooleanField projectIntoConvexHull = new BooleanField(SettablePawPlannerParameters::getProjectInsideUsingConvexHull, SettablePawPlannerParameters::setProjectInsideUsingConvexHull);

   public FootstepPlannerParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultPawPlannerParameters());
   }

   public FootstepPlannerParametersProperty(Object bean, String name, PawPlannerParametersReadOnly pawPlannerParameters)
   {
      super(bean, name, new SettablePawPlannerParameters(pawPlannerParameters));
   }

   public void setPlannerParameters(PawPlannerParametersReadOnly parameters)
   {
      setValue(new SettablePawPlannerParameters(parameters));
   }

   @Override
   protected SettablePawPlannerParameters getValueCopy(SettablePawPlannerParameters valueToCopy)
   {
      return new SettablePawPlannerParameters(valueToCopy);
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
