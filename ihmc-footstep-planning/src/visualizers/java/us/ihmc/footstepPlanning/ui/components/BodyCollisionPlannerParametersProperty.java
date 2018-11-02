package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

import java.util.concurrent.atomic.AtomicReference;

public class BodyCollisionPlannerParametersProperty extends ParametersProperty<SettableFootstepPlannerParameters>
{
   private BooleanField checkForBodyBoxCollision = new BooleanField(SettableFootstepPlannerParameters::checkForBodyBoxCollisions, (p, v) -> p.setCheckForBodyBoxCollisions(v));
   private DoubleField bodyBoxWidth = new DoubleField(SettableFootstepPlannerParameters::getBodyBoxWidth, (p, v) -> p.setBodyBoxWidth(v));
   private DoubleField bodyBoxDepth = new DoubleField(SettableFootstepPlannerParameters::getBodyBoxDepth, (p, v) -> p.setBodyBoxDepth(v));
   private DoubleField bodyBoxHeight = new DoubleField(SettableFootstepPlannerParameters::getBodyBoxHeight, (p, v) -> p.setBodyBoxHeight(v));
   private DoubleField bodyBoxBaseX = new DoubleField(SettableFootstepPlannerParameters::getBodyBoxBaseX, (p, v) -> p.setBodyBoxBaseZ(v));
   private DoubleField bodyBoxBaseY = new DoubleField(SettableFootstepPlannerParameters::getBodyBoxBaseY, (p, v) -> p.setBodyBoxBaseZ(v));
   private DoubleField bodyBoxBaseZ = new DoubleField(SettableFootstepPlannerParameters::getBodyBoxBaseZ, (p, v) -> p.setBodyBoxBaseZ(v));

   public BodyCollisionPlannerParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultFootstepPlanningParameters());
   }

   public BodyCollisionPlannerParametersProperty(Object bean, String name, FootstepPlannerParameters footstepPlannerParameters)
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

   public void bidirectionalBindCheckBodyBoxCollisions(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, checkForBodyBoxCollision);
   }

   public void bidirectionalBindBodyBoxWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxWidth);
   }

   public void bidirectionalBindBodyBoxDepth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxDepth);
   }

   public void bidirectionalBindBodyBoxHeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxHeight);
   }

   public void bidirectionalBindBodyBoxBaseX(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxBaseX);
   }

   public void bidirectionalBindBodyBoxBaseY(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxBaseY);
   }

   public void bidirectionalBindBodyBoxBaseZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, bodyBoxBaseZ);
   }
}
