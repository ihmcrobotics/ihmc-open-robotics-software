package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

import java.util.concurrent.atomic.AtomicReference;

public class MessagerPlannerParametersProperty extends ParametersProperty<SettableMessagerPlannerParameters>
{
   private DoubleField idealFootstepWidth = new DoubleField(SettableMessagerPlannerParameters::getIdealFootstepWidth, (p, v) -> p.setIdealFootstepWidth(v));
   private DoubleField idealFootstepLength = new DoubleField(SettableMessagerPlannerParameters::getIdealFootstepLength, (p, v) -> p.setIdealFootstepLength(v));
   private DoubleField maxStepReach = new DoubleField(SettableMessagerPlannerParameters::getMaximumStepReach, (p, v) -> p.setMaximumStepReach(v));
   private DoubleField maxStepYaw = new DoubleField(SettableMessagerPlannerParameters::getMaximumStepYaw, (p, v) -> p.setMaximumStepYaw(v));
   private DoubleField minStepWidth = new DoubleField(SettableMessagerPlannerParameters::getMinimumStepWidth, (p, v) -> p.setMinimumStepWidth(v));
   private DoubleField minStepLength = new DoubleField(SettableMessagerPlannerParameters::getMinimumStepLength, (p, v) -> p.setMinimumStepLength(v));
   private DoubleField minStepYaw = new DoubleField(SettableMessagerPlannerParameters::getMinimumStepYaw, (p, v) -> p.setMinimumStepYaw(v));
   private DoubleField maxStepZ = new DoubleField(SettableMessagerPlannerParameters::getMaximumStepZ, (p, v) -> p.setMaximumStepZ(v));
   private DoubleField minFootholdPercent = new DoubleField(SettableMessagerPlannerParameters::getMinimumFootholdPercent, (p, v) -> p.setMinimumFootholdPercent(v));
   private DoubleField minSurfaceIncline = new DoubleField(SettableMessagerPlannerParameters::getMinimumSurfaceInclineRadians, (p, v) -> p.setMinimumSurfaceInclineRadians(v));
   private DoubleField maxStepWidth = new DoubleField(SettableMessagerPlannerParameters::getMaximumStepWidth, (p, v) -> p.setMaximumStepWidth(v));


   public MessagerPlannerParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultFootstepPlanningParameters());
   }

   public MessagerPlannerParametersProperty(Object bean, String name, FootstepPlannerParameters footstepPlannerParameters)
   {
      super(bean, name, new SettableMessagerPlannerParameters(footstepPlannerParameters));
   }

   public void setPlannerParameters(FootstepPlannerParameters parameters)
   {
      setValue(new SettableMessagerPlannerParameters(parameters));
   }

   @Override
   protected SettableMessagerPlannerParameters getValueCopy(SettableMessagerPlannerParameters valueToCopy)
   {
      return new SettableMessagerPlannerParameters(valueToCopy);
   }

   public void bidirectionalBindIdealFootstepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, idealFootstepWidth);
   }

   public void bidirectionalBindIdealFootstepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, idealFootstepLength);
   }

   public void bidirectionalBindMaxStepReach(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepReach);
   }

   public void bidirectionalBindMaxStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepYaw);
   }

   public void bidirectionalBindMinStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepWidth);
   }

   public void bidirectionalBindMinStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepLength);
   }

   public void bidirectionalBindMinStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepYaw);
   }

   public void bidirectionalBindMaxStepZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepZ);
   }

   public void bidirectionalBindMinFootholdPercent(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minFootholdPercent);
   }

   public void bidirectionalBindMinSurfaceIncline(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minSurfaceIncline);
   }

   public void bidirectionalBindMaxStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepWidth);
   }
}
