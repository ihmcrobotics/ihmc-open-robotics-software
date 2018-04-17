package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;

public class BoundingBoxParametersProperty extends ParametersProperty<BoundingBoxParametersMessage>
{
   private final FloatField minX = new FloatField(BoundingBoxParametersMessage::getMinX, (p, v) -> p.setMinX(v));
   private final FloatField minY = new FloatField(BoundingBoxParametersMessage::getMinY, (p, v) -> p.setMinY(v));
   private final FloatField minZ = new FloatField(BoundingBoxParametersMessage::getMinZ, (p, v) -> p.setMinZ(v));
   private final FloatField maxX = new FloatField(BoundingBoxParametersMessage::getMaxX, (p, v) -> p.setMaxX(v));
   private final FloatField maxY = new FloatField(BoundingBoxParametersMessage::getMaxY, (p, v) -> p.setMaxY(v));
   private final FloatField maxZ = new FloatField(BoundingBoxParametersMessage::getMaxZ, (p, v) -> p.setMaxZ(v));

   public BoundingBoxParametersProperty(Object bean, String name)
   {
      super(bean, name, new BoundingBoxParametersMessage());
   }

   public void binBidirectionalMinX(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minX);
   }

   public void binBidirectionalMinY(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minY);
   }

   public void binBidirectionalMinZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minZ);
   }
   
   public void binBidirectionalMaxX(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxX);
   }
   
   public void binBidirectionalMaxY(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxY);
   }
   
   public void binBidirectionalMaxZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxZ);
   }

   @Override
   protected BoundingBoxParametersMessage getValueCopy(BoundingBoxParametersMessage valueToCopy)
   {
      return new BoundingBoxParametersMessage(valueToCopy);
   }
}
