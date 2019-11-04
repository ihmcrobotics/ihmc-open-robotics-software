package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;

public class ImageSegmentationParametersProperty extends ParametersProperty<ImageSegmentationParameters>
{
   private final IntegerField pixelSize = new IntegerField(ImageSegmentationParameters::getPixelSize, (p, v) -> p.setPixelSize(v));
   private final DoubleField pixelRuler = new DoubleField(ImageSegmentationParameters::getPixelRuler, (p, v) -> p.setPixelRuler(v));
   private final IntegerField iterate = new IntegerField(ImageSegmentationParameters::getIterate, (p, v) -> p.setIterate(v));
   private final BooleanField enableConnectivity = new BooleanField(ImageSegmentationParameters::getEnableConnectivity, (p, v) -> p.setEnableConnectivity(v));
   private final IntegerField minElementSize = new IntegerField(ImageSegmentationParameters::getMinElementSize, (p, v) -> p.setMinElementSize(v));

   public ImageSegmentationParametersProperty(Object bean, String name)
   {
      super(bean, name, new ImageSegmentationParameters());
   }

   public void bindBidirectionalPixelSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, pixelSize);
   }

   public void bindBidirectionalPixelRuler(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, pixelRuler);
   }

   public void bindBidirectionalIterate(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, iterate);
   }

   public void bindBidirectionalEnableConnectivity(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableConnectivity);
   }

   public void bindBidirectionalMinElementSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minElementSize);
   }

   @Override
   protected ImageSegmentationParameters getValueCopy(ImageSegmentationParameters valueToCopy)
   {
      return new ImageSegmentationParameters(valueToCopy);
   }
}
