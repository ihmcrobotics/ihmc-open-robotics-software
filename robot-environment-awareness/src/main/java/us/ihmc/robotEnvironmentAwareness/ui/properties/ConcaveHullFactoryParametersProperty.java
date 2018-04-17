package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;

public class ConcaveHullFactoryParametersProperty extends ParametersProperty<ConcaveHullFactoryParameters>
{
   private final DoubleField edgeLengthThreshold = new DoubleField(ConcaveHullFactoryParameters::getEdgeLengthThreshold, (p, v) -> p.setEdgeLengthThreshold(v));
   private final BooleanField removeAllTrianglesWithTwoBorderEdges = new BooleanField(ConcaveHullFactoryParameters::doRemoveAllTrianglesWithTwoBorderEdges, (p, v) -> p.setRemoveAllTrianglesWithTwoBorderEdges(v));
   private final BooleanField allowSplittingConcaveHull = new BooleanField(ConcaveHullFactoryParameters::isSplittingConcaveHullAllowed, (p, v) -> p.setAllowSplittingConcaveHull(v));
   private final IntegerField maxNumberOfIterations = new IntegerField(ConcaveHullFactoryParameters::getMaxNumberOfIterations, (p, v) -> p.setMaxNumberOfIterations(v));

   public ConcaveHullFactoryParametersProperty(Object bean, String name)
   {
      super(bean, name, new ConcaveHullFactoryParameters());
   }

   public void bindBidirectionalEdgeLengthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, edgeLengthThreshold);
   }
   
   public void bindBidirectionalRemoveAllTrianglesWithTwoBorderEdges(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, removeAllTrianglesWithTwoBorderEdges);
   }
   
   public void bindBidirectionalAllowSplittingConcaveHull(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, allowSplittingConcaveHull);
   }
   
   public void bindBidirectionalMaxNumberOfIterations(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxNumberOfIterations);
   }

   @Override
   protected ConcaveHullFactoryParameters getValueCopy(ConcaveHullFactoryParameters valueToCopy)
   {
      return new ConcaveHullFactoryParameters(valueToCopy);
   }
}
