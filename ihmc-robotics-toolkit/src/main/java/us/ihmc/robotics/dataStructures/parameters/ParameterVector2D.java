package us.ihmc.robotics.dataStructures.parameters;

import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterVector2D implements Vector2DReadOnly
{
   private final DoubleParameter x;
   private final DoubleParameter y;

   public ParameterVector2D(String prefix, YoRegistry registry)
   {
      this(prefix, null, registry);
   }

   public ParameterVector2D(String prefix, Vector2DReadOnly defaults, YoRegistry registry)
   {
      if (defaults == null)
      {
         x = new DoubleParameter(prefix + "X", registry);
         y = new DoubleParameter(prefix + "Y", registry);
      }
      else
      {
         x = new DoubleParameter(prefix + "X", registry, defaults.getX());
         y = new DoubleParameter(prefix + "Y", registry, defaults.getY());
      }
   }

   @Override
   public double getX()
   {
      return x.getValue();
   }

   @Override
   public double getY()
   {
      return y.getValue();
   }

   public DoubleParameter getXParameter()
   {
      return x;
   }

   public DoubleParameter getYParameter()
   {
      return y;
   }
}
