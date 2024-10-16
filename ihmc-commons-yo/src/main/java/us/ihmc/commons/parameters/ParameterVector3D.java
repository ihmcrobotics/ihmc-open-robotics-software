package us.ihmc.commons.parameters;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterVector3D implements Vector3DReadOnly
{
   private final DoubleParameter x;
   private final DoubleParameter y;
   private final DoubleParameter z;

   public ParameterVector3D(String prefix, YoRegistry registry)
   {
      this(prefix, null, registry);
   }

   public ParameterVector3D(String prefix, Tuple3DReadOnly defaults, YoRegistry registry)
   {
      if (defaults == null)
      {
         x = new DoubleParameter(prefix + "X", registry);
         y = new DoubleParameter(prefix + "Y", registry);
         z = new DoubleParameter(prefix + "Z", registry);
      }
      else
      {
         x = new DoubleParameter(prefix + "X", registry, defaults.getX());
         y = new DoubleParameter(prefix + "Y", registry, defaults.getY());
         z = new DoubleParameter(prefix + "Z", registry, defaults.getZ());
      }
   }

   public ParameterVector3D(String prefix, double defaultX, double defaultY, double defaultZ, YoRegistry registry)
   {
      x = new DoubleParameter(prefix + "X", registry, defaultX);
      y = new DoubleParameter(prefix + "Y", registry, defaultY);
      z = new DoubleParameter(prefix + "Z", registry, defaultZ);
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

   @Override
   public double getZ()
   {
      return z.getValue();
   }
}
