package us.ihmc.robotics.dataStructures.parameters;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterVector3D implements Vector3DReadOnly
{
   private final DoubleParameter x;
   private final DoubleParameter y;
   private final DoubleParameter z;

   public ParameterVector3D(String prefix, YoVariableRegistry registry)
   {
      this(prefix, null, registry);
   }

   public ParameterVector3D(String prefix, Vector3DReadOnly defaults, YoVariableRegistry registry)
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
