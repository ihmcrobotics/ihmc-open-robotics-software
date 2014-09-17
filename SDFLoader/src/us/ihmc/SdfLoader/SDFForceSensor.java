package us.ihmc.SdfLoader;

import us.ihmc.utilities.math.geometry.Transform3d;

public class SDFForceSensor
{
   private final String name;
   private final Transform3d transform;

   public String getName()
   {
      return name;
   }

   public Transform3d getTransform()
   {
      return transform;
   }

   public SDFForceSensor(String name, Transform3d transform)
   {
      this.name = name;
      this.transform = transform;
   }

}
