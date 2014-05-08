package us.ihmc.SdfLoader;

import javax.media.j3d.Transform3D;

public class SDFForceSensor
{
   private final String name;
   private final Transform3D transform;

   public String getName()
   {
      return name;
   }

   public Transform3D getTransform()
   {
      return transform;
   }

   public SDFForceSensor(String name, Transform3D transform)
   {
      this.name = name;
      this.transform = transform;
   }

}
