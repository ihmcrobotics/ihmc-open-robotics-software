package us.ihmc.graphics3DAdapter.jme;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

import com.jme3.scene.Geometry;

public class JMEGraphics3DObject extends Graphics3DObject
{
   private Geometry geo;

   public JMEGraphics3DObject(Geometry geo)
   {
      this.geo = geo;
   }

   public Geometry getGeometry()
   {
      return geo;
   }
}
