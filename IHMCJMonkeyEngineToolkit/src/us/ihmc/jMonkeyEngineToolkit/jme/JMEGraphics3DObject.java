package us.ihmc.jMonkeyEngineToolkit.jme;

import com.jme3.scene.Geometry;

import us.ihmc.graphicsDescription.Graphics3DObject;

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
