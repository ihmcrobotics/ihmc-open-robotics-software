package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.rdx.tools.GDXModelBuilder;
import us.ihmc.rdx.tools.GDXTools;

public class GDXSphereAndArrowGraphic
{
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private ModelInstance arrow;
   private ModelInstance sphere;
   private boolean cleared = true;

   public void create(double sphereRadius, double arrowLength, Color color)
   {
      sphere = GDXModelBuilder.createSphere((float) sphereRadius, color);
      arrow = GDXModelBuilder.createArrow(arrowLength, color);
   }

   public void clear()
   {
      cleared = true;
   }

   public void setToPose(Pose3DReadOnly pose)
   {
      GDXTools.toGDX(pose, tempTransform, sphere.transform);
      GDXTools.toGDX(pose, tempTransform, arrow.transform);
      cleared = false;
   }

   public void setToTransform(RigidBodyTransformReadOnly transform)
   {
      tempTransform.set(transform);
      GDXTools.toGDX(tempTransform, sphere.transform);
      GDXTools.toGDX(tempTransform, arrow.transform);
      cleared = false;
   }

   public RigidBodyTransformReadOnly getTransform()
   {
      GDXTools.toEuclid(arrow.transform, tempTransform);
      return tempTransform;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (!cleared)
      {
         sphere.getRenderables(renderables, pool);
         arrow.getRenderables(renderables, pool);
      }
   }
}
