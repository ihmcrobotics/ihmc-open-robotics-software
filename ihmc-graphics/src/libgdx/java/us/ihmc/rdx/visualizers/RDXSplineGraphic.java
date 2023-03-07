package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;

public class RDXSplineGraphic implements RenderableProvider
{
   private static final float sphereRadius = 0.02f;
   private static final float lineWidth = 0.01f;

   private final RDXSplineBody line = new RDXSplineBody(lineWidth);
   private ModelInstance sphereStartPoint;
   private ModelInstance sphereEndPoint;
   private boolean cleared = true;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private Point3DReadOnly lastPoint;

   public void createStart(Point3DReadOnly point, Color color)
   {
      sphereStartPoint = RDXModelBuilder.createSphere(sphereRadius, color);
      Pose3DReadOnly pose = new Pose3D(point.getX(), point.getY(), point.getZ(), 0, 0, 0);
      LibGDXTools.toLibGDX(pose, tempTransform, sphereStartPoint.transform);
      cleared = false;
      lastPoint = point;
   }

   public void createEnd(Color color)
   {
      sphereEndPoint = RDXModelBuilder.createSphere(sphereRadius, color);
      Pose3DReadOnly pose = new Pose3D(lastPoint.getX(), lastPoint.getY(), lastPoint.getZ(), 0, 0, 0);
      LibGDXTools.toLibGDX(pose, tempTransform, sphereEndPoint.transform);
   }

   public void createAdditionalPoint(Point3DReadOnly point, Color color)
   {
      line.setColor(color);
      line.generateMeshes(lastPoint, point);
      lastPoint = point;
   }

   public void clear()
   {
      cleared = true;
      line.clear();
      sphereEndPoint = null;
      lastPoint = null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (!cleared)
      {
         sphereStartPoint.getRenderables(renderables, pool);
         line.getRenderables(renderables, pool);
         if (sphereEndPoint != null)
            sphereEndPoint.getRenderables(renderables, pool);
      }
   }
}
