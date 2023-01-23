package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;

import java.util.ArrayList;

public class RDXSplineGraphic implements RenderableProvider
{
   private final ArrayList<ModelInstance> lineSegments = new ArrayList<>();
   private ModelInstance sphereStartPoint;
   private ModelInstance sphereEndPoint;
   private Point3DReadOnly lastPoint;
   private boolean cleared = true;
   private RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void createStart(Point3DReadOnly point, Color color)
   {
      sphereStartPoint = RDXModelBuilder.createSphere((float) 0.02, color);
      Pose3DReadOnly pose = new Pose3D(point.getX(), point.getY(), point.getZ(), 0, 0, 0);
      LibGDXTools.toLibGDX(pose, tempTransform, sphereStartPoint.transform);
      lastPoint = point;
      cleared = false;
   }

   public void createEnd(Color color)
   {
      sphereEndPoint = RDXModelBuilder.createSphere((float) 0.02, color);
      Pose3DReadOnly pose = new Pose3D(lastPoint.getX(), lastPoint.getY(), lastPoint.getZ(), 0, 0, 0);
      LibGDXTools.toLibGDX(pose, tempTransform, sphereEndPoint.transform);
   }

   public void createAdditionalPoint(Point3DReadOnly point, Color color)
   {
      lineSegments.add(RDXModelBuilder.createLine(lastPoint, point, 0.01, color));
      lastPoint = point;
   }

   public void clear()
   {
      cleared = true;
      lineSegments.clear();
      sphereEndPoint = null;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (!cleared)
      {
         sphereStartPoint.getRenderables(renderables, pool);
         for (var lineSegment : lineSegments)
            lineSegment.getRenderables(renderables, pool);
         if (sphereEndPoint != null)
            sphereEndPoint.getRenderables(renderables, pool);
      }
   }
}
