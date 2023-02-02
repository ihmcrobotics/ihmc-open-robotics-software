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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;

import java.util.ArrayList;

public class RDXSplineGraphic implements RenderableProvider
{
   private static final float sphereRadius = 0.02f;
   private static final float lineWidth = 0.01f;

   private RDXLineMeshModel lineModel = new RDXLineMeshModel(lineWidth);
   private final ArrayList<Point3D> linePoints = new ArrayList<>();
   private ModelInstance sphereStartPoint;
   private ModelInstance sphereEndPoint;
   private boolean cleared = true;
   private RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void createStart(Point3D point, Color color)
   {
      sphereStartPoint = RDXModelBuilder.createSphere(sphereRadius, color);
      Pose3DReadOnly pose = new Pose3D(point.getX(), point.getY(), point.getZ(), 0, 0, 0);
      LibGDXTools.toLibGDX(pose, tempTransform, sphereStartPoint.transform);
      linePoints.add(point);
      cleared = false;
   }

   public void createEnd(Color color)
   {
      sphereEndPoint = RDXModelBuilder.createSphere(sphereRadius, color);
      var lastPoint = linePoints.get(linePoints.size() - 1);
      Pose3DReadOnly pose = new Pose3D(lastPoint.getX(), lastPoint.getY(), lastPoint.getZ(), 0, 0, 0);
      LibGDXTools.toLibGDX(pose, tempTransform, sphereEndPoint.transform);
   }

   public void createAdditionalPoint(Point3D point, Color color)
   {
      linePoints.add(point);
      lineModel.setColor(color);
      lineModel.generateMeshes(linePoints,1);
      lineModel.update();
   }

   public void clear()
   {
      cleared = true;
      lineModel.clear();
      linePoints.clear();
      sphereEndPoint = null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (!cleared)
      {
         sphereStartPoint.getRenderables(renderables, pool);
         lineModel.getRenderables(renderables, pool);
         if (sphereEndPoint != null)
            sphereEndPoint.getRenderables(renderables, pool);
      }
   }
}