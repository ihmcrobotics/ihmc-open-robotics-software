package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;
import georegression.struct.shapes.Sphere3D_F64;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import org.opensphere.geometry.algorithm.ConcaveHull;
import org.opensphere.geometry.triangulation.model.Edge;
import org.opensphere.geometry.triangulation.model.Triangle;
import org.opensphere.geometry.triangulation.model.Vertex;

import us.ihmc.sensorProcessing.concaveHull.BoundConcavePolygon;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;
import bubo.clouds.detect.alg.BoundCylinderBasic;
import bubo.clouds.detect.alg.BoundPlaneRectangle;

import com.jme3.app.SimpleApplication;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.material.RenderState.FaceCullMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Line;
import com.jme3.scene.shape.Sphere;
import com.jme3.util.BufferUtils;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;

public class ShapeTranslator
{
   SimpleApplication ui;

   public ShapeTranslator(SimpleApplication ui)
   {
      this.ui = ui;
   }

   public void translateShape(Shape s, ColorRGBA color, Node nodeToAddTo)
   {
      color.a = 0.5f;

      System.out.println("type " + s.type);

      if (s.type.equals(CloudShapeTypes.CYLINDER))
      {
         createCylinder(s, color, nodeToAddTo);
      }

      else if (s.type.equals(CloudShapeTypes.PLANE))
      {
         createPlane(s, color, nodeToAddTo);
         //createPolygon(s, color, nodeToAddTo);
      }
      else if (s.type.equals(CloudShapeTypes.SPHERE))
      {
         createSphere(s, color, nodeToAddTo);
      }

   }

   public void createCylinder(Shape s, ColorRGBA color, Node nodeToAddTo)
   {
      if (s.type.equals(CloudShapeTypes.CYLINDER))
      {
         BoundCylinderBasic cyl = new BoundCylinderBasic();

         ((Cylinder3D_F64) s.parameters).line.slope.normalize();

         cyl.bound((Cylinder3D_F64) s.parameters, s.points);

         Point3D_F64 center = ((Cylinder3D_F64) s.parameters).line.p;
         Point3D_F64 a = cyl.getPointA();
         Point3D_F64 b = cyl.getPointB();

         Vector3f start = new Vector3f(new Float(a.x), new Float(a.y), new Float(a.z));
         Vector3f end = new Vector3f(new Float(b.x), new Float(b.y), new Float(b.z));

         nodeToAddTo.attachChild(generateCylinder(start, end, new Float(((Cylinder3D_F64) s.parameters).radius), color));

      }
   }

   public void createSphere(Shape s, ColorRGBA color, Node nodeToAddTo)
   {
      if (s.type.equals(CloudShapeTypes.SPHERE))
      {
         Point3D_F64 center = ((Sphere3D_F64) s.parameters).center;
         Vector3f centerF = new Vector3f(new Float(center.x), new Float(center.y), new Float(center.z));
         float radius = new Float(((Sphere3D_F64) s.parameters).radius);

         nodeToAddTo.attachChild(generateSphere(centerF, radius, color));

      }
   }

   public void createPlane(Shape s, ColorRGBA color, Node nodeToAddTo)
   {
      if (s.type.equals(CloudShapeTypes.PLANE))
      {
         BoundPlaneRectangle plane = new BoundPlaneRectangle();
         plane.process((PlaneGeneral3D_F64) s.parameters, s.points);
         Point3D_F64[] points = plane.getRect();

         if (points.length == 4)
         {
            nodeToAddTo.attachChild(generatePlane(points[0], points[1], points[3], points[2], color));

         }
         else
            System.out.println("NOT RIGHT NUMBER OF POINTS IN PLANE");
      }
   }

   public void createPolygon(PlaneGeneral3D_F64 plane, List<Point3D_F64> points, ColorRGBA color, Node nodeToAddTo)
   {
      // get the concave polygon around and get the resulting triangulation
      BoundConcavePolygon polygon = new BoundConcavePolygon();
      ConcaveHull prep_c = prepareConcaveHull(plane, points, polygon);

      // draw the polygon outer edges
      List<Point3D_F64> points3d = getConcaveHull(prep_c, polygon);

      // get the triangles from the concave hull
      System.out.println(prep_c.triangles.size());
      nodeToAddTo.attachChild(generateCustomPolygon(prep_c.triangles.values(), polygon, color));

      for (int i = 1; i < points3d.size(); i++)
      {
         Vector3f start = new Vector3f((float) points3d.get(i - 1).x, (float) points3d.get(i - 1).y, (float) points3d.get(i - 1).z);
         Vector3f end = new Vector3f((float) points3d.get(i).x, (float) points3d.get(i).y, (float) points3d.get(i).z);
         nodeToAddTo.attachChild(drawLine(start, end));
      }

      //    }
   }

   public void createPolygon(Shape s, ColorRGBA color, Node nodeToAddTo)
   {
      if (s.type.equals(CloudShapeTypes.PLANE))
         createPolygon((PlaneGeneral3D_F64) s.parameters, s.points, color, nodeToAddTo);
   }

   // prepares to build the concave hull
   public ConcaveHull prepareConcaveHull(PlaneGeneral3D_F64 plane, List<Point3D_F64> points3d, BoundConcavePolygon polygon)
   {
     
      polygon.process(plane, points3d);

      List<Point2D_F64> points2d = polygon.getPoints2D();
      Coordinate[] points = new Coordinate[points2d.size()];
      System.out.println(points2d.size() + "total 2d points");

      for (int i = 0; i < points2d.size(); i++)
      {
         Point2D_F64 po = points2d.get(i);
         points[i] = new Coordinate(po.x, po.y);

         // System.out.println(points[i]);
      }

      GeometryCollection geometry = new GeometryFactory().createMultiPoint(points);
      ConcaveHull c = new ConcaveHull(geometry, 0.7);

      return c;

   }

   // mathod to get the concave hull from the concave hull object
   public List<Point3D_F64> getConcaveHull(ConcaveHull c, BoundConcavePolygon polygon)
   {
      List<Point3D_F64> points3d = new ArrayList<Point3D_F64>();

      com.vividsolutions.jts.geom.Geometry concaveHull = c.getConcaveHull();
      System.out.println(concaveHull.toText());
      Coordinate[] boundary = concaveHull.getCoordinates();

      for (int i = 0; i < boundary.length; i++)
      {
         points3d.add(polygon.convertTo3D(boundary[i].x, boundary[i].y, polygon.getCenter()));

      }

      return points3d;
   }

   public Node drawLine(Vector3f start, Vector3f end)
   {
      Node lineNode = new Node();
      Line line = new Line(start, end);
      line.setLineWidth(5);
      Geometry lineGeometry = new Geometry("line", line);

      Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      objectMaterial.setColor("Color", ColorRGBA.White);
      lineGeometry.setMaterial(objectMaterial);
      lineGeometry.setQueueBucket(Bucket.Translucent);

      lineNode.attachChild(lineGeometry);

      return lineNode;
   }

   public Node generatePlane(Point3D_F64 p1, Point3D_F64 p2, Point3D_F64 p3, Point3D_F64 p4, ColorRGBA color)
   {
      Node plane = new Node();
      {
         Vector3f[] vertices = new Vector3f[5];
         vertices[0] = new Vector3f(new Float(p1.x), new Float(p1.y), new Float(p1.z));
         vertices[1] = new Vector3f(new Float(p2.x), new Float(p2.y), new Float(p2.z));
         vertices[2] = new Vector3f(new Float(p3.x), new Float(p3.y), new Float(p3.z));
         vertices[3] = new Vector3f(new Float(p4.x), new Float(p4.y), new Float(p4.z));
         Vector2f[] texCoord = new Vector2f[4];

         texCoord[0] = new Vector2f(0, 0);
         texCoord[1] = new Vector2f(1, 0);
         texCoord[2] = new Vector2f(0, 1);
         texCoord[3] = new Vector2f(1, 1);

         int[] indexes = { 2, 0, 1, 1, 3, 2 };

         Mesh plane1 = new Mesh();

         plane1.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
         plane1.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
         plane1.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indexes));
         plane1.updateBound();

         Geometry geo = new Geometry("OurMesh", plane1); // using our custom mesh object
         Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
         objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
         objectMaterial.setColor("Color", color);
         geo.setMaterial(objectMaterial);
         geo.setQueueBucket(Bucket.Transparent);
         geo.setShadowMode(ShadowMode.CastAndReceive);

         plane.attachChild(geo);

      }

      {
         Vector3f[] vertices = new Vector3f[4];
         vertices[0] = new Vector3f(new Float(p1.x), new Float(p1.y), new Float(p1.z));
         vertices[1] = new Vector3f(new Float(p2.x), new Float(p2.y), new Float(p2.z));
         vertices[2] = new Vector3f(new Float(p3.x), new Float(p3.y), new Float(p3.z));
         vertices[3] = new Vector3f(new Float(p4.x), new Float(p4.y), new Float(p4.z));

         Vector2f[] texCoord = new Vector2f[4];

         texCoord[0] = new Vector2f(0, 0);
         texCoord[1] = new Vector2f(1, 0);
         texCoord[2] = new Vector2f(0, 1);
         texCoord[3] = new Vector2f(1, 1);

         int[] indexes = { 2, 3, 1, 1, 0, 2 };

         Mesh plane1 = new Mesh();

         plane1.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
         plane1.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
         plane1.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indexes));
         plane1.updateBound();

         Geometry geo = new Geometry("OurMesh", plane1); // using our custom mesh object
         Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
         objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
         objectMaterial.setColor("Color", color);
         geo.setMaterial(objectMaterial);
         geo.setQueueBucket(Bucket.Transparent);
         geo.setShadowMode(ShadowMode.CastAndReceive);

         plane.attachChild(geo);

      }

      plane.setShadowMode(ShadowMode.CastAndReceive);

      return plane;

   }

   // generate a custom polygon mesh

   public Node generateCustomPolygon(Collection<Triangle> triangles, BoundConcavePolygon polygon, ColorRGBA color)
   {
      Node plane = new Node();
      {
         HashMap<Integer, Point3D_F64> verts = new HashMap<Integer, Point3D_F64>();
         HashMap<Integer, Vertex> triVertex = new HashMap<Integer, Vertex>();
         int[] indexes = new int[triangles.size() * 3];
         int index = 0;
         Vertex ov, ev;
         for (Triangle triangle : triangles)
         {
            List<Edge> edges = triangle.getEdges();
            for (Edge e : edges)
            {
               ov = e.getOV();
               verts.put(Integer.valueOf(ov.getId()), polygon.convertTo3D(ov.getCoordinate().x, ov.getCoordinate().y, polygon.getCenter()));
               triVertex.put(Integer.valueOf(ov.getId()), ov);
               ev = e.getEV();
               verts.put(Integer.valueOf(ev.getId()), polygon.convertTo3D(ov.getCoordinate().x, ov.getCoordinate().y, polygon.getCenter()));
               triVertex.put(Integer.valueOf(ev.getId()), ev);
            }

            for (Integer i : triVertex.keySet())
            {
               indexes[index++] = i.intValue();

            }
            triVertex.clear();
         }
         
         System.out.println("Index? " + index + " of " + indexes.length);

         Vector3f[] vertices = new Vector3f[verts.size()];
         for (Integer key : verts.keySet())
         {
            Point3D_F64 p = verts.get(key);
            vertices[key.intValue()] = new Vector3f(new Float(p.x), new Float(p.y), new Float(p.z));

         }

         Vector2f[] texCoord = new Vector2f[4];

         texCoord[0] = new Vector2f(0, 0);
         texCoord[1] = new Vector2f(1, 0);
         texCoord[2] = new Vector2f(0, 1);
         texCoord[3] = new Vector2f(1, 1);

         Mesh plane1 = new Mesh();

         plane1.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
         plane1.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
         plane1.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indexes));
         plane1.updateBound();

         Geometry geo = new Geometry("OurMesh", plane1); // using our custom mesh object
         Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
         objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
         objectMaterial.setColor("Color", color);
         objectMaterial.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
         geo.setMaterial(objectMaterial);
         geo.setQueueBucket(Bucket.Transparent);
         geo.setShadowMode(ShadowMode.CastAndReceive);

         plane.attachChild(geo);

      }
      plane.setShadowMode(ShadowMode.CastAndReceive);

      return plane;

   }

   public Node generateCylinder(Vector3f start, Vector3f end, float radius, ColorRGBA color)
   {
      float length = start.distance(end);
      Cylinder c = new Cylinder(10, 10, radius, length, true);

      Geometry g = new Geometry("cyl", c);
      g.setLocalTranslation(0, 0, length / 2.0f);

      Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      objectMaterial.setColor("Color", color);
      g.setMaterial(objectMaterial);
      g.setQueueBucket(Bucket.Transparent);
      g.setShadowMode(ShadowMode.CastAndReceive);

      Node cylNode = new Node();
      cylNode.attachChild(g);

      cylNode.setLocalTranslation(start);

      cylNode.lookAt(end, Vector3f.UNIT_Z);

      return cylNode;
   }

   public Node generateSphere(Vector3f center, float radius, ColorRGBA color)
   {
      Sphere s = new Sphere(10, 10, radius);

      Geometry g = new Geometry("cyl", s);
      Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      objectMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      objectMaterial.setColor("Color", color);
      g.setMaterial(objectMaterial);
      g.setQueueBucket(Bucket.Transparent);
      g.setShadowMode(ShadowMode.CastAndReceive);
      Node sphereNode = new Node();
      sphereNode.attachChild(g);

      sphereNode.setLocalTranslation(center);

      return sphereNode;
   }

   //   
}
