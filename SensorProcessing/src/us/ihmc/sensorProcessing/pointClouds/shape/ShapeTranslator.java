package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import bubo.ptcloud.CloudShapeTypes;
import bubo.ptcloud.PointCloudShapeFinder.Shape;
import bubo.ptcloud.alg.BoundCylinderBasic;
import bubo.ptcloud.alg.BoundPlaneRectangle;

import com.jme3.app.SimpleApplication;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;
import com.jme3.util.BufferUtils;

public class ShapeTranslator
{
   SimpleApplication ui;

   public ShapeTranslator(SimpleApplication ui)
   {
      this.ui = ui;
   }

   public void translateShape(Shape s, ColorRGBA color, Node nodeToAddTo)
   {
      System.out.println("type " + s.type);

      if (s.type.equals(CloudShapeTypes.CYLINDER))
      {
         createCylinder(s, color, nodeToAddTo);
      }

      else if (s.type.equals(CloudShapeTypes.PLANE))
      {
         createPlane(s, color, nodeToAddTo);
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

         System.out.println(points.length);

         if (points.length == 4)
         {
            System.out.println(points[0] + " " + points[1] + " " + points[2] + " " + points[3]);
            nodeToAddTo.attachChild(generatePlane(points[0], points[1], points[3], points[2], color));

         }
         else
            System.out.println("NOT RIGHT NUMBER OF POINTS IN PLANE");
      }
   }

   private Node generatePlane(Point3D_F64 p1, Point3D_F64 p2, Point3D_F64 p3, Point3D_F64 p4, ColorRGBA color)
   {
      Node plane = new Node();
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

         int[] indexes =
         {
            2, 0, 1, 1, 3, 2
         };

         Mesh plane1 = new Mesh();

         plane1.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
         plane1.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
         plane1.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indexes));
         plane1.updateBound();

         Geometry geo = new Geometry("OurMesh", plane1);    // using our custom mesh object
         Material mat = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
         mat.setColor("Color", color);
         geo.setMaterial(mat);
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

         int[] indexes =
         {
            2, 3, 1, 1, 0, 2
         };

         Mesh plane1 = new Mesh();

         plane1.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
         plane1.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
         plane1.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indexes));
         plane1.updateBound();

         Geometry geo = new Geometry("OurMesh", plane1);    // using our custom mesh object
         Material mat = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
         mat.setColor("Color", color);
         geo.setMaterial(mat);
         geo.setShadowMode(ShadowMode.CastAndReceive);

         plane.attachChild(geo);

      }


      Material objectMaterial = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      objectMaterial.setColor("Color", color);
      plane.setShadowMode(ShadowMode.CastAndReceive);
      plane.setCullHint(CullHint.Dynamic);
      plane.setMaterial(objectMaterial);
      plane.setQueueBucket(Bucket.Opaque);

      return plane;

   }

   public Node generateCylinder(Vector3f start, Vector3f end, float radius, ColorRGBA color)
   {
      float length = start.distance(end);
      Cylinder c = new Cylinder(10, 10, radius, length, true);
      Material mat = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      mat.setColor("Color", color);

      Geometry g = new Geometry("cyl", c);
      g.setLocalTranslation(0, 0, length / 2.0f);
      Node cylNode = new Node();
      cylNode.attachChild(g);
      cylNode.setMaterial(mat);

      cylNode.setLocalTranslation(start);

      cylNode.lookAt(end, Vector3f.UNIT_Z);

      return cylNode;
   }

   public Node generateSphere(Vector3f center, float radius, ColorRGBA color)
   {
      Sphere s = new Sphere(10, 10, radius);
      Material mat = new Material(ui.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      mat.setColor("Color", color);

      Geometry g = new Geometry("cyl", s);
      Node sphereNode = new Node();
      sphereNode.attachChild(g);
      sphereNode.setMaterial(mat);

      sphereNode.setLocalTranslation(center);

      return sphereNode;
   }


//   
}
