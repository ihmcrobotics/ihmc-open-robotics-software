package us.ihmc.jMonkeyEngineToolkit.tralala;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import com.jme3.app.SimpleApplication;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Triangle;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryCollectionIterator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.OctagonalEnvelope;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.PrecisionModel;
import com.vividsolutions.jts.geom.util.AffineTransformation;
import com.vividsolutions.jts.geom.util.LinearComponentExtracter;
import com.vividsolutions.jts.simplify.DouglasPeuckerSimplifier;
import com.vividsolutions.jts.triangulate.DelaunayTriangulationBuilder;

public class ShapeUtilities
{
   private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory(new PrecisionModel());
   private static final int BLACK_COLOR_THRESHOLD = 60;
   private static final float PIXEL_DISTANCE  = 1;
   private static final float SIMPLIFY_AMOUNT = 1;

   /** Reads the colors of first column of an image and creates a gradient texture.*/
   public static void testMain(SimpleApplication scene)
   {
      String shape = "shapes/public domain/tribal_star.png";
      float height = 0.05f;

      //Apply image effects.
      BufferedImage shapeImage = ImageUtilities.loadImage(shape, scene.getAssetManager());
      shapeImage = ImageUtilities.symmetrifyX(shapeImage, true, false);

      //Get shape.
      com.jme3.scene.Geometry g = createShape(shapeImage, height);
      g.setMaterial(Utilities.getUnshadedMaterial(scene.getAssetManager().loadTexture("Textures/CobbleStone.png"), null, BlendMode.Off, scene.getAssetManager()));
      g.scale(10f);
      g.rotate(FastMath.HALF_PI*3, 0, 0);

      scene.getRootNode().attachChild(g);
      scene.getViewPort().setBackgroundColor(new ColorRGBA(0.7f, 0.8f, 1f, 1f));
   }

   public static com.jme3.scene.Geometry createShape(BufferedImage shapeImage, float height)
   {
      Geometry geom = createGeometry(shapeImage);
      geom = ShapeUtilities.combineComponents(geom, PIXEL_DISTANCE);
      geom = simplify(geom, SIMPLIFY_AMOUNT);
      geom = scale(geom, 1d / shapeImage.getWidth(), 1d / shapeImage.getHeight());//we scale it to [0,1] so it have correct uv.

      MeshData mesh = createShape(geom, 0.05f);
      Coordinate center = getCenter(geom);
      mesh = mesh.translate(new Vector3f( (float)-center.x, 0, (float)-center.y) );

      com.jme3.scene.Geometry g = new com.jme3.scene.Geometry("", mesh.createMesh());
      return g;
   }
   
   public static MeshData createShape(Geometry geom, float height)
   {
      Geometry polygon = ShapeUtilities.triangulate(geom);
      ArrayList<Triangle> triangles = getTriangles(polygon);
      ArrayList<LineString> lines = getEdges(geom);

      int numOfIndexesInTriangle = 3;
      int p = triangles.size() * numOfIndexesInTriangle;
      int indexSize = 2 * p + lines.size() * 6;
      float[] vertexArray = new float[indexSize * 3];
      float[] normalArray = new float[indexSize * 3];
      float[] uvArray = new float[indexSize * 2];
      short[] indexArray = new short[indexSize];
      Vector3f neg = Vector3f.UNIT_Y.mult(-1f);

      int index = 0;
      for (Triangle t : triangles)
      {
         Vector3f t1 = t.get1();
         Vector3f t2 = t.get2();
         Vector3f t3 = t.get3();

         //top
         addTriangle(new Triangle(t1.add(0, height, 0), t2.add(0, height, 0), t3.add(0, height, 0)), Vector3f.UNIT_Y, index, vertexArray, normalArray, uvArray, indexArray);
         index += 3;

         //bottom
         addTriangle(new Triangle(t3, t2, t1), neg, index, vertexArray, normalArray, uvArray, indexArray);
         index += 3;
      }

      for (int i = 0; i < lines.size(); i++) //periphery
      {
         Point start = lines.get(i).getStartPoint();
         Point end = lines.get(i).getEndPoint();
         Vector3f vStart = new Vector3f((float) start.getX(), 0, (float) start.getY());
         Vector3f vStart1 = vStart.add(0, height, 0);
         Vector3f vEnd = new Vector3f((float) end.getX(), 0, (float) end.getY());
         Vector3f vEnd1 = vEnd.add(0, height, 0);

         Triangle t = new Triangle(vStart1, vStart, vEnd1);
         t.calculateNormal();
         addTriangle(t, t.getNormal(), index, vertexArray, normalArray, uvArray, indexArray);
         index += 3;

         t = new Triangle(vStart, vEnd, vEnd1);
         t.calculateNormal();
         addTriangle(t, t.getNormal(), index, vertexArray, normalArray, uvArray, indexArray);
         index += 3;
      }

      return new MeshData(vertexArray, normalArray, uvArray, indexArray).removeDuplicateData();
   }

   /** Creates a point where there is black color.*/
   public static Geometry createGeometry(BufferedImage image)
   {
      ArrayList<Point> points = new ArrayList<Point>(128 * 128);
      image = ImageUtilities.rotateImage(image, FastMath.PI);
      for (int i = 0; i < image.getWidth(); i++)
      {
         for (int j = 0; j < image.getHeight(); j++)
         {
            Color color = ImageUtilities.getColor(image, i, j, true);
            boolean opaque = (color.getAlpha() > BLACK_COLOR_THRESHOLD);
            boolean black = ((color.getRed() + color.getGreen() + color.getBlue()) / 3 < BLACK_COLOR_THRESHOLD);
            if (opaque && black)
            {
               points.add(createPoint(new Coordinate(i, j)));
            }
         }
      }
      GeometryCollection g = new GeometryCollection(points.toArray(new Point[points.size()]), GEOMETRY_FACTORY);
      return g;
   }

   public static Point createPoint(Coordinate c)
   {
      return GEOMETRY_FACTORY.createPoint(c);
   }

   /** Combines Geometry. Points are converted into circle with size "pointSize" and then combined. */
   public static Geometry combineComponents(Geometry g, double pointSize)
   {
      return componentBuffers(g, pointSize).union();
   }

   public static Geometry simplify(Geometry g, float amount)
   {
      return DouglasPeuckerSimplifier.simplify(g, amount);
   }

   /** Converts it into a convexHull shape.*/
   public static Geometry convertToConvexHull(Geometry g)
   {
      return g.convexHull();
   }

   /** Converts it into a octagonal shape.*/
   public static Geometry convertToOctagonal(Geometry g)
   {
      OctagonalEnvelope octEnv = new OctagonalEnvelope(g);
      return octEnv.toGeometry(g.getFactory());
   }

   public static Geometry intersection(Geometry a, Geometry b)
   {
      return a.intersection(b);
   }

   public static Geometry union(Geometry a, Geometry b)
   {
      return a.union(b);
   }

   public static Geometry symDifference(Geometry a, Geometry b)
   {
      return a.symDifference(b);
   }

   public static Geometry difference(Geometry a, Geometry b)
   {
      return a.difference(b);
   }

   public static Geometry triangulate(Geometry geom)
   {
      DelaunayTriangulationBuilder builder = new DelaunayTriangulationBuilder();
      builder.setSites(geom);
      builder.setTolerance(0);
      Geometry tris = builder.getTriangles(geom.getFactory());
      return intersection(tris, geom);
   }

   public static Coordinate getCenter(Geometry g)
   {
      return g.getEnvelopeInternal().centre();
   }

   public static Geometry translateCentreToOrigin(Geometry g)
   {
      Coordinate centre = getCenter(g);
      AffineTransformation trans = AffineTransformation.translationInstance(-centre.x, -centre.y);
      return trans.transform(g);
   }

   public static Geometry scale(Geometry g, double scaleX, double scaleY)
   {
      AffineTransformation trans = AffineTransformation.scaleInstance(scaleX, scaleY);
      return trans.transform(g);
   }

   public static Geometry reflectInX(Geometry g)
   {
      Coordinate centre = getCenter(g);
      AffineTransformation trans = AffineTransformation.scaleInstance(1, -1, centre.x, centre.y);
      return trans.transform(g);
   }

   public static Geometry reflectInY(Geometry g)
   {
      Coordinate centre = getCenter(g);
      AffineTransformation trans = AffineTransformation.scaleInstance(-1, 1, centre.x, centre.y);
      return trans.transform(g);
   }

   public static ArrayList<Triangle> getTriangles(Geometry geom)
   {
      ArrayList<Triangle> triangles = new ArrayList<Triangle>();

      LinkedList<Geometry> gc = new LinkedList<Geometry>();
      gc.add(geom);

      while (!gc.isEmpty())
      {
         Geometry g = gc.pop();

         if (g instanceof GeometryCollection)
         {
            GeometryCollection collection = (GeometryCollection) g;
            for (int i = 0; i < collection.getNumGeometries(); i++) gc.add(collection.getGeometryN(i));
         }
         else if (g instanceof Polygon)
         {
            Polygon p = (Polygon) g;
            Coordinate[] cords = p.getCoordinates();
            triangles.add(new Triangle(toVector3f(cords[0]), toVector3f(cords[1]), toVector3f(cords[2])));

            if (g.getNumPoints() == 5) //when triangulation failed.
            {
               triangles.add(new Triangle(toVector3f(cords[2]), toVector3f(cords[3]), toVector3f(cords[0])));
               if (!cords[4].equals2D(cords[0])) triangles.add(new Triangle(toVector3f(cords[3]), toVector3f(cords[4]), toVector3f(cords[0])));
            }
            if (g.getNumPoints() == 6)
            {
               triangles.add(new Triangle(toVector3f(cords[2]), toVector3f(cords[3]), toVector3f(cords[4])));
               triangles.add(new Triangle(toVector3f(cords[4]), toVector3f(cords[5]), toVector3f(cords[2])));
               if (!cords[5].equals2D(cords[0])) triangles.add(new Triangle(toVector3f(cords[5]), toVector3f(cords[0]), toVector3f(cords[2])));
            }
         }
      }
      return triangles;
   }

   public static Vector3f toVector3f(Coordinate c)
   {
      return new Vector3f((float) c.x, 0, (float) c.y);
   }

   public static Vector3f toVector3f(Point c)
   {
      return toVector3f(c.getCoordinate());
   }

   /** Extracts all edges of target geometry.*/
   public static ArrayList<LineString> getEdges(Geometry geom)
   {
      @SuppressWarnings("unchecked")
      List<LineString> lines = LinearComponentExtracter.getLines(geom);
      ArrayList<LineString> segments = new ArrayList<LineString>(lines.size());
      for (Iterator<LineString> it = lines.iterator(); it.hasNext();)
      {
         LineString line = it.next();
         for (int i = 1; i < line.getNumPoints(); i++)
         {
            LineString seg = GEOMETRY_FACTORY.createLineString(
                    new Coordinate[]
                    {
                       line.getCoordinateN(i - 1), line.getCoordinateN(i)
                    });
            segments.add(seg);
         }
      }
      return segments;
   }

   private static GeometryCollection componentBuffers(Geometry g, double distance)
   {
      List<Geometry> bufs = new ArrayList<Geometry>();
      for (Iterator it = new GeometryCollectionIterator(g); it.hasNext();)
      {
         Geometry comp = (Geometry) it.next();
         if (comp instanceof GeometryCollection) continue;
         bufs.add(comp.buffer(distance));
      }
      return GEOMETRY_FACTORY.createGeometryCollection(GeometryFactory.toGeometryArray(bufs));
   }

   private static void addTriangle(Triangle t, Vector3f normal, int index, float[] vertexArray, float[] normalArray, float[] uvArray, short[] indexArray)
   {
      addCord(t.get1(), normal, index, vertexArray, normalArray, uvArray, indexArray);
      addCord(t.get2(), normal, index + 1, vertexArray, normalArray, uvArray, indexArray);
      addCord(t.get3(), normal, index + 2, vertexArray, normalArray, uvArray, indexArray);
   }

   private static void addCord(Vector3f triangle, Vector3f normal, int index, float[] vertexArray, float[] normalArray, float[] uvArray, short[] indexArray)
   {
      indexArray[index] = (short) index;
      Utilities.setInArray(triangle, vertexArray, index);
      Utilities.setInArray(normal, normalArray, index);
      Utilities.setInArray(new Vector2f(triangle.getX(), triangle.getZ()), uvArray, index);
   }

   public static void main(String[] args)
   {
      SimpleApplication scene = new SimpleApplication()
      {
         public void simpleInitApp()
         {
            testMain(this);
         }
      };
      scene.start();
   }
}
