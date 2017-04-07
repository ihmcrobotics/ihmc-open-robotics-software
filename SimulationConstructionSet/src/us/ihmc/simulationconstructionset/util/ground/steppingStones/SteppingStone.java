package us.ihmc.simulationconstructionset.util.ground.steppingStones;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SteppingStone
{
   private final String name;

   private final double baseZ;
   private final double height;
   private final ConvexPolygon2d convexPolygon2d;
   private final ConvexPolygon2d shrunkenPolygon;

   private final ConvexPolygon2d polygonToShrink;

   public SteppingStone(String name, double baseZ, double height, ConvexPolygon2d convexPolygon2d, ConvexPolygon2d footPolygon)
   {
      this.name = name;
      this.baseZ = baseZ;
      this.height = height;
      this.convexPolygon2d = new ConvexPolygon2d(convexPolygon2d);
      polygonToShrink = new ConvexPolygon2d(footPolygon);
      shrunkenPolygon = ConvexPolygonTools.shrinkInto(polygonToShrink, new Point2D(0.0, 0.0), convexPolygon2d);
   }

   public SteppingStone(String name, double baseZ, double height, ArrayList<Point2D> points, ConvexPolygon2d footPolygon)
   {
      this.name = name;
      this.baseZ = baseZ;
      this.height = height;
      convexPolygon2d = new ConvexPolygon2d(points);
      polygonToShrink = new ConvexPolygon2d(footPolygon);
      shrunkenPolygon = ConvexPolygonTools.shrinkInto(polygonToShrink, new Point2D(0.0, 0.0), convexPolygon2d);
   }

   public String getName()
   {
      return name;
   }

   public double getHeight()
   {
      return height;
   }

   public ConvexPolygon2d getConvexPolygon2d()
   {
      return convexPolygon2d;
   }

   public ConvexPolygon2d getShrunkenConvexPolygon2d()
   {
      return shrunkenPolygon;
   }


   public Graphics3DObject createLinkGraphics(AppearanceDefinition yoAppearance)
   {
      Graphics3DObject ret = new Graphics3DObject();
      double polygonExtrusionHeight = height-baseZ;
      ret.translate(new Vector3D(0.0, 0.0, baseZ));
      ret.addExtrudedPolygon(convexPolygon2d, polygonExtrusionHeight, yoAppearance);
            
      if (this.shrunkenPolygon != null)
      {
         //      ret.translate(new Vector3d(0.0, 0.0, baseZ));
         ret.addExtrudedPolygon(shrunkenPolygon, polygonExtrusionHeight + 0.001, YoAppearance.DarkGreen());
      }
      
      return ret;
   }


   public boolean intersectsLocation(double x, double y)
   {
      if (convexPolygon2d.isPointInside(x, y))
         return true;


      return false;
   }

   public static SteppingStone generateRandomCicularStone(String name, Random random, double xCenter, double yCenter, double baseZ, double height,
           double radius, ConvexPolygon2d shrunkenPolygon)
   {
      ArrayList<Point2D> points = generateRandomCircularPoints(xCenter, yCenter, radius, 40);
      ConvexPolygon2d polygon2d = new ConvexPolygon2d(points);
      SteppingStone steppingStone = new SteppingStone(name, baseZ, height, polygon2d, shrunkenPolygon);

      return steppingStone;
   }

   public static SteppingStone createRectangularStone(String name, double xMin, double xMax, double yMin, double yMax, double baseZ, double height,
           ConvexPolygon2d shrunkenPolygon)
   {
      ArrayList<Point2D> points = generateRectangularPoints(xMin, xMax, yMin, yMax);
      SteppingStone steppingStone = new SteppingStone(name, baseZ, height, points, shrunkenPolygon);

      return steppingStone;
   }


   public static SteppingStone generateRandomPolygonalStone(String name, Random random, double xCenter, double yCenter, double baseZ, double height,
           double radius, int numSides, ConvexPolygon2d shrunkenPolygon)
   {
      ArrayList<Point2D> points = generateRandomPolygonalPoints(random, xCenter, yCenter, radius, numSides);
      SteppingStone steppingStone = new SteppingStone(name, baseZ, height, points, shrunkenPolygon);

      return steppingStone;
   }

   public static SteppingStone generateRegularPolygonalStone(String name, double xCenter, double yCenter, double baseZ, double height, double radius,
           int numSides, ConvexPolygon2d shrunkenPolygon)
   {
      ArrayList<Point2D> points = generateRegularPolygonalPoints(xCenter, yCenter, radius, numSides);
      SteppingStone steppingStone = new SteppingStone(name, baseZ, height, points, shrunkenPolygon);

      return steppingStone;
   }

   private static ArrayList<Point2D> generateRegularPolygonalPoints(double xCenter, double yCenter, double radius, int numberOfPoints)
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      double alpha = (2.0 * Math.PI) / (numberOfPoints);
      Point2D zeroPoint = new Point2D(xCenter, yCenter);
      Point2D firstPoint = new Point2D(xCenter + radius, yCenter);
      points.add(firstPoint);

      for (int i = 1; i < numberOfPoints; i++)
      {
         double x = Math.cos(alpha) * (firstPoint.getX() - zeroPoint.getX()) - Math.sin(alpha) * (firstPoint.getY() - zeroPoint.getY()) + zeroPoint.getX();
         double y = Math.sin(alpha) * (firstPoint.getX() - zeroPoint.getX()) + Math.cos(alpha) * (firstPoint.getY() - zeroPoint.getY()) + zeroPoint.getY();
         Point2D nextPoint = new Point2D(x, y);
         points.add(nextPoint);
         firstPoint = nextPoint;
      }

      return points;
   }

   private static ArrayList<Point2D> generateRandomPolygonalPoints(Random random, double xCenter, double yCenter, double radius, int numberOfPoints)
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      double alpha = (2.0 * Math.PI) / (numberOfPoints);
      Point2D zeroPoint = new Point2D(xCenter, yCenter);
      Point2D firstPoint = new Point2D(xCenter + radius, yCenter);
      points.add(firstPoint);
      double minAngle = 0.0, maxAngle = alpha;

      for (int i = 1; i < numberOfPoints; i++)
      {
         double randonAngle = randomDoubleInRange(random, minAngle, maxAngle);
         double x = Math.cos(randonAngle) * (firstPoint.getX() - zeroPoint.getX()) - Math.sin(randonAngle) * (firstPoint.getY() - zeroPoint.getY())
                    + zeroPoint.getX();
         double y = Math.sin(randonAngle) * (firstPoint.getX() - zeroPoint.getX()) + Math.cos(randonAngle) * (firstPoint.getY() - zeroPoint.getY())
                    + zeroPoint.getY();
         Point2D nextPoint = new Point2D(x, y);
         points.add(nextPoint);
         minAngle = maxAngle;
         maxAngle = maxAngle + alpha;
      }

      return points;
   }

   private static ArrayList<Point2D> generateRandomCircularPoints(double xCenter, double yCenter, double radius, int numberOfPoints)
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();

      Random random = new Random(1972L);

      Point2D zeroFramePoint = new Point2D(xCenter, yCenter);

      double xMin = xCenter - radius;
      double xMax = xCenter + radius;
      double yMin = yCenter - radius;
      double yMax = yCenter + radius;

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point2D randomPoint = FramePoint2d.generateRandomFramePoint2d(random, ReferenceFrame.getWorldFrame(), xMin, xMax, yMin, yMax).getPointCopy();

         if (randomPoint.distance(zeroFramePoint) > radius)
            continue;

         points.add(randomPoint);
      }

      return points;
   }


   private static ArrayList<Point2D> generateRectangularPoints(double xMin, double xMax, double yMin, double yMax)
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      Point2D randomPoint = new Point2D(xMin, yMin);
      points.add(randomPoint);
      randomPoint = new Point2D(xMin, yMax);
      points.add(randomPoint);
      randomPoint = new Point2D(xMax, yMax);
      points.add(randomPoint);
      randomPoint = new Point2D(xMax, yMin);
      points.add(randomPoint);

      return points;
   }

   private static double randomDoubleInRange(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

}
