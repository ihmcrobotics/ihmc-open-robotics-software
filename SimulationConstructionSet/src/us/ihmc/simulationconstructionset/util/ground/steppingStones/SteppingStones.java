package us.ihmc.simulationconstructionset.util.ground.steppingStones;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */

public class SteppingStones
{
   private final ArrayList<SteppingStone> steppingStones = new ArrayList<SteppingStone>();

   public SteppingStones()
   {
   }

   public void addSteppingStone(SteppingStone steppingStone)
   {
      steppingStones.add(steppingStone);
   }

   public SteppingStone getSteppingStone(int i)
   {
      return steppingStones.get(i);
   }

   public SteppingStone getSteppingStone(String name)
   {
      SteppingStone ret = null;
      for (SteppingStone steppingStone : steppingStones)
      {
         if (steppingStone.getName().equals(name))
            ret = steppingStone;
      }

      return ret;
   }



   public ArrayList<ConvexPolygon2d> getConvexPolygons()
   {
      ArrayList<ConvexPolygon2d> ret = new ArrayList<ConvexPolygon2d>();

      for (SteppingStone steppingStone : steppingStones)
      {
         ret.add(steppingStone.getConvexPolygon2d());
      }

      return ret;
   }

   public ArrayList<ConvexPolygon2d> getShrunkenConvexPolygons()
   {
      ArrayList<ConvexPolygon2d> ret = new ArrayList<ConvexPolygon2d>();

      for (SteppingStone steppingStone : steppingStones)
      {
         ret.add(steppingStone.getShrunkenConvexPolygon2d());
      }

      return ret;
   }


   public double[][][] getConvexPolygonVertices()
   {
      double[][][] ret = new double[steppingStones.size()][][];

      for (int i = 0; i < steppingStones.size(); i++)
      {
         SteppingStone steppingStone = steppingStones.get(i);

         ConvexPolygon2d convexPolygon2d = steppingStone.getConvexPolygon2d();

         ret[i] = new double[convexPolygon2d.getNumberOfVertices()][];

         for (int j = 0; j < convexPolygon2d.getNumberOfVertices(); j++)
         {
            Point2DReadOnly point2d = convexPolygon2d.getVertex(j);

            ret[i][j] = new double[] {point2d.getX(), point2d.getY()};
         }
      }

      return ret;
   }

   public double[][][] getShrunkenConvexPolygonVertices()
   {
      double[][][] ret = new double[steppingStones.size()][][];

      for (int i = 0; i < steppingStones.size(); i++)
      {
         SteppingStone steppingStone = steppingStones.get(i);

         ConvexPolygon2d shrunkenConvexPolygon2d = steppingStone.getShrunkenConvexPolygon2d();

         ret[i] = new double[shrunkenConvexPolygon2d.getNumberOfVertices()][];

         for (int j = 0; j < shrunkenConvexPolygon2d.getNumberOfVertices(); j++)
         {
            Point2DReadOnly point2d = shrunkenConvexPolygon2d.getVertex(j);

            ret[i][j] = new double[] {point2d.getX(), point2d.getY()};
         }
      }

      return ret;
   }




   public ArrayList<Graphics3DObject>  createLinkGraphics()
   {
      ArrayList<Graphics3DObject> linkGraphicsArray = new ArrayList<Graphics3DObject>();
      
      for (SteppingStone steppingStone : steppingStones)
      {
         Graphics3DObject linkGraphics = steppingStone.createLinkGraphics(YoAppearance.Red()); //YoAppearance.Black());
//       LinkGraphics linkGraphics = steppingStone.createLinkGraphics(YoAppearance.Red());
         
         linkGraphicsArray.add(linkGraphics);
      }

      return linkGraphicsArray;
   }

   public static SteppingStones generateRandomSteppingStones(Random random, int numStones, ConvexPolygon2d polygonToShrink)
   {
      double xMin = -2.0;
      double xMax = 2.0;
      double yMin = -2.0;
      double yMax = 2.0;

      double baseZ = -0.2;
      double zMin = -0.2;
      double zMax = 0.0;
      double minRadius = 0.2;
      double maxRadius = 0.3;

      return generateRandomSteppingStones(random, xMin, xMax, yMin, yMax, baseZ, zMin, zMax, minRadius, maxRadius, numStones, polygonToShrink);
   }

   public static SteppingStones generateRandomSteppingStones(Random random, double xMin, double xMax, double yMin, double yMax, double baseZ, double minHeight,
           double maxHeight, double minRadius, double maxRadius, int numStones, ConvexPolygon2d polygonToShrink)
   {
      SteppingStones ret = new SteppingStones();

      for (int i = 0; i < numStones; i++)
      {
         double xCenter = randomDoubleInRange(random, xMin, xMax);
         double yCenter = randomDoubleInRange(random, yMin, yMax);
         double height = randomDoubleInRange(random, minHeight, maxHeight);
         double radius = randomDoubleInRange(random, minRadius, maxRadius);

         SteppingStone steppingStone = SteppingStone.generateRandomCicularStone("stone" + i, random, xCenter, yCenter, baseZ, height, radius, polygonToShrink);    // class new SteppingStone("stone" + i , zCenter, points);
         ret.addSteppingStone(steppingStone);

      }

      return ret;
   }

   public static SteppingStones generateRectangularCheckeredStripSteppingStones(double startXPosition, double startYPosition, double stoneXDimension,
           double stoneYDimension, double spacingInX, double spacingInY, double baseZ, double height, int numStones, ConvexPolygon2d polygonToShrink,
           boolean variableHeight)
   {
      double minX, maxX, minY, maxY;
      int count = 0;
      SteppingStones ret = new SteppingStones();
      Random random = new Random(1972L);

      while (count < numStones)
      {
         minX = startXPosition - stoneXDimension / 2.0;
         maxX = startXPosition + stoneXDimension / 2.0;
         minY = startYPosition - stoneYDimension / 2.0;
         maxY = startYPosition + stoneYDimension / 2.0;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         SteppingStone steppingStone = SteppingStone.createRectangularStone("stone" + count, minX, maxX, minY, maxY, baseZ, height, polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;

         minX = startXPosition - stoneXDimension / 2.0;
         maxX = startXPosition + stoneXDimension / 2.0;
         minY = startYPosition + 2.0 * spacingInY + 2.0 * stoneYDimension - stoneYDimension / 2.0;
         maxY = startYPosition + 2.0 * spacingInY + 2.0 * stoneYDimension + stoneYDimension / 2.0;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         steppingStone = SteppingStone.createRectangularStone("stone" + count, minX, maxX, minY, maxY, baseZ, height, polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;

         minX = startXPosition + 2.0 * spacingInX + 2.0 * stoneXDimension - stoneXDimension / 2.0;
         maxX = startXPosition + 2.0 * spacingInX + 2.0 * stoneXDimension + stoneXDimension / 2.0;
         minY = startYPosition - stoneYDimension / 2.0;
         maxY = startYPosition + stoneYDimension / 2.0;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         steppingStone = SteppingStone.createRectangularStone("stone" + count, minX, maxX, minY, maxY, baseZ, height, polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;
         startXPosition = startXPosition + spacingInX + stoneXDimension;
         startYPosition = startYPosition + spacingInY + stoneYDimension;
      }

      return ret;
   }

   public static SteppingStones generateRectangularChessBoardSteppingStones(double startXPosition, double startYPosition, double stoneXDimension,
           double stoneYDimension, double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns,
           ConvexPolygon2d polygonToShrink, boolean variableHeight)
   {
      double minX, maxX, minY, maxY;
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      SteppingStones ret = new SteppingStones();
      Random random = new Random(1972L);

      for (int i = 0; i < numRows; i++)
      {
         if (i % 2 == 0)
         {
            startXPosition = x;
            startYPosition = y;
            y = startYPosition + 2.0 * spacingInY + 2.0 * stoneYDimension;
            if (numColumns % 2 != 0)
               columns = numColumns + 1;
         }
         else
         {
            startXPosition = x + spacingInX + stoneXDimension;
            startYPosition = startYPosition + spacingInY + stoneYDimension;
            if (numColumns % 2 != 0)
               columns = numColumns - 1;
         }

         for (int j = 0; j < columns / 2; j++)
         {
            minX = startXPosition - stoneXDimension / 2.0;
            maxX = startXPosition + stoneXDimension / 2.0;
            minY = startYPosition - stoneYDimension / 2.0;
            maxY = startYPosition + stoneYDimension / 2.0;

            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            SteppingStone steppingStone = SteppingStone.createRectangularStone("stone" + i + j, minX, maxX, minY, maxY, baseZ, height, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            startXPosition = startXPosition + 2.0 * spacingInX + 2.0 * stoneXDimension;
         }
      }

      return ret;
   }


   public static SteppingStones generateRectangularUniformSteppingStones(double startXPosition, double startYPosition, double stoneXDimension,
           double stoneYDimension, double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns,
           ConvexPolygon2d polygonToShrink, boolean variableHeight)
   {
      double minX, maxX, minY, maxY;
      int rows = 2 * numRows;
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      SteppingStones ret = new SteppingStones();
      Random random = new Random(1972L);

      for (int i = 0; i < rows; i++)
      {
         if (i % 2 == 0)
         {
            startXPosition = x;
            startYPosition = y;
            y = startYPosition + spacingInY + stoneYDimension;

            for (int j = 0; j < columns; j++)
            {
               minX = startXPosition - stoneXDimension / 2.0;
               maxX = startXPosition + stoneXDimension / 2.0;
               minY = startYPosition - stoneYDimension / 2.0;
               maxY = startYPosition + stoneYDimension / 2.0;

               if (variableHeight)
               {
                  height = randomDoubleInRange(random, 0.0, 0.2);
               }

               SteppingStone steppingStone = SteppingStone.createRectangularStone("stone" + i + j, minX, maxX, minY, maxY, baseZ, height, polygonToShrink);
               ret.addSteppingStone(steppingStone);

               startXPosition = startXPosition + spacingInX + stoneXDimension;
            }
         }
      }

      return ret;
   }

   public static SteppingStones generateRectangularBeamBalance(double startXPosition, double startYPosition, double stoneXDimension, double stoneYDimension,
           double spacingInX, double spacingInYSmall, double spacingInYLarge, double baseZ, double height, int numRows, int numColumns,
           ConvexPolygon2d polygonToShrink)
   {
      double minX, maxX, minY, maxY;
      int rows = 2 * numRows;
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      SteppingStones ret = new SteppingStones();

      for (int i = 0; i < rows; i++)
      {
         if (i % 2 == 0)
         {
            if (i % 4 == 0)
            {
               startXPosition = x;
               startYPosition = y;
               y = startYPosition + spacingInYLarge + stoneYDimension;
            }
            else
            {
               startXPosition = x;
               startYPosition = y;
               y = startYPosition + spacingInYSmall + stoneYDimension;
            }

            for (int j = 0; j < columns; j++)
            {
               minX = startXPosition - stoneXDimension / 2.0;
               maxX = startXPosition + stoneXDimension / 2.0;
               minY = startYPosition - stoneYDimension / 2.0;
               maxY = startYPosition + stoneYDimension / 2.0;
               SteppingStone steppingStone = SteppingStone.createRectangularStone("stone" + i + j, minX, maxX, minY, maxY, baseZ, height, polygonToShrink);
               ret.addSteppingStone(steppingStone);

               startXPosition = startXPosition + spacingInX + stoneXDimension;
            }
         }
      }

      return ret;
   }

   public static SteppingStones generateRectangularCrissCrossBeams(double startXPosition, double startYPosition, double stoneXDimension,
           double stoneYDimension, double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns,
           ConvexPolygon2d polygonToShrink, boolean variableHeight)
   {
      SteppingStones ret = new SteppingStones();
      ret = SteppingStones.generateRectangularUniformSteppingStones(startXPosition, startYPosition, stoneXDimension, stoneYDimension, spacingInX, spacingInY,
              baseZ, height, numRows, numColumns, polygonToShrink, false);

      ArrayList<Point2D> points0 = new ArrayList<Point2D>();
      points0.add(new Point2D(0.0, -0.3));
      points0.add(new Point2D(0.0, -0.1));
      points0.add(new Point2D(12.0, -0.1));
      points0.add(new Point2D(12.0, -0.3));

      SteppingStone steppingStone = new SteppingStone("stone0", -0.1, 0.0, points0, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points1 = new ArrayList<Point2D>();
      points1.add(new Point2D(1.1, -3.1));
      points1.add(new Point2D(0.9, -2.9));
      points1.add(new Point2D(4.9, 3.1));
      points1.add(new Point2D(5.1, 2.9));

      steppingStone = new SteppingStone("stone1", -0.1, 0.0, points1, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points2 = new ArrayList<Point2D>();
      points2.add(new Point2D(0.0, 1.9));
      points2.add(new Point2D(0.0, 2.1));
      points2.add(new Point2D(10.0, 2.1));
      points2.add(new Point2D(10.0, 1.9));

      steppingStone = new SteppingStone("stone2", -0.1, 0.0, points2, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points3 = new ArrayList<Point2D>();
      points3.add(new Point2D(0.9, 0.9));
      points3.add(new Point2D(1.1, 1.1));
      points3.add(new Point2D(12.1, -2.4));
      points3.add(new Point2D(11.9, -2.6));

      steppingStone = new SteppingStone("stone3", -0.1, 0.0, points3, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points4 = new ArrayList<Point2D>();
      points4.add(new Point2D(5.9, 2.4));
      points4.add(new Point2D(6.1, 2.6));
      points4.add(new Point2D(12.0, -3.9));
      points4.add(new Point2D(11.8, -4.1));

      steppingStone = new SteppingStone("stone4", -0.1, 0.0, points4, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points5 = new ArrayList<Point2D>();
      points5.add(new Point2D(0.0, -2.5));
      points5.add(new Point2D(0.0, -2.3));
      points5.add(new Point2D(10.3, -2.3));
      points5.add(new Point2D(10.3, -2.5));

      steppingStone = new SteppingStone("stone5", -0.1, 0.0, points5, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points6 = new ArrayList<Point2D>();
      points6.add(new Point2D(0.9, -1.6));
      points6.add(new Point2D(1.1, -1.4));
      points6.add(new Point2D(12.1, -3.4));
      points6.add(new Point2D(11.9, -3.6));

      steppingStone = new SteppingStone("stone6", -0.1, 0.0, points6, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points7 = new ArrayList<Point2D>();
      points7.add(new Point2D(3.1, -3.1));
      points7.add(new Point2D(2.9, -2.9));
      points7.add(new Point2D(6.4, 1.1));
      points7.add(new Point2D(6.6, 0.9));

      steppingStone = new SteppingStone("stone7", -0.1, 0.0, points7, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points8 = new ArrayList<Point2D>();
      points8.add(new Point2D(11.0, -1.95));
      points8.add(new Point2D(11.0, -1.75));
      points8.add(new Point2D(12.5, -1.8));
      points8.add(new Point2D(12.5, -2.0));

      steppingStone = new SteppingStone("stone8", -0.1, 0.0, points8, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points9 = new ArrayList<Point2D>();
      points9.add(new Point2D(11.0, -1.55));
      points9.add(new Point2D(11.0, -1.35));
      points9.add(new Point2D(13.0, -1.35));
      points9.add(new Point2D(13.0, -1.55));

      steppingStone = new SteppingStone("stone9", -0.1, 0.0, points9, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points10 = new ArrayList<Point2D>();
      points10.add(new Point2D(10.5, -1.15));
      points10.add(new Point2D(10.5, -0.95));
      points10.add(new Point2D(12.5, -0.9));
      points10.add(new Point2D(12.5, -1.1));

      steppingStone = new SteppingStone("stone10", -0.1, 0.0, points10, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      ArrayList<Point2D> points11 = new ArrayList<Point2D>();
      points11.add(new Point2D(9.75, -0.25));
      points11.add(new Point2D(9.75, -0.05));
      points11.add(new Point2D(13.25, -0.7));
      points11.add(new Point2D(13.25, -0.9));

      steppingStone = new SteppingStone("stone11", -0.1, 0.0, points11, polygonToShrink);
      ret.addSteppingStone(steppingStone);

      return ret;
   }


   public static SteppingStones generateRegularPolygonalCheckeredStripSteppingStones(double startXPosition, double startYPosition, double radius, int numSides,
           double spacingInX, double spacingInY, double baseZ, double height, int numStones, ConvexPolygon2d polygonToShrink, boolean variableHeight)
   {
      Random random = new Random(1972L);
      double alpha = (2.0 * Math.PI) / (numSides);
      @SuppressWarnings("unused")
      double xCenter, yCenter, distance = 0.0, xbuffer = 0.0, ybuffer = 0.0;
      int count = 0;
      double xDistance = 0.0, yDistance = 0.0;
      SteppingStones ret = new SteppingStones();
      SteppingStone steppingStoneInitial = SteppingStone.generateRegularPolygonalStone("stone", startXPosition, startYPosition, baseZ, 0.0, radius, numSides,
                                              polygonToShrink);

      if (numSides % 2 == 0)
         if (numSides % 4 == 0)
         {
            yDistance = 2.0 * radius;
            xDistance = 2.0 * radius;
         }
         else
         {
            yDistance = 2.0 * radius * Math.cos(alpha / 2.0);
            xDistance = 2.0 * radius;
         }
      else
      {
         xDistance = radius + radius * Math.cos(alpha / 2.0);
         ConvexPolygon2d convexPolygon2d = steppingStoneInitial.getConvexPolygon2d();

         for (int k = ((numSides + 1) / 2) - 1, j = (numSides + 1) / 2; k > 0; k--, j++)
         {
            double dis = convexPolygon2d.getVertex(k).distance(convexPolygon2d.getVertex(j));
            if (dis >= yDistance)
            {
               yDistance = dis;
            }
         }
      }


      while (count < numStones)
      {
         xCenter = startXPosition;
         yCenter = startYPosition;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         SteppingStone steppingStone = SteppingStone.generateRegularPolygonalStone("stone" + count, xCenter, yCenter, baseZ, height, radius, numSides,
                                          polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;

         xCenter = startXPosition;
         yCenter = startYPosition + spacingInY + yDistance;
         ybuffer = yCenter;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         steppingStone = SteppingStone.generateRegularPolygonalStone("stone" + count, xCenter, yCenter, baseZ, height, radius, numSides, polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;

         xCenter = startXPosition + spacingInX + xDistance;
         yCenter = startYPosition;
         xbuffer = xCenter;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         steppingStone = SteppingStone.generateRegularPolygonalStone("stone" + count, xCenter, yCenter, baseZ, height, radius, numSides, polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;
         startXPosition = xbuffer;
         startYPosition = ybuffer;
      }

      return ret;
   }

   public static SteppingStones generateRegularPolygonalChessBoardSteppingStones(double startXPosition, double startYPosition, double radius, int numSides,
           double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns, ConvexPolygon2d polygonToShrink,
           boolean variableHeight)
   {
      Random random = new Random(1972L);
      double alpha = (2.0 * Math.PI) / (numSides);
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      double xDistance = 0.0, yDistance = 0.0;
      SteppingStones ret = new SteppingStones();
      SteppingStone steppingStoneInitial = SteppingStone.generateRegularPolygonalStone("stone", startXPosition, startYPosition, baseZ, 0.0, radius, numSides,
                                              polygonToShrink);
      if (numSides % 2 == 0)
         if (numSides % 4 == 0)
         {
            yDistance = 2.0 * radius;
            xDistance = 2.0 * radius;
         }
         else
         {
            yDistance = 2.0 * radius * Math.cos(alpha / 2.0);
            xDistance = 2.0 * radius;
         }
      else
      {
         xDistance = radius + radius * Math.cos(alpha / 2.0);
         ConvexPolygon2d convexPolygon2d = steppingStoneInitial.getConvexPolygon2d();

         for (int k = ((numSides + 1) / 2) - 1, j = (numSides + 1) / 2; k > 0; k--, j++)
         {
            double dis = convexPolygon2d.getVertex(k).distance(convexPolygon2d.getVertex(j));
            if (dis >= yDistance)
            {
               yDistance = dis;
            }
         }
      }

      for (int i = 0; i < numRows; i++)
      {
         if (i % 2 == 0)
         {
            startXPosition = x;
            startYPosition = y;
            y = startYPosition + 2.0 * spacingInY + 2.0 * yDistance;
            if (numColumns % 2 != 0)
               columns = numColumns + 1;
         }
         else
         {
            startXPosition = x + spacingInX + xDistance;
            startYPosition = startYPosition + spacingInY + yDistance;
            if (numColumns % 2 != 0)
               columns = numColumns - 1;
         }

         for (int j = 0; j < columns / 2; j++)
         {
            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            SteppingStone steppingStone = SteppingStone.generateRegularPolygonalStone("stone" + i + j, startXPosition, startYPosition, baseZ, height, radius,
                                             numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);
            startXPosition = startXPosition + 2.0 * spacingInX + 2.0 * xDistance;
         }
      }

      return ret;
   }

   public static SteppingStones generateRegularPolygonalUniformSteppingStones(double startXPosition, double startYPosition, double radius, int numSides,
           double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns, ConvexPolygon2d polygonToShrink,
           boolean variableHeight)
   {
      Random random = new Random(1972L);
      double alpha = (2.0 * Math.PI) / (numSides);
      int rows = 2 * numRows;
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      double xDistance = 0.0, yDistance = 0.0;
      SteppingStones ret = new SteppingStones();
      SteppingStone steppingStoneInitial = SteppingStone.generateRegularPolygonalStone("stone", startXPosition, startYPosition, baseZ, 0.0, radius, numSides,
                                              polygonToShrink);
      if (numSides % 2 == 0)
         if (numSides % 4 == 0)
         {
            yDistance = 2.0 * radius;
            xDistance = 2.0 * radius;
         }
         else
         {
            yDistance = 2.0 * radius * Math.cos(alpha / 2.0);
            xDistance = 2.0 * radius;
         }
      else
      {
         xDistance = radius + radius * Math.cos(alpha / 2.0);
         ConvexPolygon2d convexPolygon2d = steppingStoneInitial.getConvexPolygon2d();

         for (int k = ((numSides + 1) / 2) - 1, j = (numSides + 1) / 2; k > 0; k--, j++)
         {
            double dis = convexPolygon2d.getVertex(k).distance(convexPolygon2d.getVertex(j));
            if (dis >= yDistance)
            {
               yDistance = dis;
            }
         }
      }

      for (int i = 0; i < rows; i++)
      {
         if (i % 2 == 0)
         {
            startXPosition = x;
            startYPosition = y;
            y = startYPosition + spacingInY + yDistance;

            for (int j = 0; j < columns; j++)
            {
               if (variableHeight)
               {
                  height = randomDoubleInRange(random, 0.0, 0.2);
               }

               SteppingStone steppingStone = SteppingStone.generateRegularPolygonalStone("stone" + i + j, startXPosition, startYPosition, baseZ, height,
                                                radius, numSides, polygonToShrink);
               ret.addSteppingStone(steppingStone);
               startXPosition = startXPosition + spacingInX + xDistance;
            }
         }
      }

      return ret;
   }

   public static SteppingStones generateRandomPolygonalCheckeredStripSteppingStones(double startXPosition, double startYPosition, double radius, int numSides,
           double spacingInX, double spacingInY, double baseZ, double height, int numStones, ConvexPolygon2d polygonToShrink, boolean variableHeight)
   {
      double xCenter, yCenter, xbuffer = 0.0, ybuffer = 0.0;
      int count = 0;
      SteppingStones ret = new SteppingStones();
      Random random = new Random(1972L);

      while (count < numStones)
      {
         xCenter = startXPosition;
         yCenter = startYPosition;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         SteppingStone steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + count, random, xCenter, yCenter, baseZ, height, radius, numSides,
                                          polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;

         xCenter = startXPosition;
         yCenter = startYPosition + spacingInY + 2.0 * radius;
         ybuffer = yCenter;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + count, random, xCenter, yCenter, baseZ, height, radius, numSides,
                 polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;

         xCenter = startXPosition + spacingInX + 2.0 * radius;
         xbuffer = xCenter;
         yCenter = startYPosition;

         if (variableHeight)
         {
            height = randomDoubleInRange(random, 0.0, 0.2);
         }

         steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + count, random, xCenter, yCenter, baseZ, height, radius, numSides,
                 polygonToShrink);
         ret.addSteppingStone(steppingStone);
         count++;
         startXPosition = xbuffer;
         startYPosition = ybuffer;
      }

      return ret;
   }

   public static SteppingStones generateRandomPolygonalChessBoardSteppingStones(double startXPosition, double startYPosition, double radius, int numSides,
           double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns, ConvexPolygon2d polygonToShrink,
           boolean variableHeight)
   {
      Random random = new Random(1972L);
      @SuppressWarnings("unused")
      double alpha = (2.0 * Math.PI) / (numSides);
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      SteppingStones ret = new SteppingStones();

      for (int i = 0; i < numRows; i++)
      {
         if (i % 2 == 0)
         {
            startXPosition = x;
            startYPosition = y;
            y = startYPosition + 2.0 * spacingInY + 4.0 * radius;
            if (numColumns % 2 != 0)
               columns = numColumns + 1;
         }
         else
         {
            startXPosition = x + spacingInX + 2.0 * radius;
            startYPosition = startYPosition + spacingInY + 2.0 * radius;
            if (numColumns % 2 != 0)
               columns = numColumns - 1;
         }

         for (int j = 0; j < columns / 2; j++)
         {
            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            SteppingStone steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + i + j, random, startXPosition, startYPosition, baseZ, height,
                                             radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);
            startXPosition = startXPosition + 2.0 * spacingInX + 4.0 * radius;
         }
      }

      return ret;
   }

   public static SteppingStones generateRandomPolygonalUniformSteppingStones(double startXPosition, double startYPosition, double radius, int numSides,
           double spacingInX, double spacingInY, double baseZ, double height, int numRows, int numColumns, ConvexPolygon2d polygonToShrink,
           boolean variableHeight)
   {
      Random random = new Random(1972L);
      int rows = 2 * numRows;
      int columns = numColumns;
      double x = startXPosition;
      double y = startYPosition;
      SteppingStones ret = new SteppingStones();

      for (int i = 0; i < rows; i++)
      {
         if (i % 2 == 0)
         {
            startXPosition = x;
            startYPosition = y;
            y = startYPosition + spacingInY + 2.0 * radius;

            for (int j = 0; j < columns; j++)
            {
               if (variableHeight)
               {
                  height = randomDoubleInRange(random, 0.0, 0.2);
               }

               SteppingStone steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + i + j, random, startXPosition, startYPosition, baseZ, height,
                                                radius, numSides, polygonToShrink);
               ret.addSteppingStone(steppingStone);
               startXPosition = startXPosition + spacingInX + 2.0 * radius;
            }
         }
      }

      return ret;
   }

   public static SteppingStones generateRandomPolygonalRandomPatternOneSteppingStones(double startXPosition, double startYPosition, double radius,
           int numSides, double spacingInX, double baseZ, double height, int numStones, ConvexPolygon2d polygonToShrink, boolean variableHeight)
   {
      Random random = new Random(1972L);
      @SuppressWarnings("unused")
      double alpha = (2.0 * Math.PI) / (numSides);
      int columns = numStones / 3;
      @SuppressWarnings("unused")
      double x = startXPosition;
      @SuppressWarnings("unused")
      double y = startYPosition;
      SteppingStones ret = new SteppingStones();

      for (int j = 0; j < columns; j++)
      {
         if (j % 2 == 1)
         {
            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            SteppingStone steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + j + "0", random, startXPosition, startYPosition, baseZ, height,
                                             radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + j + "1", random, startXPosition + 1.414 * radius,
                    startYPosition + 1.414 * radius, baseZ, height, radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + j + "2", random, startXPosition + 2.0 * 1.414 * radius, startYPosition, baseZ,
                    height, radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            startXPosition = startXPosition + spacingInX + 2.0 * 1.414 * radius + 2.0 * radius;
            startYPosition = startYPosition + 1.414 * radius;
         }
         else
         {
            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            SteppingStone steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + j + "0", random, startXPosition, startYPosition, baseZ, height,
                                             radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + j + "1", random, startXPosition + 1.414 * radius,
                    startYPosition - 1.414 * radius, baseZ, height, radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            if (variableHeight)
            {
               height = randomDoubleInRange(random, 0.0, 0.2);
            }

            steppingStone = SteppingStone.generateRandomPolygonalStone("stone" + j + "2", random, startXPosition + 2.0 * 1.414 * radius, startYPosition, baseZ,
                    height, radius, numSides, polygonToShrink);
            ret.addSteppingStone(steppingStone);

            startXPosition = startXPosition + spacingInX + 2.0 * 1.414 * radius + 2.0 * radius;
            startYPosition = startYPosition - 1.414 * radius;
         }
      }

      return ret;
   }

   private static double randomDoubleInRange(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   public static void main(String[] args)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Null"));
      ConvexPolygon2d footPolygon = new ConvexPolygon2d(new double[][]
      {
         {-0.2, -0.2}, {-0.2, 0.2}, {0.2, 0.2}, {0.2, -0.2}
      });


//    SteppingStones steppingStones = SteppingStones.generateRandomSteppingStones(new Random(1776L), 20, footPolygon); // Inside out?
//    SteppingStones steppingStones = SteppingStones.generateRectangularCheckeredStripSteppingStones(0.0,0.0,0.2,0.2,0.1,0.1,-0.1, 0.0,33); // Inside out?
//    SteppingStones steppingStones = SteppingStones.generateRectangularChessBoardSteppingStones(0.0,0.0,0.2,0.2,0.0,0.0,-0.1,0.0,6,6); // Inside out?
//      SteppingStones steppingStones = SteppingStones.generateRectangularUniformSteppingStones(0.0, 0.0, 2.0, 2.0, 0.5, 0.5, -0.1, 0.0, 6, 6, footPolygon,
//                                         false);    // Inside out?

//    SteppingStones steppingStones = SteppingStones.generateRegularPolygonalCheckeredStripSteppingStones(0.0,0.0,0.2,5,0.0,0.0,-0.1,0.0,33);
//    SteppingStones steppingStones = SteppingStones.generateRegularPolygonalChessBoardSteppingStones(0.0,0.0,0.2,7,0.0,0.0,-0.1,0.0,8,8);
//    SteppingStones steppingStones = SteppingStones.generateRegularPolygonalUniformSteppingStones(0.0,0.0,0.2,5,0.0,0.0,-0.1,0.0,5,5);
//    SteppingStones steppingStones = SteppingStones.generateRandomPolygonalCheckeredStripSteppingStones(0.0,0.0,0.2,7,0.0,0.0,-0.1,0.0,33);
//    SteppingStones steppingStones = SteppingStones.generateRandomPolygonalChessBoardSteppingStones(0.0,0.0,0.2,5,0.0,0.0,-0.2,0.0,8,8);
    SteppingStones steppingStones = SteppingStones.generateRandomPolygonalUniformSteppingStones(0.0,0.0,0.5,6,0.0, 0.0, -0.1, 0.01, 5, 5, footPolygon, false);

      ArrayList<Graphics3DObject> linkGraphics = steppingStones.createLinkGraphics();

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(linkGraphics);

      scs.startOnAThread();
   }

   /**
    * getStonesIntersectingLocation
    *
    * @param x double
    * @param y double
    * @param stonesIntersectingLocation ArrayList
    */
   public void getStonesIntersectingLocation(double x, double y, ArrayList<SteppingStone> stonesIntersectingLocation)
   {
      for (SteppingStone steppingStone : steppingStones)
      {
         if (steppingStone.intersectsLocation(x, y))
            stonesIntersectingLocation.add(steppingStone);
      }
   }
}
