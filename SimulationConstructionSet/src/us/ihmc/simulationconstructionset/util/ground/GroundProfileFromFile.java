package us.ihmc.simulationconstructionset.util.ground;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.StringTokenizer;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.KDTree;


//Ground file must be a point cloud with 1 cm resolution
public class GroundProfileFromFile extends GroundProfileFromHeightMap
{
   private KDTree kdTree;
   private final BoundingBox3D boundingBox;

   public static enum VariableType {X, Y, Z}

   public GroundProfileFromFile(String BDITerrainFilePath, int maxPointsInLeaves, RigidBodyTransform transform3D)
   {
      this(BDITerrainFilePath, maxPointsInLeaves, transform3D, new VariableType[] {VariableType.X, VariableType.Y, VariableType.Z});
   }

   public GroundProfileFromFile(String BDITerrainFilePath, RigidBodyTransform transform3D)
   {
      this(BDITerrainFilePath, 10, transform3D, new VariableType[] {VariableType.X, VariableType.Y, VariableType.Z});
   }

   /**
    * Creates a KDTree from an array of (X, Y) terrain points and an equally sized array of
    * (Z) terrain heights.  The value MaxPointsInLeaves specifies the maximum number of
    * points in a leaf Node.   Use a small value (5-20) unless building the
    * tree takes too long.
    *
    * @param points double[][]
    * @param maxPointsInLeaves int
    */
   public GroundProfileFromFile(String BDITerrainFilePath, int maxPointsInLeaves, RigidBodyTransform transform3D, VariableType[] variableOrder)
   {
      double[][] rawPoints = loadPoints3D(BDITerrainFilePath, transform3D, variableOrder);

      // for(int i=0; i<rawPoints.length; i++)
      // {
      // System.out.println("(" + rawPoints[i][0] + ", " + rawPoints[i][1] + ", " + rawPoints[i][2] + ")");
      // }

      double[][] XYpoints = new double[rawPoints.length][2];

      // Copy X,Y data components to allow for (X,Y) queries.
      for (int i = 0; i < rawPoints.length; i++)
      {
         XYpoints[i][0] = rawPoints[i][0];
         XYpoints[i][1] = rawPoints[i][1];
      }

      double xMin, yMin, zMin;
      double xMax, yMax, zMax;

      // Find bounds of point data.
      xMin = yMin = zMin = Double.POSITIVE_INFINITY;
      xMax = yMax = zMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < XYpoints.length; i++)
      {
         if (XYpoints[i][0] < xMin)
            xMin = XYpoints[i][0];
         if (XYpoints[i][0] > xMax)
            xMax = XYpoints[i][0];
         if (XYpoints[i][1] < yMin)
            yMin = XYpoints[i][1];
         if (XYpoints[i][1] > yMax)
            yMax = XYpoints[i][1];
         if (rawPoints[i][2] < zMin)
            zMin = rawPoints[i][2];
         if (rawPoints[i][2] > zMax)
            zMax = rawPoints[i][2];
      }

      boundingBox = new BoundingBox3D(xMin, yMin, zMin, xMax, yMax, zMax);
      
      System.out.println(BDITerrainFilePath + ": " + "(" + xMin + ", " + yMin + ", " + zMin + ") (" + xMax + ", " + yMax + ", " + zMax + ")");

      kdTree = new KDTree(XYpoints, rawPoints, maxPointsInLeaves);
   }


   /**
    * Loads an ASCII file of 3D points.  The first line of the file must contain the
    * number of points, and all subsequent lines must contain three scalar values.
    *
    * @param filename String
    * @return OneDTerrainGrid
    */
   public static double[][] loadPoints3D(String filename, RigidBodyTransform transform3D, VariableType[] variableOrder)
   {
      BufferedReader br;

      System.out.println("loading points");
      System.out.flush();

      try
      {
         br = new BufferedReader(new FileReader(filename));
         System.out.println("Found " + filename);
         System.out.flush();
         double[][] points = loadPoints3D(br, transform3D, variableOrder);
         br.close();

         return points;
      }
      catch (IOException e)
      {
         System.err.println("Could not open " + filename);
         System.err.flush();
      }

      return null;
   }

   /**
    * Loads terrain data from a BufferedReader and returns a 2D array of doubles.
    * The first line of the file must contain the number of points, and all
    * subsequent lines must contain three scalar values.
    *
    * Returns null if the operation does not succeed.
    *
    * @param bufferedReader BufferedReader
    * @return BreadthFirstStateEnumerator
    */
   public static double[][] loadPoints3D(BufferedReader bufferedReader, RigidBodyTransform transform3D, VariableType[] variableOrder)
   {
      Point3D point3d = new Point3D();
      double[] values = new double[3];

      try
      {
         // int numPoints = Integer.parseInt(bufferedReader.readLine());
         ArrayList<double[]> pointsArrayList = new ArrayList<double[]>();

         // double[][] points = new double[numPoints][3];
         // for (int i=0; i<numPoints; i++)
         boolean moreToRead = true;

         do
         {
            String lineIn = bufferedReader.readLine();

            if (lineIn == null)
               moreToRead = false;    // +++ Is this the end of the file?

            else
            {
               // StringTokenizer s = new StringTokenizer(lineIn, " ");
               StringTokenizer s = new StringTokenizer(lineIn);


               double x = 0.0, y = 0.0, z = 0.0;

               values[0] = Double.parseDouble(s.nextToken()) / 1000.0;
               values[1] = Double.parseDouble(s.nextToken()) / 1000.0;
               values[2] = Double.parseDouble(s.nextToken()) / 1000.0;

               for (int i = 0; i < 3; i++)
               {
                  VariableType type = variableOrder[i];

                  if (type == VariableType.X)
                     x = values[i];
                  else if (type == VariableType.Y)
                     y = values[i];
                  else if (type == VariableType.Z)
                     z = values[i];
               }

               // @todo: Throw away points on the bottom.
               if (z > 0.005)
               {
                  point3d.set(x, y, z);
                  transform3D.transform(point3d);

                  double[] point = new double[] {point3d.getX(), point3d.getY(), point3d.getZ()};

                  // System.out.println("(" + point[0] + ", " + point[1] + ", " + point[2] + ")");

                  pointsArrayList.add(point);
               }

               if (s.hasMoreTokens())
               {
                  System.err.println("KDTree::loadPoints3D(): extra scalar encountered on line: " + lineIn);
               }
            }
         }
         while (moreToRead);

         double[][] points = new double[pointsArrayList.size()][];

         pointsArrayList.toArray(points);

         return points;
      }
      catch (IOException ex)
      {
         System.err.println(ex);
      }
      catch (NumberFormatException ex)
      {
         System.err.println(ex);
      }

      return null;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return height;
   }
   
   private final double[] query = new double[2];

   @Override
   public double heightAt(double x, double y, double z)
   {
      if (!boundingBox.isXYInsideInclusive(x, y)) return 0.0;

      // double[] query = new double[]{x, y};
      query[0] = x;
      query[1] = y;
      double[] closest = (double[]) kdTree.closestObject(query, 0.02);    // At most can be 2 cm away. Otherwise return 0.0;

      // if (closest == null) System.out.println(x + ", " + y);
      if ((closest == null) || (KDTree.distanceSquared(query, new double[] {closest[0], closest[1]}) > 0.01 * 0.01))
         return 0.0;

      return closest[2];
   }

   /*
    * public double[] closestPoint(double x, double y)
    * {
    *   double[] closest = (double[]) kdTree.closestObject(new double[]{x, y});
    *   return closest;
    * }
    */


   public void surfaceNormalAt(double x, double y, double z, Vector3D vector3d)
   {
      vector3d.set(0.0, 0.0, 1.0);
   }


   public static void main(String[] args)
   {
      // String datafile = "TerrainFiles/wood1.asc";
      String datafile = "TerrainFiles/rocks1.asc";
      @SuppressWarnings("unused") double xOffset = 0.0;    // 0.5;
      @SuppressWarnings("unused") double yOffset = 0.0;    // 0.3;
      @SuppressWarnings("unused") double unitScale = 0.01;

      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.setRotationYawAndZeroTranslation(Math.PI / 8.0);

      GroundProfileFromFile groundProfile = new GroundProfileFromFile(datafile, 10, transform3D, new VariableType[] {VariableType.X, VariableType.Z,
              VariableType.Y});

      // TerrainFromGroundProfile terrain = new TerrainFromGroundProfile(groundProfile, unitScale, xOffset, yOffset);
      BoundingBox3D boundingBox = groundProfile.getBoundingBox();

      System.out.println("xMin: " + boundingBox.getMinX() + ", xMax: " + boundingBox.getMaxX());
      System.out.println("yMin: " + boundingBox.getMinY() + ", yMax: " + boundingBox.getMaxY());
      System.out.println("zMin: " + boundingBox.getMinZ() + ", zMax: " + boundingBox.getMaxZ());

      Robot rob = new Robot("")
      {
         /**
          *
          */
      };

      SimulationConstructionSet scs = new SimulationConstructionSet(rob);
      scs.setGroundVisible(false);

      Graphics3DObject groundLinkGraphics = new Graphics3DObject();
      
      HeightMapWithNormals heightMap = groundProfile.getHeightMapIfAvailable();
      groundLinkGraphics.addHeightMap(heightMap, 100, 100, YoAppearance.Green());
      scs.addStaticLinkGraphics(groundLinkGraphics);
      

      Link link = new Link("");
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0);
      link.setLinkGraphics(linkGraphics);
      scs.addStaticLink(link);

      Thread thread = new Thread(scs);
      thread.start();

      // 608.799561 21.673950 38.070194

      /*
       * double x = 0.608799;
       * double y = 0.02167;
       * double z = groundProfile.heightAt(x, y, 0.0);
       *
       * double[] closest = groundProfile.closestPoint(x, y);
       *
       *
       *
       * System.out.print("(" + x + ", " + y + ", " + z + ")   ");
       * System.out.println("(" + closest[0] + ", " + closest[1]  + ", " + closest[2]  + ")");
       */

   }
   
   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
