package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PointCloudTools
{
   public enum WindingOrder
   {
      CW, CCW
   };

   private static double getAngle(Point3D pt1, Point3D refPt)
   {
      return Math.atan2(pt1.getY() - refPt.getY(), pt1.getX() - refPt.getX());
   }

   public static ArrayList<Point3D> orderPoints(ArrayList<Point3D> pointsToBeOrdered, WindingOrder windingOrder, Point3D referencePoint)
   {
      ArrayList<PointOrderHolder> listOfPointHolders = new ArrayList<>();
      for (Point3D pt : pointsToBeOrdered)
      {
         listOfPointHolders.add(new PointOrderHolder(new Point2D(pt.getX(), pt.getY()), getAngle(pt, referencePoint)));
      }

      ArrayList<PointOrderHolder> orderedPointHolders = new ArrayList<>();

      if (windingOrder == WindingOrder.CW)
      {
         while (!listOfPointHolders.isEmpty())
         {
            double maxAngle = -190.0;
            PointOrderHolder maxPointHolder = null;
            for (PointOrderHolder pointHolder : listOfPointHolders)
            {
               if (pointHolder.angle > maxAngle)
               {
                  maxAngle = pointHolder.angle;
                  maxPointHolder = pointHolder;
               }
            }

            listOfPointHolders.remove(maxPointHolder);
            //            System.out.println("Adding " + Math.toDegrees(maxPointHolder.angle));
            orderedPointHolders.add(maxPointHolder);
         }
      }
      else
      {
         while (!listOfPointHolders.isEmpty())
         {
            double minAngle = 190.0;
            PointOrderHolder maxPointHolder = null;
            for (PointOrderHolder pointHolder : listOfPointHolders)
            {
               if (pointHolder.angle < minAngle)
               {
                  minAngle = pointHolder.angle;
                  maxPointHolder = pointHolder;
               }
            }

            listOfPointHolders.remove(maxPointHolder);
            //            System.out.println("Adding " + Math.toDegrees(maxPointHolder.angle));
            orderedPointHolders.add(maxPointHolder);
         }
      }

      for (int i = 0; i < orderedPointHolders.size() - 1; i++)
      {
         Point2D ptA = orderedPointHolders.get(i).point;
         Point2D ptB = orderedPointHolders.get(i + 1).point;

         isClockwise(new Point2D(ptA.getX(), ptA.getY()), new Point2D(ptB.getX(), ptB.getY()), new Point2D(referencePoint.getX(), referencePoint.getY()));
      }

      ArrayList<Point3D> orderedList = new ArrayList<>();
      for (PointOrderHolder pointOrderHolder : orderedPointHolders)
      {
         orderedList.add(new Point3D((float) pointOrderHolder.point.getX(), (float) pointOrderHolder.point.getY(), 0));
      }

      return orderedList;
   }

   private static class PointOrderHolder
   {
      private Point2D point;
      private double angle;

      public PointOrderHolder(Point2D point, double angle)
      {
         this.point = point;
         this.angle = angle;
      }
   }

   public static Point3D getCentroid(ArrayList<Point3D> listOfPoints)
   {
      double x = 0;
      double y = 0;
      double z = 0;
      for (int i = 0; i < listOfPoints.size(); i++)
      {
         x = x + listOfPoints.get(i).getX();
         y = y + listOfPoints.get(i).getY();
         z = z + listOfPoints.get(i).getZ();
      }
      Point3D centroid1 = new Point3D((x / listOfPoints.size()), (y / listOfPoints.size()), (z / listOfPoints.size()));

      return centroid1;
   }

   private static boolean isClockwise(Point2D ptA, Point2D ptB, Point2D refPt)
   {
      if (calculateDeterminant(ptA, ptB, refPt) > 0)
      {
         //         System.out.println("CCW");
         return false;
      }
      if (calculateDeterminant(ptA, ptB, refPt) < 0)
      {
         //         System.out.println("CW");
         return true;
      }

      return true;
   }

   private static double calculateDeterminant(Point2D ptA, Point2D ptB, Point2D refPt)
   {
      return (ptA.getX() - refPt.getX()) * (ptB.getY() - refPt.getY()) - (ptB.getX() - refPt.getX()) * (ptA.getY() - refPt.getY());
   }

   public static void main(String args[])
   {
      new PointCloudTools();
   }

   public static void doBrakeDownOn2DPoints(List<Point2D> pointsToBrakeDown, double brakeDownThreshold)
   {
      for (int i = 1; i < pointsToBrakeDown.size(); i++)
      {
         Point2D point1 = pointsToBrakeDown.get(i - 1);
         Point2D point2 = pointsToBrakeDown.get(i);

         doBrakeDown2D(pointsToBrakeDown, i, point1, point2, brakeDownThreshold);
      }
   }

   private static void doBrakeDown2D(List<Point2D> points, int index, Point2D point1, Point2D point2, double brakeDownThreshold)
   {
      double nOfPointsToAddToSegment = Math.floor(point2.distance(point1) / brakeDownThreshold);
      Vector2D direction = new Vector2D(point2.getX() - point1.getX(), point2.getY() - point1.getY());
      direction.normalize();

      for (int i = 0; i < nOfPointsToAddToSegment; i++)
      {
         double xPos = point1.getX() + direction.getX() * brakeDownThreshold * (i + 1);
         double yPos = point1.getY() + direction.getY() * brakeDownThreshold * (i + 1);
         Point3D point = new Point3D(xPos, yPos, 0);
         points.add(index, new Point2D(xPos, yPos));
         index++;
      }
   }

   public static void savePlanarRegionsToFile(PlanarRegionsList planarRegionsList)
   {
      savePlanarRegionsToFile(planarRegionsList, null, null);
   }

   public static void savePlanarRegionsToFile(PlanarRegionsList planarRegionsList, Point3D start, Point3D goal)
   {
      Thread thread = new Thread()
      {
         @Override
         public void run()
         {
            if (planarRegionsList != null)
            {
               System.out.println("Saving planar regions to file");
               String data = "";

               String filename = "PlanarRegions_";
               if (filename.length() < 1)
               {
                  filename = "LidarDefault_";
               }

               filename = filename + new SimpleDateFormat("yyyyMMddhhmm'.txt'").format(new Date());

               File file = new File(filename);

               try
               {
                  // if file doesnt exists, then create it
                  if (!file.exists())
                  {
                     file.createNewFile();
                  }

                  FileWriter fw = new FileWriter(file.getAbsoluteFile());
                  BufferedWriter bw = new BufferedWriter(fw);

                  for (int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++) //planarRegionsList.getNumberOfPlanarRegions()
                  {
                     ConvexPolygon2D cp2d = planarRegionsList.getPlanarRegion(j).getConvexHull();
                     RigidBodyTransform transformToWorld = new RigidBodyTransform();
                     planarRegionsList.getPlanarRegion(j).getTransformToWorld(transformToWorld);

                     bw.write("PR_" + j);
                     bw.write(System.getProperty("line.separator"));

                     Vector3D translation = new Vector3D();
                     transformToWorld.getTranslation(translation);

                     Quaternion quat = new Quaternion();
                     transformToWorld.getRotation(quat);

                     bw.write("RBT," + translation + ", " + quat);
                     bw.write(System.getProperty("line.separator"));

                     for (int i = 0; i < cp2d.getNumberOfVertices(); i++)
                     {
                        Point2DReadOnly pt = cp2d.getVertexCCW(i);

                        FramePoint3D fpt = new FramePoint3D();
                        fpt.set(pt.getX(), pt.getY(), 0);
                        //                        fpt.applyTransform(transformToWorld);
                        data = fpt.getX() + ", " + fpt.getY() + ", " + fpt.getZ();
                        bw.write(data);
                        bw.write(System.getProperty("line.separator"));
                     }
                  }

                  if (start != null)
                  {
                     bw.write("start," + start + System.getProperty("line.separator"));
                  }
                  if (goal != null)
                  {
                     bw.write("goal," + goal + System.getProperty("line.separator"));
                  }

                  bw.close();
               }
               catch (IOException e1)
               {
                  // TODO Auto-generated catch block
                  e1.printStackTrace();
               }
            }
         }
      };

      thread.start();
   }

   @Deprecated
   public static ArrayList<PlanarRegion> loadPlanarRegionsFromFile(String fileName)
   {
      return loadPlanarRegionsFromFile(fileName, new Point3D(), new Point3D());
   }

   @Deprecated
   public static ArrayList<PlanarRegion> loadPlanarRegionsFromFile(String fileName, Point3D start, Point3D goal)
   {
      ArrayList<PlanarRegion> regions = new ArrayList<>();
      ArrayList<Cluster> clusters = new ArrayList<>();
      BufferedReader br = null;
      FileReader fr = null;

      start.setToNaN();
      goal.setToNaN();

      try
      {

         //br = new BufferedReader(new FileReader(FILENAME));
         fr = new FileReader(fileName);
         br = new BufferedReader(fr);

         String sCurrentLine;

         double averageX = 0.0;
         double averageY = 0.0;
         double averageZ = 0.0;

         int index = 0;

         Cluster cluster = new Cluster();
         int nPacketsRead = 0;

         ArrayList<Point3D> pointsTemp = new ArrayList<>();

         while ((sCurrentLine = br.readLine()) != null)
         {
            //                        System.out.println(sCurrentLine);

            if (sCurrentLine.contains("PR_"))
            {
               //               System.out.println("Contains PR_");
               if (!pointsTemp.isEmpty())
               {
                  //                  System.out.println("adding points");
                  cluster.addRawPointsInWorld3D(pointsTemp);
                  pointsTemp.clear();
               }

               cluster = new Cluster();
               clusters.add(cluster);
               nPacketsRead++;
               //               System.out.println("New cluster created");
            }

            else if (sCurrentLine.contains("RBT,"))
            {
               //                              System.out.println("Transformation read");
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());

               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);
               Vector3D translation = new Vector3D(x, y, z);

               float qx = (float) Double.parseDouble(points[3]);
               float qy = (float) Double.parseDouble(points[4]);
               float qz = (float) Double.parseDouble(points[5]);
               float qs = (float) Double.parseDouble(points[6]);
               Quaternion quat = new Quaternion(qx, qy, qz, qs);

               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(quat, translation);
               cluster.setTransformToWorld(rigidBodyTransform);
            }
            else if (sCurrentLine.contains("start,"))
            {
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());
               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");
               double x = Double.parseDouble(points[0]);
               double y = Double.parseDouble(points[1]);
               double z = Double.parseDouble(points[2]);
               start.set(x, y, z);
            }
            else if (sCurrentLine.contains("goal,"))
            {
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());
               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");
               double x = Double.parseDouble(points[0]);
               double y = Double.parseDouble(points[1]);
               double z = Double.parseDouble(points[2]);
               goal.set(x, y, z);
            }
            else
            {
               //                              System.out.println("adding point");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);

               pointsTemp.add(new Point3D(x, y, z));

               averageX = averageX + x;
               averageY = averageY + y;
               averageZ = averageZ + z;

               index++;
            }
         }

         if (!pointsTemp.isEmpty())
         {
            //            System.out.println("adding points");
            cluster.addRawPointsInWorld3D(pointsTemp);
            pointsTemp.clear();
         }

         for (Cluster cluster1 : clusters)
         {
            ConvexPolygon2D convexPolygon = new ConvexPolygon2D(cluster1.getRawPointsInLocal2D());

            PlanarRegion planarRegion = new PlanarRegion(cluster1.getTransformToWorld(), convexPolygon);
            

            regions.add(planarRegion);
         }

         System.out.println("Loaded " + regions.size() + " regions");
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }

      }
      return regions;
   }
}
