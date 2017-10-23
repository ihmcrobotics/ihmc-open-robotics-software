package us.ihmc.footstepPlanning.polygonSnapping;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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

      //      Point3f pt1 = new Point3f();
      //      Point3f refPt = new Point3f();
      //      
      //      pt1 = new Point3f(1,1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //
      //      pt1 = new Point3f(1,0,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(1,-1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(0,-1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(-1,-1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(-1,0,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
      //      
      //      pt1 = new Point3f(0,1,0);
      //      refPt = new Point3f(0,0,0);
      //      System.out.println(Math.toDegrees(Math.atan2(pt1.y - refPt.y, pt1.x - refPt.x)));
   }

   public static void doBrakeDownOn3DPoints(ArrayList<Point3D> pointsToBrakeDown, double brakeDownThreshold)
   {
      for (int i = 1; i < pointsToBrakeDown.size(); i++)
      {
         Point3D point1 = pointsToBrakeDown.get(i - 1);
         Point3D point2 = pointsToBrakeDown.get(i);

         doBrakeDown3D(pointsToBrakeDown, i, point1, point2);
      }
   }

   private static void doBrakeDown3D(ArrayList<Point3D> points, int index, Point3D point1, Point3D point2)
   {
      if (point2.distance(point1) > 0.2)
      {
         Point3D newPointToAdd = new Point3D(point1.getX() + ((point2.getX() - point1.getX()) / 2.0), point1.getY() + ((point2.getY() - point1.getY()) / 2.0),
                                             point1.getZ() + ((point2.getZ() - point1.getZ()) / 2.0));
         points.add(index, new Point3D(newPointToAdd.getX(), newPointToAdd.getY(), 0));
         doBrakeDown3D(points, index, point1, new Point3D(newPointToAdd.getX(), newPointToAdd.getY(), 0));
      }
   }

   public static void doBrakeDownOn2DPoints(ArrayList<Point2D> pointsToBrakeDown, double brakeDownThreshold)
   {
      for (int i = 1; i < pointsToBrakeDown.size(); i++)
      {
         Point2D point1 = pointsToBrakeDown.get(i - 1);
         Point2D point2 = pointsToBrakeDown.get(i);

         doBrakeDown2D(pointsToBrakeDown, i, point1, point2, brakeDownThreshold);
      }
   }

   private static void doBrakeDown2D(ArrayList<Point2D> points, int index, Point2D point1, Point2D point2, double brakeDownThreshold)
   {
      if (point2.distance(point1) > brakeDownThreshold)
      {
         Point2D newPointToAdd = new Point2D(point1.getX() + ((point2.getX() - point1.getX()) / 2.0), point1.getY() + ((point2.getY() - point1.getY()) / 2.0));
         points.add(newPointToAdd);
         doBrakeDown2D(points, index, point1, newPointToAdd, brakeDownThreshold);
      }
   }
   
   public static void savePlanarRegionsToFile(PlanarRegionsList planarRegionsList)
   {
      Thread thread = new Thread()
      {
         public void run()
         {
            int regions = 0;
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
                     
                     regions++;
                  }

                  bw.close();
               }
               catch (IOException e1)
               {
                  // TODO Auto-generated catch block
                  e1.printStackTrace();
               }
            }
            
            System.out.println("Finished saving " + regions + " regions");
         }
      };

      thread.start();
   }
}
