package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;

/*
 * TODO That class does not seem to be used anymore.
 */
public class AutomaticClusteringTool
{
   public static List<Cluster> createClusters(List<Point3D> listOfPoints, double clusteringDistanceThreshold)
   {
      ArrayList<Cluster> listOfClusters = new ArrayList<>();

      Cluster cluster = new Cluster();

      for (int i = 0; i < listOfPoints.size() - 1; i++)
      {
         cluster.addRawPointInWorld(listOfPoints.get(i));

         if (listOfPoints.get(i).distance(listOfPoints.get(i + 1)) > clusteringDistanceThreshold)
         {
            if (cluster.getRawPointsInLocal().size() > 1)
            {
               listOfClusters.add(cluster);
            }
            cluster = new Cluster();
         }

         if (i == listOfPoints.size() - 2)
         {
            cluster.addRawPointInWorld(listOfPoints.get(listOfPoints.size() - 1));

            if (cluster.getRawPointsInLocal().size() > 1)
            {
               listOfClusters.add(cluster);
            }

         }
      }

      System.out.println("Calculated a total of " + listOfClusters.size() + " clusters");
      //
      //      for (int i = 0; i < listOfClusters.size(); i++)
      //      {
      //         System.out.println("     Cluster - " + i + " has size of " + listOfClusters.get(i).getPointsInCluster().size());
      //      }

      return listOfClusters;
   }

   public static List<Cluster> createClusters2(List<Point3D> listOfPoints, double clusteringDistanceThreshold)
   {
      ArrayList<Cluster> listOfClusters = new ArrayList<>();

      Cluster cluster = new Cluster();

      for (int i = 0; i < listOfPoints.size() - 1; i++)
      {
         cluster.addRawPointInWorld(listOfPoints.get(i));

         if (listOfPoints.get(i).distance(listOfPoints.get(i + 1)) > 1.0)
         {
            if (cluster.getRawPointsInLocal().size() > 1)
            {
               listOfClusters.add(cluster);
            }
            cluster = new Cluster();
         }

         if (i == listOfPoints.size() - 2)
         {
            cluster.addRawPointInWorld(listOfPoints.get(listOfPoints.size() - 1));

            if (cluster.getRawPointsInLocal().size() > 1)
            {
               listOfClusters.add(cluster);
            }

         }
      }

      System.out.println("Calculated a total of " + listOfClusters.size() + " clusters");

      return listOfClusters;
   }

}
