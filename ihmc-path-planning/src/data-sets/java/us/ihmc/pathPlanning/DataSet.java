package us.ihmc.pathPlanning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class DataSet
{
   private final String name;
   private final PlanarRegionsList planarRegionsList;

   private Point3D startPosition = new Point3D();
   private Point3D goalPosition = new Point3D();
   private double startYaw = Double.NaN;
   private double goalYaw = Double.NaN;
   private HashMap<String, List<String>> additionalData = new HashMap<>();

   public DataSet(String name, PlanarRegionsList planarRegionsList)
   {
      this.name = name;
      this.planarRegionsList = planarRegionsList;
   }

   public String getName()
   {
      return name;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public Point3DReadOnly getStartPosition()
   {
      return startPosition;
   }

   public Point3DReadOnly getGoalPosition()
   {
      return goalPosition;
   }

   public double getStartYaw()
   {
      return startYaw;
   }

   public double getGoalYaw()
   {
      return goalYaw;
   }

   public List<String> getAdditionalData(String key)
   {
      return additionalData.get(key);
   }

   void setStartPosition(double x, double y, double z)
   {
      this.startPosition.set(x, y, z);
   }

   void setGoalPosition(double x, double y, double z)
   {
      this.goalPosition.set(x, y, z);
   }

   void setStartYaw(double yaw)
   {
      this.startYaw = yaw;
   }

   void setGoalYaw(double yaw)
   {
      this.goalYaw = yaw;
   }

   void addAdditionalData(String key, String value)
   {
      additionalData.computeIfAbsent(key, k -> new ArrayList<>()).add(value);
   }
}
