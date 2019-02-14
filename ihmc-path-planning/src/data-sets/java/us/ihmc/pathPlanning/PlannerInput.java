package us.ihmc.pathPlanning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlannerInput
{
   private Point3D startPosition = new Point3D();
   private Point3D goalPosition = new Point3D();
   private double startYaw = 0.0;
   private double goalYaw = 0.0;
   private HashMap<String, List<String>> additionalData = new HashMap<>();

   public Point3D getStartPosition()
   {
      return startPosition;
   }

   public Point3D getGoalPosition()
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

   public void setStartPosition(Point3DReadOnly startPosition)
   {
      this.startPosition.set(startPosition);
   }

   public void setStartPosition(double x, double y, double z)
   {
      this.startPosition.set(x, y, z);
   }

   public void setGoalPosition(Point3DReadOnly goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   public void setGoalPosition(double x, double y, double z)
   {
      this.goalPosition.set(x, y, z);
   }

   public void setStartYaw(double yaw)
   {
      this.startYaw = yaw;
   }

   public void setGoalYaw(double yaw)
   {
      this.goalYaw = yaw;
   }

   public void addAdditionalData(String key, String value)
   {
      additionalData.computeIfAbsent(key, k -> new ArrayList<>()).add(value);
   }

   public boolean getBooleanFlag(String key)
   {
      return additionalData.containsKey(key) && additionalData.get(key).get(0).equals("true");
   }

   /* package-private */
   HashMap<String, List<String>> getAdditionDataMap()
   {
      return additionalData;
   }
}
