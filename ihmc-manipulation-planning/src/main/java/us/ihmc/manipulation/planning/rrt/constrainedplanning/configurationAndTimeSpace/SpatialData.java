package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.geometry.AngleTools;

/**
 * this is data structure for plotting of the {@link SpatialNode}
 * @author shadylady
 *
 */
public class SpatialData
{
   private static boolean VERBOSE = true;

   private final List<String> rigidBodyNames;
   private final List<Pose3D> rigidBodySpatials;

   private final List<String> configurationNames;
   private final List<Double> configurationData;

   public SpatialData()
   {
      rigidBodyNames = new ArrayList<String>();
      rigidBodySpatials = new ArrayList<Pose3D>();
      configurationNames = new ArrayList<String>();
      configurationData = new ArrayList<Double>();
   }

   public SpatialData(SpatialData other)
   {
      this();
      rigidBodyNames.addAll(other.getRigidBodyNames());
      rigidBodySpatials.addAll(other.getRigidBodySpatials());
      configurationNames.addAll(other.getConfigurationNames());
      configurationData.addAll(other.getConfigurationData());
   }

   public void appendSpatial(String rigidBodyName, String[] configurationNames, double[] configurationData, Pose3D pose)
   {
      this.rigidBodyNames.add(rigidBodyName);
      this.rigidBodySpatials.add(pose);
      for (int i = 0; i < configurationNames.length; i++)
         this.configurationNames.add(configurationNames[i]);
      for (int i = 0; i < configurationData.length; i++)
         this.configurationData.add(configurationData[i]);
   }

   public void interpolate(SpatialData dataOne, SpatialData dataTwo, double alpha)
   {
      for (int i = 0; i < rigidBodySpatials.size(); i++)
         dataOne.getRigidBodySpatials().get(i).interpolate(dataOne.getRigidBodySpatials().get(i), dataTwo.getRigidBodySpatials().get(i), alpha);
   }

   public double getPositionDistance(SpatialData other)
   {
      double distance = 0.0;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         if (getRigidBodyNames().get(i) != other.getRigidBodyNames().get(i))
            PrintTools.warn("other spatial data has different order");
         distance = distance + rigidBodySpatials.get(i).getPositionDistance(other.getRigidBodySpatials().get(i));
      }

      return distance;
   }

   public double getOrientationDistance(SpatialData other)
   {
      double distance = 0.0;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {         
         double orientationDistance = rigidBodySpatials.get(i).getOrientationDistance(other.getRigidBodySpatials().get(i));
         orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
         orientationDistance = Math.abs(orientationDistance);
         
         distance = orientationDistance + rigidBodySpatials.get(i).getOrientationDistance(other.getRigidBodySpatials().get(i));
      }

      return distance;
   }

   public double getMaximumPositionDistance(SpatialData other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double positionDistance = rigidBodySpatials.get(i).getPositionDistance(other.getRigidBodySpatials().get(i));
         if (distance < positionDistance)
            distance = positionDistance;
      }

      return distance;
   }

   public double getMaximumOrientationDistance(SpatialData other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double orientationDistance = rigidBodySpatials.get(i).getOrientationDistance(other.getRigidBodySpatials().get(i));

         orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
         orientationDistance = Math.abs(orientationDistance);
         if (distance < orientationDistance)
            distance = orientationDistance;
      }

      return distance;
   }

   public List<String> getRigidBodyNames()
   {
      return rigidBodyNames;
   }

   public List<Pose3D> getRigidBodySpatials()
   {
      return rigidBodySpatials;
   }

   public List<String> getConfigurationNames()
   {
      return configurationNames;
   }

   public List<Double> getConfigurationData()
   {
      return configurationData;
   }

}
