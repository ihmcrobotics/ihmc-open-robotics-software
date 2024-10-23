package us.ihmc.manipulation.planning.exploringSpatial;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.commons.AngleTools;

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
      for (int i = 0; i < other.getRigidBodySpatials().size(); i++)
         rigidBodySpatials.add(new Pose3D(other.getRigidBodySpatials().get(i)));
      rigidBodyNames.addAll(other.getRigidBodyNames());
      configurationNames.addAll(other.getConfigurationNames());
      configurationData.addAll(other.getConfigurationData());
   }

   public void initializeData()
   {
      for (int i = 0; i < configurationData.size(); i++)
      {
         configurationData.set(i, 0.0);
      }
      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         rigidBodySpatials.set(i, new Pose3D());
      }
   }

   public void appendSpatial(String rigidBodyName, String[] configurationNames, double[] configurationData, RigidBodyTransform pose)
   {
      this.rigidBodyNames.add(rigidBodyName);
      this.rigidBodySpatials.add(new Pose3D(pose));
      for (int i = 0; i < configurationNames.length; i++)
         this.configurationNames.add(configurationNames[i]);
      for (int i = 0; i < configurationData.length; i++)
         this.configurationData.add(configurationData[i]);
   }

   public void interpolate(SpatialData dataOne, SpatialData dataTwo, double alpha)
   {
      for (int i = 0; i < rigidBodySpatials.size(); i++)
         rigidBodySpatials.get(i).interpolate(dataOne.getRigidBodySpatials().get(i), dataTwo.getRigidBodySpatials().get(i), alpha);

      for (int i = 0; i < configurationData.size(); i++)
      {
         double double1 = dataOne.getConfigurationData().get(i);
         double double2 = dataTwo.getConfigurationData().get(i);
         double doubleInterpolate = double1 + (double2 - double1) * alpha;

         configurationData.remove(i);
         configurationData.add(i, doubleInterpolate);
      }
   }

   public double getPositionDistance(SpatialData other)
   {
      double distance = 0.0;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         if (rigidBodyNames.get(i) != other.getRigidBodyNames().get(i))
            PrintTools.warn("other spatial data has different order");

         distance = distance + rigidBodySpatials.get(i).getPosition().distance(other.getRigidBodySpatials().get(i).getPosition());
      }

      return distance;
   }

   public double getOrientationDistance(SpatialData other)
   {
      double distance = 0.0;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double orientationDistance;

         if (rigidBodySpatials.get(i).getOrientation().equals(other.getRigidBodySpatials().get(i).getOrientation()))
            orientationDistance = 0.0;
         else
            orientationDistance = rigidBodySpatials.get(i).getOrientation().distance(other.getRigidBodySpatials().get(i).getOrientation());

         orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
         orientationDistance = Math.abs(orientationDistance);

         distance = distance + orientationDistance;
      }

      return distance;
   }

   public double getMaximumPositionDistance(SpatialData other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < rigidBodySpatials.size(); i++)
      {
         double positionDistance = rigidBodySpatials.get(i).getPosition().distance(other.getRigidBodySpatials().get(i).getPosition());

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
         double orientationDistance = rigidBodySpatials.get(i).getOrientation().distance(other.getRigidBodySpatials().get(i).getOrientation());
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
