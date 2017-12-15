package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SpatialNode
{
   private double time;
   private SpatialData spatialData;

   private SpatialNode parent;

   private boolean validity = true;
   private KinematicsToolboxOutputStatus configuration;

   public SpatialNode()
   {
   }

   public SpatialNode(SpatialData spatialData)
   {
      this(0.0, spatialData);
   }

   public SpatialNode(double time, SpatialData spatialData)
   {
      this.time = time;
      this.spatialData = spatialData;
   }

   public SpatialNode(SpatialNode other)
   {
      time = other.time;
      spatialData = new SpatialData(other.spatialData);

      if (other.parent != null)
         parent = new SpatialNode(other.parent);
      else
         parent = null;

      validity = other.validity;
      if (other.configuration != null)
         configuration = new KinematicsToolboxOutputStatus(other.configuration);
   }

   public void initializeSpatialData()
   {
      spatialData.initializeData();
   }

   public double getTimeGap(SpatialNode other)
   {
      if (getTime() > other.getTime())
         return Double.MAX_VALUE;
      else
         return other.getTime() - getTime();
   }

   public double getPositionDistance(SpatialNode other)
   {
      return spatialData.getPositionDistance(other.getSpatialData());
   }

   public double getOrientationDistance(SpatialNode other)
   {
      return spatialData.getOrientationDistance(other.getSpatialData());
   }

   /**
    * Compute distance from this to other.
    */
   public double computeDistance(double timeWeight, double positionWeight, double orientationWeight, SpatialNode other)
   {
      double timeDistance = timeWeight * getTimeGap(other);
      double positionDistance = positionWeight * getPositionDistance(other);
      double orientationDistance = orientationWeight * getOrientationDistance(other);

      double distance = timeDistance + positionDistance + orientationDistance;
      return distance;
   }

   /**
    * Compute distance within maximum distance
    */
   public double computeDistanceWithinMaxDistance(double timeWeight, double positionWeight, double orientationWeight, SpatialNode other, double maxTimeInterval,
                                                  double maxPositionDistance, double maxOrientationDistance)
   {
      double timeDistance = timeWeight * getTimeGap(other);
      double positionDistance = positionWeight * getPositionDistance(other);
      double orientationDistance = orientationWeight * getOrientationDistance(other);

      double greatestPositionDistance = spatialData.getMaximumPositionDistance(other.getSpatialData());
      double greatestOrientationDistance = spatialData.getMaximumOrientationDistance(other.getSpatialData());

      if (greatestPositionDistance / getTimeGap(other) > maxPositionDistance / maxTimeInterval)
         return Double.MAX_VALUE;
      if (greatestOrientationDistance / getTimeGap(other) > maxOrientationDistance / maxTimeInterval)
         return Double.MAX_VALUE;

      double distance = timeDistance + positionDistance + orientationDistance;

      return distance;
   }

   public void interpolate(SpatialNode nodeOne, SpatialNode nodeTwo, double alpha)
   {
      time = TupleTools.interpolate(nodeOne.time, nodeTwo.time, alpha);

      spatialData.interpolate(nodeOne.getSpatialData(), nodeTwo.getSpatialData(), alpha);
   }

   public SpatialNode createNodeWithinMaxDistance(double maxTimeInterval, double maxPositionDistance, double maxOrientationDistance, SpatialNode query)
   {
      double alpha = 1.0;

      double timeGap = getTimeGap(query);

      double greatestPositionDistance = spatialData.getMaximumPositionDistance(query.getSpatialData());
      double greatestOrientationDistance = spatialData.getMaximumOrientationDistance(query.getSpatialData());

      if (timeGap > maxTimeInterval)
      {
         alpha = Math.min(alpha, maxTimeInterval / timeGap);
      }

      if (greatestPositionDistance > maxPositionDistance)
      {
         alpha = Math.min(alpha, maxPositionDistance / greatestPositionDistance);
      }

      if (greatestOrientationDistance > maxOrientationDistance)
      {
         alpha = Math.min(alpha, maxOrientationDistance / greatestOrientationDistance);
      }

      if (alpha >= 1.0)
      {
         return new SpatialNode(query);
      }
      else if (alpha <= 0.0)
      {
         return new SpatialNode(this);
      }
      else
      {
         SpatialNode spatialNode = new SpatialNode(this);
         spatialNode.interpolate(this, query, alpha);
         return spatialNode;
      }
   }

   public SpatialNode createNodeWithinTimeStep(double maxTimeInterval, SpatialNode query)
   {
      SpatialNode spatialNode = new SpatialNode(this);

      double alpha = 1.0;
      double timeGap = getTimeGap(query);
      if (timeGap > maxTimeInterval)
      {
         alpha = Math.min(alpha, maxTimeInterval / timeGap);
      }
      spatialNode.interpolate(this, query, alpha);

      return spatialNode;
   }

   public double getConfigurationData(int index)
   {
      return spatialData.getConfigurationData().get(index);
   }

   public String getConfigurationName(int index)
   {
      return spatialData.getConfigurationNames().get(index);
   }

   public SpatialData getSpatialData()
   {
      return spatialData;
   }

   public void setParent(SpatialNode parent)
   {
      this.parent = parent;
   }

   public void clearParent()
   {
      parent = null;
   }

   public SpatialNode getParent()
   {
      return parent;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public double getTime()
   {
      return time;
   }

   public void setConfiguration(KinematicsToolboxOutputStatus configuration)
   {
      this.configuration = new KinematicsToolboxOutputStatus(configuration);
   }

   public KinematicsToolboxOutputStatus getConfiguration()
   {
      return configuration;
   }

   public void setValidity(boolean validity)
   {
      this.validity = validity;
   }

   public boolean isValid()
   {
      return validity;
   }

   public int getSize()
   {
      return spatialData.getRigidBodyNames().size();
   }

   public String getName(int index)
   {
      return spatialData.getRigidBodyNames().get(index);
   }

   public Pose3D getSpatialData(int index)
   {
      return spatialData.getRigidBodySpatials().get(index);
   }

   public Pose3D getSpatialData(RigidBody rigidBody)
   {
      for (int i = 0; i < spatialData.getRigidBodyNames().size(); i++)
      {
         if (spatialData.getRigidBodyNames().get(i).equals(rigidBody.getName()))
            return getSpatialData(i);
      }
      return null;
   }

   public void setSpatialsAndConfiguration(SpatialNode other)
   {
      time = other.time;
      spatialData = new SpatialData(other.getSpatialData());
      configuration = new KinematicsToolboxOutputStatus(other.configuration);
      validity = other.validity;
   }
}
