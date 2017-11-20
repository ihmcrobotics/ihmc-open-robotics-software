package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.robotics.geometry.AngleTools;

public class SpatialNode
{
   private double time;
   private String[] names;
   private Pose3D[] spatials;

   private SpatialNode parent;
   private final List<SpatialNode> children = new ArrayList<>();

   private boolean validity = true;
   private KinematicsToolboxOutputStatus configuration;

   public SpatialNode()
   {
   }

   public SpatialNode(String[] names, Pose3D[] spatials)
   {
      this(0.0, names, spatials);
   }

   public SpatialNode(double time, String[] names, Pose3D[] spatials)
   {
      this.time = time;
      this.names = names;
      this.spatials = spatials;
   }

   public SpatialNode(SpatialNode other)
   {
      time = other.time;
      names = new String[other.getSize()];
      spatials = new Pose3D[other.getSize()];

      for (int i = 0; i < other.getSize(); i++)
      {
         names[i] = other.names[i];
         spatials[i] = new Pose3D(other.spatials[i]);
      }

      parent = other.parent;
      children.addAll(other.children);

      validity = other.validity;
      if (other.configuration != null)
         configuration = new KinematicsToolboxOutputStatus(other.configuration);
   }

   public double getTimeGap(SpatialNode other)
   {
      return other.time - time;
   }

   public double getGreatestPositionDistance(SpatialNode other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < spatials.length; i++)
         distance = Math.max(distance, spatials[i].getPositionDistance(other.spatials[i]));

      return distance;
   }

   public double getGreatestOrientationDistance(SpatialNode other)
   {
      double distance = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < spatials.length; i++)
      {
         double orientationDistance = spatials[i].getOrientationDistance(other.spatials[i]);
         orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
         orientationDistance = Math.abs(orientationDistance);
         distance = Math.max(distance, orientationDistance);
      }

      return distance;
   }

   public double computeDistance(double timeWeight, double positionWeight, double orientationWeight, SpatialNode other)
   {
      double distanceSquared = MathTools.square(timeWeight * getTimeGap(other));
      for (int i = 0; i < spatials.length; i++)
      {
         distanceSquared += MathTools.square(positionWeight * getGreatestPositionDistance(other));
         distanceSquared += MathTools.square(orientationWeight * getGreatestOrientationDistance(other));
      }
      return Math.sqrt(distanceSquared);
   }

   public void interpolate(SpatialNode nodeOne, SpatialNode nodeTwo, double alpha)
   {
      names = nodeOne.names;
      spatials = new Pose3D[names.length];

      time = TupleTools.interpolate(nodeOne.time, nodeTwo.time, alpha);

      for (int i = 0; i < names.length; i++)
      {
         spatials[i] = new Pose3D();
         spatials[i].interpolate(nodeOne.spatials[i], nodeTwo.spatials[i], alpha);
      }
   }

   public SpatialNode createNodeWithinMaxDistance(double maxTimeInterval, double maxPositionDistance, double maxOrientationDistance, SpatialNode query)
   {
      double alpha = 1.0;

      double timeGap = getTimeGap(query);
      double greatestPositionDistance = getGreatestPositionDistance(query);
      double greatestOrientationDistance = getGreatestOrientationDistance(query);

      if (timeGap > maxTimeInterval)
         alpha = Math.min(alpha, maxTimeInterval / timeGap);
      if (greatestPositionDistance > maxPositionDistance)
         alpha = Math.min(alpha, maxPositionDistance / greatestPositionDistance);
      if (greatestOrientationDistance > maxOrientationDistance)
         alpha = Math.min(alpha, maxOrientationDistance / greatestOrientationDistance);

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
         SpatialNode spatialNode = new SpatialNode();
         spatialNode.interpolate(this, query, alpha);
         return spatialNode;
      }
   }

   public void addChild(SpatialNode child)
   {
      child.setParent(this);
      children.add(child);
   }

   public SpatialNode getChild(int index)
   {
      return children.get(index);
   }

   public int getNumberOfChildren()
   {
      return children.size();
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
      this.configuration = configuration;
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
      return names.length;
   }

   public String getName(int index)
   {
      return names[index];
   }

   public Pose3D getSpatial(int index)
   {
      return spatials[index];
   }

   public void setSpatialsAndConfiguration(SpatialNode other)
   {
      spatials = other.spatials;
      configuration = other.configuration;
   }
}
