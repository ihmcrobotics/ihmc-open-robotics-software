package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;

public class CTTaskNode
{
   protected NodeData nodeData;
   protected NodeData normalizedNodeData;

   private ArrayList<CTTaskNode> childNodes;
   private CTTaskNode parentNode;

   protected boolean validity = true;

   protected KinematicsToolboxOutputStatus configuration;

   public CTTaskNode(CTTaskNode node)
   {
      this.nodeData = new NodeData(node.nodeData);
      this.normalizedNodeData = new NodeData(node.normalizedNodeData);
      this.configuration = node.configuration;

      this.childNodes = node.childNodes;
      this.parentNode = node.parentNode;

      this.validity = node.validity;
   }

   public CTTaskNode(double[] rootData)
   {
      this.nodeData = new NodeData(rootData.length);
      this.nodeData.q = rootData;
      this.childNodes = new ArrayList<CTTaskNode>();
      this.normalizedNodeData = new NodeData(rootData.length);

   }

   public CTTaskNode(int dimensionOfData)
   {
      this.nodeData = new NodeData(dimensionOfData);
      this.childNodes = new ArrayList<CTTaskNode>();
      this.normalizedNodeData = new NodeData(dimensionOfData);
   }

   public final int getDimensionOfNodeData()
   {
      return nodeData.getDimension();
   }

   public final double getNodeData(int index)
   {
      return nodeData.getQ(index);
   }

   public final double getNormalizedNodeData(int index)
   {
      return normalizedNodeData.getQ(index);
   }

   public final double getDistance(CTTaskNode targetNode)
   {
      return nodeData.distance(targetNode.nodeData);
   }

   public final double getNormailzedDistance(CTTaskNode targetNode)
   {
      return normalizedNodeData.distance(targetNode.normalizedNodeData);
   }

   public final double getTimeGap(CTTaskNode targetNode)
   {
      return targetNode.getNodeData(0) - getNodeData(0);
   }

   public final double getNormalizedTimeGap(CTTaskNode targetNode)
   {
      return targetNode.getNormalizedNodeData(0) - getNormalizedNodeData(0);
   }

   public final void setNodeData(int index, double data)
   {
      nodeData.setQ(index, data);
   }

   public final void setNormalizedNodeData(int index, double data)
   {
      normalizedNodeData.setQ(index, data);
   }
   
   public final void setTimeData(double time)
   {
      nodeData.setQ(0, time);
   }

   public final void addChildNode(CTTaskNode node)
   {
      childNodes.add(node);
      node.setParentNode(this);
   }

   public final CTTaskNode getChildNode(int indexOfChild)
   {
      return childNodes.get(indexOfChild);
   }

   public final int getNumberOfChild()
   {
      return childNodes.size();
   }

   public final void setParentNode(CTTaskNode node)
   {
      parentNode = node;
   }

   public final void clearParentNode()
   {
      parentNode = null;
   }

   public final CTTaskNode getParentNode()
   {
      return parentNode;
   }

   public final void printNodeData()
   {
      for (int i = 0; i < getDimensionOfNodeData(); i++)
         PrintTools.info("" + i + " " + getNodeData(i));
   }

   public final double getTime()
   {
      return getNodeData(0);
   }

   public final void convertDataToNormalizedData(TaskRegion nodeRegion)
   {
      normalizedNodeData = new NodeData(getDimensionOfNodeData());
      for (int i = 0; i < getDimensionOfNodeData(); i++)
      {
         double normalizedValue = 0;
         if (i == 0)
         {
            normalizedValue = (getNodeData(i) - nodeRegion.getLowerLimit(i)) / nodeRegion.getTrajectoryTime();
         }
         else
         {
            if (nodeRegion.isEnable(i))
               normalizedValue = (getNodeData(i) - nodeRegion.getLowerLimit(i)) / nodeRegion.sizeOfRegion(i);
            else
               normalizedValue = 0;
         }
         normalizedNodeData.setQ(i, normalizedValue);
      }
   }

   public final void convertNormalizedDataToData(TaskRegion nodeRegion)
   {
      nodeData = new NodeData(getDimensionOfNodeData());
      for (int i = 0; i < getDimensionOfNodeData(); i++)
      {
         double value = 0;
         if (i == 0)
         {
            value = getNormalizedNodeData(i) * nodeRegion.getTrajectoryTime() + nodeRegion.getLowerLimit(i);
         }
         else
         {
            value = getNormalizedNodeData(i) * nodeRegion.sizeOfRegion(i) + nodeRegion.getLowerLimit(i);
         }
         nodeData.setQ(i, value);
      }
   }

   /**
    * alpha is within 0 ~ 1 and a ratio from this to towardNode.
    * if i, the createdNewNode would be towardNode.
    * if 0, the createdNewNode would be this.
    */
   public final CTTaskNode createNewNodeTowardNode(CTTaskNode towardNode, double alpha)
   {
      /*
       * clamping alpha first.
       */
      if (alpha > 1)
         alpha = 1;
      else if (alpha < 0)
         alpha = 0;
      else
         ;

      CTTaskNode createdNewNode = new CTTaskNode(this);

      for (int i = 0; i < createdNewNode.getDimensionOfNodeData(); i++)
      {
         double stepToward;
         stepToward = (towardNode.getNodeData(i) - this.getNodeData(i)) * alpha;
         createdNewNode.setNodeData(i, this.getNodeData(i) + stepToward);

         stepToward = (towardNode.getNormalizedNodeData(i) - this.getNormalizedNodeData(i)) * alpha;
         createdNewNode.setNormalizedNodeData(i, this.getNormalizedNodeData(i) + stepToward);
      }
      createdNewNode.setParentNode(this);
      return createdNewNode;
   }

   public final void setConfigurationJoints(KinematicsToolboxOutputStatus outputStatus)
   {
      this.configuration = new KinematicsToolboxOutputStatus(outputStatus);
   }

   public KinematicsToolboxOutputStatus getConfiguration()
   {
      return configuration;
   }

   public void setValidity(boolean value)
   {
      validity = value;
   }

   public boolean getValidity()
   {
      return validity;
   }

}