package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.RobotKinematicsConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.NodeData;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class CTTaskNode
{
   private NodeData nodeData;
   private NodeData normalizedNodeData;

   private ArrayList<CTTaskNode> childNodes;
   private CTTaskNode parentNode;

   protected boolean validity = true;

   protected RobotKinematicsConfiguration configuration;

   public CTTaskNode(CTTaskNode node)
   {
      this(node.getDimensionOfNodeData());
   }

   public CTTaskNode(double[] rootData)
   {
      this.nodeData = new NodeData(rootData.length);
      this.nodeData.q = rootData;
      this.childNodes = new ArrayList<CTTaskNode>();
      this.normalizedNodeData = new NodeData(rootData.length);

      this.configuration = new RobotKinematicsConfiguration();
   }

   public CTTaskNode(int dimensionOfData)
   {
      this.nodeData = new NodeData(dimensionOfData);
      this.childNodes = new ArrayList<CTTaskNode>();
      this.normalizedNodeData = new NodeData(dimensionOfData);

      this.configuration = new RobotKinematicsConfiguration();
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

   public final void setNodeData(int index, double data)
   {
      nodeData.setQ(index, data);
   }

   public final void setNormalizedNodeData(int index, double data)
   {
      normalizedNodeData.setQ(index, data);
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

   /**
    * this method is only used to create optimal path in CTTaskNodeTree.
    */
   public final CTTaskNode createNodeCopy()
   {
      CTTaskNode nodeCopy = new CTTaskNode(this);

      nodeCopy.nodeData = new NodeData(this.nodeData);
      nodeCopy.normalizedNodeData = new NodeData(this.normalizedNodeData);
      nodeCopy.configuration = this.configuration;
      
      nodeCopy.parentNode = this.parentNode;
      
      nodeCopy.validity = this.validity;
      
      return nodeCopy;
   }

   public final void setConfigurationJoints(FullHumanoidRobotModel robot)
   {
      this.configuration.setRobotConfigurationData(robot);
   }

   public RobotKinematicsConfiguration getConfiguration()
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
