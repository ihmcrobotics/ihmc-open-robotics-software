package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;

public class GenericTaskNode extends CTTaskNode
{
   public static double handCoordinateOffsetX = -0.2;
   public static int nodeDimension = 11;

   public GenericTaskNode()
   {
      super(11);
   }
   
   public GenericTaskNode(CTTaskNode node)
   {
      super(node);
   }

   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, double eeX, double eeY, double eeZ,
                          double eeRoll, double eePitch, double eeYaw)
   {
      super(11);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
      setNodeData(4, chestRoll);
      setNodeData(5, eeX);
      setNodeData(6, eeY);
      setNodeData(7, eeZ);
      setNodeData(8, eeRoll);
      setNodeData(9, eePitch);
      setNodeData(10, eeYaw);
   }

   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, ConfigurationSpace eeConfigurationSpace)
   {
      super(11);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
      setNodeData(4, chestRoll);
      setNodeData(5, eeConfigurationSpace.getTranslationX());
      setNodeData(6, eeConfigurationSpace.getTranslationY());
      setNodeData(7, eeConfigurationSpace.getTranslationZ());
      setNodeData(8, eeConfigurationSpace.getRotationRoll());
      setNodeData(9, eeConfigurationSpace.getRotationPitch());
      setNodeData(10, eeConfigurationSpace.getRotationYaw());
   }

   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll)
   {
      super(11);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
      setNodeData(4, chestRoll);
      ConfigurationSpace eeConfigurationSpace = new ConfigurationSpace();
      setNodeData(5, eeConfigurationSpace.getTranslationX());
      setNodeData(6, eeConfigurationSpace.getTranslationY());
      setNodeData(7, eeConfigurationSpace.getTranslationZ());
      setNodeData(8, eeConfigurationSpace.getRotationRoll());
      setNodeData(9, eeConfigurationSpace.getRotationPitch());
      setNodeData(10, eeConfigurationSpace.getRotationYaw());
   }

   @Override
   public CTTaskNode createNode()
   {
      return new GenericTaskNode();
   }
}
