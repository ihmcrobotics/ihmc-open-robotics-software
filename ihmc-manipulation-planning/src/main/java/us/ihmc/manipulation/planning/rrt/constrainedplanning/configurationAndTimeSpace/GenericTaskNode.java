package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

public class GenericTaskNode extends CTTaskNode
{ 
   /**
    * 0  time
    * 1  pelvis height
    * 2  chest yaw
    * 3  chest pitch
    * 4  chest roll
    * 5  left x
    * 6  left y
    * 7  left z
    * 8  left roll
    * 9  left pitch
    * 10 left yaw
    * 11 right x
    * 12 right y
    * 13 right z
    * 14 right roll
    * 15 right pitch
    * 16 right yaw
    * 
    */
   public static int nodeDimension = 17;  // 11

   public GenericTaskNode()
   {
      super(nodeDimension);
   }
   
   public GenericTaskNode(CTTaskNode node)
   {
      super(node);
   }
   
//   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, double eeX, double eeY, double eeZ,
//                          double eeRoll, double eePitch, double eeYaw)
//   {
//      super(nodeDimension);
//      setNodeData(0, time);
//      setNodeData(1, pelvisHeight);
//      setNodeData(2, chestYaw);
//      setNodeData(3, chestPitch);
//      setNodeData(4, chestRoll);
//      setNodeData(5, eeX);
//      setNodeData(6, eeY);
//      setNodeData(7, eeZ);
//      setNodeData(8, eeRoll);
//      setNodeData(9, eePitch);
//      setNodeData(10, eeYaw);
//   }
//
//   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, ConfigurationSpace eeConfigurationSpace)
//   {
//      super(nodeDimension);
//      setNodeData(0, time);
//      setNodeData(1, pelvisHeight);
//      setNodeData(2, chestYaw);
//      setNodeData(3, chestPitch);
//      setNodeData(4, chestRoll);
//      setNodeData(5, eeConfigurationSpace.getTranslationX());
//      setNodeData(6, eeConfigurationSpace.getTranslationY());
//      setNodeData(7, eeConfigurationSpace.getTranslationZ());
//      setNodeData(8, eeConfigurationSpace.getRotationRoll());
//      setNodeData(9, eeConfigurationSpace.getRotationPitch());
//      setNodeData(10, eeConfigurationSpace.getRotationYaw());
//   }
//
//   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll)
//   {
//      super(nodeDimension);
//      setNodeData(0, time);
//      setNodeData(1, pelvisHeight);
//      setNodeData(2, chestYaw);
//      setNodeData(3, chestPitch);
//      setNodeData(4, chestRoll);
//      
//      ConfigurationSpace eeConfigurationSpace = new ConfigurationSpace();
//      setNodeData(5, eeConfigurationSpace.getTranslationX());
//      setNodeData(6, eeConfigurationSpace.getTranslationY());
//      setNodeData(7, eeConfigurationSpace.getTranslationZ());
//      setNodeData(8, eeConfigurationSpace.getRotationRoll());
//      setNodeData(9, eeConfigurationSpace.getRotationPitch());
//      setNodeData(10, eeConfigurationSpace.getRotationYaw());
//   }

}