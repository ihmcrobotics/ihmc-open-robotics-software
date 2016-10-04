package us.ihmc.robotModels;

public interface OutputWriter
{

   void initialize();

   void setFullRobotModel(FullRobotModel fullRobotModel);

   void write();

}