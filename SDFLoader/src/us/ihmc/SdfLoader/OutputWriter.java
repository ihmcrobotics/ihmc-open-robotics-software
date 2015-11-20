package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.models.FullRobotModel;

public interface OutputWriter
{

   void initialize();

   void setFullRobotModel(FullRobotModel fullRobotModel);

   void write();

}