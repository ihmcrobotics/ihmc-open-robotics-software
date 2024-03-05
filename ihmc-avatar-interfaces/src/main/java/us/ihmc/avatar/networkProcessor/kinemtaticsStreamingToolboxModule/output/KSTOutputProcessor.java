package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;

public interface KSTOutputProcessor
{
   void initialize();

   void update(double time, boolean wasStreaming, boolean isStreaming, KinematicsToolboxOutputStatus latestOutput);

   KinematicsToolboxOutputStatus getProcessedOutput();
}
