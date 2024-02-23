package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;

public interface KSTInputStateEstimator
{
   void reset();

   void update(double time,
               boolean isNewInput,
               KinematicsStreamingToolboxInputCommand latestInputCommand,
               KinematicsStreamingToolboxInputCommand previousRawInputCommand);
}
