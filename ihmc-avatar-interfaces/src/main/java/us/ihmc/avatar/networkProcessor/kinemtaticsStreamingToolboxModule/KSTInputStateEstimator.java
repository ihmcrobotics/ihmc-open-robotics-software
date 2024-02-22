package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;

public interface KSTInputStateEstimator
{
   void reset();

   void update(boolean isNewInput,
               KinematicsStreamingToolboxInputCommand inputCommandToFilter,
               KinematicsStreamingToolboxInputCommand currentRawInputCommand,
               KinematicsStreamingToolboxInputCommand previousRawInputCommand);
}
