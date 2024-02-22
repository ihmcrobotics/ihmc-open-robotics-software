package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.robotics.lists.RingBuffer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTInputSplineFitStateEstimator implements KSTInputStateEstimator
{

   public static final ReferenceFrame workdFrame = ReferenceFrame.getWorldFrame();

   @Override
   public void reset()
   {

   }

   @Override
   public void update(boolean isNewInput,
                      KinematicsStreamingToolboxInputCommand latestInputCommand,
                      KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {

   }

   private static class SingleEndEffectorStateEstimator
   {
      private final RingBuffer<YoFramePose3D> rawPoseHistory;

      public SingleEndEffectorStateEstimator(int historyLength, YoRegistry registry)
      {
         rawPoseHistory = new RingBuffer<>(historyLength, SupplierBuilder.indexedSupplier(i -> new YoFramePose3D("rawPoseHistory" + i, workdFrame, registry)));
      }
   }
}
