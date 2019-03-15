package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commons.lists.RecyclingArrayList;

public class FeedbackControlCommandBuffer extends FeedbackControlCommandList
{
   private final RecyclingArrayList<JointspaceFeedbackControlCommand> jointspaceFeedbackControlCommandBuffer = new RecyclingArrayList<>(JointspaceFeedbackControlCommand.class);
   private final RecyclingArrayList<OrientationFeedbackControlCommand> orientationFeedbackControlCommandBuffer = new RecyclingArrayList<>(OrientationFeedbackControlCommand.class);
   private final RecyclingArrayList<PointFeedbackControlCommand> pointFeedbackControlCommandBuffer = new RecyclingArrayList<>(PointFeedbackControlCommand.class);
   private final RecyclingArrayList<SpatialFeedbackControlCommand> spatialFeedbackControlCommandBuffer = new RecyclingArrayList<>(SpatialFeedbackControlCommand.class);
   private final RecyclingArrayList<CenterOfMassFeedbackControlCommand> centerOfMassFeedbackControlCommandBuffer = new RecyclingArrayList<>(CenterOfMassFeedbackControlCommand.class);

   public FeedbackControlCommandBuffer()
   {
   }

   @Override
   public void clear()
   {
      super.clear();
      jointspaceFeedbackControlCommandBuffer.clear();
      orientationFeedbackControlCommandBuffer.clear();
      pointFeedbackControlCommandBuffer.clear();
      spatialFeedbackControlCommandBuffer.clear();
      centerOfMassFeedbackControlCommandBuffer.clear();
   }

   @Override
   public void set(FeedbackControlCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommand(FeedbackControlCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommandList(FeedbackControlCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public FeedbackControlCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
   }

   public JointspaceFeedbackControlCommand addJointspaceFeedbackControlCommand()
   {
      JointspaceFeedbackControlCommand command = jointspaceFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public OrientationFeedbackControlCommand addOrientationFeedbackControlCommand()
   {
      OrientationFeedbackControlCommand command = orientationFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public PointFeedbackControlCommand addPointFeedbackControlCommand()
   {
      PointFeedbackControlCommand command = pointFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public SpatialFeedbackControlCommand addSpatialFeedbackControlCommand()
   {
      SpatialFeedbackControlCommand command = spatialFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public CenterOfMassFeedbackControlCommand addCenterOfMassFeedbackControlCommand()
   {
      CenterOfMassFeedbackControlCommand command = centerOfMassFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }
}
