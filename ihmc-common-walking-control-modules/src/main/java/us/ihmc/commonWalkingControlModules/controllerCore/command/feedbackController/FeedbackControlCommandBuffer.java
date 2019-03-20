package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * This class is not for general user, it is used for performing cross-robot command conversion in a
 * garbage free manner.
 * <p>
 * This class should only be used with {@link CrossRobotCommandResolver} and
 * {@link ControllerCoreCommandBuffer} to resolve a {@link ControllerCoreCommand}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class FeedbackControlCommandBuffer extends FeedbackControlCommandList
{
   private final RecyclingArrayList<OneDoFJointFeedbackControlCommand> oneDoFJointFeedbackControlCommandBuffer = new RecyclingArrayList<>(OneDoFJointFeedbackControlCommand.class);
   private final RecyclingArrayList<OrientationFeedbackControlCommand> orientationFeedbackControlCommandBuffer = new RecyclingArrayList<>(OrientationFeedbackControlCommand.class);
   private final RecyclingArrayList<PointFeedbackControlCommand> pointFeedbackControlCommandBuffer = new RecyclingArrayList<>(PointFeedbackControlCommand.class);
   private final RecyclingArrayList<SpatialFeedbackControlCommand> spatialFeedbackControlCommandBuffer = new RecyclingArrayList<>(SpatialFeedbackControlCommand.class);
   private final RecyclingArrayList<CenterOfMassFeedbackControlCommand> centerOfMassFeedbackControlCommandBuffer = new RecyclingArrayList<>(CenterOfMassFeedbackControlCommand.class);

   public FeedbackControlCommandBuffer()
   {
   }

   /**
    * In addition to clearing the list of commands declared in the super-type, it clears the internal
    * buffers marking the commands previously used as available.
    */
   @Override
   public void clear()
   {
      super.clear();
      oneDoFJointFeedbackControlCommandBuffer.clear();
      orientationFeedbackControlCommandBuffer.clear();
      pointFeedbackControlCommandBuffer.clear();
      spatialFeedbackControlCommandBuffer.clear();
      centerOfMassFeedbackControlCommandBuffer.clear();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void set(FeedbackControlCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommand(FeedbackControlCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommandList(FeedbackControlCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public FeedbackControlCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Gets an available {@link OneDoFJointFeedbackControlCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public OneDoFJointFeedbackControlCommand addOneDoFJointFeedbackControlCommand()
   {
      OneDoFJointFeedbackControlCommand command = oneDoFJointFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link JointspaceFeedbackControlCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public OrientationFeedbackControlCommand addOrientationFeedbackControlCommand()
   {
      OrientationFeedbackControlCommand command = orientationFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link PointFeedbackControlCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public PointFeedbackControlCommand addPointFeedbackControlCommand()
   {
      PointFeedbackControlCommand command = pointFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link SpatialFeedbackControlCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public SpatialFeedbackControlCommand addSpatialFeedbackControlCommand()
   {
      SpatialFeedbackControlCommand command = spatialFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link CenterOfMassFeedbackControlCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public CenterOfMassFeedbackControlCommand addCenterOfMassFeedbackControlCommand()
   {
      CenterOfMassFeedbackControlCommand command = centerOfMassFeedbackControlCommandBuffer.add();
      super.addCommand(command);
      return command;
   }
}
