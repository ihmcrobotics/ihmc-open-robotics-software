package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class RepeatedlyWalkFootstepListBehavior extends AbstractBehavior
{
   private static final int defaultNumberOfStepsToTake = 10;
   private static final int defaultNumberOfIterations = 8;
   private static final RobotSide defaultInitialSwingSide = RobotSide.LEFT;

   private final YoBoolean walkingForward = new YoBoolean("walkingFotward", registry);
   private final YoDouble footstepLength = new YoDouble("footstepLength", registry);
   private final YoDouble footstepWidth = new YoDouble("footstepWidth", registry);
   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble transferTime = new YoDouble("transferTime", registry);
   private final YoInteger numberOfStepsToTake = new YoInteger("numberOfStepsToTake", registry);
   private final YoInteger iterations = new YoInteger("iterations", registry);
   private final YoInteger iterationCounter = new YoInteger("iterationCounter", registry);
   private final YoInteger stepsAlongPath = new YoInteger("stepsAlongPath", registry);
   private final YoEnum<RobotSide> initialSwingSide = YoEnum.create("initialSwingSide", RobotSide.class, registry);

   private final FootstepDataListMessage forwardFootstepList = new FootstepDataListMessage();
   private final FootstepDataListMessage backwardFootstepList = new FootstepDataListMessage();

   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(null);
   private final SideDependentList<MovingReferenceFrame> soleFrames;
   private final ReferenceFrame midFootZUpFrame;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher;

   public RepeatedlyWalkFootstepListBehavior(String robotName, Ros2Node ros2Node, HumanoidReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      super(robotName, ros2Node);

      soleFrames = referenceFrames.getSoleFrames();
      midFootZUpFrame = referenceFrames.getMidFeetZUpFrame();
      createSubscriberFromController(FootstepStatusMessage.class, footstepStatusMessage::set);
      footstepPublisher = createPublisherForController(FootstepDataListMessage.class);

      walkingForward.set(true);
      initialSwingSide.set(defaultInitialSwingSide);

      numberOfStepsToTake.set(defaultNumberOfStepsToTake);
      iterations.set(defaultNumberOfIterations);
      swingTime.set(1.5);
      transferTime.set(0.6);
      footstepLength.set(0.3);
      footstepWidth.set(0.25);

      parentRegistry.addChild(registry);
   }

   @Override
   public void onBehaviorEntered()
   {
      computeForwardFootstepList();
      computeBackwardFootstepList();

      footstepPublisher.publish(forwardFootstepList);
      walkingForward.set(true);
      stepsAlongPath.set(0);
   }

   private void computeForwardFootstepList()
   {
      forwardFootstepList.getFootstepDataList().clear();

      RobotSide swingSide = initialSwingSide.getEnumValue();

      for (int i = 0; i < numberOfStepsToTake.getIntegerValue(); i++)
      {
         FootstepDataMessage footstepDataMessage = constructFootstepDataMessage(midFootZUpFrame, footstepLength.getDoubleValue() * (i + 1),
                                                                                0.5 * swingSide.negateIfRightSide(footstepWidth.getDoubleValue()), swingSide);
         forwardFootstepList.getFootstepDataList().add().set(footstepDataMessage);

         swingSide = swingSide.getOppositeSide();
      }

      forwardFootstepList.setDefaultSwingDuration(swingTime.getDoubleValue());
      forwardFootstepList.setDefaultTransferDuration(transferTime.getDoubleValue());
   }

   private void computeBackwardFootstepList()
   {
      backwardFootstepList.getFootstepDataList().clear();

      ArrayList<FootstepDataMessage> footstepDataList = new ArrayList<>();
      List<FootstepDataMessage> dataList = forwardFootstepList.getFootstepDataList();
      for (int i = 0; i < dataList.size(); i++)
      {
         FootstepDataMessage step = dataList.get(i);
         footstepDataList.add(step);
      }
      footstepDataList.remove(footstepDataList.size() - 1);

      Collections.reverse(footstepDataList);

      RobotSide initialStanceSide = initialSwingSide.getEnumValue().getOppositeSide();
      FootstepDataMessage initialStanceFoot = constructFootstepDataMessage(soleFrames.get(initialStanceSide), 0.0, 0.0, initialStanceSide);
      footstepDataList.add(initialStanceFoot);
      MessageTools.copyData(footstepDataList, backwardFootstepList.getFootstepDataList());

      backwardFootstepList.setDefaultSwingDuration(swingTime.getDoubleValue());
      backwardFootstepList.setDefaultTransferDuration(transferTime.getDoubleValue());
   }

   private static FootstepDataMessage constructFootstepDataMessage(ReferenceFrame frame, double xOffset, double yOffset, RobotSide side)
   {
      FootstepDataMessage footstepDataMessage = new FootstepDataMessage();

      FramePose3D footstepPose = new FramePose3D();
      footstepPose.setToZero(frame);
      footstepPose.setPosition(xOffset, yOffset, 0.0);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepDataMessage.getLocation().set(footstepPose.getPosition());
      footstepDataMessage.getOrientation().set(footstepPose.getOrientation());
      footstepDataMessage.setRobotSide(side.toByte());

      return footstepDataMessage;
   }

   @Override
   public void doControl()
   {
      if (isDone())
      {
         return;
      }

      FootstepStatusMessage footstepStatus = this.footstepStatusMessage.getAndSet(null);
      if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         stepsAlongPath.increment();
      }

      if (stepsAlongPath.getIntegerValue() == forwardFootstepList.getFootstepDataList().size())
      {
         stepsAlongPath.set(0);

         if (walkingForward.getBooleanValue())
         {
            footstepPublisher.publish(backwardFootstepList);
            walkingForward.set(false);
         }
         else
         {
            iterationCounter.increment();
            if (isDone())
            {
               return;
            }

            footstepPublisher.publish(forwardFootstepList);
            walkingForward.set(true);
         }
      }
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public boolean isDone(double timeinState)
   {
      return iterationCounter.getIntegerValue() == iterations.getIntegerValue();
   }
}
