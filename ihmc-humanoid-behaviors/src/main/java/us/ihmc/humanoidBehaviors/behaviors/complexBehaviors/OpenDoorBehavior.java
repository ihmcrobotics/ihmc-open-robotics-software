package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.yoVariables.variable.YoDouble;

public class OpenDoorBehavior extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine;

   private PoseReferenceFrame doorPoseFrame = null;


   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;

   public OpenDoorBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      uiPositionCheckerPacketpublisher = createBehaviorOutputPublisher(UIPositionCheckerPacket.class);
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
   }

   private void setupPipeline()
   {

      BehaviorAction moveHandsToDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior,atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand( 0.298, -0.147,  1.097,1.2554068994570775, 0.03416782147174632, 0.26586161890007015,RobotSide.LEFT,"Moving Left Hand To Door"));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.833, -0.102,  1.079 , 1.551252338779563, 0.048351007951384285, 0.007252343575301105,RobotSide.RIGHT, "Moving Right Hand Above Door Knob"));

            
         }
      };

      BehaviorAction moveRightHandToDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.780, -0.101,  0.879,  1.551252338779563, 0.048351007951384285, 0.007252343575301105, RobotSide.RIGHT,"Moving Hand To Door Knob"));

         }
      };
      BehaviorAction pushDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.455,  0.218,  1.154,  1.7318790859631, 0.9163508562370669, -0.2253954188985998  , RobotSide.LEFT,"Pushing Door"));

         }
      };
      
      BehaviorAction pullHandsBack = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.274, -0.208,  0.798 ,  1.2609443582725661, 0.02096196100421688, 0.27326972080173334, RobotSide.LEFT,"Pushing Door"));

         }
      };



      pipeLine.clearAll();
      pipeLine.submitSingleTaskStage(moveHandsToDoor);
      pipeLine.submitSingleTaskStage(moveRightHandToDoorKnob);
      pipeLine.submitSingleTaskStage(pushDoor);
      pipeLine.submitSingleTaskStage(pullHandsBack);



   }



   private HandTrajectoryMessage moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll,final RobotSide side, final String description)
   {
      publishTextToSpeech(description);

    
      FramePose3D point = offsetPointFromDoorInWorldFrame(x, y, z, yaw, pitch, roll);

      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(side, 10, point.getPosition(),
                                                                                                     point.getOrientation(),
                                                                                                     CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

     return handTrajectoryMessage;
   }

   public void setGrabLocation(Pose3D doorPose3D)
   {
      
      PoseReferenceFrame doorPose = new PoseReferenceFrame("OpenDoorReferenceFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(doorPose3D));
      this.doorPoseFrame = doorPose;
   }

   @Override
   public void onBehaviorExited()
   {
      doorPoseFrame = null;
   }

   @Override
   public void doControl()
   {
      if (!isPaused())
         pipeLine.doControl();
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   private FramePose3D offsetPointFromDoorInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      FramePoint3D point1 = new FramePoint3D(doorPoseFrame, x, y, z);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(doorPoseFrame, yaw, pitch, roll);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D pose = new FramePose3D(point1, orient);

      return pose;
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

}