package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.packets.HandPosePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHandPoseProvider implements ObjectConsumer<HandPosePacket>
{

   private final SideDependentList<AtomicReference<HandPosePacket>> packets = new SideDependentList<AtomicReference<HandPosePacket>>();
   
   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();

   private final ReferenceFrame chestFrame;
   private boolean relativeToWorld = false;

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         packets.put(robotSide, new AtomicReference<HandPosePacket>());
         
         homePositions.put(robotSide, new FramePose(chestFrame, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame().get(robotSide)));

         desiredHandPoses.put(robotSide, homePositions.get(robotSide));
      }

   }

   public boolean checkForNewPose(RobotSide robotSide)
   {
      return packets.get(robotSide).get() != null;
      
      
   }

   public FramePose getDesiredHandPose(RobotSide robotSide)
   {
      HandPosePacket object = packets.get(robotSide).getAndSet(null);
      
      if(object != null)
      {
        
         if(object.isToHomePosition())
         {
            relativeToWorld = false;
            desiredHandPoses.put(robotSide, homePositions.get(robotSide));
         }
         else
         {
            ReferenceFrame referenceFrame;
            switch(object.getReferenceFrame())
            {
            case WORLD:
               referenceFrame = ReferenceFrame.getWorldFrame();
               relativeToWorld = true;
               break;
            case CHEST:
               referenceFrame = chestFrame;
               relativeToWorld = false;
               break;
            default:
               throw new RuntimeException("Unkown frame");
            }
            
            FramePose pose = new FramePose(referenceFrame, object.getPosition(), object.getOrientation());
            desiredHandPoses.put(robotSide, pose);
         }
      }

      return desiredHandPoses.get(robotSide);
   }
   
   public boolean isRelativeToWorld()
   {
      return relativeToWorld;
   }

   public void consumeObject(HandPosePacket object)
   {
      packets.get(object.getRobotSide()).set(object);
   }
}
