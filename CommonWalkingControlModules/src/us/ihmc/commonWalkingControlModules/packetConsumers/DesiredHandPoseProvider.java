package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

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
   private double trajectoryTime = 1.0;

   private final BooleanYoVariable useChestFrameForArms;
   
   private final ReferenceFrame chestFrame;

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters, YoVariableRegistry registry)
   {
      useChestFrameForArms = new BooleanYoVariable("useChestFrameForArms", registry);
      
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
   
   public boolean checkFoHomePosition(RobotSide robotSide)
   {
      if (!checkForNewPose(robotSide))
         return false;
      
      if (!packets.get(robotSide).get().isToHomePosition())
         return false;

      HandPosePacket object = packets.get(robotSide).getAndSet(null);
      trajectoryTime = object.getTrajectoryTime();
      desiredHandPoses.put(robotSide, homePositions.get(robotSide));
      return true;
   }

   public FramePose getDesiredHandPose(RobotSide robotSide)
   {
      HandPosePacket object = packets.get(robotSide).getAndSet(null);
      
      if(object != null)
      {
         trajectoryTime = object.getTrajectoryTime();
         
         if(object.isToHomePosition())
         {
            desiredHandPoses.put(robotSide, homePositions.get(robotSide));
         }
         else
         {
            ReferenceFrame referenceFrame;
            switch(object.getReferenceFrame())
            {
            case WORLD:
               referenceFrame = ReferenceFrame.getWorldFrame();
               break;
            case CHEST:
               referenceFrame = chestFrame;
               break;
            default:
               throw new RuntimeException("Unkown frame");
            }
            
            FramePose pose = new FramePose(referenceFrame, object.getPosition(), object.getOrientation());
            
            if (useChestFrameForArms.getBooleanValue())
            {
               referenceFrame = chestFrame;
               pose.changeFrame(referenceFrame);
            }
            
            desiredHandPoses.put(robotSide, pose);
         }
      }

      return desiredHandPoses.get(robotSide);
   }
   
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public void consumeObject(HandPosePacket object)
   {
      packets.get(object.getRobotSide()).set(object);
   }
}
