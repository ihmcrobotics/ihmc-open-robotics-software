package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.packets.HandPosePacket;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class DesiredHandPoseProvider implements ObjectConsumer<HandPosePacket>
{
   private final SideDependentList<AtomicReference<HandPosePacket>> packets = new SideDependentList<AtomicReference<HandPosePacket>>();
   
   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();
   private final SideDependentList<Map<OneDoFJoint, Double>> finalDesiredJointAngleMaps = new SideDependentList<Map<OneDoFJoint, Double>>();
   private double trajectoryTime = 1.0;

   private final ReferenceFrame chestFrame;
   
   private final SideDependentList<ReferenceFrame> packetReferenceFrames;
   private final FullRobotModel fullRobotModel;

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters, YoVariableRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      packetReferenceFrames = new SideDependentList<ReferenceFrame>(chestFrame, chestFrame);
      for (RobotSide robotSide : RobotSide.values)
      {
         packets.put(robotSide, new AtomicReference<HandPosePacket>());
         
         homePositions.put(robotSide, new FramePose(chestFrame, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame().get(robotSide)));

         desiredHandPoses.put(robotSide, homePositions.get(robotSide));
         
         finalDesiredJointAngleMaps.put(robotSide, new LinkedHashMap<OneDoFJoint, Double>());
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
            switch(object.getReferenceFrame())
            {
            case WORLD:
               packetReferenceFrames.put(robotSide, ReferenceFrame.getWorldFrame());
               break;
            case CHEST:
               packetReferenceFrames.put(robotSide, chestFrame);
               break;
            default:
               throw new RuntimeException("Unkown frame");
            }
            
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
            pose.changeFrame(packetReferenceFrames.get(robotSide));
            
            desiredHandPoses.put(robotSide, pose);
         }
         
         Map<OneDoFJoint, Double> finalDesiredJointAngleMap = finalDesiredJointAngleMaps.get(robotSide);
         
         int i = -1;
         for(ArmJointName armJoint: fullRobotModel.getRobotSpecificJointNames().getArmJointNames())
         {
            if(object.getJointAngles() != null)
            	finalDesiredJointAngleMap.put(fullRobotModel.getArmJoint(robotSide, armJoint), object.getJointAngles()[++i]);
         }
      }

      return desiredHandPoses.get(robotSide);
   }
   
   public Map<OneDoFJoint, Double> getFinalDesiredJointAngleMaps(RobotSide robotSide)
   {
      return finalDesiredJointAngleMaps.get(robotSide);
   }

   public ReferenceFrame getDesiredReferenceFrame(RobotSide robotSide)
   {
      return packetReferenceFrames.get(robotSide);
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
