package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import org.apache.commons.lang.mutable.MutableBoolean;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHandPoseProvider implements ObjectConsumer<HandPosePacket>
{

   private final SideDependentList<FramePose> homePositions = new SideDependentList<FramePose>();
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();
   private final SideDependentList<MutableBoolean> contactStates  = new SideDependentList<MutableBoolean>();

   private SideDependentList<Boolean> hasNewPose = new SideDependentList<Boolean>();

   private final ReferenceFrame chestFrame;
   private boolean relativeToWorld = false;

   public DesiredHandPoseProvider(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         homePositions.put(robotSide, new FramePose(chestFrame, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame().get(robotSide)));

         desiredHandPoses.put(robotSide, homePositions.get(robotSide));
         contactStates.put(robotSide, new MutableBoolean(false));
         hasNewPose.put(robotSide, false);
      }

   }

   public synchronized boolean checkForNewPose(RobotSide robotSide)
   {
      return hasNewPose.get(robotSide);
   }

   public synchronized FramePose getDesiredHandPose(RobotSide robotSide)
   {
      hasNewPose.put(robotSide, false);
      return desiredHandPoses.get(robotSide);
   }
   
   public synchronized boolean isInContact(RobotSide robotSide)
   {
      return contactStates.get(robotSide).booleanValue();
   }
   
   public synchronized boolean isRelativeToWorld()
   {
      return relativeToWorld;
   }

   public void consumeObject(HandPosePacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasNewPose.put(robotSide, true);
      contactStates.get(robotSide).setValue(object.isLoadBearing);
      
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
}
