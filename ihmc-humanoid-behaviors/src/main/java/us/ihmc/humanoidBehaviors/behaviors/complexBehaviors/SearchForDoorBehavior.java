package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchForDoorBehavior.SearchForDoorBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class SearchForDoorBehavior extends StateMachineBehavior<SearchForDoorBehaviorState>
{

   public enum SearchForDoorBehaviorState
   {
      SETUP,
      LOOK_UP_RIGHT, //search for general door location
      LOOK_DOWN_RIGHT,
      LOOK_DOWN_LEFT,
      LOOK_UP_LEFT,
      RESET_ROBOT,
      DONE,
      WAIT_FOR_DOOR_PACKET_NO_SEARCHING,

   }

   private Pose3D doorTransformToWorld;
   private byte detectedDoorType;
   private AtlasPrimitiveActions atlasPrimitiveActions;

   private boolean recievedNewDoorLocation = false;

   private final ResetRobotBehavior resetRobotBehavior;

   private final HumanoidReferenceFrames referenceFrames;

   private boolean scan = false;

   protected final ConcurrentListeningQueue<DoorLocationPacket> doorLocationQueue = new ConcurrentListeningQueue<DoorLocationPacket>(10);

   public SearchForDoorBehavior(String robotName, String yoNamePrefix, ROS2Node ros2Node, YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
                                AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "SearchForDoorBehavior", SearchForDoorBehaviorState.class, yoTime, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      createSubscriber(DoorLocationPacket.class, ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput(), doorLocationQueue::put);
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      this.referenceFrames = referenceFrames;

      setupStateMachine();

   }
   
   public void setScanForDoor(boolean scan)
   {
      this.scan = scan;
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("entering search for door behavior action");
      clearDoorLocationHistory();
      super.onBehaviorEntered();
   }
   
   public void clearDoorLocationHistory()
   {
      recievedNewDoorLocation = false;
      doorTransformToWorld = null;
      doorLocationQueue.clear();
      detectedDoorType= DoorLocationPacket.UNKNOWN_TYPE;
   }
   
   

   @Override
   public void doControl()
   {
      if (doorLocationQueue.isNewPacketAvailable())
      {
         recievedDoorLocation(doorLocationQueue.getLatestPacket());
      }

      super.doControl();
   }

   @Override
   public void onBehaviorExited()
   {
      scan = false;
   }

   public Pose3D getLocation()
   {
      return doorTransformToWorld;
   }

   public byte getDoorType()
   {
      return detectedDoorType;
   }

   private void recievedDoorLocation(DoorLocationPacket doorLocationPacket)
   {
      setDoorType(doorLocationPacket.detected_door_type_);
      setDoorLocation(doorLocationPacket.getDoorTransformToWorld());
   }

   public void setDoorLocation(Pose3D pose)
   {
      System.out.println("SET DOOR LOCATION " + pose);
      doorTransformToWorld = pose;
      recievedNewDoorLocation = true;
   }

   public void setDoorType(byte doorType)
   {
      detectedDoorType = doorType;
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
   public boolean isDone()
   {

      try
      {
         return super.isDone();
      }
      catch (Exception e)
      {
         return false;
      }

   }

   @Override
   protected SearchForDoorBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<SearchForDoorBehaviorState, BehaviorAction> factory)
   {

      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      BehaviorAction lookUpRight = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, Math.toRadians(-10), Math.toRadians(-10));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(6, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }

      };

      BehaviorAction waitForDoorPacket = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         public boolean isDone()
         {
            return recievedNewDoorLocation && doorTransformToWorld != null;
         }
      };
      
      BehaviorAction setup = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
      };
      BehaviorAction lookDownRight = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, Math.toRadians(10), Math.toRadians(-10));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(6, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }
      };

      BehaviorAction lookDownLeft = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, Math.toRadians(10), Math.toRadians(10));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(6, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }

      };

      BehaviorAction lookUpLeft = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, Math.toRadians(-10), Math.toRadians(10));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(6, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }

      };
      
      factory.addStateChangedListener((from, to) ->
      {
         publishTextToSpeech((from == null ? null : from.name()) + " -> " + (to == null ? null : to.name()));
      });

      
      
      factory.addState(SearchForDoorBehaviorState.SETUP, setup);

      factory.addTransition(SearchForDoorBehaviorState.SETUP,
                            SearchForDoorBehaviorState.RESET_ROBOT,
                            t -> recievedNewDoorLocation && doorTransformToWorld != null);

      factory.addTransition(SearchForDoorBehaviorState.SETUP,
                            SearchForDoorBehaviorState.LOOK_UP_RIGHT,
                            t -> scan);
      
      factory.addTransition(SearchForDoorBehaviorState.SETUP,
                            SearchForDoorBehaviorState.WAIT_FOR_DOOR_PACKET_NO_SEARCHING,
                            t -> !scan);
      
     

         factory.addStateAndDoneTransition(SearchForDoorBehaviorState.LOOK_UP_RIGHT, lookUpRight, SearchForDoorBehaviorState.LOOK_DOWN_RIGHT);
         factory.addStateAndDoneTransition(SearchForDoorBehaviorState.LOOK_DOWN_RIGHT, lookDownRight, SearchForDoorBehaviorState.LOOK_DOWN_LEFT);
         factory.addStateAndDoneTransition(SearchForDoorBehaviorState.LOOK_DOWN_LEFT, lookDownLeft, SearchForDoorBehaviorState.LOOK_UP_LEFT);

         factory.addStateAndDoneTransition(SearchForDoorBehaviorState.LOOK_UP_LEFT, lookUpLeft, SearchForDoorBehaviorState.LOOK_UP_RIGHT);
         factory.addState(SearchForDoorBehaviorState.RESET_ROBOT, resetRobot);

         factory.addTransition(SearchForDoorBehaviorState.LOOK_UP_RIGHT,
                               SearchForDoorBehaviorState.RESET_ROBOT,
                               t -> recievedNewDoorLocation && doorTransformToWorld != null);

         factory.addTransition(SearchForDoorBehaviorState.LOOK_DOWN_RIGHT,
                               SearchForDoorBehaviorState.RESET_ROBOT,
                               t -> recievedNewDoorLocation && doorTransformToWorld != null);

         factory.addTransition(SearchForDoorBehaviorState.LOOK_DOWN_LEFT,
                               SearchForDoorBehaviorState.RESET_ROBOT,
                               t -> recievedNewDoorLocation && doorTransformToWorld != null);

         factory.addTransition(SearchForDoorBehaviorState.LOOK_UP_LEFT,
                               SearchForDoorBehaviorState.RESET_ROBOT,
                               t -> recievedNewDoorLocation && doorTransformToWorld != null);

    
         factory.addState(SearchForDoorBehaviorState.WAIT_FOR_DOOR_PACKET_NO_SEARCHING, waitForDoorPacket);
         return SearchForDoorBehaviorState.SETUP;

    


   }

}
