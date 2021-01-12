package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.DoorOpenDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.OpenPullDoorBehavior.OpenDoorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.yoVariables.variable.YoDouble;

public class OpenPullDoorBehavior extends StateMachineBehavior<OpenDoorState>
{

   enum OpenDoorState
   {
      START,
      LOWER_HEIGHT,
      MOVE_LEFT_HAND_TO_INITIAL_LOCATION,
      MOVE_RIGHT_HAND_TO_INITIAL_LOCATION,
      TURN_ON_OPEN_DOOR_DETECTOR,
      OPEN_RIGHT_HAND,

      GRAB_DOOR_KNOB,
      CLOSE_RIGHT_HAND,
      TURN_DOOR_KNOB,
      PULL_ON_DOOR,
      PULL_ON_DOOR_MORE,
      PUT_LEFT_HAND_IN_DOOR,
      RELEASE_DOOR_KNOB,
      MOVE_RIGHT_HAND_CLEAR,
      SAFE_RIGHT_HAND,
      PUSH_OPEN_DOOR,
      TURN_RIGHT_TO_CLEAR_DOOR,
      TURN_BACK_LEFT,      
      PULL_BACK_HANDS,
      DONE,
      FAILED
   }

   private PoseReferenceFrame doorPoseFrame = null;

   private boolean succeeded;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SleepBehavior sleepBehavior;
   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;
   protected final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<DoorLocationPacket>();
   private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;

   private long timeFirstDoorPullFinished = Long.MAX_VALUE;

   
   private final IHMCROS2Publisher<AutomaticManipulationAbortMessage> abortMessagePublisher;
   
   private final HumanoidReferenceFrames referenceFrames;
   
   //move left foot 1.172,  0.602,  0.092 
//pinch close
   
   public OpenPullDoorBehavior(String robotName, String behaviorPrefix, YoDouble yoTime, ROS2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions,
                               DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService, HumanoidReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, "OpenDoorBehavior", OpenDoorState.class, yoTime, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;
      this.doorOpenDetectorBehaviorService = doorOpenDetectorBehaviorService;
      uiPositionCheckerPacketpublisher = createBehaviorOutputPublisher(UIPositionCheckerPacket.class);
      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);
      abortMessagePublisher = createPublisherForController(AutomaticManipulationAbortMessage.class);

      createSubscriber(DoorLocationPacket.class, ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput(), doorLocationPacket::set);

      setupStateMachine();

   }

   @Override
   public void onBehaviorEntered()
   {
      succeeded = false;
      super.onBehaviorEntered();
   }

   public boolean succeeded()
   {
      return succeeded;
   }

   @Override
   protected OpenDoorState configureStateMachineAndReturnInitialKey(StateMachineFactory<OpenDoorState, BehaviorAction> factory)
   {

      
      BehaviorAction start = new BehaviorAction(atlasPrimitiveActions.leftHandDesiredConfigurationBehavior,
                                                atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
           // doorLocationPacket.set(null);
         }

         @Override
         protected void setBehaviorInput()
         {
            
            HandDesiredConfigurationMessage leftHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.LEFT,
                                                                                                                         HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            
         }
//         @Override
//         public boolean isDone()
//         {
//            //wait for the door to be located and a baseline set for open detection
//            if (doorLocationPacket.get() != null)
//            {
//               setGrabLocation(doorLocationPacket.get().getDoorTransformToWorld());
//            }
//            return doorLocationPacket.get() != null;
//         }

      };
      
      
      BehaviorAction lowerHeightTask = new BehaviorAction(atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1, 0.75, referenceFrames.getWorldFrame(),referenceFrames.getMidFeetZUpFrame());
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(message);
            publishTextToSpeech("Decrease heigth");
         }
      };
      
     

      BehaviorAction moveLeftHandToDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(false);

            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand( 1.091,  0.140,  1.127,
                                                                                -2.842152676032728, -0.014314520562289174, -0.18564764268543277,
                                                                               RobotSide.LEFT,
                                                                               "Moving Left Hand To Door",
                                                                               4));

            
//            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand( 1.022,  0.157,  1.144,
//                                                                                -2.8441230686376677, -0.016485014310185484, -0.17486318044083837,
//                                                                               RobotSide.LEFT,
//                                                                               "Moving Left Hand To Door",
//                                                                               4));

         }
      };

      BehaviorAction moveRightHandToDoor = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(false);

            //pre speedup values for distance from door -0.102
            //            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.833, -0.0635, 1.079, 1.551252338779563, 0.048351007951384285, 0.007252343575301105, RobotSide.RIGHT, "Moving Right Hand Above Door Knob",3));

            //RIGHT hand in MultiClickdoor_0_objID1219 ( 0.769, -0.095,  1.032 ) orientation 1.549469789243062, 0.08444685410187032, 0.037877956817564146

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.775,  0.243,  0.909,
                                                                                -1.586084090324106, 0.020808916332101107, -1.6129154247182247,
                                                                                RobotSide.RIGHT,
                                                                                "Moving Right Hand Above Door Knob",
                                                                                4));

            //  atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.298, -0.147, 1.097, 1.2554068994570775, 0.03416782147174632,
            //                                                                    0.26586161890007015, RobotSide.LEFT, "Moving Left Hand To Door",4));

         }
      };

      BehaviorAction setDoorDetectorStart = new BehaviorAction()
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Starting Door Open Detector Service");

            doorOpenDetectorBehaviorService.reset();
            doorOpenDetectorBehaviorService.run(true);
         }

         @Override
         public boolean isDone()
         {
            //wait for the door to be located and a baseline set for open detection
            if (doorOpenDetectorBehaviorService.doorDetected())
            {
               publishTextToSpeech("Door Open Detector Service has closed door position saved");

            }
            return doorOpenDetectorBehaviorService.doorDetected();

         }

      };
      
      BehaviorAction openRightHand = new BehaviorAction(atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            
            
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.OPEN);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
 
         }
      };
      
      
      BehaviorAction moveRightHandToDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.855,  0.143,  0.903     ,
                                                                                 -1.562742720704193, 0.09604396595308502, -1.31030776262876,
                                                                                RobotSide.RIGHT,
                                                                                "Moving Hand To Door Knob",
                                                                                4));
            
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.847,  0.091,  0.917,
//                                                                                -1.3096309586379535, 0.019801129381768357, -1.5948559796213917,
//                                                                                RobotSide.RIGHT,
//                                                                                "Moving Hand To Door Knob",
//                                                                                4));

         }
      };
      
      BehaviorAction closeRightHand = new BehaviorAction(atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
           
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.PINCH_GRIP);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
 
         }
      };
      
      BehaviorAction moveRightHandToSafeLocation = new BehaviorAction(atlasPrimitiveActions.rightHandDesiredConfigurationBehavior,atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.589,  1.055,  0.830,
                                                                                -1.554026654268604, 0.759223557761935, -1.6572679687794496,
                                                                                RobotSide.RIGHT,
                                                                                "Move Right Hand Clear Of Closing Door",
                                                                                2));
            
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
 
         }
      };

      BehaviorAction turnDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);

            //            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.780, -0.0635, 0.879, 1.551252338779563, 0.048351007951384285,
            //                                                                                0.007252343575301105, RobotSide.RIGHT, "Moving Hand To Door Knob", 2));

            //atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.8, -0.1, 0.879, 1.551252338779563, 0.048351007951384285,0.007252343575301105, RobotSide.RIGHT, "Moving Hand To Door Knob", 2));

            //      RIGHT hand in MultiClickdoor_0_objID1219 ( 0.769, -0.096,  0.932 ) orientation 1.5511648101378044, 0.08462087065219358, 0.03818089607481523

            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.870,  0.143,  0.858 ,
                                                                                 -1.5627423555966327, 0.09604417393163554, -0.6296291985824343,
                                                                                RobotSide.RIGHT,
                                                                                "Turn Door Knob",
                                                                                4));
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.794,  0.186,  0.792 ,
//                                                                                 -1.5431936567280218, 0.13501342218408519, -0.7156860167842839,
//                                                                                RobotSide.RIGHT,
//                                                                                "Turn Door Knob",
//                                                                                4));

//            
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.856,  0.125,  0.791,
//                                                                                -1.2856865907790618, 0.0386255602637629, -0.9492873833196996,
//                                                                                RobotSide.RIGHT,
//                                                                                "Turn Door Knob",
//                                                                                4));

         }
      };
      BehaviorAction pullDoorALittle = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.75, -0.00, 0.879, 1.551252338779563, 0.048351007951384285,0.007252343575301105, RobotSide.RIGHT, "Push Door A Little", 1));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.765,  0.345,  0.861 ,
                                                                                -1.150674730539914, 0.061222681558504306, -0.6444483189913179,
                                                                                RobotSide.RIGHT,
                                                                                "Pull Door A Little",
                                                                                2));
            //RIGHT hand in MultiClickdoor_0_objID197 ( 0.750, -0.049,  0.896 ) orientation 1.5911238903674156, 0.038548649273740986, -7.31590778193919E-4

         }
         
         @Override
         public void onEntry()
         {
            super.onEntry();
            timeFirstDoorPullFinished = Long.MAX_VALUE;
         }
         @Override
         public boolean isDone()
         {
            if(super.isDone()&&timeFirstDoorPullFinished==Long.MAX_VALUE)
               timeFirstDoorPullFinished = System.currentTimeMillis();
            return super.isDone();
         }
         
      };
      
      BehaviorAction pullDoorMore = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.75, -0.00, 0.879, 1.551252338779563, 0.048351007951384285,0.007252343575301105, RobotSide.RIGHT, "Push Door A Little", 1));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.736,  0.424,  0.858 ,
                                                                                 -1.151342832129056, 0.05916110896787938, -0.6444342718082302,
                                                                                RobotSide.RIGHT,
                                                                                "Pull Door More",
                                                                                2));
            //
            //RIGHT hand in MultiClickdoor_0_objID197 ( 0.750, -0.049,  0.896 ) orientation 1.5911238903674156, 0.038548649273740986, -7.31590778193919E-4

         }
      };
      
      BehaviorAction putLeftHandInDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //otherwise the robot stops the arm motion because it is to fast
            setAutomaticArmAbort(false);
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.818,  0.112,  1.133,
                                                                               -3.005059183962542, -0.08222760637317995, 0.09712104081764292,
                                                                               RobotSide.LEFT,
                                                                               "Put Left Hand In Door",
                                                                               2));
         }
      };
      
      BehaviorAction releaseHandle = new BehaviorAction(atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            
            
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.OPEN);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

          
 
         }
      };
      
      BehaviorAction moveRightHandClear = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.75, -0.00, 0.879, 1.551252338779563, 0.048351007951384285,0.007252343575301105, RobotSide.RIGHT, "Push Door A Little", 1));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.492,  0.536,  0.949,
                                                                                 -1.0010885536716199, 0.13559477363777, -1.5315488331624716,
                                                                                RobotSide.RIGHT,
                                                                                "Removing Hand From Door Knob",
                                                                                2));
          //  [mRIGHT hand in MultiClickdoor_0_objID184 ( 0.492,  0.536,  0.949 ) orientation -1.0010885536716199, 0.13559477363777, -1.5315488331624716

            //

         }
        
      };
      
      
      BehaviorAction pushDoorOpen = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //otherwise the robot stops the arm motion because it is to fast
            setAutomaticArmAbort(false);
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.363,  0.546,  1.137,
                                                                               2.4835904948189986, 0.018198548283333248, 0.13293536111042364,
                                                                               RobotSide.LEFT,
                                                                               "Pushing Door Open With Left Hand",
                                                                               2));
         }

      };
      
      BehaviorAction turnRight = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, Math.toRadians(0), Math.toRadians(-15));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(6, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }

      };
      
      BehaviorAction turnBackCenter = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, 0, Math.toRadians(0));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(10, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }

      };

      BehaviorAction pullHandsBack = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setAutomaticArmAbort(true);
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(1.329,  0.642,  0.843,
                                                                               -2.367683866476793, 0.5316805288956675, -1.2625699425496124,
                                                                               RobotSide.LEFT,
                                                                               "Pulling Left Hand Back",
                                                                               1));
         }
      };

      BehaviorAction done = new BehaviorAction()
      {
         @Override
         protected void setBehaviorInput()
         {
            succeeded = true;
            publishTextToSpeech("DOOR OPENING COMPLETE");
         }
      };

      BehaviorAction failed = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior,atlasPrimitiveActions.rightHandTrajectoryBehavior,atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            
            
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.OPEN);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            
            
            succeeded = false;
            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand( 1.041,  0.206,  1.131,
                                                                                -2.833001530795148, -0.030807973085045466, -0.12069474717811632,
                                                                               RobotSide.LEFT,
                                                                               "Pulling Left Hand Back",
                                                                               4));
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.775,  0.243,  0.909,
                                                                                -1.586084090324106, 0.020808916332101107, -1.6129154247182247,
                                                                                RobotSide.RIGHT,
                                                                                "Pulling Right Hand Back",
                                                                                4));

            publishTextToSpeech("DOOR OPENING FAILED Reseting");
         }
      };

      factory.addStateAndDoneTransition(OpenDoorState.START, start, OpenDoorState.LOWER_HEIGHT);
      factory.addStateAndDoneTransition(OpenDoorState.LOWER_HEIGHT, lowerHeightTask, OpenDoorState.MOVE_LEFT_HAND_TO_INITIAL_LOCATION);
      factory.addStateAndDoneTransition(OpenDoorState.MOVE_LEFT_HAND_TO_INITIAL_LOCATION,moveLeftHandToDoor,OpenDoorState.MOVE_RIGHT_HAND_TO_INITIAL_LOCATION);
      factory.addStateAndDoneTransition(OpenDoorState.MOVE_RIGHT_HAND_TO_INITIAL_LOCATION, moveRightHandToDoor, OpenDoorState.OPEN_RIGHT_HAND);
      factory.addStateAndDoneTransition(OpenDoorState.TURN_ON_OPEN_DOOR_DETECTOR, setDoorDetectorStart, OpenDoorState.OPEN_RIGHT_HAND);
      
      factory.addStateAndDoneTransition(OpenDoorState.OPEN_RIGHT_HAND, openRightHand, OpenDoorState.GRAB_DOOR_KNOB);
      factory.addStateAndDoneTransition(OpenDoorState.GRAB_DOOR_KNOB, moveRightHandToDoorKnob, OpenDoorState.CLOSE_RIGHT_HAND);
      factory.addStateAndDoneTransition(OpenDoorState.CLOSE_RIGHT_HAND, closeRightHand, OpenDoorState.TURN_DOOR_KNOB);


                                        
                                        

      factory.addStateAndDoneTransition(OpenDoorState.TURN_DOOR_KNOB, turnDoorKnob, OpenDoorState.PULL_ON_DOOR);

      //      
      factory.addState(OpenDoorState.PULL_ON_DOOR, pullDoorALittle);
      factory.addState(OpenDoorState.PULL_ON_DOOR_MORE, pullDoorMore);

      factory.addState(OpenDoorState.DONE, done);
      factory.addState(OpenDoorState.FAILED, failed);

     // factory.addTransition(OpenDoorState.PULL_ON_DOOR, OpenDoorState.FAILED, t -> pullDoorALittle.isDone() &&doorOpenDetectorBehaviorService.getLastupdateTime()>=timeFirstDoorPullFinished && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PULL_ON_DOOR, OpenDoorState.PULL_ON_DOOR_MORE, t -> pullDoorMore.isDone());//doorOpenDetectorBehaviorService.isDoorOpen());
      
      
     // factory.addTransition(OpenDoorState.PULL_ON_DOOR_MORE, OpenDoorState.FAILED, t -> pullDoorMore.isDone() && !doorOpenDetectorBehaviorService.isDoorOpen());
      factory.addTransition(OpenDoorState.PULL_ON_DOOR_MORE, OpenDoorState.PUT_LEFT_HAND_IN_DOOR, t -> pullDoorMore.isDone());//&&doorOpenDetectorBehaviorService.isDoorOpen());
      
      
      factory.addStateAndDoneTransition(OpenDoorState.PUT_LEFT_HAND_IN_DOOR, putLeftHandInDoor, OpenDoorState.RELEASE_DOOR_KNOB);
      factory.addStateAndDoneTransition(OpenDoorState.RELEASE_DOOR_KNOB, releaseHandle, OpenDoorState.MOVE_RIGHT_HAND_CLEAR);
      factory.addStateAndDoneTransition(OpenDoorState.MOVE_RIGHT_HAND_CLEAR, moveRightHandClear, OpenDoorState.SAFE_RIGHT_HAND);
      factory.addStateAndDoneTransition(OpenDoorState.SAFE_RIGHT_HAND, moveRightHandToSafeLocation, OpenDoorState.PUSH_OPEN_DOOR);
      factory.addStateAndDoneTransition(OpenDoorState.PUSH_OPEN_DOOR, pushDoorOpen, OpenDoorState.TURN_RIGHT_TO_CLEAR_DOOR);
      factory.addStateAndDoneTransition(OpenDoorState.TURN_RIGHT_TO_CLEAR_DOOR, turnRight, OpenDoorState.TURN_BACK_LEFT);

      factory.addStateAndDoneTransition(OpenDoorState.TURN_BACK_LEFT, turnBackCenter, OpenDoorState.PULL_BACK_HANDS);

      factory.addStateAndDoneTransition(OpenDoorState.PULL_BACK_HANDS, pullHandsBack, OpenDoorState.DONE);

      return OpenDoorState.START;

   }

   /*
    * @Override public boolean isDone() {
    * //System.out.println("done check "+super.isDone()+" "+getStateMachine().
    * getCurrentBehaviorKey()+" " +getStateMachine().getCurrentAction().isDone()); return
    * super.isDone(); }
    */

   private HandTrajectoryMessage moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll,
                                          final RobotSide side, final String description, double trajectoryTime)
   {
      publishTextToSpeech(description);

      FramePose3D point = offsetPointFromDoorInWorldFrame(x, y, z, yaw, pitch, roll);

      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(point.getPosition()));

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(side,
                                                                                                     trajectoryTime,
                                                                                                     point.getPosition(),
                                                                                                     point.getOrientation(),
                                                                                                     CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      return handTrajectoryMessage;
   }

   private void setAutomaticArmAbort(boolean enableAbort)
   {
      AutomaticManipulationAbortMessage automaticManipulationAbortPacket = HumanoidMessageTools.createAutomaticManipulationAbortMessage(enableAbort);
      abortMessagePublisher.publish(automaticManipulationAbortPacket);
   }

   public void setGrabLocation(Pose3D doorPose3D)
   {
      publishTextToSpeech("grab location set " + doorPose3D);
      PoseReferenceFrame doorPose = new PoseReferenceFrame("OpenDoorReferenceFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(doorPose3D));
      this.doorPoseFrame = doorPose;
   }

   @Override
   public void doControl()
   {

      super.doControl();
   }

   @Override
   public void onBehaviorExited()
   {
      doorPoseFrame = null;
      doorLocationPacket.set(null);

   }

   private FramePose3D offsetPointFromDoorInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      System.out.println("doorPoseFrame " + doorPoseFrame);
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