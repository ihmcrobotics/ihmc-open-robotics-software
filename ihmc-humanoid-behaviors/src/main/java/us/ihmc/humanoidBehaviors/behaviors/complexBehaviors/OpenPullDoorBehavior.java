package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.*;
import perception_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
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
      MOVE_HANDS_TO_INITIAL_LOCATION,
      MOVE_RIGHT_HAND_TO_INITIAL_LOCATION,
      TURN_ON_OPEN_DOOR_DETECTOR,
      OPEN_RIGHT_HAND,

      GRAB_DOOR_KNOB,
      CLOSE_RIGHT_HAND,
      TURN_DOOR_KNOB,
      PULL_ON_DOOR,
      PULL_ON_DOOR_MORE,
      PUT_LEFT_HAND_IN_DOOR,
      OPEN_RIGHT_HAND_TO_RELEASE_DOOR_KNOB,
      REMOVE_RIGHT_HAND_FROM_DOOR_KNOB,
      MOVE_RIGHT_HAND_CLEAR,
      SAFE_RIGHT_HAND,
      PUSH_OPEN_DOOR,
      TURN_RIGHT_TO_CLEAR_DOOR,
      PUT_RIGHT_HAND_AT_DOOR_APPROACH_LOCATION,
      PUT_RIGHT_HAND_IN_DOOR,
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
   private final ROS2PublisherBasics<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;
   protected final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<DoorLocationPacket>();
   private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;

   private long timeFirstDoorPullFinished = Long.MAX_VALUE;

   
   private final ROS2PublisherBasics<AutomaticManipulationAbortMessage> abortMessagePublisher;
   
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

      createSubscriber(DoorLocationPacket.class, PerceptionAPI.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput(), doorLocationPacket::set);

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
            referenceFrames.updateFrames();

            PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1, 0.8, referenceFrames.getMidFeetZUpFrame(),referenceFrames.getMidFeetZUpFrame());
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(message);
            publishTextToSpeech("Decrease heigth");
         }
      };
      
     

      BehaviorAction moveHandsToDoor = new BehaviorAction(atlasPrimitiveActions.leftHandTrajectoryBehavior,atlasPrimitiveActions.rightHandTrajectoryBehavior)
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

            
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand(0.775,  0.243,  0.909,
                    -1.586084090324106, 0.020808916332101107, -1.6129154247182247,
                    RobotSide.RIGHT,
                    "Moving Right Hand Above Door Knob",
                    4));


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
      
      
      BehaviorAction moveRightHandToDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
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
            
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.BASIC_GRIP);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

         }
      };
      
      BehaviorAction closeRightHand = new BehaviorAction(atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
           
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);
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
            //RIGHT hand in MultiClickdoor_0_objID316 ( 0.905,  0.161,  0.861 ) orientation -1.6028650668735689, 0.11697876570833832, -0.4508481230631935
            //Sending HandPosePacket with joint angles: [0.9121657167940591, 0.7459740703314905, 1.6321662808749469, -1.8572450854485256, 0.6954361027225711, 0.9566209710658681, 0.6554421664700405]

//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.905,  0.161,  0.861  ,
//                                                                                 -1.6028650668735689, 0.11697876570833832, -0.4508481230631935,
//                                                                                 RobotSide.RIGHT,
//                                                                                 "Turn Door Knob",
//                                                                                 3));



            //
            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.870,  0.143,  0.858 ,
                                                                                 -1.5627423555966327, 0.09604417393163554, -0.6296291985824343,
                                                                                RobotSide.RIGHT,
                                                                                "Turn Door Knob",
                                                                                4));

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
      
      BehaviorAction removeHandFromDoorKnob = new BehaviorAction(atlasPrimitiveActions.rightArmTrajectoryBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.492,  0.536,  0.949,
//                                                                                 -1.0010885536716199, 0.13559477363777, -1.5315488331624716,
//                                                                                RobotSide.RIGHT,
//                                                                                "Removing Hand From Door Knob",
//                                                                                2));
          //RIGHT hand in rightGrabber_objID160 ( 0.000,  0.000,  0.000 ) orientation -6.938893903907231E-18, -2.40692882291782E-16, -1.3877787807814463E-17
            //Sending HandPosePacket with joint angles: [-0.785398, -0.21587146258631892, 3.14159, -2.116894000339932, 2.9237335374002837, 1.3455840128526875, 0.10012868484905206]


            double[] rightArmPose = new double[] {-0.06297748877453922, 0.6173491524232326, 1.6879726724476818, -1.6583433722760346, 0.6848646542070285, -0.583852926735062, 1.5583772407261367};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);

            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);




            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.BASIC_GRIP);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

         }
        
      };
      
      
      BehaviorAction moveRightHandClear = new BehaviorAction(atlasPrimitiveActions.rightArmTrajectoryBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
//            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.492,  0.536,  0.949,
//                                                                                 -1.0010885536716199, 0.13559477363777, -1.5315488331624716,
//                                                                                RobotSide.RIGHT,
//                                                                                "Removing Hand From Door Knob",
//                                                                                2));
          //RIGHT hand in rightGrabber_objID160 ( 0.000,  0.000,  0.000 ) orientation -6.938893903907231E-18, -2.40692882291782E-16, -1.3877787807814463E-17
            //Sending HandPosePacket with joint angles: [-0.785398, -0.21587146258631892, 3.14159, -2.116894000339932, 2.9237335374002837, 1.3455840128526875, 0.10012868484905206]


            double[] rightArmPose = new double[] {-0.785398, -0.21587146258631892, 3.14159, -2.116894000339932, 2.9237335374002837, 1.3455840128526875, 0.10012868484905206};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 3, rightArmPose);

            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);




            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.BASIC_GRIP);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

         }
        
      };
      
      
      BehaviorAction pushDoorOpen = new BehaviorAction(atlasPrimitiveActions.leftArmTrajectoryBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //otherwise the robot stops the arm motion because it is to fast
            setAutomaticArmAbort(false);


            double[] leftArmPose = new double[] {-1.5708, 0.10368274319678998, 1.547510524212296, 1.4039946506162546, -0.7724023185840884, -0.11032864092855531, 2.489235716646133};

            ArmTrajectoryMessage leftPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 3, leftArmPose);

            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);

            //LEFT hand in MultiClickdoor_0_objID1639 ( 0.904, -0.178,  1.123 ) orientation -3.0881447248030516, 0.07115788522883558, 1.6537273636240406
            //Sending HandPosePacket with joint angles: [-1.5708, 0.10368274319678998, 1.547510524212296, 1.4039946506162546, -0.7724023185840884, -0.11032864092855531, 2.489235716646133]
//
//            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(moveHand(0.363,  0.546,  1.137,
//                                                                               2.4835904948189986, 0.018198548283333248, 0.13293536111042364,
//                                                                               RobotSide.LEFT,
//                                                                               "Pushing Door Open With Left Hand",
//                                                                               2));
            
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
         }

      };
      
      BehaviorAction turnRight = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            referenceFrames.updateFrames();
            Quaternion rot = new Quaternion();
            rot.setEuler(0, Math.toRadians(0), Math.toRadians(-30));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(3, rot, referenceFrames.getPelvisZUpFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);
         }

      };

      BehaviorAction moveRightHandIntoDoorApproachLocation = new BehaviorAction(atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.492,  0.536,  0.949,
            //                                                                                 -1.0010885536716199, 0.13559477363777, -1.5315488331624716,
            //                                                                                RobotSide.RIGHT,
            //                                                                                "Removing Hand From Door Knob",
            //                                                                                2));
            //RIGHT hand in rightGrabber_objID160 ( 0.000,  0.000,  0.000 ) orientation -6.938893903907231E-18, -2.40692882291782E-16, -1.3877787807814463E-17
            //Sending HandPosePacket with joint angles: [-0.785398, -0.21587146258631892, 3.14159, -2.116894000339932, 2.9237335374002837, 1.3455840128526875, 0.10012868484905206]


            double[] rightArmPose = new double[] {-0.785398, 0.5261852384215046, 3.14159, -1.5645836237956119, 2.9237335374002837, 1.1230074654573774, 0.0859542570502406};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);

            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);

         }

      };



      BehaviorAction moveRightHandIntoDoor = new BehaviorAction(atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //            atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(moveHand( 0.492,  0.536,  0.949,
            //                                                                                 -1.0010885536716199, 0.13559477363777, -1.5315488331624716,
            //                                                                                RobotSide.RIGHT,
            //                                                                                "Removing Hand From Door Knob",
            //                                                                                2));
            //RIGHT hand in rightGrabber_objID160 ( 0.000,  0.000,  0.000 ) orientation -6.938893903907231E-18, -2.40692882291782E-16, -1.3877787807814463E-17
            //Sending HandPosePacket with joint angles: [-0.785398, -0.21587146258631892, 3.14159, -2.116894000339932, 2.9237335374002837, 1.3455840128526875, 0.10012868484905206]


            double[] rightArmPose = new double[] {-0.30194139245512475, 1.4652392947714175, 2.5612754904865938, -1.2070877752672502, 2.845152091226715, 0.5345985142106401, 0.20836574432616292};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);

            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);

         }

      };


      BehaviorAction turnBackCenter = new BehaviorAction(atlasPrimitiveActions.chestTrajectoryBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, 0, Math.toRadians(0));
            referenceFrames.updateFrames();
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(3, rot, referenceFrames.getPelvisZUpFrame());
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
      factory.addStateAndDoneTransition(OpenDoorState.LOWER_HEIGHT, lowerHeightTask, OpenDoorState.MOVE_HANDS_TO_INITIAL_LOCATION);
      factory.addStateAndDoneTransition(OpenDoorState.MOVE_HANDS_TO_INITIAL_LOCATION,moveHandsToDoor,OpenDoorState.OPEN_RIGHT_HAND);
//      factory.addStateAndDoneTransition(OpenDoorState.MOVE_RIGHT_HAND_TO_INITIAL_LOCATION, moveRightHandToDoor, OpenDoorState.OPEN_RIGHT_HAND);
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
      
      
      factory.addStateAndDoneTransition(OpenDoorState.PUT_LEFT_HAND_IN_DOOR, putLeftHandInDoor, OpenDoorState.OPEN_RIGHT_HAND_TO_RELEASE_DOOR_KNOB);
      factory.addStateAndDoneTransition(OpenDoorState.OPEN_RIGHT_HAND_TO_RELEASE_DOOR_KNOB, releaseHandle, OpenDoorState.REMOVE_RIGHT_HAND_FROM_DOOR_KNOB);
      factory.addStateAndDoneTransition(OpenDoorState.REMOVE_RIGHT_HAND_FROM_DOOR_KNOB, removeHandFromDoorKnob, OpenDoorState.MOVE_RIGHT_HAND_CLEAR);

      factory.addStateAndDoneTransition(OpenDoorState.MOVE_RIGHT_HAND_CLEAR, moveRightHandClear, OpenDoorState.SAFE_RIGHT_HAND);
      factory.addStateAndDoneTransition(OpenDoorState.SAFE_RIGHT_HAND, moveRightHandToSafeLocation, OpenDoorState.PUSH_OPEN_DOOR);
      factory.addStateAndDoneTransition(OpenDoorState.PUSH_OPEN_DOOR, pushDoorOpen, OpenDoorState.TURN_RIGHT_TO_CLEAR_DOOR);
      factory.addStateAndDoneTransition(OpenDoorState.TURN_RIGHT_TO_CLEAR_DOOR, turnRight, OpenDoorState.PUT_RIGHT_HAND_AT_DOOR_APPROACH_LOCATION);
      factory.addStateAndDoneTransition(OpenDoorState.PUT_RIGHT_HAND_AT_DOOR_APPROACH_LOCATION, moveRightHandIntoDoorApproachLocation, OpenDoorState.PUT_RIGHT_HAND_IN_DOOR);

      factory.addStateAndDoneTransition(OpenDoorState.PUT_RIGHT_HAND_IN_DOOR, moveRightHandIntoDoor, OpenDoorState.TURN_BACK_LEFT);


      factory.addStateAndDoneTransition(OpenDoorState.TURN_BACK_LEFT, turnBackCenter, OpenDoorState.DONE);

     // factory.addStateAndDoneTransition(OpenDoorState.PULL_BACK_HANDS, pullHandsBack, OpenDoorState.DONE);

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