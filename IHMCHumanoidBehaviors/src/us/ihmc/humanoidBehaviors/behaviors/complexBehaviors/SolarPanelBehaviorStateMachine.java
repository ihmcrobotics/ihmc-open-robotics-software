package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SolarPanelBehaviorStateMachine.SolarPanelStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTNode1DTimeDomain;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelMotionPlanner;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelMotionPlanner.CleaningMotion;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class SolarPanelBehaviorStateMachine extends StateMachineBehavior<SolarPanelStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;   
   
   private SolarPanel solarPanel;   
   private SolarPanelMotionPlanner solarPanelPlanner;
   
   private SolarPanelPoseValidityTester solarPanelPoseValidityTestBehavior;
   
   private RequestSolarPanelParamBehavior requestSolarPanelParamBehavior;
   private PlanningBehavior planningBehavior;
   
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
   
   public enum SolarPanelStates
   {
      HOME_MOTION, GET_SOLARPANEL, READY_MOTION, TESTER_ACTIVATE, PLANNING, CLEANING_MOTION, DONE
   }
   
   public SolarPanelBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions, 
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("SolarPanelBehaviorStateMachine", SolarPanelStates.class, yoTime, communicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      this.atlasPrimitiveActions = atlasPrimitiveActions;
    
      
      

      setUpSolarPanel();
      this.solarPanelPlanner = new SolarPanelMotionPlanner(solarPanel);
      
      
      this.requestSolarPanelParamBehavior = new RequestSolarPanelParamBehavior(communicationBridge);      
      solarPanelPoseValidityTestBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);

      planningBehavior = new PlanningBehavior(communicationBridge, solarPanelPlanner, solarPanelPoseValidityTestBehavior);
      
      
      
      
      
      
      
      setUpStateMachine();
      PrintTools.info("SetUp State Machine Done");
   }
   
   private void setUpStateMachine()
   {
      BehaviorAction<SolarPanelStates> homeMotionAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.HOME_MOTION,
            atlasPrimitiveActions.rightArmGoHomeBehavior,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("homeMotionAction");
            GoHomeMessage goHomeRightArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2);
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(goHomeRightArmMessage);
            HandDesiredConfigurationMessage handMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(handMessage);
         }
      };
      
      BehaviorAction<SolarPanelStates> getSolarPanelAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.GET_SOLARPANEL,
            requestSolarPanelParamBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("getSolarPanelAction");
            TextToSpeechPacket p1 = new TextToSpeechPacket("getSolarPanelAction");
            sendPacket(p1);
         }  
      };
      
      BehaviorAction<SolarPanelStates> readyMotionAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.READY_MOTION,
            atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("readyMotionAction");
            TextToSpeechPacket p1 = new TextToSpeechPacket("readyMotionAction");
            sendPacket(p1);
            
            // -------------------------------------------------- TEMP -------------------------------------------------- //
            FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 0.8, -0.35, 1.1);            

            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3.0);

            FrameOrientation tmpOr = new FrameOrientation(point.getReferenceFrame());
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.RIGHT, point, tmpOr);
            atlasPrimitiveActions.wholeBodyBehavior.holdCurrentChestOrientation();
            atlasPrimitiveActions.wholeBodyBehavior.holdCurrentPelvisHeight();
            atlasPrimitiveActions.wholeBodyBehavior.holdCurrentPelvisOrientation();
            
            atlasPrimitiveActions.wholeBodyBehavior.onBehaviorEntered();
            // -------------------------------------------------- TEMP -------------------------------------------------- //
         }
      };
      
      BehaviorAction<SolarPanelStates> testerActiveAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.TESTER_ACTIVATE,            
            solarPanelPoseValidityTestBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("testerActiveAction");
            TextToSpeechPacket p1 = new TextToSpeechPacket("testerActiveAction");
            sendPacket(p1);
            
            solarPanelPoseValidityTestBehavior.setIsDone(true);
         }
         @Override
         public void doTransitionOutOfAction()
         {    
            // no clean up behavior
         }
      };
      
      BehaviorAction<SolarPanelStates> planningAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.PLANNING,
            planningBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("planningAction ");
            TextToSpeechPacket p1 = new TextToSpeechPacket("planningAction");
            sendPacket(p1);            
         }
         
         @Override
         public void doTransitionOutOfAction()
         {
            if(solarPanelPlanner.setWholeBodyTrajectoryMessage(CleaningMotion.LinearCleaningMotion) == true)
            {
               
            } 
         }
      };
         
      BehaviorAction<SolarPanelStates> cleaningMotionAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.CLEANING_MOTION,
            atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("cleaningMotionAction");
            TextToSpeechPacket p1 = new TextToSpeechPacket("cleaningMotionAction");
            sendPacket(p1);
         }
      };
      
      BehaviorAction<SolarPanelStates> doneState = new BehaviorAction<SolarPanelStates>(SolarPanelStates.DONE,
            atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("doneState");
            TextToSpeechPacket p1 = new TextToSpeechPacket("doneState");
            sendPacket(p1);
         }
      };
      
      
      statemachine.addStateWithDoneTransition(homeMotionAction, SolarPanelStates.GET_SOLARPANEL);
      statemachine.addStateWithDoneTransition(getSolarPanelAction, SolarPanelStates.READY_MOTION);
      statemachine.addStateWithDoneTransition(readyMotionAction, SolarPanelStates.TESTER_ACTIVATE);
      statemachine.addStateWithDoneTransition(testerActiveAction, SolarPanelStates.PLANNING);
      statemachine.addStateWithDoneTransition(planningAction, SolarPanelStates.CLEANING_MOTION);
      statemachine.addStateWithDoneTransition(cleaningMotionAction, SolarPanelStates.DONE);
      statemachine.addState(doneState);
      
      statemachine.setStartState(SolarPanelStates.HOME_MOTION);
   }
   
   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Starting SolarPanel Behavior");
      sendPacket(p1);
      super.onBehaviorEntered();
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
      
   }
   
   
   
   public void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.75, -0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      solarPanel = new SolarPanel(poseSolarPanel, SolarPanel.defaultSizeX, SolarPanel.defaultSizeY);
   }
   
   public void setUpSolarPanel(Pose extractedPose, double sizeX, double sizeY)
   {
      Pose poseSolarPanel = new Pose();
      poseSolarPanel.setPosition(extractedPose.getPoint());
      poseSolarPanel.setOrientation(extractedPose.getOrientation());
      
      solarPanel = new SolarPanel(poseSolarPanel, sizeX, sizeY);
   }
   
   
   // ****************************************************** Behaviors .. ****************************************************** //
   
   private class RequestSolarPanelParamBehavior extends AbstractBehavior
   {
      public int cnt = 0;
      public RequestSolarPanelParamBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         PrintTools.info("RequestSolarPanelParamBehavior :: doControl");
         cnt++;
      }

      @Override
      public void onBehaviorEntered()
      {
         PrintTools.info("RequestSolarPanelParamBehavior :: onBehaviorEntered");
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
         PrintTools.info("RequestSolarPanelParamBehavior :: onBehaviorExited");
         
//         solarPanelPoseValidityTestBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);            
//         solarPanelPoseValidityTestBehavior.setSolarPanel(solarPanel);            
//         planningBehavior = new PlanningBehavior(communicationBridge, solarPanelPlanner, solarPanelPoseValidityTestBehavior);
         
         RRTNode1DTimeDomain.nodeValidityTester = solarPanelPoseValidityTestBehavior;
         RRTNode1DTimeDomain.nodeValidityTester.setSolarPanel(solarPanel);
      }

      @Override
      public boolean isDone()
      {
         if(cnt == 10)
            return true;
         else            
            return false;
      }
   }
   
   private class PlanningBehavior extends AbstractBehavior
   {
      private SolarPanelMotionPlanner solarPanelPlanner;
      
      private boolean isDone = false;
   
      public PlanningBehavior(CommunicationBridge communicationBridge, SolarPanelMotionPlanner solarPanelPlanner, SolarPanelPoseValidityTester solarPanelPoseValidityTestBehavior)
      {
         super(communicationBridge);
         this.solarPanelPlanner = solarPanelPlanner; 
      }

      @Override
      public void doControl()
      {
         PrintTools.info("PlanningBehavior :: doControl");
//         ThreadTools.sleep(1000);
//         PrintTools.info("");
//         ThreadTools.sleep(1000);
//         PrintTools.info("");
//         ThreadTools.sleep(1000);
//         PrintTools.info("");
//         if(solarPanelPlanner.setWholeBodyTrajectoryMessage(CleaningMotion.LinearCleaningMotion) == true)
//         {
//            
//         }      
         
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {
         ThreadTools.sleep(1500);
         PrintTools.info("PlanningBehavior :: onBehaviorEntered");
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
         PrintTools.info("PlanningBehavior :: onBehaviorExited");
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   
   
   
   
   
   
   
   
   
   
}
