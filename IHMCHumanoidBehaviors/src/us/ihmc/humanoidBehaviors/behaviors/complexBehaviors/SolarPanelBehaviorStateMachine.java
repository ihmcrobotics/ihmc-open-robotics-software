package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SolarPanelBehaviorStateMachine.SolarPanelStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTNode1DTimeDomain;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelMotionPlanner;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelMotionPlanner.CleaningMotion;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelBehaviorStateMachine extends StateMachineBehavior<SolarPanelStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private BehaviorAction<SolarPanelStates> bahaviorAction;
   
   private SolarPanel solarPanel;
   
   private SolarPanelMotionPlanner solarPanelPlanner;
   
   
   
   
   
   public enum SolarPanelStates
   {
      GOTO_READYPOSE, ACTIVATE_TESTER, PLANNING, CLEANING_MOTION
   }
   
   public SolarPanelBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("SolarPanelBehaviorStateMachine", SolarPanelStates.class, yoTime, communicationBridge);

      this.atlasPrimitiveActions = atlasPrimitiveActions;
      
      // ************************* Immigration ******************************* //

      setUpSolarPanel();
      this.atlasPrimitiveActions.solarPanelPoseValidityTestBehavior.setSolarPanel(solarPanel);
      this.solarPanelPlanner = new SolarPanelMotionPlanner(solarPanel);
      
//      RRTNode1DTimeDomain.nodeValidityTester = this.atlasPrimitiveActions.solarPanelPoseValidityTestBehavior;
//      
//      RRTNode1DTimeDomain.nodeValidityTester.setSolarPanel(solarPanel);

      
      
      
      
      
      
      
      
      PrintTools.info("Check");
      setUpStateMachine();
   }
   
   private void setUpStateMachine()
   {
      BehaviorAction<SolarPanelStates> gotoReadyPoseAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.GOTO_READYPOSE, atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            TextToSpeechPacket p1 = new TextToSpeechPacket("Goto Ready Pose");
            sendPacket(p1);
            PrintTools.info("gotoReadyPoseAction");
            
            WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
                            
            if(solarPanelPlanner.setWholeBodyTrajectoryMessage(CleaningMotion.ReadyPose) == true)
            {
               wholeBodyTrajectoryMessage = solarPanelPlanner.getWholeBodyTrajectoryMessage();
            }
          
            FramePoint rHandPoint = new FramePoint(ReferenceFrame.getWorldFrame(), 0.7, -0.35, 1.2);
            FrameOrientation rHandOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), new Quaternion());
                        
            PrintTools.info(""+solarPanelPlanner.getMotionTime());
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(solarPanelPlanner.getMotionTime());
            atlasPrimitiveActions.wholeBodyBehavior.holdCurrentChestOrientation();
            atlasPrimitiveActions.wholeBodyBehavior.holdCurrentPelvisHeight();
            atlasPrimitiveActions.wholeBodyBehavior.holdCurrentPelvisOrientation();
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.RIGHT, rHandPoint, rHandOrientation);            
         }
         

         
      };
      
      BehaviorAction<SolarPanelStates> activateTesterAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.ACTIVATE_TESTER, atlasPrimitiveActions.solarPanelPoseValidityTestBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("activateTesterAction");
            sendPacket(p1);
            PrintTools.info("activateTesterAction");
          
//            RRTNode1DTimeDomain.nodeValidityTester = atlasPrimitiveActions.solarPanelPoseValidityTestBehavior;            
//            RRTNode1DTimeDomain.nodeValidityTester.setSolarPanel(solarPanel);
//            
//            Pose dummyPose = new Pose(new Point3D(0.7, -0.35, 1.2), new Quaternion());
//            RRTNode1DTimeDomain.nodeValidityTester.setWholeBodyPose(dummyPose, 0.0);
         }
         
         
//         @Override
//         public void doTransitionOutOfAction()
//         {
//            PrintTools.info("do transition without cleanup actions");
//            //doPostBehaviorCleanup();
//         }
      };
      
      BehaviorAction<SolarPanelStates> planningAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.PLANNING, atlasPrimitiveActions.solarPanelPoseValidityTestBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("planningAction");
            sendPacket(p1);
            PrintTools.info("planningAction");            
            
            //atlasPrimitiveActions.solarPanelPoseValidityTestBehavior.setIsDone(true);
            
//            if(solarPanelPlanner.setWholeBodyTrajectoryMessage(CleaningMotion.LinearCleaningMotion) == true)
//            {
//               // atlasPrimitiveActions.solarPanelPoseValidityTestBehavior.onBehaviorExited();
//            }
//            
//            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(10.0);
         }
         
//         @Override
//         public void doTransitionOutOfAction()
//         {
//            doPostBehaviorCleanup();
//         }
      };
      
      
      
      BehaviorAction<SolarPanelStates> cleaningAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.CLEANING_MOTION, atlasPrimitiveActions.solarPanelPoseValidityTestBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Cleaning Linear Path");
            sendPacket(p1);
            PrintTools.info("cleaningAction");            
            

         }
      };

      statemachine.addStateWithDoneTransition(gotoReadyPoseAction, SolarPanelStates.ACTIVATE_TESTER);
      statemachine.addStateWithDoneTransition(activateTesterAction, SolarPanelStates.PLANNING);
      statemachine.addStateWithDoneTransition(planningAction, SolarPanelStates.CLEANING_MOTION);
      statemachine.addState(cleaningAction);      
      
      statemachine.setStartState(SolarPanelStates.GOTO_READYPOSE);

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
      
      solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
}
