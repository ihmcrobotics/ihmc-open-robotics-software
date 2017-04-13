package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SolarPanelBehaviorStateMachine.SolarPanelStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelMotionPlanner;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelBehaviorStateMachine extends StateMachineBehavior<SolarPanelStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private BehaviorAction<SolarPanelStates> bahaviorAction;
   
   
   
   SolarPanel solarPanel;
   SolarPanelMotionPlanner solarPanelPlanner = new SolarPanelMotionPlanner(solarPanel);
   
   
   
   
   
   
   public enum SolarPanelStates
   {
      GOTO_READYPOSE, CLEANING_LINEARPATH, CLEANING_MULTIPLEPATH
   }
   
   public SolarPanelBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("SolarPanelBehaviorStateMachine", SolarPanelStates.class, yoTime, communicationBridge);

      this.atlasPrimitiveActions = atlasPrimitiveActions;
      
      // ************************* Immigration ******************************* //

      
      
      
      
      
      
      setUpMotionPlanner();
      
      
      
      
      
      
      
      setUpStateMachine();
   }
   
   private void setUpStateMachine()
   {
      BehaviorAction<SolarPanelStates> gotoReadyPoseAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.GOTO_READYPOSE, atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            TextToSpeechPacket p1 = new TextToSpeechPacket("Goto Ready Pose");
            sendPacket(p1);
            PrintTools.info("Goto Ready Pose");
            

          
          PrintTools.info("send message");
          
          
          double motionTime = 3.0;
          WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
          HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, new Point3D(0.6, -0.3, 1.2), new Quaternion(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
          wholeBodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
          atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT));
          
          PrintTools.info("sent message");

            
         }
      };
      
      BehaviorAction<SolarPanelStates> cleaningAction = new BehaviorAction<SolarPanelStates>(SolarPanelStates.CLEANING_LINEARPATH, atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Cleaning Linear Path");
            sendPacket(p1);
            PrintTools.info("Cleaning Linear Path");
            //super.setBehaviorInput();
            
//            DRCRobotModel robotModel = getRobotModel();      
//            kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, true);
//            toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
            

         }
      };

      //statemachine.addStateWithDoneTransition(gotoReadyPoseAction, SolarPanelStates.CLEANING_LINEARPATH);
      //statemachine.addState(cleaningAction);
      statemachine.addState(gotoReadyPoseAction);
      
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
   
   
   public void setUpMotionPlanner()
   {
      
   }
   
   
   private void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
}
