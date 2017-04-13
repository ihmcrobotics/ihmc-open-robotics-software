package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.io.IOException;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SolarPanelBehaviorStateMachine.SolarPanelStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelBehaviorStateMachine extends StateMachineBehavior<SolarPanelStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private BehaviorAction<SolarPanelStates> bahaviorAction;
   
   
   private PacketCommunicator toolboxCommunicator;   
   private final PacketRouter<PacketDestination> networkProcessor;   
   private KinematicsToolboxModule kinematicsToolboxModule;
   
   
   
   
   
   public enum SolarPanelStates
   {
      GOTO_READYPOSE, CLEANING_LINEARPATH, CLEANING_MULTIPLEPATH
   }
   
   public SolarPanelBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("SolarPanelBehaviorStateMachine", SolarPanelStates.class, yoTime, communicationBridge);

      this.atlasPrimitiveActions = atlasPrimitiveActions;
      
      // ************************* Immigration ******************************* //
      networkProcessor = new PacketRouter<>(PacketDestination.class);
      toolboxCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      try
      {
         toolboxCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      networkProcessor.attachPacketCommunicator(PacketDestination.KINEMATICS_TOOLBOX_MODULE, toolboxCommunicator);
      
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, true);
      
      
      
      
      
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
      KinematicsToolboxController kinematicsToolBoxController = (KinematicsToolboxController) kinematicsToolboxModule.getToolboxController();
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
}
