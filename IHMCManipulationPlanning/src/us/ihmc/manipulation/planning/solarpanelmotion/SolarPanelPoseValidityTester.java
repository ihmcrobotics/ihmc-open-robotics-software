package us.ihmc.manipulation.planning.solarpanelmotion;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.robotcollisionmodel.CollisionModelBox;
import us.ihmc.tools.thread.ThreadTools;

public class SolarPanelPoseValidityTester extends WholeBodyPoseValidityTester
{
   private SolarPanel solarPanel;
   private PacketCommunicator toolboxCommunicator;
   

   public SolarPanelPoseValidityTester(SolarPanel solarPanel, PacketCommunicator toolboxCommunicator, KinematicsToolboxController ikToolboxController)
   {
      super(ikToolboxController);
      this.solarPanel = solarPanel;
      this.toolboxCommunicator = toolboxCommunicator;
      addEnvironmentCollisionModel();
   }
   
   public void sendWholebodyTrajectoryMessage(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      wholebodyTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);      
      toolboxCommunicator.send(wholebodyTrajectoryMessage);
      ThreadTools.sleep(50);
   }

   public boolean isValidWholeBodyPose(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      sendWholebodyTrajectoryMessage(wholebodyTrajectoryMessage);
      // wait      
      return isValid();   
   }
      
   public int cnt = 0;
   public void setSolarPanelWholeBodyPose(SolarPanelPath cleaningPath, double tempNodeData0, double tempNodeData1)
   {
      // *********************************************************************** wholebody ***********************************************************************          
      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = cleaningPath.getWholeBodyMessageForValidityTest(-Math.PI*0.2, tempNodeData1, 0.0, tempNodeData0);      
      
      sendWholebodyTrajectoryMessage(wholebodyTrajectoryMessage);
      for(int i =0;i<50;i++)
      {
         isValid = false;
         if(ikToolboxController.getSolution().solutionQuality < 0.015)
         {            
            isValid = true;
            break;
         }         
         ThreadTools.sleep(5);         
      }
      cnt++;
      
      // *********************************************************************** temp ***********************************************************************
//      if(tempNodeData0 > 1.5 && tempNodeData0 < 2.5 && tempNodeData1 < Math.PI*0.1 && tempNodeData1 > -Math.PI*0.1)
//      {
//         isValid = false;         
//      }
//      else
//      {
//         isValid = true;         
//      }      
   }

   @Override
   public void addEnvironmentCollisionModel()
   {
      CollisionModelBox solarPanelCollisionModel;
      solarPanelCollisionModel = new CollisionModelBox(getRobotCollisionModel().getCollisionShapeFactory(), solarPanel.getRigidBodyTransform(),
                                                       solarPanel.getSizeX(), solarPanel.getSizeY(), solarPanel.getSizeZ());
      solarPanelCollisionModel.getCollisionShape().setCollisionGroup(0b11111111111111);
      solarPanelCollisionModel.getCollisionShape().setCollisionMask(0b11111111111111);
   }

}
