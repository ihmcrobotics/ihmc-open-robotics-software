package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.robotcollisionmodel.CollisionModelBox;

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
      toolboxCommunicator.send(wholebodyTrajectoryMessage);
   }

   public boolean isValidWholeBodyPose(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      sendWholebodyTrajectoryMessage(wholebodyTrajectoryMessage);
      // wait      
      return isValid();   
   }
   
   public void setSolarPanelWholeBodyPose(double tempNodeData0, double tempNodeData1)
   {      
      if(tempNodeData0 > 1.5 && tempNodeData0 < 2.5 && tempNodeData1 < Math.PI*0.1 && tempNodeData1 > -Math.PI*0.1)
      {
         isValid = false;         
      }
      else
      {
         isValid = true;         
      }      
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
