package us.ihmc.manipulation.planning.solarpanelmotion;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelPath;
import us.ihmc.manipulation.planning.robotcollisionmodel.CollisionModelBox;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
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
   }

   public boolean isValidWholeBodyPose(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      sendWholebodyTrajectoryMessage(wholebodyTrajectoryMessage);
      // wait      
      return isValid();   
   }
   
   public void setSolarPanelWholeBodyPose(SolarPanelPath cleaningPath, double tempNodeData0, double tempNodeData1)
   {
      // *********************************************************************** wholebody ***********************************************************************
//      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
//      
//      Point3D desHandPosition = new Point3D(0.5, -0.35, 1.0);
//      Quaternion desHandOrientation = new Quaternion();
//      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2.0, desHandPosition, desHandOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
//      
//      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
//      
//      sendWholebodyTrajectoryMessage(wholebodyTrajectoryMessage);
      
      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = cleaningPath.getWholeBodyMessageForValidityTest(-Math.PI*0.2, tempNodeData1, 0.0, tempNodeData0);      
      
      sendWholebodyTrajectoryMessage(wholebodyTrajectoryMessage);
      for(int i =0;i<100;i++)
      {
         isValid = false;
         if(ikToolboxController.getSolution().solutionQuality < 0.01)
         {            
            isValid = true;
            break;
         }         
         ThreadTools.sleep(100);         
      }
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
