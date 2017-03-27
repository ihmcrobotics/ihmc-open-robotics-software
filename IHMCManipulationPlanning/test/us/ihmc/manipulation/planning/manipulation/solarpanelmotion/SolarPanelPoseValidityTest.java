package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public class SolarPanelPoseValidityTest extends WholeBodyPoseValidityTester
{
   private SolarPanel solarPanel;
   
   public SolarPanelPoseValidityTest(SolarPanel solarPanel, KinematicsToolboxController ikToolboxController, WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      super(ikToolboxController, wholebodyTrajectoryMessage);
      this.solarPanel = solarPanel;
      
   }
   
   public void addCollisionShapeForSolarPanel()
   {
      
   }

   
}
