package us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.manipulation.planning.robotcollisionmodel.CollisionModelBox;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SolarPanelPoseValidityTester extends WholeBodyPoseValidityTester
{
   private SolarPanel solarPanel;
   
   public SolarPanelPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, CommunicationBridgeInterface outgoingCommunicationBridge, 
                                       FullHumanoidRobotModel fullRobotModel, SolarPanel solarPanel)
   {
      super(fullRobotModelFactory, outgoingCommunicationBridge, fullRobotModel);
      
      this.solarPanel = solarPanel;
      addEnvironmentCollisionModel();
      
   }

   @Override
   public void addEnvironmentCollisionModel()
   {
//      CollisionModelBox solarPanelCollisionModel;
//      solarPanelCollisionModel = new CollisionModelBox(getRobotCollisionModel().getCollisionShapeFactory(), solarPanel.getRigidBodyTransform(),
//                                                       solarPanel.getSizeX(), solarPanel.getSizeY(), solarPanel.getSizeZ());
//      solarPanelCollisionModel.getCollisionShape().setCollisionGroup(0b11111111111111);
//      solarPanelCollisionModel.getCollisionShape().setCollisionMask(0b11111111111111);
   }

}
