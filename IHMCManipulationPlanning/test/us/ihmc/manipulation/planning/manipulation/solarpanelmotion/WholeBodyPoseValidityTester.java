package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public abstract class WholeBodyPoseValidityTester
{
   private KinematicsToolboxController ikToolboxController;
   public FullHumanoidRobotModel ikFullRobotModel;
   public RobotCollisionModel robotCollisionModel;
   
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   
   private boolean isValid = true;
   private boolean collisionFree = true;
   private boolean jointlimitFree = true;

   
   
   
   public ArrayList<RigidBodyTransform> debugAxis = new ArrayList<RigidBodyTransform>();      
 
   public WholeBodyPoseValidityTester(KinematicsToolboxController ikToolboxController, WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.ikToolboxController = ikToolboxController;
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
      this.ikFullRobotModel = this.ikToolboxController.getDesiredFullRobotModel();
      
      this.robotCollisionModel = new RobotCollisionModel(this.ikFullRobotModel);
      
      robotCollisionModel.getCollisionShape();      
      robotCollisionModel.setCollisionMaskAndGroup();
   }
   

   
   public abstract void getEnvironmentCollisionShape();   

   public void getInverseKienamticsModule()
   {
      ikToolboxController.updateTools();
   }
   
   
   public void getResult()
   {
      robotCollisionModel.update();
      
      robotCollisionModel.getCollisionResult();
      
      // joint limit tester
      // collision tester
   }   
   
   public void update()
   {
      
   }
   
   public boolean isValid()
   {
      if(collisionFree == false || jointlimitFree == false)
      {
         isValid = false;
      }
      else
      {
         isValid = true;
      }
      return isValid;
   }   

   
   
   
   
   
  
   
}
