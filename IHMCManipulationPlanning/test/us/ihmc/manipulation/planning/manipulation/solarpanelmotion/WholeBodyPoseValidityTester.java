package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public abstract class WholeBodyPoseValidityTester implements ValidityTester
{
   private KinematicsToolboxController ikToolboxController;
   public FullHumanoidRobotModel ikFullRobotModel;
   public RobotCollisionModel robotCollisionModel;
   
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   
   private boolean isValid = false;
   
   
   
   public ArrayList<RigidBodyTransform> debugAxis = new ArrayList<RigidBodyTransform>();
      
 
   public WholeBodyPoseValidityTester(KinematicsToolboxController ikToolboxController, WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.ikToolboxController = ikToolboxController;
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
      this.ikFullRobotModel = this.ikToolboxController.getDesiredFullRobotModel();
      
      this.robotCollisionModel = new RobotCollisionModel(this.ikFullRobotModel);
   }
   

   
   public abstract void getEnvironmentCollisionShape();   

   public void getInverseKienamticsModule()
   {
      ikToolboxController.updateTools();
   }
   
   public void getCollisionShape()
   {
      robotCollisionModel.getCollisionShape();
      
      
      
      
   }   
   
   public void getCollisionResult()
   {
      isValid = true;
   }   
   
   public void update()
   {
      
   }
   
   @Override
   public boolean isValid()
   {
      return isValid;
   }   

   
   
   
   
   
  
   
}
