package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;

public class WholeBodyPoseValidityTester implements ValidityTester
{
   private KinematicsToolboxController ikToolboxController;
   private FullHumanoidRobotModel ikFullRobotModel;
   
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   
   private boolean isValid = false;
   
   private SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   private CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   private SimpleCollisionShapeFactory shapeFactorRobot;
   
   private CollisionShape csRightShoulderYawArm;
   private CollisionShape csRightUpperArm;
   private CollisionShape csRightLowerArm;
   private CollisionShape csRightEndArm;
   private CollisionShape csRightHand;
   
   private CollisionShape csRightUpperLeg;
   private CollisionShape csRightLowerLeg;
   private CollisionShape csRightFoot;
   
   private CollisionShape csLeftShoulderYawArm;
   private CollisionShape csLeftUpperArm;
   private CollisionShape csLeftLowerArm;
   private CollisionShape csLeftEndArm;
   private CollisionShape csLeftHand;
   
   private CollisionShape csLeftUpperLeg;
   private CollisionShape csLeftLowerLeg;
   private CollisionShape csLeftFoot;
   
   private CollisionShape csHead;
   private CollisionShape csBody;
   private CollisionShape csPelvis;
   
   
   
   public WholeBodyPoseValidityTester(KinematicsToolboxController ikToolboxController, WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      PrintTools.info("!");
      this.ikToolboxController = ikToolboxController;
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
      this.ikFullRobotModel = this.ikToolboxController.getDesiredFullRobotModel();
      
      getInverseKienamticsModule();
      getCollisionShape();
      getCollisionResult();
   }

   public void getInverseKienamticsModule()
   {
      
   }
   
   public void getCollisionShape()
   {
      shapeFactorRobot = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      
      
      // addBox make box from center
      // addCapsule has maximul length in endof both half-sphere
      // addCylinder make cylinder from center

      RigidBodyTransform transformToWorldFrame;
      transformToWorldFrame = ikFullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL).getSuccessor().getBodyFixedFrame().getTransformToWorldFrame();
      PrintTools.info(""+transformToWorldFrame.getM03()+" "+transformToWorldFrame.getM13()+" "+transformToWorldFrame.getM23());
      
      transformToWorldFrame = ikFullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL).getPredecessor().getBodyFixedFrame().getTransformToWorldFrame();
      PrintTools.info(""+transformToWorldFrame.getM03()+" "+transformToWorldFrame.getM13()+" "+transformToWorldFrame.getM23());
      
      CollisionShapeDescription<?> capsuleRightShoulderYawArm = shapeFactorRobot.createCapsule(0.1, 0.15);
      CollisionShapeDescription<?> capsuleRightUpperArm = shapeFactorRobot.createCapsule(0.1, 0.15);
      CollisionShapeDescription<?> capsuleRightLowerArm = shapeFactorRobot.createCapsule(0.1, 0.15);
      CollisionShapeDescription<?> capsuleRightHand = shapeFactorRobot.createCapsule(0.1, 0.15);
      
      CollisionShapeDescription<?> capsuleRightUpperLeg = shapeFactorRobot.createCapsule(0.1, 0.15);
      CollisionShapeDescription<?> capsuleRightLowerLeg = shapeFactorRobot.createCapsule(0.1, 0.15);
      CollisionShapeDescription<?> boxRightFoot = shapeFactorRobot.createCapsule(0.1, 0.15);
      
//      CollisionShapeDescription<?> boxHead = shapeFactorRobot.createBox(halfLengthX, halfWidthY, halfHeightZ);
//      CollisionShapeDescription<?> boxBody = shapeFactorRobot.createBox(halfLengthX, halfWidthY, halfHeightZ);
//      CollisionShapeDescription<?> cylinderPelvis = shapeFactorRobot.createCylinder(radius, height);
      
   }
   
   public void getCollisionResult()
   {
      collisionDetectionResult.clear();
      collisionDetector.performCollisionDetection(collisionDetectionResult);
      
      if(collisionDetectionResult.getNumberOfCollisions() > 0)
      {
         for (int i = 0; i<collisionDetectionResult.getNumberOfCollisions();i++)
         {
//            if(collisionDetectionResult.getCollision(i).getShapeA().getCollisionMask() == collisionShapeUpperArm.getCollisionMask())
//               PrintTools.info("upper Arm is collided with "+collisionDetectionResult.getCollision(i).getShapeB().getCollisionMask());
         }
         isValid = false;
      }     

      isValid = true;
   }
   
   
   
   @Override
   public boolean isValid()
   {
      return isValid;
   }

}
