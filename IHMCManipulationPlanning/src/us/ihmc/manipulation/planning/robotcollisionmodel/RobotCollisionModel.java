package us.ihmc.manipulation.planning.robotcollisionmodel;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;

public class RobotCollisionModel
{
   private FullHumanoidRobotModel fullRobotModel;
   
   private SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   private CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   private SimpleCollisionShapeFactory shapeFactory;
   
   private RobotBox chestBody;
   private RobotBox pelvisBody;
   
   private RobotCapsule rightUpperArm;
   private RobotCapsule rightLowerArm;
   private RobotBox rightHand;
   
   private RobotCapsule leftUpperArm;
   private RobotCapsule leftLowerArm;
   private RobotBox leftHand;
   
   private RobotCapsule rightUpperLeg;
   private RobotCapsule rightLowerLeg; 
   private RobotBox rightFoot;
   
   private RobotCapsule leftUpperLeg;
   private RobotCapsule leftLowerLeg;
   private RobotBox leftFoot;
   

   
   public RobotCollisionModel(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      
      this.shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      
   }
   
   public void getCollisionShape()
   {
      Point3D translationChest = new Point3D(-0.03, 0, -0.12);
      chestBody = new RobotBox(shapeFactory, fullRobotModel.getChest().getBodyFixedFrame(), translationChest, 0.55, 0.38, 0.5);
      Point3D translationPelvis = new Point3D(-0.0, 0, -0.0);
      pelvisBody = new RobotBox(shapeFactory, fullRobotModel.getPelvis().getBodyFixedFrame(), translationPelvis, 0.35, 0.35, 0.2);
      
      rightUpperArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL), fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.ELBOW_ROLL), 0.06);
      rightLowerArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.ELBOW_ROLL), fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.WRIST_ROLL), 0.06);      
      Point3D translationRHand = new Point3D(-0.0, 0.03, -0.0);
      rightHand = new RobotBox(shapeFactory, fullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame(), translationRHand, 0.1, 0.10, 0.1);
      
      leftUpperArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL), fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL), 0.06);
      leftLowerArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL), fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_ROLL), 0.06);      
      Point3D translationLHand = new Point3D(-0.0, 0.03, -0.0);
      leftHand = new RobotBox(shapeFactory, fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame(), translationLHand, 0.1, 0.1, 0.1);
      
      rightUpperLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH), fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH), 0.06);
      rightLowerLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH), fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH), 0.06);
      Point3D translationRFoot = new Point3D(0.02, 0, 0.01);
      rightFoot = new RobotBox(shapeFactory, fullRobotModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame(), translationRFoot, 0.26, 0.135, 0.06);
      
      leftUpperLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH), fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH), 0.06);
      leftLowerLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH), fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH), 0.06);
      Point3D translationLFoot = new Point3D(0.02, 0, 0.01);
      leftFoot = new RobotBox(shapeFactory, fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame(), translationLFoot, 0.26, 0.135, 0.06);      
      
      PrintTools.info("CollisionShape Define Finished");
   }
   
   public void setCollisionMaskAndGroup()
   {
      PrintTools.info("setCollisionMaskAndGroup Started");
      
      chestBody.getCollisionShape().setCollisionMask        (0b00000000000001);
      pelvisBody.getCollisionShape().setCollisionMask       (0b00000000000010);
      rightUpperArm.getCollisionShape().setCollisionMask    (0b00000000000100);
      rightLowerArm.getCollisionShape().setCollisionMask    (0b00000000001000);
      rightHand.getCollisionShape().setCollisionMask        (0b00000000010000);
      leftUpperArm.getCollisionShape().setCollisionMask     (0b00000000100000);
      leftLowerArm.getCollisionShape().setCollisionMask     (0b00000001000000);
      leftHand.getCollisionShape().setCollisionMask         (0b00000010000000);
      rightUpperLeg.getCollisionShape().setCollisionMask    (0b00000100000000);
      rightLowerLeg.getCollisionShape().setCollisionMask    (0b00001000000000);
      rightFoot.getCollisionShape().setCollisionMask        (0b00010000000000);
      leftUpperLeg.getCollisionShape().setCollisionMask     (0b00100000000000);
      leftLowerLeg.getCollisionShape().setCollisionMask     (0b01000000000000);
      leftFoot.getCollisionShape().setCollisionMask         (0b10000000000000);
      
      chestBody.getCollisionShape().setCollisionGroup       (0b00000011011000);  // R lower arm, R hand, L lower arm, L hand                       3 4 6 7
      pelvisBody.getCollisionShape().setCollisionGroup      (0b00000011011000);  // R lower arm, R hand, L lower arm, L hand                       3 4 6 7
      rightUpperArm.getCollisionShape().setCollisionGroup   (0b00000011000000);  // L lower arm, L hand                                            6 7
      rightLowerArm.getCollisionShape().setCollisionGroup   (0b00000011100011);  // L lower arm, L hand, chest, pelvis , L upper arm               6 7 0 1 5
      rightHand.getCollisionShape().setCollisionGroup       (0b00000111100011);  // L lower arm, L hand, chest, pelvis, R upper Leg, L upper arm   6 7 0 1 8 5
      leftUpperArm.getCollisionShape().setCollisionGroup    (0b00000000011000);  // R lower arm, R hand                                            3 4
      leftLowerArm.getCollisionShape().setCollisionGroup    (0b00000000011111);  // R lower arm, R hand, chest, pelvis , R upper arm               3 4 0 1 2
      leftHand.getCollisionShape().setCollisionGroup        (0b00100000011111);  // R lower arm, R hand, chest, pelvis, L upper Leg, R upper arm   3 4 0 1 11 2
      
      rightUpperLeg.getCollisionShape().setCollisionGroup   (0b11100000010000);  // R hand, L upper leg, L lower leg, L foot                       4 11 12 13
      rightLowerLeg.getCollisionShape().setCollisionGroup   (0b11100000000000);  // L upper leg, L lower leg, L foot                               11 12 13
      rightFoot.getCollisionShape().setCollisionGroup       (0b11100000000000);  // L upper leg, L lower leg, L foot                               11 12 13
      leftUpperLeg.getCollisionShape().setCollisionGroup    (0b00011110000000);  // L hand, R upper leg, R lower leg, R foot                       7 8 9 10
      leftLowerLeg.getCollisionShape().setCollisionGroup    (0b00011100000000);  // R upper leg, R lower leg, R foot                               8 9 10
      leftFoot.getCollisionShape().setCollisionGroup        (0b00011100000000);  // R upper leg, R lower leg, R foot                               8 9 10
      
      
      
      
      
      PrintTools.info("chestBody Mask " + chestBody.getCollisionShape().getCollisionMask() +" Group "+chestBody.getCollisionShape().getCollisionGroup());
      PrintTools.info("pelvisBody Mask " + pelvisBody.getCollisionShape().getCollisionMask() +" Group "+pelvisBody.getCollisionShape().getCollisionGroup());
      PrintTools.info("rightUpperArm Mask " + rightUpperArm.getCollisionShape().getCollisionMask() +" Group "+rightUpperArm.getCollisionShape().getCollisionGroup());
      PrintTools.info("rightLowerArm Mask " + rightLowerArm.getCollisionShape().getCollisionMask() +" Group "+rightLowerArm.getCollisionShape().getCollisionGroup());
      PrintTools.info("leftUpperArm Mask " + leftUpperArm.getCollisionShape().getCollisionMask() +" Group "+leftUpperArm.getCollisionShape().getCollisionGroup());
      PrintTools.info("leftLowerArm Mask " + leftLowerArm.getCollisionShape().getCollisionMask() +" Group "+leftLowerArm.getCollisionShape().getCollisionGroup());
      PrintTools.info("rightUpperLeg Mask " + rightUpperLeg.getCollisionShape().getCollisionMask() +" Group "+rightUpperLeg.getCollisionShape().getCollisionGroup());
      PrintTools.info("rightLowerLeg Mask " + rightLowerLeg.getCollisionShape().getCollisionMask() +" Group "+rightLowerLeg.getCollisionShape().getCollisionGroup());
      PrintTools.info("leftUpperLeg Mask " + leftUpperLeg.getCollisionShape().getCollisionMask() +" Group "+leftUpperLeg.getCollisionShape().getCollisionGroup());
      PrintTools.info("leftLowerLeg Mask " + leftLowerLeg.getCollisionShape().getCollisionMask() +" Group "+leftLowerLeg.getCollisionShape().getCollisionGroup());
      
      PrintTools.info("setCollisionMaskAndGroup Finished");
   }
   
   public void update()
   {
      PrintTools.info("update Start");
      chestBody.updateRighdBodyTransform();
      pelvisBody.updateRighdBodyTransform();
      
      rightUpperArm.updateRighdBodyTransform();
      rightLowerArm.updateRighdBodyTransform();
      rightHand.updateRighdBodyTransform();
      
      leftUpperArm.updateRighdBodyTransform();
      leftLowerArm.updateRighdBodyTransform();
      leftHand.updateRighdBodyTransform();
      
      rightUpperLeg.updateRighdBodyTransform();
      rightLowerLeg.updateRighdBodyTransform();
      rightFoot.updateRighdBodyTransform();
      
      leftUpperLeg.updateRighdBodyTransform();
      leftLowerLeg.updateRighdBodyTransform();
      leftFoot.updateRighdBodyTransform();
      
      chestBody.updateCollisionShape();
      pelvisBody.updateCollisionShape();
      
      rightUpperArm.updateCollisionShape();
      rightLowerArm.updateCollisionShape();
      rightHand.updateCollisionShape();
      
      leftUpperArm.updateCollisionShape();
      leftLowerArm.updateCollisionShape();
      leftHand.updateCollisionShape();
      
      rightUpperLeg.updateCollisionShape();
      rightLowerLeg.updateCollisionShape();
      rightFoot.updateCollisionShape();
      
      leftUpperLeg.updateCollisionShape();
      leftLowerLeg.updateCollisionShape();
      leftFoot.updateCollisionShape();
      
      PrintTools.info("update Finished");
   }
   
   public void getCollisionResult()
   {       
      collisionDetectionResult.clear();
      collisionDetector.performCollisionDetection(collisionDetectionResult);
      
      
      
      
      if(collisionDetectionResult.getNumberOfCollisions() > 0)
      {
         for (int i = 0; i<collisionDetectionResult.getNumberOfCollisions();i++)
         {
            PrintTools.info("collid A "+collisionDetectionResult.getCollision(i).getShapeA().getCollisionMask()+ " collid B "+ collisionDetectionResult.getCollision(i).getShapeB().getCollisionMask());
            
         }
         PrintTools.info("Collid! "+collisionDetectionResult.getNumberOfCollisions());
      }
   }
   
   public ArrayList<Graphics3DObject> getCollisionGraphics()
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
      
      ret.add(chestBody.getGraphicObject());
      ret.add(pelvisBody.getGraphicObject());
      
      ret.add(rightUpperArm.getGraphicObject());
      ret.add(rightLowerArm.getGraphicObject());
      ret.add(rightHand.getGraphicObject());
      ret.add(leftUpperArm.getGraphicObject());
      ret.add(leftLowerArm.getGraphicObject());
      ret.add(leftHand.getGraphicObject());
      
      ret.add(rightUpperLeg.getGraphicObject());
      ret.add(rightLowerLeg.getGraphicObject());
      ret.add(rightFoot.getGraphicObject());
      ret.add(leftUpperLeg.getGraphicObject());
      ret.add(leftLowerLeg.getGraphicObject());
      ret.add(leftFoot.getGraphicObject());
      
      return ret;
   }
   
}
