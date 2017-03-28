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
   
   private RobotCapsule rightUpperArm;
   private RobotCapsule rightLowerArm;
   
   private RobotCapsule leftUpperArm;
   private RobotCapsule leftLowerArm;
   
   private RobotCapsule rightUpperLeg;
   private RobotCapsule rightLowerLeg; 
   
   private RobotCapsule leftUpperLeg;
   private RobotCapsule leftLowerLeg;
   
   private RobotBox chestBody;
   private RobotBox pelvisBody;
   
   private RobotBox rightFoot;
   private RobotBox leftFoot;
   
   public RobotCollisionModel(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      
      this.shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      
   }
   
   public void getCollisionShape()
   {
      rightUpperArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL), fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.ELBOW_ROLL), 0.06);
      rightLowerArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.ELBOW_ROLL), fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.WRIST_ROLL), 0.06);
      
      leftUpperArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL), fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL), 0.06);
      leftLowerArm = new RobotCapsule(shapeFactory, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL), fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_ROLL), 0.06);
      
      rightUpperLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH), fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH), 0.06);
      rightLowerLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH), fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH), 0.06);
      
      leftUpperLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH), fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH), 0.06);
      leftLowerLeg = new RobotCapsule(shapeFactory, fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH), fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH), 0.06);
      
      Point3D translationChest = new Point3D(-0.03, 0, -0.12);
      chestBody = new RobotBox(shapeFactory, fullRobotModel.getChest().getBodyFixedFrame(), translationChest, 0.55, 0.38, 0.5);
      Point3D translationPelvis = new Point3D(-0.0, 0, -0.0);
      pelvisBody = new RobotBox(shapeFactory, fullRobotModel.getPelvis().getBodyFixedFrame(), translationPelvis, 0.35, 0.35, 0.2);
      
      PrintTools.info("CollisionShape Define Finished");
   }
   
   public void setCollisionMaskAndGroup()
   {
      
   }
   
   public void getCollisionResult()
   {
//    collisionDetectionResult.clear();
//    collisionDetector.performCollisionDetection(collisionDetectionResult);
//    
//    if(collisionDetectionResult.getNumberOfCollisions() > 0)
//    {
//       for (int i = 0; i<collisionDetectionResult.getNumberOfCollisions();i++)
//       {
//
//       }
//       isValid = false;
//    }  
   }
   
   public ArrayList<Graphics3DObject> getCollisionGraphics()
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
      
      ret.add(rightUpperArm.getGraphicObject());
      ret.add(rightLowerArm.getGraphicObject());
      ret.add(leftUpperArm.getGraphicObject());
      ret.add(leftLowerArm.getGraphicObject());
      
      ret.add(rightUpperLeg.getGraphicObject());
      ret.add(rightLowerLeg.getGraphicObject());
      ret.add(leftUpperLeg.getGraphicObject());
      ret.add(leftLowerLeg.getGraphicObject());
      
      ret.add(chestBody.getGraphicObject());
      ret.add(pelvisBody.getGraphicObject());

      return ret;
   }
   
}
