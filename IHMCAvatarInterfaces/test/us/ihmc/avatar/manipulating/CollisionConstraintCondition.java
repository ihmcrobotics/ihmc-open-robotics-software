package us.ihmc.avatar.manipulating;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.CapsuleShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.tools.io.printing.PrintTools;

public class CollisionConstraintCondition extends ConstraintCondition
{  
   GhostRobotArm ghostRobot;
   
   SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   SimpleCollisionShapeFactory shapeFactorRobot;
   
   double XsizeTable = 0.9;
   double YsizeTable = 1.2;
   double ZsizeTable = 0.05;
   
   double heightOfTableColumn = 0.8;
   double radiusOfTableColumn = 0.075;
   
   Point3d translationTableColumn = new Point3d(0.8, 0.0, 0);
   
   private Point3d targetPosition = new Point3d(0.8, -0.3, 0);
   private double targetRadius = 0.05;
   
   private double heightOfTargetColumn = 0.25;
   
   private Point3d obstaclePosition = new Point3d(0.5, -0.7, 1.0);
   private double radiusOfObstacle = 0.05;
      
   CollisionShape collisionShapeUpperArm;
   CollisionShape collisionShapeLowerArm;
   CollisionShape collisionShapeEndArm;
   


   
   public CollisionConstraintCondition(GhostRobotArm ghostRobotArm)
   {
      this.ghostRobot = ghostRobotArm;
      
      shapeFactorRobot = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
                     
      CollisionShapeDescription<?> upperCapsule = shapeFactorRobot.createCapsule(ghostRobotArm.upperArmCapsule.getRadius(), ghostRobotArm.upperArmCapsule.getHeight());
      CollisionShapeDescription<?> lowerCapsule = shapeFactorRobot.createCapsule(ghostRobotArm.lowerArmCapsule.getRadius(), ghostRobotArm.lowerArmCapsule.getHeight());
      CollisionShapeDescription<?> endCapsule = shapeFactorRobot.createCapsule(ghostRobotArm.endArmCapsule.getRadius(), ghostRobotArm.endArmCapsule.getHeight());
      
      collisionShapeUpperArm = shapeFactorRobot.addShape(upperCapsule);       
      collisionShapeLowerArm = shapeFactorRobot.addShape(lowerCapsule);
      collisionShapeEndArm = shapeFactorRobot.addShape(endCapsule);
     
      
      collisionShapeUpperArm.setCollisionMask(0b100000);
      collisionShapeUpperArm.setCollisionGroup(0b000111);
      collisionShapeLowerArm.setCollisionMask(0b010000);
      collisionShapeLowerArm.setCollisionGroup(0b000111);
      collisionShapeEndArm.setCollisionMask(0b001000);
      collisionShapeEndArm.setCollisionGroup(0b000111);
           
      CollisionShapeDescription<?> tableCylinder = shapeFactorRobot.createCylinder(radiusOfTableColumn, heightOfTableColumn);
      CollisionShapeDescription<?> tableBox = shapeFactorRobot.createBox(XsizeTable/2, YsizeTable/2, ZsizeTable/2);      
      CollisionShapeDescription<?> obstacleSphere = shapeFactorRobot.createSphere(radiusOfObstacle);
      
      CollisionShape collisionShapeTableCylinder = shapeFactorRobot.addShape(tableCylinder);
      CollisionShape collisionShapeTableBox = shapeFactorRobot.addShape(tableBox);
      CollisionShape collisionShapeObstacleSphere = shapeFactorRobot.addShape(obstacleSphere);      

      RigidBodyTransform transformTableCylinder = new RigidBodyTransform();
      RigidBodyTransform transformTableBox = new RigidBodyTransform();
      RigidBodyTransform transformObstacleSphere = new RigidBodyTransform();
      
      transformTableCylinder.setTranslation(translationTableColumn);
      transformTableBox.setTranslation(translationTableColumn.x,translationTableColumn.y,heightOfTableColumn);
      transformObstacleSphere.setTranslation(obstaclePosition);
      
      collisionShapeTableCylinder.setTransformToWorld(transformTableCylinder);
      collisionShapeTableBox.setTransformToWorld(transformTableBox);
      collisionShapeObstacleSphere.setTransformToWorld(transformObstacleSphere);

      collisionShapeTableCylinder.setCollisionMask(0b000100);
      collisionShapeTableBox.setCollisionMask(0b000010);
      collisionShapeObstacleSphere.setCollisionMask(0b000001);
      collisionShapeTableCylinder.setCollisionGroup(0b111000);
      collisionShapeTableBox.setCollisionGroup(0b111000);
      collisionShapeObstacleSphere.setCollisionGroup(0b111000);
      
      
   }

   @Override
   public void updateCurrentState()
   {
      collisionShapeUpperArm.setTransformToWorld(ghostRobot.upperArmCapsule.getTransform());      
      collisionShapeLowerArm.setTransformToWorld(ghostRobot.lowerArmCapsule.getTransform());      
      collisionShapeEndArm.setTransformToWorld(ghostRobot.endArmCapsule.getTransform());
   }

   @Override
   public boolean getResult()
   {      
      collisionDetectionResult.clear();
      collisionDetector.performCollisionDetection(collisionDetectionResult);
      
      if(collisionDetectionResult.getNumberOfCollisions() > 0)
      {
         for (int i = 0; i<collisionDetectionResult.getNumberOfCollisions();i++)
         {
//            if(collisionDetectionResult.getCollision(i).getShapeA().getCollisionMask() == collisionShapeUpperArm.getCollisionMask())
//               PrintTools.info("upper Arm is collided with "+collisionDetectionResult.getCollision(i).getShapeB().getCollisionMask());     
//            if(collisionDetectionResult.getCollision(i).getShapeA().getCollisionMask() == collisionShapeLowerArm.getCollisionMask())
//               PrintTools.info("lower Arm is collided with "+collisionDetectionResult.getCollision(i).getShapeB().getCollisionMask());     
//            if(collisionDetectionResult.getCollision(i).getShapeA().getCollisionMask() == collisionShapeEndArm.getCollisionMask())
//               PrintTools.info("end Arm is collided with "+collisionDetectionResult.getCollision(i).getShapeB().getCollisionMask());            
         }
         return false;
      }     

      return true;
   }
   
   public ArrayList<Graphics3DObject> getEnvironment()
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject tableColumn = new Graphics3DObject();
      Graphics3DObject tablePlate = new Graphics3DObject();
      Graphics3DObject target = new Graphics3DObject();
      Graphics3DObject obstacleOne = new Graphics3DObject();
      
      tableColumn.translate(translationTableColumn);
      tableColumn.addCylinder(heightOfTableColumn, radiusOfTableColumn, YoAppearance.Black());
      
      tablePlate.translate(translationTableColumn.x,translationTableColumn.y,heightOfTableColumn);
      tablePlate.addCube(XsizeTable, YsizeTable, ZsizeTable, YoAppearance.Wheat());
            
      Point3d translationTargetColumn = new Point3d(targetPosition.x, targetPosition.y, heightOfTableColumn + ZsizeTable);
      target.translate(translationTargetColumn);
      target.addCylinder(heightOfTargetColumn, targetRadius, YoAppearance.Blue());
              
      obstacleOne.translate(obstaclePosition);            
      obstacleOne.addSphere(radiusOfObstacle, YoAppearance.Red());
      
      ret.add(tableColumn);
      ret.add(tablePlate);
      ret.add(target);
      ret.add(obstacleOne);

      return ret;
   }

   
}
