package us.ihmc.manipulation.planning.walkingpath;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.tools.io.printing.PrintTools;

public class RRT2DNodeWalkingPath extends RRTNode
{
   private SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   private CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();
   
   SimpleCollisionShapeFactory shapeFactory;
   public static BoxInfo[] boxes;
   
   public RRT2DNodeWalkingPath()
   {
      super(2);
      super.setNodeData(0, 0);
      super.setNodeData(1, 0);
      
      setUpCollisionDetector();
   }
   
   public RRT2DNodeWalkingPath(double px, double py)
   {
      super(2);
      super.setNodeData(0, px);
      super.setNodeData(1, py);
      
      setUpCollisionDetector();
   }
      
   @Override
   public boolean isValidNode()
   {
      Point3D translationOfNode = new Point3D(getNodeData(0), getNodeData(1), 0.0);
      
      RigidBodyTransform transform;
      transform = new RigidBodyTransform();
      transform.setTranslation(translationOfNode);
           
      
      collisionDetector.getCollisionObjects().get(0).setTransformToWorld(transform);
      collisionDetectionResult.clear();
      collisionDetector.performCollisionDetection(collisionDetectionResult);
      
      if (collisionDetectionResult.getNumberOfCollisions() > 0)
      {
         return false;
      }
      
      return true;
   }

   @Override
   public RRTNode createNode()
   {
      return  new RRT2DNodeWalkingPath();
   }

   private void setUpCollisionDetector()
   {            
      boxes = new BoxInfo[5];
      boxes[0] = new BoxInfo(new Point3D(1.5, -0.5, 0), new double[]{0.2, 1.0, 0.5});
      boxes[1] = new BoxInfo(new Point3D(1.0, -2.0, 0), new double[]{0.5, 0.5, 0.5});
      boxes[2] = new BoxInfo(new Point3D(4.0,  2.0, 0), new double[]{1.0, 1.0, 0.5});
      boxes[3] = new BoxInfo(new Point3D(4.5, -1.5, 0), new double[]{1.8, 1.0, 0.5});
      boxes[4] = new BoxInfo(new Point3D(3.5, -5.5, 0), new double[]{1.5, 1.5, 0.5});  
            
      shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      
      shapeFactory.addShape(shapeFactory.createCapsule(0.4, 2.0));
            
      for (int i =0;i<boxes.length;i++)
      {
         shapeFactory.addShape(shapeFactory.createBox(boxes[i].sizeX/2, boxes[i].sizeY/2, boxes[i].sizeZ/2));
         
         RigidBodyTransform transform;
         transform = new RigidBodyTransform();
         transform.setTranslation(boxes[i].center);
         collisionDetector.getCollisionObjects().get(i+1).setTransformToWorld(transform);
      }      
   }
   
   public static class BoxInfo
   {
      public double centerX;
      public double centerY;
      public double sizeX;
      public double sizeY;
      public double sizeZ;
      public Point3D center;

      public BoxInfo(Point3D center, double[] size)
      {
         this.center = center;
         centerX = center.getX();
         centerY = center.getY();
         
         sizeX = size[0];
         sizeY = size[1];
         sizeZ = size[2];
      }
   }
}
