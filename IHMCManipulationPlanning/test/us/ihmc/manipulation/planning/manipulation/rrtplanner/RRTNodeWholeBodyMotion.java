package us.ihmc.manipulation.planning.manipulation.rrtplanner;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;

public class RRTNodeWholeBodyMotion extends RRTNode
{
   private SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   private CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();
   
   private SimpleCollisionShapeFactory shapeFactory;
   
   @Override
   public boolean isValidNode()
   {
      // TODO Auto-generated method stub
      return true;
   }

   @Override
   public RRTNode createNode()
   {
      // TODO Auto-generated method stub
      return new RRTNodeWholeBodyMotion();
   }

}
