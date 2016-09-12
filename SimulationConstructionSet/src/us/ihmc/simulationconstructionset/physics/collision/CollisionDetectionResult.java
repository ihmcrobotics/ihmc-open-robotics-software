package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.simulationconstructionset.physics.CollisionShape;

public class CollisionDetectionResult
{
   private final ArrayList<DetectedCollision> results = new ArrayList<>();

   public void addContact(CollisionShape shapeA, CollisionShape shapeB, Point3d pointOnA, Point3d pointOnB)
   {
      DetectedCollision result = new DetectedCollision(shapeA, shapeB, pointOnA, pointOnB);
      results.add(result);
   }




}
