package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import us.ihmc.robotics.geometry.ConvexPolygon2d;

/**
 * Created by agrabertilton on 1/20/15.
 */
public class GenericFootstepSnappingParameters extends FootstepSnappingParameters
{
   public GenericFootstepSnappingParameters(){
      super(null, null, 0.3, Math.PI/4, 0.0, 0.01, 0.025, Math.PI/24, 1);
      collisionPolygon = new ConvexPolygon2d();
      collisionPolygon.addVertex(0.1, 0.05);
      collisionPolygon.addVertex(0.1, -0.05);
      collisionPolygon.addVertex(-0.05,-0.05);
      collisionPolygon.addVertex(-0.05, 0.05);
      collisionPolygon.update();

      supportPolygon = collisionPolygon;
   }

   @Override
   public double getMinArea()
   {
      return supportPolygon.getArea() * 0.01;
   }
}
