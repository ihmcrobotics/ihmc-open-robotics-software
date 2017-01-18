package us.ihmc.SdfLoader;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class SDFRoadVisual extends Graphics3DObject
{
   public SDFRoadVisual(Road road)
   {
      double width = Double.parseDouble(road.getWidth());
      
      AppearanceDefinition appearance = YoAppearance.Texture("models/road1.jpg");
      
      Point3d startLeft = new Point3d();
      Point3d startRight = new Point3d();
      Point3d endLeft = new Point3d();
      Point3d endRight = new Point3d();
      for(int i = 0 ; i < road.getPoints().size() - 1; i++)
      {
         Point3d start = new Point3d(SDFConversionsHelper.stringToVector3d(road.getPoints().get(i)));
         Point3d end = new Point3d(SDFConversionsHelper.stringToVector3d(road.getPoints().get(i+1)));
         
         Vector3d direction = getDirection(start, end);
         
         
         Vector3d toSide = getPerpendicularVectorOfLength(width, direction);
         
         if(i == 0)
         {
            startLeft.add(start, toSide);
            startRight.sub(start, toSide);
         }
         else
         {
            startLeft.set(endLeft);
            startRight.set(endRight);
         }
         
         if(i < road.getPoints().size() - 2)
         {
            Point3d startNext = end;
            Point3d endNext = new Point3d(SDFConversionsHelper.stringToVector3d(road.getPoints().get(i+2)));
            Vector3d directionNext = getDirection(startNext, endNext);
            Vector3d nextSide = getPerpendicularVectorOfLength(width, directionNext);

            Vector2d toSide2d = new Vector2d(toSide.getX(), toSide.getY());
            Vector2d nextSide2d = new Vector2d(nextSide.getX(), nextSide.getY());
            double angle = GeometryTools.getAngleFromFirstToSecondVector(toSide2d, nextSide2d);
            
            RigidBodyTransform rotZ = new RigidBodyTransform();
            rotZ.setRotationYawAndZeroTranslation(angle/2.0);
            
            rotZ.transform(toSide);
            toSide.normalize();
            toSide.scale((width/2.0)/Math.cos(angle/2.0));
            
            
         }
         
         endLeft.add(end, toSide);
         endRight.sub(end, toSide);
         
         addPolygon(appearance, startRight, endRight, endLeft, startLeft);
         
      }
   }


   private Vector3d getPerpendicularVectorOfLength(double width, Vector3d direction)
   {
      Vector3d toSide = new Vector3d(-direction.getY(), direction.getX(), 0);
      toSide.normalize();
      toSide.scale(width/2.0);
      return toSide;
   }
   
   
   private Vector3d getDirection(Point3d a, Point3d b)
   {
      Vector3d direction = new Vector3d(b);
      direction.sub(a);
      direction.scale(0.5);
      return direction;
   }
   
}
