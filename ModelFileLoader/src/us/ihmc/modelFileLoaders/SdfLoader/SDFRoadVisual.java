package us.ihmc.modelFileLoaders.SdfLoader;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.robotics.geometry.GeometryTools;

public class SDFRoadVisual extends Graphics3DObject
{
   public SDFRoadVisual(Road road)
   {
      double width = Double.parseDouble(road.getWidth());
      
      AppearanceDefinition appearance = YoAppearance.Texture("models/road1.jpg");
      
      Point3D startLeft = new Point3D();
      Point3D startRight = new Point3D();
      Point3D endLeft = new Point3D();
      Point3D endRight = new Point3D();
      for(int i = 0 ; i < road.getPoints().size() - 1; i++)
      {
         Point3D start = new Point3D(ModelFileLoaderConversionsHelper.stringToVector3d(road.getPoints().get(i)));
         Point3D end = new Point3D(ModelFileLoaderConversionsHelper.stringToVector3d(road.getPoints().get(i+1)));
         
         Vector3D direction = getDirection(start, end);
         
         
         Vector3D toSide = getPerpendicularVectorOfLength(width, direction);
         
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
            Point3D startNext = end;
            Point3D endNext = new Point3D(ModelFileLoaderConversionsHelper.stringToVector3d(road.getPoints().get(i+2)));
            Vector3D directionNext = getDirection(startNext, endNext);
            Vector3D nextSide = getPerpendicularVectorOfLength(width, directionNext);

            Vector2D toSide2d = new Vector2D(toSide.getX(), toSide.getY());
            Vector2D nextSide2d = new Vector2D(nextSide.getX(), nextSide.getY());
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


   private Vector3D getPerpendicularVectorOfLength(double width, Vector3D direction)
   {
      Vector3D toSide = new Vector3D(-direction.getY(), direction.getX(), 0);
      toSide.normalize();
      toSide.scale(width/2.0);
      return toSide;
   }
   
   
   private Vector3D getDirection(Point3D a, Point3D b)
   {
      Vector3D direction = new Vector3D(b);
      direction.sub(a);
      direction.scale(0.5);
      return direction;
   }
   
}
