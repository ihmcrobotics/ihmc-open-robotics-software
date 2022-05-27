package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.simulation.environment.object.GDXSimpleObject;

import java.util.ArrayList;
import java.util.HashMap;

public class GDXBuildingObject
{
   public enum ComponentType
   {
      WALLS, DOORS, WINDOWS, STAIRS, FLOORS, PLATFORMS
   }

   private final ArrayList<Point3D> corners = new ArrayList<>();

   private final ArrayList<GDXSimpleObject> allObjects = new ArrayList<>();
   private final HashMap<ComponentType, ArrayList<GDXSimpleObject>> components = new HashMap<>();

   private float height = 5.0f;

   public GDXBuildingObject()
   {
      for (ComponentType type : ComponentType.values())
      {
         components.put(type, new ArrayList<>());
      }
   }

   public Point3D getClosestRectangularCorner(Point3D lastPickPoint)
   {
      Point3D cornerPoint;
      if (corners.size() > 0)
      {
         Point3D corner = corners.get(corners.size() - 1);
         int secondVectorX = 1;
         int secondVectorY = 0;
         double angle = EuclidGeometryTools.angleFromFirstToSecondVector2D(corner.getX() - lastPickPoint.getX(),
                                                                           corner.getY() - lastPickPoint.getY(),
                                                                           secondVectorX,
                                                                           secondVectorY);

         if (StrictMath.abs(angle - StrictMath.PI / 2.0f) < 0.1f || StrictMath.abs(angle + StrictMath.PI / 2.0f) < 0.1f)
         {
            Vector3D direction = new Vector3D(0.0f, 1.0f, 0.0f);
            cornerPoint = EuclidGeometryTools.orthogonalProjectionOnLine3D(lastPickPoint, corner, direction);
         }
         else if (StrictMath.abs(angle) < 0.1f || StrictMath.abs(angle - StrictMath.PI) < 0.1f)
         {
            Vector3D direction = new Vector3D(1.0f, 0.0f, 0.0f);
            cornerPoint = EuclidGeometryTools.orthogonalProjectionOnLine3D(lastPickPoint, corner, direction);
         }
         else
         {
            cornerPoint = lastPickPoint;
         }
      }
      else
         cornerPoint = lastPickPoint;
      return cornerPoint;
   }

   public Point3D getLastCorner()
   {
      if (corners.size() >= 1)
         return corners.get(corners.size() - 1);
      else
         return null;
   }

   public ArrayList<Point3D> getCorners()
   {
      return corners;
   }

   public void addCorner(Point3D corner)
   {
      corners.add(corner);
   }

   public void setHeight(float height)
   {
      this.height = height;
   }

   public float getHeight()
   {
      return height;
   }

   public void insertComponent(ComponentType type, GDXSimpleObject component)
   {
      components.get(type).add(component);
      allObjects.add(component);
   }

   public HashMap<ComponentType, ArrayList<GDXSimpleObject>> getComponents()
   {
      return components;
   }

   public ArrayList<GDXSimpleObject> getAllObjects()
   {
      return allObjects;
   }
}
