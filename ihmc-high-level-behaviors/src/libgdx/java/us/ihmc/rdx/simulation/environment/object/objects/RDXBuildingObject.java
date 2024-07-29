package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.rdx.simulation.environment.object.RDXSimpleObject;
import us.ihmc.rdx.tools.RDXModelBuilder;

import java.util.ArrayList;
import java.util.HashMap;

public class RDXBuildingObject
{
   public enum ComponentType
   {
      WALLS, DOORS, WINDOWS, STAIRS, FLOORS, PLATFORMS
   }

   private final ArrayList<Point3D> corners = new ArrayList<>();
   private final ArrayList<RDXSimpleObject> allObjects = new ArrayList<>();
   private final RigidBodyTransform translationTransform = new RigidBodyTransform();
   private final HashMap<ComponentType, ArrayList<RDXSimpleObject>> components = new HashMap<>();
   private final ColorAttribute highlightColor = ColorAttribute.createDiffuse(0.3f, 0.8f, 0.4f, 1.0f);

   private float height = 2.5f;

   public RDXBuildingObject()
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

   public void construct()
   {
      for (int i = 0; i < corners.size(); i++)
      {
         Point3D corner = corners.get((i + 1) % corners.size());
         Point3D previousCorner = corners.get(i % corners.size());
         double yaw = EuclidGeometryTools.angleFromFirstToSecondVector2D(corner.getX() - previousCorner.getX(), corner.getY() - previousCorner.getY(), 1, 0);
         float length = (float) EuclidGeometryTools.distanceBetweenPoint3Ds(corner.getX(),
                                                                            corner.getY(),
                                                                            corner.getZ(),
                                                                            previousCorner.getX(),
                                                                            previousCorner.getY(),
                                                                            previousCorner.getZ());
         Point3D midPoint = new Point3D(0.0, 0.0, 0.0);
         midPoint.add(corner);
         midPoint.add(previousCorner);
         midPoint.scale(0.5);

         RDXSimpleObject objectToPlace = new RDXSimpleObject("BuildingWall_" + i);
         Model objectModel = RDXModelBuilder.createBox(length, 0.1f, height, Color.LIGHT_GRAY).model;

         Vector3DBasics translation = objectToPlace.getObjectTransform().getTranslation();
         translationTransform.setTranslationAndIdentityRotation(translation);
         objectToPlace.getObjectTransform().setRotationYawAndZeroTranslation(-yaw);
         objectToPlace.getObjectTransform().multiply(translationTransform);
         objectToPlace.setRealisticModel(objectModel);
         //                  objectToPlace.setCollisionModel(objectModel);

         Box3D collisionBox = new Box3D(length, 0.1f, height);
         objectToPlace.setCollisionGeometryObject(collisionBox);
         objectToPlace.setCollisionModelColor(highlightColor, 0.2f);

         objectToPlace.getCollisionShapeOffset().getTranslation().add(0.0f, 0.0f, height / 2.0f);
         objectToPlace.getRealisticModelOffset().getTranslation().add(0.0f, 0.0f, height / 2.0f);
         objectToPlace.setPositionInWorld(midPoint);
         objectToPlace.getBoundingSphere().setRadius(height / 2.0f);

         insertComponent(RDXBuildingObject.ComponentType.WALLS, objectToPlace);
      }
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

   public void insertComponent(ComponentType type, RDXSimpleObject component)
   {
      components.get(type).add(component);
      allObjects.add(component);
   }

   public HashMap<ComponentType, ArrayList<RDXSimpleObject>> getComponents()
   {
      return components;
   }

   public ArrayList<RDXSimpleObject> getAllObjects()
   {
      return allObjects;
   }
}
