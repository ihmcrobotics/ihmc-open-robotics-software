package us.ihmc.robotics.graphics;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class Graphics3DObjectTools
{

   /** Adds the PlanarRegionsList transforming from the current coordinate system.
    *
    * @param planarRegions
    */
   public static void addPlanarRegionsList(Graphics3DObject graphics3DObject, PlanarRegionsList planarRegions)
   {
      addPlanarRegionsList(graphics3DObject, planarRegions, YoAppearance.Black());
   }

   /** Adds the PlanarRegionsList transforming from the current coordinate system.
    * Uses the given appearances in order, one for each PlanarRegion. Then loops on the appearances.
    *
    * @param planarRegions
    */
   public static void addPlanarRegionsList(Graphics3DObject graphics3DObject, PlanarRegionsList planarRegions, AppearanceDefinition... appearances)
   {
      int numberOfPlanarRegions = planarRegions.getNumberOfPlanarRegions();
      for (int i = 0; i < numberOfPlanarRegions; i++)
      {
         addPlanarRegion(graphics3DObject, planarRegions.getPlanarRegion(i), appearances[i % appearances.length]);
      }
   }

   /**
    * Adds a PlanarRegion transforming from the current coordinate system.
    *
    * @param planarRegion
    */
   public static void addPlanarRegion(Graphics3DObject graphics3DObject, PlanarRegion planarRegion)
   {
      addPlanarRegion(graphics3DObject, planarRegion, YoAppearance.Black());
   }

   /**
    * Adds a PlanarRegion transforming from the current coordinate system.
    * Uses the given appearances in order, one for each Polygon in the PlanarRegion. Then loops on the appearances.
    *
    * @param planarRegion
    */
   public static void addPlanarRegion(Graphics3DObject graphics3DObject, PlanarRegion planarRegion, AppearanceDefinition... appearances)
   {
      double thickness = 1e-4;
      addPlanarRegion(graphics3DObject, planarRegion, thickness, appearances);
   }

   /**
    * Adds a PlanarRegion transforming from the current coordinate system.
    * Uses the given appearances in order, one for each Polygon in the PlanarRegion. Then loops on the appearances.
    * Extrudes the mesh downward in local frame by the given thickness
    *
    * @param planarRegion
    */
   public static void addPlanarRegion(Graphics3DObject graphics3DObject, PlanarRegion planarRegion, double thickness, AppearanceDefinition... appearances)
   {
      int numberOfConvexPolygons = planarRegion.getNumberOfConvexPolygons();

      RigidBodyTransform transform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transform);

      graphics3DObject.transform(transform);
      double extrusionHeight = - Math.max(thickness, 1e-4);

      for (int i = 0; i < numberOfConvexPolygons; i++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(i);
         MeshDataHolder meshDataHolder = MeshDataGenerator.ExtrudedPolygon(convexPolygon, extrusionHeight);
         graphics3DObject.addInstruction(new Graphics3DAddMeshDataInstruction(meshDataHolder, appearances[i % appearances.length]));
      }

      transform.invert();
      graphics3DObject.transform(transform);
   }

}
