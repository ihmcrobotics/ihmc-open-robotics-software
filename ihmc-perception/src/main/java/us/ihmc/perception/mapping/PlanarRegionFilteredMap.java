package us.ihmc.perception.mapping;

import com.esotericsoftware.kryo.util.IntMap;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMerger;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashSet;

public class PlanarRegionFilteredMap
{
   private static final double updateAlphaTowardsMatch = 0.05;

   private static final double angleThresholdBetweenNormalsForMatch = Math.toRadians(15);
   private static final float outOfPlaneDistanceFromOneRegionToAnother = 0.05f;
   private static final float maxDistanceBetweenRegionsForMatch = 1.5f;


   private boolean initialized = false;
   private boolean modified = false;
   private int uniqueRegionsFound = 0;
   private int uniqueIDtracker = 0;

   private final HashSet<Integer> mapRegionIDSet = new HashSet<>();

   private PlanarRegionsList finalMap;

   public PlanarRegionFilteredMap()
   {
      finalMap = new PlanarRegionsList();
   }

   public void submitRegions(PlanarRegionsList regions)
   {
      modified = true;
      if (!initialized)
      {
         for (PlanarRegion region : regions.getPlanarRegionsAsList())
         {
            if (region.getRegionId() == -1)
            {
               while (mapRegionIDSet.contains(uniqueIDtracker))
                  uniqueIDtracker++;

               region.setRegionId(uniqueIDtracker);
               mapRegionIDSet.add(uniqueIDtracker);
               uniqueRegionsFound++;
               finalMap.addPlanarRegion(region);
            }

            mapRegionIDSet.add(region.getRegionId());
         }

         initialized = true;
      }
      else
      {
         IntMap<TIntArrayList> matches = PlanarRegionSLAMTools.findPlanarRegionMatches(finalMap,
                                                                                       regions,
                                                                                       (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                                                                       outOfPlaneDistanceFromOneRegionToAnother,
                                                                                       maxDistanceBetweenRegionsForMatch);

         //LogTools.info("Regions Before: {}", regions.getPlanarRegionsAsList().size());

         for (PlanarRegion region : regions.getPlanarRegionsAsList())
            region.setRegionId(-1);

         for (int mapRegionIndex : matches.keys().toArray().toArray())
         {
            PlanarRegion mapRegion = finalMap.getPlanarRegion(mapRegionIndex);
            for (int matchingRegionIndex : matches.get(mapRegionIndex).toArray())
            {
               PlanarRegion region = regions.getPlanarRegion(matchingRegionIndex);

               //LogTools.info(String.format("Merging: Map(%d)[%.3f, %.3f, %.3f] -> Region(%d)[%.3f, %.3f, %.3f]",
               //                            mapRegionIndex,
               //                            mapRegion.getNormal().getX(),
               //                            mapRegion.getNormal().getY(),
               //                            mapRegion.getNormal().getZ(),
               //                            matchingRegionIndex,
               //                            region.getNormal().getX(),
               //                            region.getNormal().getY(),
               //                            region.getNormal().getZ()));

               mergeRegionIntoMap(mapRegion, region);
            }
         }

         uniqueRegionsFound = 0;
         for (PlanarRegion region : regions.getPlanarRegionsAsList())
         {
            if (region.getRegionId() == -1)
            {
               while (mapRegionIDSet.contains(uniqueIDtracker))
                  uniqueIDtracker++;

               region.setRegionId(uniqueIDtracker);
               mapRegionIDSet.add(uniqueIDtracker);
               uniqueRegionsFound++;
               //LogTools.info("Found Unique: {}", uniqueIDtracker);
               finalMap.addPlanarRegion(region);
            }
         }
         //map.addPlanarRegionsList(regions);

         LogTools.info("Regions: {}, Unique: {}, Map: {}",
                       regions.getPlanarRegionsAsList().size(),
                       uniqueRegionsFound,
                       finalMap.getPlanarRegionsAsList().size());

         LogTools.info("Unique Set: [{}]", mapRegionIDSet);
      }
   }

   private void mergeRegionIntoMap(PlanarRegion mapRegion, PlanarRegion region)
   {
      region.setRegionId(mapRegion.getRegionId());

      // Update Map Region Normal and Origin
      UnitVector3DReadOnly mapNormal = mapRegion.getNormal();
      Point3DReadOnly mapOrigin = mapRegion.getPoint();

      UnitVector3DReadOnly regionNormal = region.getNormal();
      Point3DReadOnly regionOrigin = region.getPoint();

      Vector3D futureNormal = new Vector3D();
      futureNormal.interpolate(mapNormal, regionNormal, updateAlphaTowardsMatch);

      double futureHeightZ = EuclidCoreTools.interpolate(mapOrigin.getZ(), regionOrigin.getZ(), updateAlphaTowardsMatch);

      Vector3D normalVector = new Vector3D(mapNormal);
      Vector3D axis = new Vector3D();
      axis.cross(normalVector, futureNormal);
      double angle = normalVector.angle(futureNormal);

      Point3D futureOrigin = new Point3D(mapOrigin.getX(), mapOrigin.getY(), futureHeightZ);
      AxisAngle rotationToFutureRegion = new AxisAngle(axis, angle);
      Vector3D translationToFutureRegion = new Vector3D();
      translationToFutureRegion.sub(futureOrigin, mapOrigin);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendOrientation(rotationToFutureRegion);
      transform.appendTranslation(translationToFutureRegion);

      mapRegion.applyTransform(transform);

      // Update Map Region Hull
      //List<Point2D> concaveHullMapRegionVertices = mapRegion.getConcaveHull();
      //List<Point2D> concaveHullRegionVertices = region.getConcaveHull();
      //List<Point2D> mergedConcaveHull = ConcaveHullMerger.mergeConcaveHulls(concaveHullMapRegionVertices, concaveHullRegionVertices, null);
      //ArrayList<ConvexPolygon2D> newPolygonsFromConcaveHull = new ArrayList<>();
      //ConcaveHullDecomposition.recursiveApproximateDecomposition(mergedConcaveHull, 0.001, newPolygonsFromConcaveHull);
      //
      //RigidBodyTransform transformToWorld = new RigidBodyTransform();
      //mapRegion.getTransformToWorld(transformToWorld);
      //
      //PlanarRegion planarRegion = new PlanarRegion(transformToWorld, mergedConcaveHull, newPolygonsFromConcaveHull);
      //planarRegion.setRegionId(mapRegion.getRegionId());
      //
      //mapRegion.set(planarRegion);

      ArrayList<PlanarRegion> mergedRegion = ConcaveHullMerger.mergePlanarRegions(mapRegion, region, 1.0f, null);

      if(mergedRegion != null)
      {
         if(mergedRegion.size() > 0)
         {
            mapRegion.set(mergedRegion.get(0));
         }
         else
         {
            // TODO: Figure out how to handle the case where no merged region is generated.
         }
      }
   }


   public PlanarRegionsList getMapRegions()
   {
      return finalMap;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }
}

