package us.ihmc.perception.mapping;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMerger;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class PlanarRegionFilteredMap
{
   private PlanarRegionsList slamMap;

   private ConcaveHullMerger merger = new ConcaveHullMerger();

   private final HashSet<Integer> mapRegionIDSet = new HashSet<>();

   private int uniqueRegionsFound = 0;
   private int uniqueIDtracker = 0;
   boolean initialized = false;
   boolean modified = false;

   public PlanarRegionFilteredMap()
   {
      slamMap = new PlanarRegionsList();
   }

   public void submitRegions(PlanarRegionsList regions)
   {
      modified = true;
      if (!initialized)
      {
         slamMap.addPlanarRegionsList(regions);

         for (PlanarRegion region : regions.getPlanarRegionsAsList())
            mapRegionIDSet.add(region.getRegionId());

         initialized = true;
      }
      else
      {
         HashMap<Integer, Integer> matches = PlanarRegionSLAMTools.findPlanarRegionMatches(slamMap, regions, 0.9f, 0.05f, 1.5f);

         LogTools.info("Regions Before: {}", regions.getPlanarRegionsAsList().size());

         for (PlanarRegion region : regions.getPlanarRegionsAsList())
            region.setRegionId(-1);

         for (Integer i : matches.keySet())
         {

            PlanarRegion mapRegion = slamMap.getPlanarRegion(i);
            PlanarRegion region = regions.getPlanarRegion(matches.get(i));

            LogTools.info(String.format("Merging: Map(%d)[%.3f, %.3f, %.3f] -> Region(%d)[%.3f, %.3f, %.3f]",
                                        i,
                                        mapRegion.getNormal().getX(),
                                        mapRegion.getNormal().getY(),
                                        mapRegion.getNormal().getZ(),
                                        matches.get(i),
                                        region.getNormal().getX(),
                                        region.getNormal().getY(),
                                        region.getNormal().getZ()));

            region.setRegionId(i);

            // Update Map Region Normal and Origin
            UnitVector3DReadOnly mapNormal = mapRegion.getNormal();
            Point3DReadOnly mapOrigin = mapRegion.getPoint();

            UnitVector3DReadOnly regionNormal = region.getNormal();
            Point3DReadOnly regionOrigin = region.getPoint();

            Vector3D futureNormal = new Vector3D(0.95 * mapNormal.getX() + 0.05 * regionNormal.getX(),
                                                 0.95 * mapNormal.getY() + 0.05 * regionNormal.getY(),
                                                 0.95 * mapNormal.getZ() + 0.05 * regionNormal.getZ());

            double futureHeightZ = 0.95 * mapOrigin.getZ() + 0.05 * regionOrigin.getZ();

            Vector3D normalVector = new Vector3D(mapNormal.getX(), mapNormal.getY(), mapNormal.getZ());
            Vector3D axis = new Vector3D();
            axis.cross(normalVector, futureNormal);
            double angle = EuclidGeometryTools.angleFromFirstToSecondVector3D(normalVector.getX(), normalVector.getY(), normalVector.getZ(),
                                                                              futureNormal.getX(), futureNormal.getY(), futureNormal.getZ());

            Point3D futureOrigin = new Point3D(mapOrigin.getX(), mapOrigin.getY(), futureHeightZ);
            AxisAngle axisAngle = new AxisAngle(axis, angle);
            Vector3D translation = new Vector3D();
            translation.sub(futureOrigin, mapOrigin);

            RigidBodyTransform transform = new RigidBodyTransform();
            transform.appendOrientation(axisAngle);
            transform.appendTranslation(translation);

            mapRegion.applyTransform(transform);

            // Update Map Region Hull
            List<Point2D> concaveHullMapRegionVertices = mapRegion.getConcaveHull();
            List<Point2D> concaveHullRegionVertices = region.getConcaveHull();
            List<Point2D> mergedConcaveHull = ConcaveHullMerger.mergeConcaveHulls(concaveHullMapRegionVertices, concaveHullRegionVertices, null);
            ArrayList<ConvexPolygon2D> newPolygonsFromConcaveHull = new ArrayList<>();
            ConcaveHullDecomposition.recursiveApproximateDecomposition(mergedConcaveHull, 0.001, newPolygonsFromConcaveHull);

            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            mapRegion.getTransformToWorld(transformToWorld);

            PlanarRegion planarRegion = new PlanarRegion(transformToWorld, mergedConcaveHull, newPolygonsFromConcaveHull);
            planarRegion.setRegionId(mapRegion.getRegionId());

            mapRegion.set(planarRegion);
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
               slamMap.addPlanarRegion(region);
            }
         }
         //slamMap.addPlanarRegionsList(regions);

         LogTools.info("Regions: {}, Unique: {}, Map: {}",
                       regions.getPlanarRegionsAsList().size(),
                       uniqueRegionsFound,
                       slamMap.getPlanarRegionsAsList().size());

         LogTools.info("Unique Set: [{}]", mapRegionIDSet);
      }
   }

   public PlanarRegionsList getMapRegions()
   {
      return slamMap;
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

