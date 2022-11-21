package us.ihmc.perception.mapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
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
import java.util.List;

public class PlanarRegionFilteredMap
{
   private PlanarRegionsList slamMap;

   private ConcaveHullMerger merger = new ConcaveHullMerger();

   boolean initialized = false;
   boolean modified = false;

   public PlanarRegionFilteredMap()
   {
      slamMap = new PlanarRegionsList();
   }

   public void submitRegions(PlanarRegionsList regions)
   {
      modified = true;
      if(!initialized)
      {
         slamMap.addPlanarRegionsList(regions);
         initialized = true;
      }
      else
      {
         HashMap<Integer, Integer> matches = PlanarRegionSLAMTools.findPlanarRegionMatches(slamMap, regions, 0.15f, 0.85f);

         LogTools.info("Regions Before: {}", regions.getPlanarRegionsAsList().size());

         for(PlanarRegion region : regions.getPlanarRegionsAsList())
            region.setRegionId(-1);

         for(Integer i : matches.keySet())
         {

            PlanarRegion mapRegion = slamMap.getPlanarRegion(i);
            PlanarRegion region = regions.getPlanarRegion(matches.get(i));

            LogTools.info("Merging: Map({}) -> Region({})", i, matches.get(i));

            region.setRegionId(i);

            // Update Map Region Normal and Origin
            UnitVector3DReadOnly mapNormal = mapRegion.getNormal();
            Point3DReadOnly mapOrigin = mapRegion.getPoint();

            UnitVector3DReadOnly regionNormal = region.getNormal();
            Point3DReadOnly regionOrigin = region.getPoint();

            Vector3D futureNormal = new Vector3D(0.95 * mapNormal.getX() + 0.05 * regionNormal.getX(),
                                                 0.95 * mapNormal.getY() + 0.05 * regionNormal.getY(),
                                                 0.95 * mapNormal.getZ() + 0.05 * regionNormal.getZ());

            Point3D futureOrigin = new Point3D(0.95 * mapOrigin.getX() + 0.05 * regionOrigin.getX(),
                                                 0.95 * mapOrigin.getY() + 0.05 * regionOrigin.getY(),
                                                 0.95 * mapOrigin.getZ() + 0.05 * regionOrigin.getZ());

            mapRegion.updatePlane(futureNormal, futureOrigin);

            // Update Map Region Hull
            //List<Point2D> concaveHullMapRegionVertices = mapRegion.getConcaveHull();
            //List<Point2D> concaveHullRegionVertices = region.getConcaveHull();
            //
            //List<Point2D> mergedConcaveHull = ConcaveHullMerger.mergeConcaveHulls(concaveHullMapRegionVertices, concaveHullRegionVertices, null);
            //
            //ArrayList<ConvexPolygon2D> newPolygonsFromConcaveHull = new ArrayList<ConvexPolygon2D>();
            //
            //ConcaveHullDecomposition.recursiveApproximateDecomposition(mergedConcaveHull, 0.001, newPolygonsFromConcaveHull);
            //
            //RigidBodyTransform transformOneToWorld = new RigidBodyTransform();
            //mapRegion.getTransformToWorld(transformOneToWorld);
            //
            //PlanarRegion planarRegion = new PlanarRegion(transformOneToWorld, mergedConcaveHull, newPolygonsFromConcaveHull);
            //planarRegion.setRegionId(mapRegion.getRegionId());
            //
            //mapRegion.set(planarRegion);

         }

         slamMap.addPlanarRegionsList(regions);

         LogTools.info("Regions Before: {}", regions.getPlanarRegionsAsList().size());
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

