package us.ihmc.sensorProcessing.pointClouds.octree;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point3D_I32;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Box3D_I32;

import java.util.List;

import bubo.construct.ConstructOctreeLeaf_I32;
import bubo.construct.OctreeOps;
import bubo.construct.Octree_I32;
import bubo.maps.d3.grid.GridMapSpacialInfo3D;
import bubo.maps.d3.grid.impl.BlurOctreeGridMap_F64;
import bubo.maps.d3.grid.impl.Kernel3D_F64;
import bubo.maps.d3.grid.impl.OctreeGridMap_F64;

/**
 * @author Peter Abeles
 */
public class OctreeOccupancyExample {

   public static double mapSize = 20.0;
   public static int NUMBER_OCCUPIED = 10;
   public static GridMapSpacialInfo3D spacial;

   public static GridMapSpacialInfo3D createSpacial() {
      Se3_F64 mapToCanonical = new Se3_F64();
      mapToCanonical.getTranslation().set(-5,-5,-5);
      spacial = new GridMapSpacialInfo3D(0.05,mapToCanonical);
      return spacial;
   }

   public static OctreeGridMap_F64 createMapFromCloud(List<Point3D_F64> cloud, GridMapSpacialInfo3D spacial , int blurRadius ) {
      //List<Point3D_F64> cloud = PointCloudTools.readPointCloud("../SensorProcessing/data/kinectcloud.txt",-1);

      int w = (int)(mapSize/spacial.getCellSize());

      ConstructOctreeLeaf_I32 pointTree = new ConstructOctreeLeaf_I32();
      pointTree.initialize(new Box3D_I32(0,0,0,w,w,w));

      for( Point3D_F64 p : cloud ) {
         spacial.canonicalToMap(p, p );
         Point3D_I32 gridPt = new Point3D_I32();
         spacial.mapToGrid(p.x, p.y, p.z, gridPt);
         pointTree.addPoint(gridPt,null);
      }

      List<Octree_I32> cells = OctreeOps.findLeafsWithPoints(pointTree.getAllNodes().toList(), null);

      // Assign probability to each occupied cell based on the number of particles found inside
      OctreeGridMap_F64 map = new OctreeGridMap_F64(w,w,w);
      int totalOccupied = 0;
      for( Octree_I32 n : cells ) {
         Point3D_I32 p = n.getLocation();
         if( n.isLeaf() ) {
            if( n.points.size() > 0 ) {
               totalOccupied++;
               map.set(p.x, p.y, p.z, 1.0);
            } 
         }
      }
      System.out.println("Leafs occupied = " + totalOccupied + "  out of " + cells.size());

      if( blurRadius > 0 ) {
         System.out.println("Applying blur");

         OctreeGridMap_F64 mapBlurred = new OctreeGridMap_F64(w, w, w);

         Kernel3D_F64 kernel = Kernel3D_F64.gaussian(blurRadius);
         BlurOctreeGridMap_F64 blurer = new BlurOctreeGridMap_F64();
         blurer.apply(map, kernel, mapBlurred);

         System.out.println("Done with blur");

         return mapBlurred;
      } else {
         return map;
      }
   }


public static double getProb(Point3D_F64 p,OctreeGridMap_F64 map){
   spacial.canonicalToMap(p, p );
   Point3D_I32 gridPt = new Point3D_I32();
   spacial.mapToGrid(p.x, p.y, p.z, gridPt);
   return map.get(gridPt.x, gridPt.y, gridPt.z);
   
}
}
