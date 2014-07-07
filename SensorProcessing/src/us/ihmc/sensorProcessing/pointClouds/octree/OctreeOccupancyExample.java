package us.ihmc.sensorProcessing.pointClouds.octree;

import bubo.construct.ConstructOctreeLeaf_I32;
import bubo.construct.OctreeOps;
import bubo.construct.Octree_I32;
import bubo.maps.d3.grid.GridMapSpacialInfo3D;
import bubo.maps.d3.grid.impl.OctreeGridMap_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point3D_I32;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Cube3D_I32;

import java.util.List;

/**
 * @author Peter Abeles
 */
public class OctreeOccupancyExample {

    public static int NUMBER_OCCUPIED = 10;

    public static GridMapSpacialInfo3D createSpacial() {
        Se3_F64 mapToWorld = new Se3_F64();
        mapToWorld.getT().set(5,5,5);
        return new GridMapSpacialInfo3D(0.05,mapToWorld);
    }

    public static OctreeGridMap_F64 createMapFromCloud(List<Point3D_F64> cloud, GridMapSpacialInfo3D spacial) {
        //List<Point3D_F64> cloud = PointCloudTools.readPointCloud("../SensorProcessing/data/kinectcloud.txt",-1);

        int w = (int)(10.0/spacial.getCellSize());

        ConstructOctreeLeaf_I32 pointTree = new ConstructOctreeLeaf_I32();
        pointTree.initialize(new Cube3D_I32(0,0,0,w,w,w));

        for( Point3D_F64 p : cloud ) {
            Point3D_I32 gridPt = new Point3D_I32();
            spacial.mapToGrid(p.x,p.y,p.z,gridPt);
            pointTree.addPoint(gridPt,null);
        }

        List<Octree_I32> cells = OctreeOps.findAllSmallest(pointTree.getAllNodes().toList(),null);

        // Assign probability to each occupied cell based on the number of particles found inside
        OctreeGridMap_F64 map = new OctreeGridMap_F64(w,w,w);
        int totalOccupied = 0;
        for( Octree_I32 n : cells ) {
            Point3D_I32 p = n.space.p0;
            if( n.isLeaf() ) {
                if( n.points.size() > NUMBER_OCCUPIED ) {
                    totalOccupied++;
                    map.set(p.x,p.y,p.z,0.8);
                } else {
                    map.set(p.x, p.y, p.z, 0.3);
                }
            }
        }
        System.out.println("Leafs occupied = " + totalOccupied + "  out of " + cells.size());

       return map;
    }



}
