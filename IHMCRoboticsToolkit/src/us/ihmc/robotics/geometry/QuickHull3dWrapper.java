package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import com.github.quickhull3d.QuickHull3D;

import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Created by agrabertilton on 1/19/15.
 */
public class QuickHull3dWrapper
{
   QuickHull3D quickHull3D;
   List<Point3D> vertices = new ArrayList<Point3D>();
   List<HullFace> faces = new ArrayList<HullFace>();


   public QuickHull3dWrapper()
   {
      quickHull3D = new QuickHull3D();
   }

   public QuickHull3dWrapper(List<Point3D> point3dList)
   {
      quickHull3D = new QuickHull3D();
      build(point3dList);
   }

   public void build(List<Point3D> point3dList)
   {
      quickHull3D.build(getPointArrayFromPointList(point3dList));
      computeWrapperVerticesAndFaces();
   }

   public double getNumVertices() {return vertices.size();}

   public double getNumFaces() {return faces.size();}

   public List<Point3D> getVertices() {return vertices;}

   public List<HullFace> getFaces() {return faces;}

   private void computeWrapperVerticesAndFaces()
   {
      vertices = getPointListFromPointArray(quickHull3D.getVertices());
      faces.clear();
      int[][] faceArrays = quickHull3D.getFaces(QuickHull3D.CLOCKWISE);
      for (int i = 0; i < faceArrays.length; i++)
      {
         faces.add(new HullFace(vertices, faceArrays[i]));
      }
   }

   private com.github.quickhull3d.Point3d[] getPointArrayFromPointList(List<Point3D> point3dList)
   {
      com.github.quickhull3d.Point3d[] pointArray = new com.github.quickhull3d.Point3d[point3dList.size()];
      int index = 0;
      for (Point3D point3d : point3dList)
      {
         pointArray[index] = createPoint3d(point3d);
         index++;
      }
      return pointArray;
   }

   private List<Point3D> getPointListFromPointArray(com.github.quickhull3d.Point3d[] pointArray)
   {
      List<Point3D> point3dList = new ArrayList<Point3D>();
      for (com.github.quickhull3d.Point3d point3d : pointArray)
      {
         point3dList.add(createPoint3d(point3d));
      }
      return point3dList;
   }


   private com.github.quickhull3d.Point3d createPoint3d(Point3D point3d)
   {
      return new com.github.quickhull3d.Point3d(point3d.getX(), point3d.getY(), point3d.getZ());
   }

   private Point3D createPoint3d(com.github.quickhull3d.Point3d point3d)
   {
      return new Point3D(point3d.x, point3d.y, point3d.z);
   }

   public double getDistanceTolerance()
   {
      return quickHull3D.getDistanceTolerance();
   }

   public void setDistanceTolerance(double tolerance)
   {
      quickHull3D.setExplicitDistanceTolerance(tolerance);
   }
}
