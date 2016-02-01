package us.ihmc.robotics.geometry;

import com.github.quickhull3d.QuickHull3D;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by agrabertilton on 1/19/15.
 */
public class QuickHull3dWrapper
{
   QuickHull3D quickHull3D;
   List<Point3d> vertices = new ArrayList<Point3d>();
   List<HullFace> faces = new ArrayList<HullFace>();


   public QuickHull3dWrapper()
   {
      quickHull3D = new QuickHull3D();
   }

   public QuickHull3dWrapper(List<Point3d> point3dList)
   {
      quickHull3D = new QuickHull3D();
      build(point3dList);
   }

   public void build(List<Point3d> point3dList)
   {
      quickHull3D.build(getPointArrayFromPointList(point3dList));
      computeWrapperVerticesAndFaces();
   }

   public double getNumVertices() {return vertices.size();}

   public double getNumFaces() {return faces.size();}

   public List<Point3d> getVertices() {return vertices;}

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

   private com.github.quickhull3d.Point3d[] getPointArrayFromPointList(List<Point3d> point3dList)
   {
      com.github.quickhull3d.Point3d[] pointArray = new com.github.quickhull3d.Point3d[point3dList.size()];
      int index = 0;
      for (Point3d point3d : point3dList)
      {
         pointArray[index] = createPoint3d(point3d);
         index++;
      }
      return pointArray;
   }

   private List<Point3d> getPointListFromPointArray(com.github.quickhull3d.Point3d[] pointArray)
   {
      List<Point3d> point3dList = new ArrayList<Point3d>();
      for (com.github.quickhull3d.Point3d point3d : pointArray)
      {
         point3dList.add(createPoint3d(point3d));
      }
      return point3dList;
   }


   private com.github.quickhull3d.Point3d createPoint3d(Point3d point3d)
   {
      return new com.github.quickhull3d.Point3d(point3d.x, point3d.y, point3d.z);
   }

   private Point3d createPoint3d(com.github.quickhull3d.Point3d point3d)
   {
      return new Point3d(point3d.x, point3d.y, point3d.z);
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
