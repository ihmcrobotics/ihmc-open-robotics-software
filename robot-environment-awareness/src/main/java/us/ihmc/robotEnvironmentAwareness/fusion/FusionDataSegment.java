package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class FusionDataSegment
{
   private int id = -1;
   
   private final int imageSegmentLabel;
   private final TIntArrayList adjacentSegmentLabels = new TIntArrayList();
   private final List<Point3D> points = new ArrayList<>();

   private final Point3D center = new Point3D();
   private final Vector3D normal = new Vector3D();

   public final Vector3D standardDeviation = new Vector3D();

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   public FusionDataSegment(int labelID)
   {
      imageSegmentLabel = labelID;
   }

   public boolean contains(int otherLabel)
   {
      return adjacentSegmentLabels.contains(otherLabel);
   }

   public void addAdjacentSegmentLabel(int otherLabel)
   {
      adjacentSegmentLabels.add(otherLabel);
   }

   public void addPoint(Point3D point)
   {
      points.add(point);
   }

   public void update()
   {
      pca.clear();
      points.stream().forEach(point -> pca.addPoint(point.getX(), point.getY(), point.getZ()));
      pca.compute();

      pca.getMean(center);
      pca.getThirdVector(normal);
      
      if(normal.getZ() < 0.0)
         normal.negate();
      
      pca.getStandardDeviation(standardDeviation);
   }
   
   public void setID(int id)
   {
      this.id = id;
   }

   // TODO: handle if this label does not have enough number of points.
   public boolean isEmpty()
   {
      return points.size() < 4;
   }

   public boolean isSparse(double threshold)
   {
      return standardDeviation.getZ() > threshold;
   }

   public int[] getAdjacentSegmentLabels()
   {
      return adjacentSegmentLabels.toArray();
   }

   public double getWeight()
   {
      return (double) points.size();
   }

   public Point3D getCenter()
   {
      return center;
   }

   public Vector3D getNormal()
   {
      return normal;
   }
   
   public int getId()
   {
      return id;
   }
   
   public int getImageSegmentLabel()
   {
      return imageSegmentLabel;
   }
   
   public List<Point3D> getPoints()
   {
      return points;
   }
}
