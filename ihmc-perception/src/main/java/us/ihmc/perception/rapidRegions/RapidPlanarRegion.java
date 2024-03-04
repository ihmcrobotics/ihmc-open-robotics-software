package us.ihmc.perception.rapidRegions;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.graphicalSegmentation.GraphicalSegment;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;

public class RapidPlanarRegion extends GraphicalSegment
{
   private final Vector3D normalAverage = new Vector3D();
   private final Vector3D normalSVD = new Vector3D();
   private final Vector3D normal = new Vector3D();
   private final Point3D centroidAverage = new Point3D();
   private final RecyclingArrayList<Point3D> patchCentroids = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Vector3D> patchNormals = new RecyclingArrayList<>(Vector3D::new);
   private final RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();
   private final RecyclingArrayList<Point3D> boundaryVertices = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<RapidRegionRing> regionRings = new RecyclingArrayList<>(RapidRegionRing::new);

   // TODO: kd tree
   private final SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
   private final DMatrixRMaj svdU = new DMatrixRMaj(3, 3);
   private final Stopwatch svdStopwatch = new Stopwatch();
   private double svdDuration = Double.NaN;
   private final TreeSet<RapidRegionRing> regionsRingsBySize = new TreeSet<>(Comparator.comparing(gpuRegionRing -> -gpuRegionRing.getConvexPolygon().getArea()));
   private final ArrayList<RapidRegionRing> holeRingsToRemove = new ArrayList<>();
   private final DMatrixRMaj patchMatrix = new DMatrixRMaj(3, 1);

   public void reset(int id)
   {
      this.id = id;
      patchCentroids.clear();
      patchNormals.clear();
      transformToWorldFrame.setIdentity();
      normalAverage.setToZero();
      normalSVD.setToZero();
      centroidAverage.setToZero();
      regionIndices.clear();
      borderIndices.clear();
      boundaryVertices.clear();
      regionRings.clear();
      numberOfPatches = 0;
   }

   public void addRegionPatch(int row, int column, double nx, double ny, double nz, double cx, double cy, double cz)
   {
      patchCentroids.add().set(cx, cy, cz);
      Vector3D patchNormal = patchNormals.add();
      patchNormal.set(nx, ny, nz);
      patchNormal.normalize();
      normalAverage.add(patchNormal);
      centroidAverage.add(cx, cy, cz);

      regionIndices.add().set(column, row);
      ++numberOfPatches;
   }

   public RecyclingArrayList<Point3D> getBoundaryVertices()
   {
      return boundaryVertices;
   }

   public RecyclingArrayList<Point2D> getBorderIndices()
   {
      return borderIndices;
   }

   public RecyclingArrayList<RapidRegionRing> getRegionRings()
   {
      return regionRings;
   }

   public int getId()
   {
      return id;
   }

   public void update(boolean useCentroidSVD, int reductionFactor)
   {
      centroidAverage.scale(1.0 / numberOfPatches);
      normalAverage.scale(1.0 / numberOfPatches);

      if (useCentroidSVD)
      {
         svdStopwatch.start();
         int leftOver = patchCentroids.size() % reductionFactor;
         int reducedSize = patchCentroids.size() - leftOver;
         int numCols = reducedSize / reductionFactor;
         patchMatrix.reshape(3, numCols);
         for (int i = 0; i < numCols; i++)
         {
            Point3D patchCentroid = patchCentroids.get(i * reductionFactor);
            patchMatrix.set(0, i, patchCentroid.getX() - centroidAverage.getX());
            patchMatrix.set(1, i, patchCentroid.getY() - centroidAverage.getY());
            patchMatrix.set(2, i, patchCentroid.getZ() - centroidAverage.getZ());
         }
         if (svd.decompose(patchMatrix))
         {
            svd.getU(svdU, true);
            normalSVD.set(svdU.get(6), svdU.get(7), svdU.get(8));
            //            normalSVD.normalize();
            if (normalSVD.dot(Axis3D.Z) < 0.0)
               normalSVD.negate();
            //            normalSVD.scale(normalSVD.getZ() / Math.abs(normalSVD.getZ()));
         }

         svdDuration = svdStopwatch.totalElapsed();
      }

      normal.set(useCentroidSVD ? normalSVD : normalAverage);

      if (normal.dot(centroidAverage) > 0.0)
      {
         normal.negate();
      }
   }

   public Point3D getCenter()
   {
      return centroidAverage;
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public double getSVDDuration()
   {
      return svdDuration;
   }

   public TreeSet<RapidRegionRing> getRegionsRingsBySize()
   {
      return regionsRingsBySize;
   }

   public ArrayList<RapidRegionRing> getHoleRingsToRemove()
   {
      return holeRingsToRemove;
   }
}
