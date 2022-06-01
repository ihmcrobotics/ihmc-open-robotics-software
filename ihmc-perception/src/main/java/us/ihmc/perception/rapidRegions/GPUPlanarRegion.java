package us.ihmc.perception.rapidRegions;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;

public class GPUPlanarRegion
{
   private final Vector3D32 normalAverage = new Vector3D32();
   private final Vector3D32 normalSVD = new Vector3D32();
   private final Vector3D32 normal = new Vector3D32();
   private final Point3D32 centroidAverage = new Point3D32();
   private final RecyclingArrayList<Point2D> regionIndices = new RecyclingArrayList<>(Point2D::new);
   private final RecyclingArrayList<Point3D> patchCentroids = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Vector3D> patchNormals = new RecyclingArrayList<>(Vector3D::new);
   private final RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();
   private final RecyclingArrayList<Point2D> borderIndices = new RecyclingArrayList<>(Point2D::new);
   private final RecyclingArrayList<Vector3D> boundaryVertices = new RecyclingArrayList<>(Vector3D::new);
   private final RecyclingArrayList<GPURegionRing> regionRings = new RecyclingArrayList<>(GPURegionRing::new);
   // TODO: kd tree
   private int numberOfPatches;
   private int id;
   private final SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
   private final DMatrixRMaj patchMatrix = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj svdU = new DMatrixRMaj(3, 3);
   private final Stopwatch svdStopwatch = new Stopwatch();
   private double svdDuration = Double.NaN;

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
      regionIndices.add().set(column, row);
      patchCentroids.add().set(cx, cy, cz);
      Vector3D patchNormal = patchNormals.add();
      patchNormal.set(nx, ny, nz);
      patchNormal.normalize();
      normalAverage.add(patchNormal);
      centroidAverage.add(cx, cy, cz);
      ++numberOfPatches;
   }

   public RecyclingArrayList<Vector3D> getBoundaryVertices()
   {
      return boundaryVertices;
   }

   public RecyclingArrayList<Point2D> getBorderIndices()
   {
      return borderIndices;
   }

   public RecyclingArrayList<GPURegionRing> getRegionRings()
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
   }

   public Point3D32 getCenter()
   {
      return centroidAverage;
   }

   public Vector3D32 getNormal()
   {
      return normal;
   }

   public RecyclingArrayList<Point2D> getRegionIndices()
   {
      return regionIndices;
   }

   public double getSVDDuration()
   {
      return svdDuration;
   }
}
