package us.ihmc.gdx.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;

public class GDXGPUPlanarRegion
{
   private static final boolean USE_SVD = false;
   private final Vector3D32 normal = new Vector3D32();
   private final Point3D32 center = new Point3D32();
   private final RecyclingArrayList<Point3D> patchCentroids = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Point2D> planarPatchCentroids = new RecyclingArrayList<>(Point2D::new);
   private final RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();
   private final RecyclingArrayList<Point2D> leafPatches = new RecyclingArrayList<>(Point2D::new);
   private final RecyclingArrayList<Vector3D> boundaryVertices = new RecyclingArrayList<>(Vector3D::new);
   private final RecyclingArrayList<GDXGPURegionRing> regionRings = new RecyclingArrayList<>(GDXGPURegionRing::new);
   // TODO: kd tree
   private boolean normalCalculated;
   private boolean centerCalculated;
   private int numberOfPatches;
   private int id;
   private int poseId;
   private int numberOfMeasurements;
   private final SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
   private final DMatrixRMaj svdU = new DMatrixRMaj(3, 3);

   public void reset(int id)
   {
      this.id = id;
      patchCentroids.clear();
      planarPatchCentroids.clear();
      transformToWorldFrame.setIdentity();
      normal.setToZero();
      center.setToZero();

      leafPatches.clear();
      boundaryVertices.clear();
      regionRings.clear();
      normalCalculated = false;
      centerCalculated = false;
      numberOfPatches = 0;
      poseId = 0;
      numberOfMeasurements = 1;
   }

   public void addPatch(double nx, double ny, double nz, double cx, double cy, double cz)
   {
      normal.add(nx, ny, nz);
      center.add(cx, cy, cz);
      Point3D patchCentroid = patchCentroids.add();
      patchCentroid.set(cx, cy, cz);
      ++numberOfPatches;
   }

   public RecyclingArrayList<Vector3D> getBoundaryVertices()
   {
      return boundaryVertices;
   }

   public RecyclingArrayList<Point2D> getLeafPatches()
   {
      return leafPatches;
   }

   public RecyclingArrayList<GDXGPURegionRing> getRegionRings()
   {
      return regionRings;
   }

   public int getId()
   {
      return id;
   }

   public Point3D32 getCenter()
   {
      if (!centerCalculated)
      {
         centerCalculated = true;
         center.scale(1.0 / numberOfPatches);
      }
      return center;
   }

   public Vector3D32 getNormal()
   {
      if (!normalCalculated)
      {
         normalCalculated = true;
         getCenter();
         DMatrixRMaj patchMatrix = new DMatrixRMaj(3, patchCentroids.size());
         for (int i = 0; i < patchCentroids.size(); i++)
         {
            Point3D patchCentroid = patchCentroids.get(i);
            patchMatrix.set(0, i, patchCentroid.getX() - center.getX());
            patchMatrix.set(1, i, patchCentroid.getY() - center.getY());
            patchMatrix.set(2, i, patchCentroid.getZ() - center.getZ());
         }
         svd.decompose(patchMatrix);
         svd.getU(svdU, false);
         normal.set(svdU.get(6), svdU.get(7), svdU.get(8));
         normal.normalize();
         normal.scale(-normal.getZ() / Math.abs(normal.getZ()));
      }
      return normal;
   }
}
