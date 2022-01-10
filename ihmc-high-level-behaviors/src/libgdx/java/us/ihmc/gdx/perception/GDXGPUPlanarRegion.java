package us.ihmc.gdx.perception;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;

public class GDXGPUPlanarRegion
{
   private final Vector3D32 normal = new Vector3D32();
   private final Point3D32 center = new Point3D32();
   private final RecyclingArrayList<Point3D> patchCentroids = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Point2D> planarPatchCentroids = new RecyclingArrayList<>(Point2D::new);
   private final RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();
   private final RecyclingArrayList<Point2D> leafPatches = new RecyclingArrayList<>(Point2D::new);
   private final RecyclingArrayList<Point3D> boundaryVertices = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<GDXGPURegionRing> regionRings = new RecyclingArrayList<>(GDXGPURegionRing::new);
   // TODO: kd tree
   private boolean normalCalculated;
   private boolean centroidCalculated;
   private int numberOfPatches;
   private int id;
   private int poseId;
   private int numberOfMeasurements;

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
      centroidCalculated = false;
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

   public void insertLeafPatch(int x, int y)
   {
      Point2D point = leafPatches.add();
      point.setX(x);
      point.setY(y);
   }

   public RecyclingArrayList<Point3D> getBoundaryVertices()
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
}
