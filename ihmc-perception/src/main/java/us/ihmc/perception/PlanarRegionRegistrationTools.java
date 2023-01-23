package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.rapidRegions.PatchFeatureGrid;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;
import java.util.Set;

public class PlanarRegionRegistrationTools
{

   public static void findPatchMatches(PatchFeatureGrid previousGrid, PatchFeatureGrid currentGrid, HashMap<Integer, Integer> matches)
   {
      Point3D previousCentroid = new Point3D();
      Point3D currentCentroid = new Point3D();
      Vector3D previousNormal = new Vector3D();
      Vector3D currentNormal = new Vector3D();

      float distanceThreshold = 0.1f;

      LogTools.info("Previous Size: {}, Current Size: {}", previousGrid.getTotal(), currentGrid.getTotal());

      for(int i = 0; i<previousGrid.getTotal(); i++)
      {
         previousGrid.getNormal(i, previousNormal);
         previousGrid.getCentroid(i, previousCentroid);
         previousNormal.normalize();

         for(int j = 0; j<currentGrid.getTotal(); j++)
         {
            currentGrid.getNormal(j, currentNormal);
            currentGrid.getCentroid(j, currentCentroid);
            currentNormal.normalize();

            double distance = EuclidGeometryTools.distanceBetweenPoint3Ds(previousCentroid.getX(), previousCentroid.getY(), previousCentroid.getZ(),
                                                                          currentCentroid.getX(), currentCentroid.getY(), currentCentroid.getZ());

            double similarity = previousNormal.dot(currentNormal);

            if(distance < distanceThreshold)
            {
               matches.put(i, j);
            }
         }
      }
   }

   public static void computeTransformFromPatches(PatchFeatureGrid previousGrid, PatchFeatureGrid currentGrid, HashMap<Integer, Integer> matches,
                                          RigidBodyTransform transformToPack)
   {
      SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
      DMatrixRMaj svdU = new DMatrixRMaj(3, 3);
      DMatrixRMaj svdVt = new DMatrixRMaj(3, 3);
      DMatrixRMaj patchMatrix = new DMatrixRMaj(3, 3);

      DMatrixRMaj matrixOne = new DMatrixRMaj(3, matches.size());
      DMatrixRMaj matrixTwo = new DMatrixRMaj(3, matches.size());

      Set<Integer> keys = matches.keySet();

      Point3D previousCentroid = new Point3D();
      Point3D currentCentroid = new Point3D();
      Vector3D previousNormal = new Vector3D();
      Vector3D currentNormal = new Vector3D();

      Point3D previousMean = new Point3D();
      Point3D currentMean = new Point3D();


      int matrixIndex = 0;
      for (Integer key : matches.keySet())
      {
         previousGrid.getCentroid(key, previousCentroid);
         previousGrid.getNormal(key, previousNormal);

         currentGrid.getCentroid(matches.get(key), currentCentroid);
         currentGrid.getNormal(matches.get(key), currentNormal);

         previousMean.add(previousCentroid);
         currentMean.add(currentCentroid);

         matrixOne.set(0, matrixIndex, previousCentroid.getX());
         matrixOne.set(1, matrixIndex, previousCentroid.getY());
         matrixOne.set(2, matrixIndex, previousCentroid.getZ());

         matrixTwo.set(0, matrixIndex, currentCentroid.getX());
         matrixTwo.set(1, matrixIndex, currentCentroid.getY());
         matrixTwo.set(2, matrixIndex, currentCentroid.getZ());

         LogTools.info("Matrix1: {}", matrixOne);
         LogTools.info("Matrix2: {}", matrixTwo);

         CommonOps_DDRM.multAddTransB(matrixOne, matrixTwo, patchMatrix);

         matrixIndex++;
      }

      previousMean.scale(1.0 / matches.size());
      currentMean.scale(1.0 / matches.size());

      Point3D translation = new Point3D();
      translation.set(currentMean);
      translation.sub(previousMean);

      if (svd.decompose(patchMatrix))
      {
         svd.getU(svdU, false);
         svd.getV(svdVt, true);

         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.mult(svdU, svdVt, rotationMatrix);

         LogTools.info("Rotation Matrix: " + rotationMatrix);

         transformToPack.setRotationAndZeroTranslation(rotationMatrix);
         transformToPack.appendTranslation(translation);

         LogTools.info("Transform: \n{}", transformToPack);

         Point3D angles = new Point3D();
         transformToPack.getRotation().getEuler(angles);

         LogTools.info("Angles: {}", angles);
      }
   }

   public static RigidBodyTransform computeTransformFromRegions(PlanarRegionsList previousRegions, PlanarRegionsList currentRegions, HashMap<Integer, Integer> matches)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();

      int totalNumOfBoundaryPoints = 0;
      for (Integer i : matches.keySet())
      {
         totalNumOfBoundaryPoints += currentRegions.getPlanarRegionsAsList().get(matches.get(i)).getConcaveHullSize();
      }
      DMatrixRMaj A = new DMatrixRMaj(totalNumOfBoundaryPoints, 6);
      DMatrixRMaj b = new DMatrixRMaj(totalNumOfBoundaryPoints, 1);

      int i = 0;
      for (Integer m : matches.keySet())
      {
         PlanarRegion previousRegion = previousRegions.getPlanarRegionsAsList().get(m);
         PlanarRegion currentRegion = currentRegions.getPlanarRegionsAsList().get(matches.get(m));
         for (int n = 0; n < currentRegion.getConcaveHullSize(); n++)
         {
            Point3D origin = new Point3D();
            Quaternion orientation = new Quaternion();
            currentRegion.getTransformToLocal().get(orientation, origin);

            Point3D latestPoint = PolygonizerTools.toPointInWorld(currentRegion.getConcaveHullVertex(n).getX(), currentRegion.getConcaveHullVertex(n).getX(), origin, orientation);
            Vector3D latestPointVector = new Vector3D(latestPoint);

            Point3D correspondingMapCentroid = new Point3D();
            previousRegion.getOrigin(correspondingMapCentroid);


            Vector3D correspondingMapNormal = new Vector3D();
            previousRegion.getNormal(correspondingMapNormal);

            Vector3D crossVec = new Vector3D();
            crossVec.cross(latestPointVector, correspondingMapNormal);

            A.set(i, 0, crossVec.getX());
            A.set(i, 1, crossVec.getY());
            A.set(i, 2, crossVec.getZ());
            A.set(i, 3, correspondingMapNormal.getX());
            A.set(i, 4, correspondingMapNormal.getY());
            A.set(i, 5, correspondingMapNormal.getZ());

            Point3D diff = new Point3D();
            diff.sub(latestPoint, correspondingMapCentroid);

            b.set(i, -diff.dot(correspondingMapNormal));
            i++;
         }
      }

      LogTools.info("PlanarICP: (A:({}, {}), b:({}))\n", A.getNumRows(), A.getNumCols(), b.getNumRows());


      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.qr(A.numRows, A.numCols);
      if( !solver.setA(A) ) {
         throw new IllegalArgumentException("Singular matrix");
      }

      if( solver.quality() <= 1e-8 )
      {
         throw new IllegalArgumentException("Nearly singular matrix");
      }

      DMatrixRMaj solution = new DMatrixRMaj(6,1);
      solver.solve(b, solution);

//      eulerAnglesToReference = Eigen::Vector3d((double) solution(0), (double) solution(1), (double) solution(2));
//      translationToReference = Eigen::Vector3d((double) solution(3), (double) solution(4), (double) solution(5));

      LogTools.info("ICP Result: Rotation({}}, {}}, {}})", solution.get(0), solution.get(1), solution.get(2));
      LogTools.info( "Translation({}}, {}}, {}})", solution.get(3), solution.get(4), solution.get(5));

      /* Update relative and total transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */
//      _sensorPoseRelative.SetAnglesAndTranslation(eulerAnglesToReference, translationToReference);
//      _sensorToMapTransform.MultiplyRight(_sensorPoseRelative);

      return transformToReturn;
   }
}
