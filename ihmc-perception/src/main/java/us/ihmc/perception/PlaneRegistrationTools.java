package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.rapidRegions.PatchFeatureGrid;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RotationTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;

public class PlaneRegistrationTools
{
   public static void findPatchMatches(PatchFeatureGrid previousGrid, PatchFeatureGrid currentGrid, HashMap<Integer, Integer> matches)
   {
      Point3D previousCentroid = new Point3D();
      Point3D currentCentroid = new Point3D();
      Vector3D previousNormal = new Vector3D();
      Vector3D currentNormal = new Vector3D();

      float distanceThreshold = 0.1f;

      LogTools.debug("Previous Size: {}, Current Size: {}", previousGrid.getTotal(), currentGrid.getTotal());

      for (int i = 0; i < previousGrid.getTotal(); i++)
      {
         previousGrid.getNormal(i, previousNormal);
         previousGrid.getCentroid(i, previousCentroid);
         previousNormal.normalize();

         for (int j = 0; j < currentGrid.getTotal(); j++)
         {
            currentGrid.getNormal(j, currentNormal);
            currentGrid.getCentroid(j, currentCentroid);
            currentNormal.normalize();

            double distance = currentCentroid.distance(previousCentroid);
            double similarity = previousNormal.dot(currentNormal);

            if (distance < distanceThreshold)
            {
               matches.put(i, j);
            }
         }
      }
   }

   public static void computeTransformFromPatches(PatchFeatureGrid previousGrid,
                                                  PatchFeatureGrid currentGrid,
                                                  HashMap<Integer, Integer> matches,
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

         previousCentroid.get(0, matrixIndex, matrixOne);
         currentCentroid.get(0, matrixIndex, matrixTwo);

         LogTools.debug("Matrix1: {}", matrixOne);
         LogTools.debug("Matrix2: {}", matrixTwo);

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

         LogTools.debug("Rotation Matrix: " + rotationMatrix);

         transformToPack.setRotationAndZeroTranslation(rotationMatrix);
         transformToPack.appendTranslation(translation);

         LogTools.debug("Transform: \n{}", transformToPack);

         Point3D angles = new Point3D();
         transformToPack.getRotation().getEuler(angles);

         LogTools.debug("Angles: {}", angles);
      }
   }

   /**
    * Computes the transform from the previous to the current planar regions list.
    * Uses approach described in the paper: https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
    * <p>
    * Use Iterative Linear Least Squares on SE3 tranform parameterized using the exponential map, as a tangent space vector. Every iteration
    * moves the current planar regions closer to the previous planar regions. Usually 4-5 iterations are enough for convergence. Minizes a similarity metric
    * between the corresponding planar regions (point-to-plane).
    *
    * @param previousRegions
    * @param currentRegions
    * @param maxIterations
    */
   public static boolean computeIterativeClosestPlane(PlanarRegionsList previousRegions, PlanarRegionsList currentRegions, int maxIterations,
                                                                 RigidBodyTransform transformToPack)
   {
      RigidBodyTransform transform;
      RigidBodyTransform finalTransform = new RigidBodyTransform();

      HashMap<Integer, Integer> matches = new HashMap<>();

      PlanarRegionSLAMTools.findBestPlanarRegionMatches(currentRegions, previousRegions, matches, 0.5f,
                                                        0.7f, 0.4f, 0.3f);
      double previousError = PlaneRegistrationTools.computeRegistrationError(previousRegions, currentRegions, matches);
      for (int i = 0; i < maxIterations; i++)
      {
         matches.clear();
         PlanarRegionSLAMTools.findBestPlanarRegionMatches(currentRegions, previousRegions, matches, 0.5f,
                                                            0.7f, 0.4f, 0.3f);
         transform = PlaneRegistrationTools.computeQuaternionAveragingTransform(previousRegions, currentRegions, matches);
         currentRegions.applyTransform(transform);

         double error = PlaneRegistrationTools.computeRegistrationError(previousRegions, currentRegions, matches);

         LogTools.info(String.format("Iteration: %d, Matches: %d, Previous Error: %.5f, Error: %.5f", i, matches.size(), previousError, error));
         if ( ((previousError - error) / previousError < 0.01) || (matches.size() < 5))
         {
            break;
         }

         finalTransform.multiply(transform);
         previousError = error;
      }

      LogTools.info("Size: {}", (int) ( (float)previousError / 0.0002f));

      LogTools.info("[{}]", ">".repeat(Math.abs((int) ((float) previousError / 0.0002f))) + ".".repeat(Math.max((int) ((0.02f - (float) previousError) / 0.0002f), 0)));

      if(previousError > 0.02)
      {
         LogTools.warn("ICP failed to converge, error: {}", previousError);
         return false;
      }

      transformToPack.set(finalTransform);
      return true;
   }

   public static RigidBodyTransform computeTransformFromRegions(PlanarRegionsList previousRegions,
                                                                PlanarRegionsList currentRegions,
                                                                HashMap<Integer, Integer> matches)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();

      int totalNumOfBoundaryPoints = 0;
      for (Integer i : matches.keySet())
      {
         totalNumOfBoundaryPoints += currentRegions.getPlanarRegionsAsList().get(matches.get(i)).getConcaveHullSize();
      }
      DMatrixRMaj A = new DMatrixRMaj(totalNumOfBoundaryPoints, 6);
      DMatrixRMaj b = new DMatrixRMaj(totalNumOfBoundaryPoints, 1);

      constructLeastSquaresProblem(previousRegions, currentRegions, matches, A, b);
      //DMatrixRMaj solution = solveUsingQRDecomposition(A, b);
      DMatrixRMaj solution = solveUsingSVDDecomposition(A, b);
      //DMatrixRMaj solution = solveUsingDampedLeastSquares(A, b);

      // TODO: Remove this
      //CommonOps_DDRM.scale(0.1, solution);

      //LogTools.info("PlanarICP: (A:({}, {}), b:({}))\n", A.getNumRows(), A.getNumCols(), b.getNumRows());
      //
      LogTools.info("[SVD] Rotation({}, {}, {})", solution.get(0), solution.get(1), solution.get(2));
      LogTools.info("[SVD] Translation({}, {}, {})", solution.get(3), solution.get(4), solution.get(5));
      //
      //LogTools.info("[QR] Rotation({}, {}, {})", solutionQR.get(0), solutionQR.get(1), solutionQR.get(2));
      //LogTools.info("[QR] Translation({}, {}, {})", solutionQR.get(3), solutionQR.get(4), solutionQR.get(5));

      RotationMatrix rotation = new RotationMatrix(solution.get(2), solution.get(1), solution.get(0));
      Point3D translation = new Point3D(solution.get(3), solution.get(4), solution.get(5));
      transformToReturn.set(rotation, translation);

      return transformToReturn;
   }

   public static RigidBodyTransform computeQuaternionAveragingTransform(PlanarRegionsList previousRegions,
                                                                        PlanarRegionsList currentRegions,
                                                                        HashMap<Integer, Integer> matches)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();

      ArrayList<QuaternionReadOnly> quaternions = findRotationEstimates(previousRegions, currentRegions, matches);
      Quaternion averageQuaternion = RotationTools.computeAverageQuaternion(quaternions);

      Point3D averageTranslation = new Point3D();
      ArrayList<Point3DReadOnly> translations = findTranslationEstimates(previousRegions, currentRegions, matches);
      averageTranslation.set(computeAverageTranslation(translations));
      averageTranslation.negate();

      RotationMatrix averageRotation = new RotationMatrix(averageQuaternion);
      transformToReturn.set(averageRotation, averageTranslation);

      return transformToReturn;
   }

   public static ArrayList<Point3DReadOnly> findTranslationEstimates(PlanarRegionsList previousRegions,
                                                                     PlanarRegionsList currentRegions,
                                                                     HashMap<Integer, Integer> matches)
   {
      ArrayList<Point3DReadOnly> translations = new ArrayList<>();

      for (Integer i : matches.keySet())
      {
         PlanarRegion currentRegion = currentRegions.getPlanarRegionsAsList().get(matches.get(i));
         PlanarRegion previousRegion = previousRegions.getPlanarRegionsAsList().get(i);

         Point3DReadOnly currentOrigin = currentRegion.getPoint();
         Point3DReadOnly previousOrigin = previousRegion.getPoint();

         Point3D translation = new Point3D();
         translation.sub(currentOrigin, previousOrigin);
         double cosineTheta = Math.abs(translation.dot(previousRegion.getNormal()) / translation.norm());
         translation.scale(cosineTheta);

         translations.add(translation);
      }

      return translations;
   }

   public static ArrayList<QuaternionReadOnly> findRotationEstimates(PlanarRegionsList previousRegions,
                                                                     PlanarRegionsList currentRegions,
                                                                     HashMap<Integer, Integer> matches)
   {
      ArrayList<QuaternionReadOnly> quaternions = new ArrayList<>();

      for (Integer i : matches.keySet())
      {
         PlanarRegion currentRegion = currentRegions.getPlanarRegionsAsList().get(matches.get(i));
         PlanarRegion previousRegion = previousRegions.getPlanarRegionsAsList().get(i);

         UnitVector3DReadOnly currentNormal = currentRegion.getNormal();
         UnitVector3DReadOnly previousNormal = previousRegion.getNormal();

         Quaternion quaternion = new Quaternion();
         Vector3D rotationAxis = new Vector3D();
         rotationAxis.cross(currentNormal, previousNormal);
         double rotationAngle = Math.acos(currentNormal.dot(previousNormal));

         quaternion.setAxisAngle(rotationAxis.getX(), rotationAxis.getY(), rotationAxis.getZ(), rotationAngle);
         quaternions.add(quaternion);
      }

      return quaternions;
   }

   public static Point3D computeAverageTranslation(ArrayList<Point3DReadOnly> translations)
   {
      Point3D averageTranslation = new Point3D();

      for (Point3DReadOnly translation : translations)
      {
         averageTranslation.add(translation);
      }

      averageTranslation.scale(1.0 / translations.size());

      return averageTranslation;
   }

   public static void constructLeastSquaresProblem(PlanarRegionsList previousRegions,
                                                   PlanarRegionsList currentRegions,
                                                   HashMap<Integer, Integer> matches,
                                                   DMatrixRMaj A,
                                                   DMatrixRMaj b)
   {
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

            Point3D latestPoint = PolygonizerTools.toPointInWorld(currentRegion.getConcaveHullVertex(n).getX(),
                                                                  currentRegion.getConcaveHullVertex(n).getY(),
                                                                  origin,
                                                                  orientation);
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

            //LogTools.info(String.format("[%d, %d] Point: (%.2f, %.2f, %.2f), Corresponding Point: (%.2f, %.2f, %.2f)", m, matches.get(m),
            //                            latestPoint.getX(),
            //                            latestPoint.getY(),
            //                            latestPoint.getZ(),
            //                            correspondingMapCentroid.getX(),
            //                            correspondingMapCentroid.getY(),
            //                            correspondingMapCentroid.getZ()));
            //
            //LogTools.info(String.format("[%d, %d] Normal: (%.2f, %.2f, %.2f), Corresponding Normal: (%.2f, %.2f, %.2f)", m, matches.get(m),
            //                            currentRegion.getNormal().getX(),
            //                            currentRegion.getNormal().getY(),
            //                            currentRegion.getNormal().getZ(),
            //                            correspondingMapNormal.getX(),
            //                            correspondingMapNormal.getY(),
            //                            correspondingMapNormal.getZ()));
         }
      }
   }

   public static DMatrixRMaj solveUsingSVDDecomposition(DMatrixRMaj A, DMatrixRMaj b)
   {
      SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);

      int singularCount = svd.numberOfSingularValues();

      DMatrixRMaj svdUt = new DMatrixRMaj(singularCount, A.numRows);
      DMatrixRMaj svdWInv = new DMatrixRMaj(singularCount, singularCount);
      DMatrixRMaj svdV = new DMatrixRMaj(singularCount, A.numCols);

      DMatrixRMaj solution = new DMatrixRMaj(6, 1);

      if (svd.decompose(A))
      {
         svd.getU(svdUt, true);
         svd.getV(svdV, false);
         svd.getW(svdWInv);
         CommonOps_DDRM.invert(svdWInv);
         CommonOps_DDRM.transpose(svdWInv);

         DMatrixRMaj svdVWinv = new DMatrixRMaj(6, A.numRows);
         DMatrixRMaj svdInverse = new DMatrixRMaj(6, A.numRows);

         //LogTools.info("SVD V, Winv, Ut: ({}, {}, {})", svdV, svdWInv, svdUt);

         CommonOps_DDRM.mult(svdV, svdWInv, svdVWinv);
         CommonOps_DDRM.mult(svdVWinv, svdUt, svdInverse);
         CommonOps_DDRM.mult(svdInverse, b, solution);
      }

      //DMatrixRMaj pInverse = new DMatrixRMaj(3, 3);
      //CommonOps_DDRM.pinv(A, pInverse);

      return solution;
   }

   public static DMatrixRMaj solveUsingDampedLeastSquares(DMatrixRMaj A, DMatrixRMaj b)
   {
      SolvePseudoInverseSvd_DDRM solver = new SolvePseudoInverseSvd_DDRM();
      DMatrixRMaj solution = new DMatrixRMaj(6, 1);

      DMatrixRMaj ATransposeTimesA = new DMatrixRMaj(6, 6);
      CommonOps_DDRM.multInner(A, ATransposeTimesA);

      DMatrixRMaj ATransposeB = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.multTransA(A, b, ATransposeB);

      DMatrixRMaj lambdaI = CommonOps_DDRM.identity(6);
      CommonOps_DDRM.scale(10, lambdaI);

      DMatrixRMaj ATransposeTimesAPlusLambdaI = new DMatrixRMaj(6, 6);
      CommonOps_DDRM.add(ATransposeTimesA, lambdaI, ATransposeTimesAPlusLambdaI);

      solver.setA(ATransposeTimesAPlusLambdaI);
      solver.solve(ATransposeB, solution);

      return solution;
   }

   public static DMatrixRMaj solveUsingQRDecomposition(DMatrixRMaj A, DMatrixRMaj b)
   {
      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.qr(A.numRows, A.numCols);
      if (!solver.setA(A))
      {
         throw new IllegalArgumentException("Singular matrix");
      }

      if (solver.quality() <= 1e-8)
      {
         throw new IllegalArgumentException("Nearly singular matrix");
      }

      DMatrixRMaj solution = new DMatrixRMaj(6, 1);
      solver.solve(b, solution);

      return solution;
   }

   public static double computeRegistrationError(PlanarRegionsList referenceRegions, PlanarRegionsList transformedRegions, HashMap<Integer, Integer> matches)
   {
      double error = 0.0;
      for(Integer key : matches.keySet())
      {
         PlanarRegion referenceRegion = referenceRegions.getPlanarRegion(key);
         PlanarRegion transformedRegion = transformedRegions.getPlanarRegion(matches.get(key));

         double cosineSimilarity = referenceRegion.getNormal().dot(transformedRegion.getNormal());

         Point3D translation = new Point3D();
         translation.sub(transformedRegion.getPoint(), referenceRegion.getPoint());
         double cosineTheta = Math.abs(translation.dot(referenceRegion.getNormal()) / translation.norm());
         translation.scale(cosineTheta);

         error += (1.0 - cosineSimilarity);
         error += Math.min((translation.norm()) * 0.5, 1.0);
      }
      error /= matches.size();
      return error;
   }
}
