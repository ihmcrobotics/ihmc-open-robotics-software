package us.ihmc.perception.tools;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.TIntIntMap;
import gnu.trove.map.hash.TIntIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMappingParameters;
import us.ihmc.perception.rapidRegions.PatchFeatureGrid;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlaneRegistrationTools
{
   /**
    * Computes the transform from the previous to the current planar regions list.
    * Uses approach described in the paper: https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
    * <p>
    * Use Iterative Linear Least Squares for quaternion averaging based estimation. Every iteration
    * moves the current planar regions closer to the previous planar regions. Usually 4-5 iterations are enough for convergence.
    *
    * @param previousRegions The previous planar regions list.
    * @param currentRegions  The current planar regions list.
    * @param transformToPack The transform from the previous to the current planar regions list.
    * @param parameters      The parameters for the registration.
    * @return True if the registration was successful, false otherwise.
    */
   public static boolean computeIterativeQuaternionAveragingBasedRegistration(PlanarLandmarkList previousRegions,
                                                                              PlanarLandmarkList currentRegions,
                                                                              RigidBodyTransform transformToPack,
                                                                              PlanarRegionMappingParameters parameters)
   {
      RigidBodyTransform transform;
      RigidBodyTransform finalTransform = new RigidBodyTransform();

      TIntIntMap matches = new TIntIntHashMap();

      LogTools.debug("Outside");
      PlaneRegistrationTools.findBestPlanarRegionMatches(currentRegions,
                                                         previousRegions,
                                                         matches,
                                                         (float) parameters.getBestMinimumOverlapThreshold(),
                                                         (float) parameters.getBestMatchAngularThreshold(),
                                                         (float) parameters.getBestMatchDistanceThreshold(),
                                                         (float) parameters.getMinimumBoundingBoxSize());
      if (matches.size() < parameters.getICPMinMatches())
      {
         LogTools.debug("Not enough matches, breaking out of IQA loop. Matches: {}", matches.size());
         return false;
      }

      double previousError = PlaneRegistrationTools.computeRegistrationError(previousRegions, currentRegions, matches);
      for (int i = 0; i < parameters.getICPMaxIterations(); i++)
      {
         matches.clear();
         PlaneRegistrationTools.findBestPlanarRegionMatches(currentRegions,
                                                            previousRegions,
                                                            matches,
                                                            (float) parameters.getBestMinimumOverlapThreshold(),
                                                            (float) parameters.getBestMatchAngularThreshold(),
                                                            (float) parameters.getBestMatchDistanceThreshold(),
                                                            (float) parameters.getMinimumBoundingBoxSize());

         if (matches.size() < parameters.getICPMinMatches())
         {
            LogTools.debug("Not enough matches, breaking out of IQA loop. Matches: {}", matches.size());
            break;
         }

         transform = PlaneRegistrationTools.computeQuaternionAveragingTransform(previousRegions, currentRegions, matches);

         if (transform.containsNaN())
         {
            LogTools.debug("Transform contains NaNs, breaking out of IQA loop. Transform: {}", transform);
            break;
         }

         currentRegions.applyTransform(transform);
         double error = PlaneRegistrationTools.computeRegistrationError(previousRegions, currentRegions, matches);
         double ratio = (previousError - error) / previousError;

         LogTools.debug("Iteration: {}, Error: {}, Ratio: {}", i, error, ratio);

         if ((Math.abs(ratio) < parameters.getICPTerminationRatio()) || (matches.size() < parameters.getICPMinMatches()))
         {
            LogTools.debug("Terminating IQA loop. Error ratio: {}, Matches: {}", ratio, matches.size());
            break;
         }

         if (previousError > parameters.getICPErrorCutoff())
         {
            LogTools.debug("Registration error too high, breaking out of IQA loop. Error: {}", previousError);
            break;
         }

         finalTransform.multiply(transform);
         previousError = error + 1e-7;
      }

      transformToPack.set(finalTransform);
      return true;
   }

   /**
    * Computes the transform from the previous to the current planar regions list using quaternion averaging based rotation estimation, and
    * orthogonal projection based translation estimation
    *
    * @param previousRegions The previous planar regions list.
    * @param currentRegions  The current planar regions list.
    * @param matches         The matches from current to previous region lists.
    * @return The transform from the current to previous planar region lists.
    */
   public static RigidBodyTransform computeQuaternionAveragingTransform(PlanarLandmarkList previousRegions,
                                                                        PlanarLandmarkList currentRegions,
                                                                        TIntIntMap matches)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();

      ArrayList<QuaternionReadOnly> quaternions = findRotationEstimates(previousRegions, currentRegions, matches);
      TDoubleArrayList weights = computeResidualWeights(previousRegions, currentRegions, matches, quaternions);
      Quaternion averageQuaternion = RotationTools.computeAverageQuaternion(quaternions, weights);

      Point3D averageTranslation = new Point3D();
      ArrayList<Point3DReadOnly> translations = findTranslationEstimates(previousRegions, currentRegions, matches);
      averageTranslation.set(computeAverageTranslation(translations));
      averageTranslation.negate();

      RotationMatrix averageRotation = new RotationMatrix(averageQuaternion);
      transformToReturn.set(averageRotation, averageTranslation);

      return transformToReturn;
   }

   /**
    * Computes the transform from the previous to the current planar regions list using least squares based optimization.
    *
    * @param previousRegions The previous planar regions list.
    * @param currentRegions  The current planar regions list.
    * @param matches         The matches between the previous and the current planar regions list.
    * @return The transform from the previous to the current planar regions list.
    */
   public static RigidBodyTransform computeTransformFromRegions(PlanarRegionsList previousRegions, PlanarRegionsList currentRegions, TIntIntMap matches)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();

      int[] keySet = matches.keySet().toArray();

      int totalNumOfBoundaryPoints = 0;
      for (Integer i : keySet)
      {
         totalNumOfBoundaryPoints += currentRegions.getPlanarRegionsAsList().get(matches.get(i)).getConcaveHullSize();
      }
      DMatrixRMaj A = new DMatrixRMaj(totalNumOfBoundaryPoints, 6);
      DMatrixRMaj b = new DMatrixRMaj(totalNumOfBoundaryPoints, 1);

      constructLeastSquaresProblem(previousRegions, currentRegions, matches, A, b);
      //DMatrixRMaj solution = solveUsingQRDecomposition(A, b);
      DMatrixRMaj solution = solveUsingSVDDecomposition(A, b);
      //DMatrixRMaj solution = solveUsingDampedLeastSquares(A, b);

      RotationMatrix rotation = new RotationMatrix(solution.get(2), solution.get(1), solution.get(0));
      Point3D translation = new Point3D(solution.get(3), solution.get(4), solution.get(5));
      transformToReturn.set(rotation, translation);

      return transformToReturn;
   }

   /**
    * Computes the translation from the current to the previous planar regions list using orthogonal distance minimization.
    *
    * @param previousRegions
    * @param currentRegions
    * @param matches
    * @return
    */
   public static ArrayList<Point3DReadOnly> findTranslationEstimates(PlanarLandmarkList previousRegions, PlanarLandmarkList currentRegions, TIntIntMap matches)
   {
      ArrayList<Point3DReadOnly> translations = new ArrayList<>();

      int[] keySet = matches.keySet().toArray();
      for (Integer i : keySet)
      {
         PlanarLandmark currentRegion = currentRegions.getPlanarLandmarksAsList().get(matches.get(i));
         PlanarLandmark previousRegion = previousRegions.getPlanarLandmarksAsList().get(i);

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

   /**
    * Computes the list of rotations (quaternions) from matching pairs from current to the previous planar regions list using plane-to-plane rotation
    * calculations.
    *
    * @param previousRegions The previous planar regions list.
    * @param currentRegions  The current planar regions list.
    * @param matches         The matches between the previous and the current planar regions list.
    * @return The list of rotations (quaternions) from matching pairs from current to the previous planar regions list.
    */
   public static ArrayList<QuaternionReadOnly> findRotationEstimates(PlanarLandmarkList previousRegions, PlanarLandmarkList currentRegions, TIntIntMap matches)
   {
      ArrayList<QuaternionReadOnly> quaternions = new ArrayList<>();

      int[] keySet = matches.keySet().toArray();
      for (Integer i : keySet)
      {
         PlanarLandmark currentRegion = currentRegions.getPlanarLandmarksAsList().get(matches.get(i));
         PlanarLandmark previousRegion = previousRegions.getPlanarLandmarksAsList().get(i);

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

   /**
    * Computes the average translation from the current to the previous planar regions list using orthogonal distance minimization.
    *
    * @param translations The list of translations from the current to the previous planar regions list.
    * @return The average translation from the current to the previous planar regions list.
    */
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

   /**
    * Computes the weights for quaternion averaging for the given list of quaternions corresponding to the matches between the previous and the current.
    * The weight for a quaternion represents the utility of that quaternion for finding the optimal quaternion average.
    *
    * @param previousRegions
    * @param currentRegions
    * @param matches
    * @param quaternions
    * @return
    */
   public static TDoubleArrayList computeResidualWeights(PlanarLandmarkList previousRegions,
                                                         PlanarLandmarkList currentRegions,
                                                         TIntIntMap matches,
                                                         ArrayList<QuaternionReadOnly> quaternions)
   {
      TDoubleArrayList weightsToPack = new TDoubleArrayList();

      RigidBodyTransform transform = new RigidBodyTransform();
      int[] keySet = matches.keySet().toArray();

      for (int i = 0; i < quaternions.size(); i++)
      {
         double weight = 0.0;
         transform.setRotationAndZeroTranslation(quaternions.get(i));

         for (Integer index : keySet)
         {
            PlanarLandmark currentRegion = currentRegions.getPlanarLandmarksAsList().get(matches.get(index));
            PlanarLandmark previousRegion = previousRegions.getPlanarLandmarksAsList().get(index);

            UnitVector3D currentNormal = new UnitVector3D(currentRegion.getNormal());
            UnitVector3D previousNormal = new UnitVector3D(previousRegion.getNormal());

            currentNormal.applyTransform(transform);
            weight += (previousNormal.dot(currentNormal));
         }

         weightsToPack.add(weight);
      }

      return weightsToPack;
   }

   /**
    * Constructs the linear least squares matrices A and b for registration between two planar regions lists.
    *
    * @param previousRegions The previous planar regions list.
    * @param currentRegions  The current planar regions list.
    * @param matches         The matches between the previous and the current planar regions list.
    * @param A               The matrix A in the linear least squares problem.
    * @param b               The vector b in the linear least squares problem.
    */
   public static void constructLeastSquaresProblem(PlanarRegionsList previousRegions,
                                                   PlanarRegionsList currentRegions,
                                                   TIntIntMap matches,
                                                   DMatrixRMaj A,
                                                   DMatrixRMaj b)
   {
      int i = 0;
      int[] keySet = matches.keySet().toArray();
      for (Integer m : keySet)
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
         }
      }
   }

   /**
    * Solves the linear least squares problem using SVD decomposition.
    *
    * @param A The matrix A in the linear least squares problem.
    * @param b The vector b in the linear least squares problem.
    * @return The solution to the linear least squares problem.
    */
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

         CommonOps_DDRM.mult(svdV, svdWInv, svdVWinv);
         CommonOps_DDRM.mult(svdVWinv, svdUt, svdInverse);
         CommonOps_DDRM.mult(svdInverse, b, solution);
      }

      return solution;
   }

   /**
    * Solves the linear least squares problem using damped least squares.
    *
    * @param A The matrix A in the linear least squares problem.
    * @param b The vector b in the linear least squares problem.
    * @return The solution to the linear least squares problem.
    */
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

   /**
    * Solves the linear least squares problem using QR decomposition.
    *
    * @param A The matrix A in the linear least squares problem.
    * @param b The vector b in the linear least squares problem.
    * @return The solution to the linear least squares problem.
    */
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

   /**
    * Computes the registration error between two planar regions lists.
    *
    * @param referenceRegions   The reference planar regions list.
    * @param transformedRegions The transformed planar regions list.
    * @param matches            The matches between the two planar regions lists.
    * @return The registration error.
    */
   public static double computeRegistrationError(PlanarLandmarkList referenceRegions, PlanarLandmarkList transformedRegions, TIntIntMap matches)
   {
      double error = 0.0;
      int[] keySet = matches.keys();
      for (Integer key : keySet)
      {
         PlanarLandmark referenceRegion = referenceRegions.getPlanarLandmarkById(key);
         PlanarLandmark transformedRegion = transformedRegions.getPlanarLandmarkById(matches.get(key));

         double cosineSimilarity = referenceRegion.getNormal().dot(transformedRegion.getNormal());

         Point3D translation = new Point3D();
         translation.sub(transformedRegion.getPoint(), referenceRegion.getPoint());
         double cosineTheta = Math.abs(translation.dot(referenceRegion.getNormal()) / translation.norm());
         translation.scale(cosineTheta);

         error += (1.0 - Math.abs(cosineSimilarity));
         //error += Math.min((translation.norm()) * 0.1, 1.0);
      }
      error /= matches.size();
      return error;
   }

   /**
    * Finds the best planar region matches between two planar regions lists.
    *
    * @param map               The map planar regions list.
    * @param incoming          The incoming planar regions list.
    * @param matches           The matches between the two planar regions lists.
    * @param overlapThreshold  The overlap threshold between two planar region bounding boxes in the world frame
    * @param normalThreshold   The normal cosine-similarity threshold between two planar regions
    * @param distanceThreshold The orthogonal distance threshold between two planar regions (origin-to-plane distance)
    * @param minBoxSize        The minimum size of the bounding box of a planar region
    */
   public static void findBestPlanarRegionMatches(PlanarLandmarkList map,
                                                  PlanarLandmarkList incoming,
                                                  TIntIntMap matches,
                                                  float overlapThreshold,
                                                  float normalThreshold,
                                                  float distanceThreshold,
                                                  float minBoxSize)
   {
      matches.clear();
      ArrayList<PlanarLandmark> newRegions = incoming.getPlanarLandmarksAsList();
      ArrayList<PlanarLandmark> mapRegions = map.getPlanarLandmarksAsList();
      Vector3D originVector = new Vector3D();

      for (int i = 0; i < newRegions.size(); i++)
      {
         double minSimilarity = Double.POSITIVE_INFINITY;
         int minSimilarityIndex = -1;

         UnitVector3DReadOnly newRegionNormal = newRegions.get(i).getNormal();
         Point3DReadOnly newRegionOrigin = newRegions.get(i).getPoint();

         for (int j = 0; j < mapRegions.size(); j++)
         {
            UnitVector3DReadOnly mapRegionNormal = mapRegions.get(j).getNormal();
            Point3DReadOnly mapRegionOrigin = mapRegions.get(j).getPoint();

            originVector.sub(newRegionOrigin, mapRegionOrigin);

            // TODO: Improve this logic with better metrics than origin-origin distance between regions.
            double distance = originVector.norm();
            double normalDistance = Math.abs(originVector.dot(mapRegionNormal));
            double normalSimilarity = mapRegionNormal.dot(newRegionNormal);

            if ((normalSimilarity < minSimilarity) && (distance < distanceThreshold) && (normalSimilarity > normalThreshold))
            {
               minSimilarity = normalSimilarity;
               minSimilarityIndex = j;
            }
         }

         if (minSimilarityIndex != -1)
         {
            matches.put(i, minSimilarityIndex);
         }
      }
   }

   // TODO: Complete this method and test it with PatchFeatureGrid from the RapidRegionsExtractor.
   public static void findPatchMatches(PatchFeatureGrid previousGrid, PatchFeatureGrid currentGrid, TIntIntMap matches)
   {
      Point3D previousCentroid = new Point3D();
      Point3D currentCentroid = new Point3D();
      Vector3D previousNormal = new Vector3D();
      Vector3D currentNormal = new Vector3D();

      float distanceThreshold = 0.1f;

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

   // TODO: Complete this method and test it with PatchFeatureGrid from the RapidRegionsExtractor.
   public static void computeTransformFromPatches(PatchFeatureGrid previousGrid,
                                                  PatchFeatureGrid currentGrid,
                                                  TIntIntMap matches,
                                                  RigidBodyTransform transformToPack)
   {
      SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
      DMatrixRMaj svdU = new DMatrixRMaj(3, 3);
      DMatrixRMaj svdVt = new DMatrixRMaj(3, 3);
      DMatrixRMaj patchMatrix = new DMatrixRMaj(3, 3);

      DMatrixRMaj matrixOne = new DMatrixRMaj(3, matches.size());
      DMatrixRMaj matrixTwo = new DMatrixRMaj(3, matches.size());

      int[] keySet = matches.keySet().toArray();

      Point3D previousCentroid = new Point3D();
      Point3D currentCentroid = new Point3D();
      Vector3D previousNormal = new Vector3D();
      Vector3D currentNormal = new Vector3D();

      Point3D previousMean = new Point3D();
      Point3D currentMean = new Point3D();

      int matrixIndex = 0;
      for (Integer key : keySet)
      {
         int currentIndex = matches.get(key);

         previousGrid.getCentroid(key, previousCentroid);
         previousGrid.getNormal(key, previousNormal);

         currentGrid.getCentroid(currentIndex, currentCentroid);
         currentGrid.getNormal(currentIndex, currentNormal);

         previousMean.add(previousCentroid);
         currentMean.add(currentCentroid);

         previousCentroid.get(0, matrixIndex, matrixOne);
         currentCentroid.get(0, matrixIndex, matrixTwo);

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

         transformToPack.setRotationAndZeroTranslation(rotationMatrix);
         transformToPack.appendTranslation(translation);

         Point3D angles = new Point3D();
         transformToPack.getRotation().getEuler(angles);
      }
   }

   public static void findPlanarRegionMatches(PlanarRegionsList map,
                                              PlanarRegionsList incoming,
                                              HashMap<Integer, TIntArrayList> matches,
                                              float overlapThreshold,
                                              float normalThreshold,
                                              float distanceThreshold,
                                              float minBoxSize)
   {
      matches.clear();
      List<PlanarRegion> newRegions = incoming.getPlanarRegionsAsList();
      List<PlanarRegion> mapRegions = map.getPlanarRegionsAsList();

      for (int i = 0; i < newRegions.size(); i++)
      {
         matches.put(i, new TIntArrayList());

         PlanarRegion newRegion = newRegions.get(i);

         for (int j = 0; j < mapRegions.size(); j++)
         {
            PlanarRegion mapRegion = mapRegions.get(j);

            if (checkRegionsForOverlap(newRegion, mapRegion, overlapThreshold, normalThreshold, distanceThreshold, minBoxSize, false))
            {
               matches.get(i).add(j);
            }
         }
      }
   }

   public static boolean checkRegionsForOverlap(PlanarRegion regionA,
                                                PlanarRegion regionB,
                                                float normalThreshold,
                                                float normalDistanceThreshold,
                                                float overlapThreshold,
                                                float minBoxSize,
                                                boolean useIntersectionOverUnion)
   {
      Point3D newOrigin = new Point3D();
      regionA.getOrigin(newOrigin);

      Point3D mapOrigin = new Point3D();
      regionB.getOrigin(mapOrigin);

      Vector3D originVector = new Vector3D();
      originVector.sub(newOrigin, mapOrigin);

      double normalDistance = 0;
      boolean intersects = false;
      double overlapScore = 0;
      double normalSimilarity = regionB.getNormal().dot(regionA.getNormal());

      // check to make sure the angles are similar enough
      boolean wasMatched = normalSimilarity > normalThreshold;
      if (wasMatched)
      {
         // check that the regions aren't too far out of plane with one another. TODO should check this normal distance measure. That's likely a problem
         normalDistance = Math.abs(originVector.dot(regionA.getNormal()));
         wasMatched &= normalDistance < normalDistanceThreshold;

         // TODO Check the logic for this minimum distance computation. In cases of vertical planar regions it generates incorrect distances.
         //         if(wasMatched)
         //         {
         //            closestDistance = planarRegionTools.getDistanceBetweenPlanarRegions(regionA, regionB);
         //            wasMatched &= closestDistance <= distanceThreshold;
         //         }

         // Check to make sure there is sufficient overlap in planar region world frame bounding boxes by computing Intersection-over-Smaller (IoS) score
         if (wasMatched)
         {
            overlapScore = computeBoundingBoxOverlapScore(regionA, regionB, minBoxSize, useIntersectionOverUnion);
            intersects = overlapScore > overlapThreshold;
            wasMatched &= intersects;
         }
      }

      //LogTools.debug("Match Metrics for [{}, {}]: " + String.format("Angular: %.2f [%.2f], Distance: %.2f [%.2f], Overlap: %.2f [%.2f]", normalSimilarity,
      //                  normalThreshold, normalDistance, normalDistanceThreshold, overlapScore, overlapThreshold), regionA.getRegionId(), regionB.getRegionId());

      return wasMatched;
   }

   public static double computeBoundingBoxOverlapScore(PlanarRegion a, PlanarRegion b, double minSize, boolean useIntersectionOverUnion)
   {
      BoundingBox3D boxA = PlanarRegionTools.getWorldBoundingBox3DWithMargin(a, minSize);
      BoundingBox3D boxB = PlanarRegionTools.getWorldBoundingBox3DWithMargin(b, minSize);

      if(useIntersectionOverUnion)
      {
         return GeometryTools.computeIntersectionOverUnionOfTwoBoundingBoxes(boxA, boxB);
      }
      else
      {
         return GeometryTools.computeIntersectionOverSmallerOfTwoBoundingBoxes(boxA, boxB);
      }

   }

   /**
    * Checks to see if the two regions intersect. This is done by checking if the normals are close to orthogonal and if the bounding boxes overlap.
    * @param regionA
    * @param regionB
    * @return true if the regions intersect, false otherwise
    */
   public static boolean checkRegionsForIntersection(PlanarRegion regionA,
                                                     PlanarRegion regionB)
   {
      // Get normal from region one
      Vector3D normalA = new Vector3D();
      regionA.getNormal(normalA);

      // Get normal from region two
      Vector3D normalB = new Vector3D();
      regionB.getNormal(normalB);

      // Find bounding box overlap score for the two regions
      double overlapScore = computeBoundingBoxOverlapScore(regionA, regionB, 0.01, true);

      // Check if normals are close to orthogonal
      if (Math.abs(normalA.dot(normalB)) < 0.2 && overlapScore > 0.02)
      {
         return true;
      }

      return false;
   }
}
