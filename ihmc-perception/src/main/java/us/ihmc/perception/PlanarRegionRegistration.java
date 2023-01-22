package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.slamWrapper.FactorGraph;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlanarRegionRegistration
{

   public static RigidBodyTransform registerRegionsToMap(PlanarRegionsList previousRegions, PlanarRegionsList currentRegions, HashMap<Integer, Integer> matches)
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
