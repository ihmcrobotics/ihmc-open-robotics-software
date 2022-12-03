package us.ihmc.perception;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
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
   private SlamWrapper.FactorGraphExternal factorGraph;
   private int frameIndex = 0;
   private boolean modified = false;

   private PlanarRegionsList currentRegions;
   private PlanarRegionsList previousRegions;

//   private String regionFilePath = "/home/quantum/Workspace/Code/MapSense/Data/Extras/Regions/Archive/Set_06_Circle/";
   private String regionFilePath = "/home/bmishra/Workspace/Code/MapSenseROS/Extras/Regions/Archive/Set_06_Circle/";



   public PlanarRegionRegistration()
   {
      factorGraph = new SlamWrapper.FactorGraphExternal();
      previousRegions = new PlanarRegionsList();
      currentRegions = loadRegions(regionFilePath + "0000.txt", 0);

      factorGraph.addPriorPoseFactor(1, new float[]{0,0,0,0,0,0});
   }

   public void incrementIndex()
   {
      frameIndex += 20;
   }

   public void update()
   {
      String fileName = String.format("%1$4s", frameIndex).replace(' ', '0') + ".txt";
      LogTools.info("Loading File: {}", fileName);

      if (currentRegions.getNumberOfPlanarRegions() != 0)
      {
         previousRegions.clear();
         previousRegions.addPlanarRegionsList(currentRegions);
      }

      currentRegions = loadRegions(regionFilePath + fileName, frameIndex);

      HashMap<Integer, Integer> matches = PlanarRegionSLAMTools.findPlanarRegionMatches(previousRegions, currentRegions, 0.1f, 0.5f);

      RigidBodyTransform transform = registerRegionsToMap(previousRegions, currentRegions, matches);

      Vector3DBasics translation = transform.getTranslation();

      Tuple3DBasics eulerAngles = new Point3D();
      transform.getRotation().getEuler(eulerAngles);

      factorGraph.addOdometryFactor(new float[]{translation.getX32(), translation.getY32(), translation.getZ32(), eulerAngles.getX32(), eulerAngles.getY32(), eulerAngles.getZ32()},2);

      // TODO: Convert to world frame and insert the initial value

      factorGraph.optimize();

      LogTools.info("Previous: {} Current: {} Matches: {}",
                    previousRegions.getPlanarRegion(0).getConcaveHullSize(),
                    currentRegions.getPlanarRegion(0).getConcaveHullSize(),
                    matches.size());

      this.modified = true;
   }

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

   public static String getValue(BufferedReader reader, String key) throws IOException
   {
      String line = reader.readLine();
      //LogTools.info("Line: {}", line);
      String[] words;
      words = line.split(":");
      if (key.equals(words[0]))
      {
         return words[1];
      }
      else
         return null;
   }

   public static Point3D getPoint3D(BufferedReader reader, String key) throws IOException
   {
      String line = reader.readLine();
      //LogTools.info("Line (Point3D): {}", line);

      String[] words;
      String[] coordinates = null;
      words = line.split(":");

      if (key == null)
         coordinates = words[0].replace(" ", "").split(",");
      else if (words[0].equals(key))
         coordinates = words[1].replace(" ", "").split(",");

      if (coordinates != null)
         return new Point3D(Float.parseFloat(coordinates[0]), Float.parseFloat(coordinates[1]), Float.parseFloat(coordinates[2]));
      else
         return null;
   }

   public static PlanarRegionsList loadMapsensePlanarRegionsFromFile(File file,
                                                                     PolygonizerParameters polygonizerParameters,
                                                                     ConcaveHullFactoryParameters concaveHullFactoryParameters)
   {
      PlanarRegionsList listToReturn = null;
      List<PlanarRegionSegmentationRawData> planarRegionRawDataList = new ArrayList<>();

      String value = null;
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(file));

         int numRegions = 0;
         value = getValue(reader, "NumRegions");
         LogTools.info("NumRegions: {}", value);
         if (value != null)
         {
            numRegions = Integer.parseInt(value);
            for (int r = 0; r < numRegions; r++)
            {

               int regionId = Integer.parseInt(getValue(reader, "RegionID"));
               Point3D origin = getPoint3D(reader, "Center");
               Point3D normal = getPoint3D(reader, "Normal");

               AxisAngle orientation = new AxisAngle();
               orientation.set(EuclidGeometryTools.axisAngleFromZUpToVector3D(new Vector3D(normal)));
               RigidBodyTransform transformToWorld = new RigidBodyTransform(orientation, origin);

               List<Point3D> loadedPoints = new ArrayList<>();

               int numPatches = Integer.parseInt(getValue(reader, "NumPatches"));

               for (int patchIndex = 0; patchIndex < numPatches; patchIndex++)
               {
                  Point3D point = getPoint3D(reader, null);
                  loadedPoints.add(point);
               }

               PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(regionId, new Vector3D(normal), origin, loadedPoints);

               planarRegionRawDataList.add(data);
            }

            listToReturn = PlanarRegionPolygonizer.createPlanarRegionsList(planarRegionRawDataList, concaveHullFactoryParameters, polygonizerParameters);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      return listToReturn;
   }

   public PlanarRegionsList loadRegions(String path, int index)
   {
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      ;
      concaveHullFactoryParameters.setEdgeLengthThreshold(0.224);
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(false);
      concaveHullFactoryParameters.setMaxNumberOfIterations(5000);
      concaveHullFactoryParameters.setTriangulationTolerance(0.0);

      PlanarRegionsList regions = loadMapsensePlanarRegionsFromFile(new File(path), polygonizerParameters, concaveHullFactoryParameters);

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
         LogTools.info("Regions: {}", region.getConcaveHullSize());

      return regions;
   }

   public boolean modified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }

   public PlanarRegionsList getCurrentRegions()
   {
      return currentRegions;
   }
}
