package us.ihmc.avatar.slamTools;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

public class LevenbergMarquardtICPVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 50.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private final YoFramePoint3D modelLocation;
   private final YoFrameQuaternion modelOrientation;
   private final YoFramePoint3D dataLocation;
   private final YoFrameQuaternion dataOrientation;

   private final List<YoDouble[]> yoModelPointsHolder;
   private final List<YoDouble[]> yoDataPointsHolder;
   private final int numberOfPoints;

   public LevenbergMarquardtICPVisualizer()
   {
      modelLocation = new YoFramePoint3D("model_location_", worldFrame, registry);
      modelOrientation = new YoFrameQuaternion("model_orientation_", worldFrame, registry);
      dataLocation = new YoFramePoint3D("data_location_", worldFrame, registry);
      dataOrientation = new YoFrameQuaternion("data_orientation_", worldFrame, registry);

      YoGraphicCoordinateSystem modelFrame = new YoGraphicCoordinateSystem("model_", modelLocation, modelOrientation, 0.5, YoAppearance.Red());
      YoGraphicCoordinateSystem dataFrame = new YoGraphicCoordinateSystem("data_", dataLocation, dataOrientation, 0.5, YoAppearance.Blue());

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("model_graphics", modelFrame);
      yoGraphicsListRegistry.registerYoGraphic("data_graphics", dataFrame);

      String cowPLYPath = "C:\\PointCloudData\\PLY\\Cow\\cow.ply";
      File cowPointCloudFile = new File(cowPLYPath);
      Point3D[] modelPointCloud = getPointsFromFile(cowPointCloudFile);
      Point3D[] driftedCowPointCloud = new Point3D[modelPointCloud.length];
      numberOfPoints = modelPointCloud.length;

      RigidBodyTransform driftingTransform = new RigidBodyTransform();
      driftingTransform.setTranslationAndIdentityRotation(0.0, 0.0, 0.1);
      driftingTransform.appendRollRotation(Math.toRadians(30.0));
      driftingTransform.appendPitchRotation(Math.toRadians(-10.0));
      driftingTransform.appendRollRotation(Math.toRadians(5.0));
      for (int i = 0; i < driftedCowPointCloud.length; i++)
      {
         driftedCowPointCloud[i] = new Point3D(modelPointCloud[i]);
         driftingTransform.transform(driftedCowPointCloud[i]);
      }
      YoGraphicPosition[] modelPointCloudViz = new YoGraphicPosition[numberOfPoints];
      YoGraphicPosition[] dataPointCloudViz = new YoGraphicPosition[numberOfPoints];

      yoModelPointsHolder = new ArrayList<>();
      yoDataPointsHolder = new ArrayList<>();
      YoGraphicsList yoGraphicPointCloudListRegistry = new YoGraphicsList("model_pointcloudviz");
      for (int i = 0; i < numberOfPoints; i++)
      {
         YoDouble[] modelPointArray = new YoDouble[3];
         YoDouble[] dataPointArray = new YoDouble[3];
         for (int j = 0; j < 3; j++)
         {
            modelPointArray[j] = new YoDouble("ModelPoint_" + i + "_" + j, registry);
            dataPointArray[j] = new YoDouble("DataPoint_" + i + "_" + j, registry);
            modelPointArray[j].set(modelPointCloud[i].getElement(j));
            dataPointArray[j].set(driftedCowPointCloud[i].getElement(j));
         }
         yoModelPointsHolder.add(modelPointArray);
         yoDataPointsHolder.add(dataPointArray);

         modelPointCloudViz[i] = new YoGraphicPosition(i
               + "modelpointviz", yoModelPointsHolder.get(i)[0], yoModelPointsHolder.get(i)[1], yoModelPointsHolder.get(i)[2], 0.02, YoAppearance.Blue());

         dataPointCloudViz[i] = new YoGraphicPosition(i
               + "datapointviz", yoDataPointsHolder.get(i)[0], yoDataPointsHolder.get(i)[1], yoDataPointsHolder.get(i)[2], 0.02, YoAppearance.Red());

         yoGraphicPointCloudListRegistry.add(modelPointCloudViz[i]);
         yoGraphicPointCloudListRegistry.add(dataPointCloudViz[i]);
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicPointCloudListRegistry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);

      // octree.
      Point3D dummySensorLocation = new Point3D(0.0, 0.0, 2.0);
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = modelPointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(modelPointCloud, dummySensorLocation));

      double octreeResolution = 0.02;
      NormalOcTree octree = new NormalOcTree(octreeResolution);
      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // define optimizer.
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, driftedCowPointCloud.length);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            Point3D[] transformedData = new Point3D[driftedCowPointCloud.length];
            for (int i = 0; i < driftedCowPointCloud.length; i++)
               transformedData[i] = new Point3D(driftedCowPointCloud[i]);
            transformPointCloud(transformedData, inputParameter.getData());

            DenseMatrix64F errorSpace = new DenseMatrix64F(transformedData.length, 1);
            for (int i = 0; i < transformedData.length; i++)
            {
               double distance = computeClosestDistance(transformedData[i], modelPointCloud);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Point3D point, Point3D[] originalCowPointCloud)
         {
            double distance = SLAMTools.computeDistanceToNormalOctree(octree, point, 10);
            return distance;
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.00001);
      purterbationVector.set(1, 0.00001);
      purterbationVector.set(2, 0.00001);
      purterbationVector.set(3, 0.00001);
      purterbationVector.set(4, 0.00001);
      purterbationVector.set(5, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      optimizer.initialize();
      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // DO: ICP
         optimizer.iterate();
         DenseMatrix64F optimalParameter = optimizer.getOptimalParameter();
         Point3D[] transformedData = new Point3D[driftedCowPointCloud.length];
         for (int i = 0; i < driftedCowPointCloud.length; i++)
            transformedData[i] = new Point3D(driftedCowPointCloud[i]);
         transformPointCloud(transformedData, optimalParameter.getData());
         for (int i = 0; i < numberOfPoints; i++)
         {
            yoDataPointsHolder.get(i)[0].set(transformedData[i].getX());
            yoDataPointsHolder.get(i)[1].set(transformedData[i].getY());
            yoDataPointsHolder.get(i)[2].set(transformedData[i].getZ());
         }

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new LevenbergMarquardtICPVisualizer();
   }

   private void transformPointCloud(Point3D[] pointCloud, double... transformParameters)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(transformParameters[0], transformParameters[1], transformParameters[2]);
      transform.appendRollRotation(transformParameters[3]);
      transform.appendPitchRotation(transformParameters[4]);
      transform.appendYawRotation(transformParameters[5]);
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         transform.transform(point);
      }
   }

   // TODO: replace or remove with PLYAsciiFormatFileLoader.
   private Point3D[] getPointsFromFile(File dataFile)
   {
      if (!dataFile.canRead())
         new NullPointerException("No dataFile");

      BufferedReader bufferedReader = null;
      try
      {
         System.out.println(dataFile.getAbsolutePath());
         bufferedReader = new BufferedReader(new FileReader(dataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int numberOfVertex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] infoArray = null;
         try
         {
            lineJustFetched = bufferedReader.readLine();
            System.out.println(lineJustFetched);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched.contains("format"))
         {
            if (!lineJustFetched.contains("ascii"))
               return null;
         }
         if (lineJustFetched.contains("element vertex"))
         {
            infoArray = lineJustFetched.split(" ");
            numberOfVertex = Integer.parseInt(infoArray[2]);
            System.out.println("number of vertex is " + numberOfVertex);
         }
         if (lineJustFetched.contains("end_header"))
         {
            break;
         }
      }
      Point3D[] pointCloudBuffer = new Point3D[numberOfVertex];

      int indexOfPoints = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] xyzArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched == null)
         {
            break;
         }
         else
         {
            if (indexOfPoints == numberOfVertex)
               break;
            xyzArray = lineJustFetched.split(" ");
            pointCloudBuffer[indexOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]), Double.parseDouble(xyzArray[1]), Double.parseDouble(xyzArray[2]));
            indexOfPoints++;
         }
      }
      Point3D[] points = new Point3D[indexOfPoints];
      for (int i = 0; i < indexOfPoints; i++)
      {
         points[i] = pointCloudBuffer[i];
      }

      return points;
   }

}
