package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.slam.tools.PLYasciiFormatFormatDataImporter;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LevenbergMarquardtICPVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final boolean ENABLE_PART_OF_MODEL = false;
   private final boolean ENABLE_PART_OF_DATA = true;

   private final double trajectoryTime = 50.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private final YoFramePoint3D modelLocation;
   private final YoFrameQuaternion modelOrientation;
   private final YoFramePoint3D dataLocation;
   private final YoFrameQuaternion dataOrientation;

   private final YoDouble locationDistance;
   private final YoDouble orientationDistance;
   private final YoDouble optimizerQuality;

   private final List<YoDouble[]> yoModelPointsHolder;
   private final List<YoDouble[]> yoDataPointsHolder;

   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(true);

   public LevenbergMarquardtICPVisualizer()
   {
      modelLocation = new YoFramePoint3D("model_location_", worldFrame, registry);
      modelOrientation = new YoFrameQuaternion("model_orientation_", worldFrame, registry);
      dataLocation = new YoFramePoint3D("data_location_", worldFrame, registry);
      dataOrientation = new YoFrameQuaternion("data_orientation_", worldFrame, registry);

      locationDistance = new YoDouble("locationDistance", registry);
      orientationDistance = new YoDouble("orientationDistance", registry);
      optimizerQuality = new YoDouble("optimizerQuality", registry);

      YoGraphicCoordinateSystem modelFrame = new YoGraphicCoordinateSystem("model_", modelLocation, modelOrientation, 0.5, YoAppearance.Red());
      YoGraphicCoordinateSystem dataFrame = new YoGraphicCoordinateSystem("data_", dataLocation, dataOrientation, 0.5, YoAppearance.Blue());

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("model_graphics", modelFrame);
      yoGraphicsListRegistry.registerYoGraphic("data_graphics", dataFrame);

      // Define model and data.
      List<Point3D> modelPointCloud = createModelPointCloud();
      List<Point3D> driftedCowPointCloud = creatDataPointCloud(modelPointCloud);
      if (ENABLE_PART_OF_MODEL)
      {
         BoundingBox3D subtractBox = new BoundingBox3D();
         subtractBox.set(-50.0, -10.0, 0.0, -1.5, 0.0, 1.5);
         modelPointCloud = subtractPointCloud(modelPointCloud, subtractBox);
      }

      YoGraphicPosition[] modelPointCloudViz = new YoGraphicPosition[modelPointCloud.size()];
      YoGraphicPosition[] dataPointCloudViz = new YoGraphicPosition[driftedCowPointCloud.size()];

      yoModelPointsHolder = new ArrayList<>();
      yoDataPointsHolder = new ArrayList<>();
      YoGraphicsList yoGraphicModelPointCloudListRegistry = new YoGraphicsList("model_pointcloudviz");
      YoGraphicsList yoGraphicDataPointCloudListRegistry = new YoGraphicsList("data_pointcloudviz");
      for (int i = 0; i < modelPointCloudViz.length; i++)
      {
         YoDouble[] modelPointArray = new YoDouble[3];
         for (int j = 0; j < 3; j++)
         {
            modelPointArray[j] = new YoDouble("ModelPoint_" + i + "_" + j, registry);
            modelPointArray[j].set(modelPointCloud.get(i).getElement(j));
         }
         yoModelPointsHolder.add(modelPointArray);

         modelPointCloudViz[i] = new YoGraphicPosition(i
               + "modelpointviz", yoModelPointsHolder.get(i)[0], yoModelPointsHolder.get(i)[1], yoModelPointsHolder.get(i)[2], 0.02, YoAppearance.Blue());

         yoGraphicModelPointCloudListRegistry.add(modelPointCloudViz[i]);
      }
      for (int i = 0; i < dataPointCloudViz.length; i++)
      {
         YoDouble[] dataPointArray = new YoDouble[3];
         for (int j = 0; j < 3; j++)
         {
            dataPointArray[j] = new YoDouble("DataPoint_" + i + "_" + j, registry);
            dataPointArray[j].set(driftedCowPointCloud.get(i).getElement(j));
         }
         yoDataPointsHolder.add(dataPointArray);

         dataPointCloudViz[i] = new YoGraphicPosition(i
               + "datapointviz", yoDataPointsHolder.get(i)[0], yoDataPointsHolder.get(i)[1], yoDataPointsHolder.get(i)[2], 0.02, YoAppearance.Red());

         yoGraphicDataPointCloudListRegistry.add(dataPointCloudViz[i]);
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicModelPointCloudListRegistry);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicDataPointCloudListRegistry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);

      // octree.
      Point3D dummySensorLocation = new Point3D();
      double octreeResolution = 0.02;
      NormalOcTree octree = SLAMTools.computeOctreeData(modelPointCloud, dummySensorLocation, octreeResolution);

      // define optimizer.
      LevenbergMarquardtParameterOptimizer optimizer = createOptimizer(octree, driftedCowPointCloud);

      Point3D initialDataLocation = new Point3D(dataLocation);
      RotationMatrix initialDataOrientation = new RotationMatrix(dataOrientation);
      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // do ICP.
         optimizer.iterate();

         // update viz.
         DMatrixRMaj optimalParameter = optimizer.getOptimalParameter();

         RigidBodyTransform transform = new RigidBodyTransform();
         optimizer.convertInputToTransform(optimalParameter, transform);
         List<Point3D> transformedData = driftedCowPointCloud.stream().map(Point3D::new).collect(Collectors.toList());
         transformedData.forEach(transform::transform);

         dataLocation.set(initialDataLocation);
         dataOrientation.set(initialDataOrientation);
         transform.transform(dataLocation);
         transform.transform(dataOrientation);

         // update yo variables.   
         locationDistance.set(dataLocation.distance(modelLocation));
         orientationDistance.set(dataOrientation.distance(modelOrientation));
         optimizerQuality.set(optimizer.getQuality());

         for (int i = 0; i < yoDataPointsHolder.size(); i++)
         {
            yoDataPointsHolder.get(i)[0].set(transformedData.get(i).getX());
            yoDataPointsHolder.get(i)[1].set(transformedData.get(i).getY());
            yoDataPointsHolder.get(i)[2].set(transformedData.get(i).getZ());
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

   private void transformPointCloud(Point3D[] pointCloud, RigidBodyTransform transform)
   {
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         transform.transform(point);
      }
   }

   private LevenbergMarquardtParameterOptimizer createOptimizer(NormalOcTree map, List<Point3D> newPointCloud)
   {
      OutputCalculator outputCalculator = new OutputCalculator()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            RigidBodyTransform transform = new RigidBodyTransform(inputFunction.apply(inputParameter));
            List<Point3D> transformedData = newPointCloud.stream().map(Point3D::new).collect(Collectors.toList());
            transformedData.forEach(transform::transform);

            DMatrixRMaj errorSpace = new DMatrixRMaj(transformedData.size(), 1);
            for (int i = 0; i < transformedData.size(); i++)
            {
               double distance = computeClosestDistance(transformedData.get(i));
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Point3D point)
         {
            double distance = SLAMTools.computeDistancePointToNormalOctree(map, point);
            return distance;
         }
      };
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction, outputCalculator, 6, newPointCloud.size());
      DMatrixRMaj purterbationVector = new DMatrixRMaj(6, 1);
      purterbationVector.set(0, 0.00001);
      purterbationVector.set(1, 0.00001);
      purterbationVector.set(2, 0.00001);
      purterbationVector.set(3, 0.00001);
      purterbationVector.set(4, 0.00001);
      purterbationVector.set(5, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.3);

      return optimizer;
   }

   private List<Point3D> subtractPointCloud(List<Point3D> pointCloud, BoundingBox3D subtractBox)
   {
      if (subtractBox == null)
         return pointCloud;

      List<Point3D> partOfPointCloud = new ArrayList<>();
      for (int i = 0; i < pointCloud.size(); i++)
      {
         if (!subtractBox.isInsideEpsilon(pointCloud.get(i), 0.01))
            partOfPointCloud.add(pointCloud.get(i));
      }
      List<Point3D> newPointCloud = new ArrayList<>();
      for (int i = 0; i < pointCloud.size(); i++)
         newPointCloud.add(new Point3D(partOfPointCloud.get(i)));

      return newPointCloud;
   }

   private List<Point3D> createModelPointCloud()
   {
      String cowPLYPath = "C:\\PointCloudData\\PLY\\Cow\\cow.ply";
      File cowPointCloudFile = new File(cowPLYPath);
      Point3D[] modelPointCloud = PLYasciiFormatFormatDataImporter.getPointsFromFile(cowPointCloudFile);

      RigidBodyTransform initializingTransform = new RigidBodyTransform();
      initializingTransform.appendRollRotation(Math.toRadians(90.0));
      for (int i = 0; i < modelPointCloud.length; i++)
      {
         initializingTransform.transform(modelPointCloud[i]);
      }

      return Arrays.asList(modelPointCloud);
   }

   private List<Point3D> creatDataPointCloud(List<? extends Point3DReadOnly> modelPointCloud)
   {
      List<Point3D> driftedCowPointCloud = new ArrayList<>();
      for (int i = 0; i < modelPointCloud.size(); i++)
      {
         driftedCowPointCloud.add(new Point3D(modelPointCloud.get(i)));
      }

      if (ENABLE_PART_OF_DATA)
      {
         BoundingBox3D subtractBox = new BoundingBox3D();
         subtractBox.set(0.0, 0.0, 1.0, 2.0, 2.0, 50.0);
         driftedCowPointCloud = subtractPointCloud(driftedCowPointCloud, subtractBox);
      }

      RigidBodyTransform driftingTransform = new RigidBodyTransform();
      driftingTransform.setTranslationAndIdentityRotation(0.15, -0.23, 1.4);
      driftingTransform.appendRollRotation(Math.toRadians(30.0));
      driftingTransform.appendPitchRotation(Math.toRadians(-10.0));
      driftingTransform.appendYawRotation(Math.toRadians(30.0));
      for (int i = 0; i < driftedCowPointCloud.size(); i++)
      {
         driftingTransform.transform(driftedCowPointCloud.get(i));
      }
      driftingTransform.transform(dataLocation);
      driftingTransform.transform(dataOrientation);

      return driftedCowPointCloud;
   }
}
