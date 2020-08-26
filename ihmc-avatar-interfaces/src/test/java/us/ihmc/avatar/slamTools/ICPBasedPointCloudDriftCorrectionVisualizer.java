package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.List;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPBasedPointCloudDriftCorrectionVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 100.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private static final DriftCase DRIFT_CASE = DriftCase.YDrift;

   private static final String DATA_PATH = DRIFT_CASE.getFilePath();
   private static final int NUMBER_OF_POINTS_TO_VISUALIZE = 2000;

   private static final boolean VISUALIZE_OCTREE = false;

   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(true);
   private final static double OCTREE_RESOLUTION = 0.02;
   private NormalOcTree octreeMap;

   private SLAMFrame frame1;
   private SLAMFrame frame2;
   private SLAMFrame frameForSourcePoints;

   private final SLAMFrameYoGraphicsManager frame1GraphicsManager;
   private final SLAMFrameYoGraphicsManager frame2GraphicsManager;
   private final SLAMFrameYoGraphicsManager sourcePointsFrameGraphicsManager;

   private final YoDouble optimizerQuality;
   private final YoInteger numberOfCorrespondingPoints;
   private final YoInteger numberOfSourcePoints;

   private final AppearanceDefinition octreeMapColor = YoAppearance.AliceBlue();
   private final AppearanceDefinition frame1Appearance = YoAppearance.Blue();
   private final AppearanceDefinition frame2Appearance = YoAppearance.Green();
   private final AppearanceDefinition sourcePointsAppearance = YoAppearance.Red();

   public ICPBasedPointCloudDriftCorrectionVisualizer()
   {
      optimizerQuality = new YoDouble("optimizerQuality", registry);
      numberOfCorrespondingPoints = new YoInteger("numberOfCorrespondingPoints", registry);
      numberOfSourcePoints = new YoInteger("numberOfSourcePoints", registry);

      // Define frames .
      setupTest();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      frame1GraphicsManager = new SLAMFrameYoGraphicsManager("Frame1_",
                                                             frame1,
                                                             NUMBER_OF_POINTS_TO_VISUALIZE,
                                                             frame1Appearance,
                                                             registry,
                                                             yoGraphicsListRegistry);
      frame2GraphicsManager = new SLAMFrameYoGraphicsManager("Frame2_",
                                                             frame2,
                                                             NUMBER_OF_POINTS_TO_VISUALIZE,
                                                             frame2Appearance,
                                                             registry,
                                                             yoGraphicsListRegistry);
      sourcePointsFrameGraphicsManager = new SLAMFrameYoGraphicsManager("SourcePointFrame_",
                                                                        frameForSourcePoints,
                                                                        NUMBER_OF_POINTS_TO_VISUALIZE,
                                                                        sourcePointsAppearance,
                                                                        registry,
                                                                        yoGraphicsListRegistry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setGroundVisible(false);

      Graphics3DObject octreeGraphics = new Graphics3DObject();
      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(octreeMap.getRoot());
      for (NormalOcTreeNode node : iterable)
      {
         Vector3D normal = new Vector3D();
         Point3D hitLocation = new Point3D();
         node.getNormal(normal);
         node.getHitLocation(hitLocation);

         octreeGraphics.identity();
         octreeGraphics.translate(hitLocation);
         RotationMatrix rotation = new RotationMatrix();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(normal, rotation);
         octreeGraphics.rotate(rotation);
         octreeGraphics.addCube(OCTREE_RESOLUTION, OCTREE_RESOLUTION, OCTREE_RESOLUTION * 0.1, octreeMapColor);
      }
      if (VISUALIZE_OCTREE)
         scs.addStaticLinkGraphics(octreeGraphics);

      // define optimizer.
      LevenbergMarquardtParameterOptimizer optimizer = createOptimizer(octreeMap,
                                                                       frameForSourcePoints.getPointCloudInLocalFrame(),
                                                                       frameForSourcePoints.getUncorrectedLocalPoseInWorld());

      RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform();
      RigidBodyTransform correctedLocalPoseInWorld = new RigidBodyTransform(frameForSourcePoints.getUncorrectedLocalPoseInWorld());
      correctedLocalPoseInWorld.multiply(driftCorrectionTransform);

      List<Point3D> correctedData = frameForSourcePoints.getPointCloudInLocalFrame().stream().map(Point3D::new).collect(Collectors.toList());
      correctedData.forEach(driftCorrectionTransform::transform);

      frame1GraphicsManager.updateGraphics();
      frame2GraphicsManager.updateGraphics();
      sourcePointsFrameGraphicsManager.updateGraphics();
      scs.tickAndUpdate();

      for (double t = 1.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // do ICP.
         optimizer.iterate();

         // get parameter.
         optimizer.convertInputToTransform(optimizer.getOptimalParameter(), driftCorrectionTransform);
         correctedLocalPoseInWorld.set(frameForSourcePoints.getUncorrectedLocalPoseInWorld());
         correctedLocalPoseInWorld.multiply(driftCorrectionTransform);
         correctedData = frameForSourcePoints.getPointCloudInLocalFrame().stream().map(Point3D::new).collect(Collectors.toList());
         correctedData.forEach(driftCorrectionTransform::transform);

         frame2.updateOptimizedCorrection(driftCorrectionTransform);
         frameForSourcePoints.updateOptimizedCorrection(driftCorrectionTransform);

         // update viz.
         frame1GraphicsManager.updateGraphics();
         frame2GraphicsManager.updateGraphics();
         sourcePointsFrameGraphicsManager.updateGraphics();

         // update yo variables.   
         optimizerQuality.set(optimizer.getQuality());
         numberOfCorrespondingPoints.set(optimizer.getNumberOfCorrespondingPoints());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ICPBasedPointCloudDriftCorrectionVisualizer();
   }

   private LevenbergMarquardtParameterOptimizer createOptimizer(NormalOcTree map, List<Point3DReadOnly> sourcePointsInLocalFrame,
                                                                RigidBodyTransformReadOnly uncorrectedLocalPoseInWorld)
   {
      OutputCalculator outputCalculator = new OutputCalculator()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform(inputFunction.apply(inputParameter));
            RigidBodyTransform correctedLocalPoseToWorld = new RigidBodyTransform(uncorrectedLocalPoseInWorld);
            correctedLocalPoseToWorld.multiply(driftCorrectionTransform);

            List<Point3D> correctedData = sourcePointsInLocalFrame.stream().map(Point3D::new).collect(Collectors.toList());
            correctedData.forEach(correctedLocalPoseToWorld::transform);

            DMatrixRMaj errorSpace = new DMatrixRMaj(correctedData.size(), 1);
            for (int i = 0; i < correctedData.size(); i++)
            {
               double distance = computeClosestDistance(correctedData.get(i));
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Point3D point)
         {
            double surfelDistance = SLAMTools.computePerpendicularDistancePointToNormalOctree(map, point);
            return surfelDistance;
         }
      };
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction,
                                                                                                outputCalculator,
                                                                                                6,
                                                                                                sourcePointsInLocalFrame.size());
      DMatrixRMaj purterbationVector = new DMatrixRMaj(6, 1);
      purterbationVector.set(0, 0.0001);
      purterbationVector.set(1, 0.0001);
      purterbationVector.set(2, 0.0001);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      return optimizer;
   }

   private void setupTest()
   {
      // load data.
      File pointCloudFile = new File(DATA_PATH);
      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      frame1 = new SLAMFrame(messages.get(0), normalEstimationParameters);
      frame2 = new SLAMFrame(frame1, messages.get(1), normalEstimationParameters);

      // octree.
      Point3D dummySensorLocation = new Point3D();
      octreeMap = SLAMTools.computeOctreeData(frame1.getCorrectedPointCloudInWorld(), dummySensorLocation, OCTREE_RESOLUTION);

      // define source points.
      double windowMargin = 0.05;
      ConvexPolygon2D windowForMap = SLAMTools.computeMapConvexHullInSensorFrame(octreeMap, frame2.getCorrectedLocalPoseInWorld());

      List<? extends Point3DReadOnly> newPointCloud = frame2.getCorrectedPointCloudInWorld();
      List<Point3DReadOnly> newPointCloudToSensorPose = frame2.getPointCloudInLocalFrame();
      boolean[] isInPreviousView = new boolean[newPointCloudToSensorPose.size()];
      int numberOfPointsInWindow = 0;
      for (int i = 0; i < newPointCloudToSensorPose.size(); i++)
      {
         Point3DReadOnly point = newPointCloudToSensorPose.get(i);
         isInPreviousView[i] = false;
         if (windowForMap.isPointInside(point.getX(), point.getY(), -windowMargin))
         {
            isInPreviousView[i] = true;
            numberOfPointsInWindow++;
         }
      }

      // TODO: do icp for all points in window.
      Point3D[] pointsInPreviousWindow = new Point3D[numberOfPointsInWindow];
      int[] colors = new int[numberOfPointsInWindow];
      int indexOfPointsInWindow = 0;
      for (int i = 0; i < newPointCloudToSensorPose.size(); i++)
      {
         if (isInPreviousView[i])
         {
            pointsInPreviousWindow[indexOfPointsInWindow] = new Point3D(newPointCloud.get(i));
            indexOfPointsInWindow++;
         }
      }
      numberOfSourcePoints.set(numberOfPointsInWindow);

      StereoVisionPointCloudMessage dummyMessageForSourcePoints = PointCloudCompression.compressPointCloud(0612L,
                                                                                                           pointsInPreviousWindow,
                                                                                                           colors,
                                                                                                           pointsInPreviousWindow.length,
                                                                                                           0.005,
                                                                                                           null);
      dummyMessageForSourcePoints.getSensorPosition().set(frame2.getCorrectedSensorPoseInWorld().getTranslation());
      dummyMessageForSourcePoints.getSensorOrientation().set(frame2.getCorrectedSensorPoseInWorld().getRotation());
      frameForSourcePoints = new SLAMFrame(frame1, dummyMessageForSourcePoints, normalEstimationParameters);
   }
}
