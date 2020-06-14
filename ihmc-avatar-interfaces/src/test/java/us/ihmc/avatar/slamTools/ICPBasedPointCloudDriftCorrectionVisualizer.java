package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

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
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ICPBasedPointCloudDriftCorrectionVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 100.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   //private static final String DATA_PATH = "C:\\PointCloudData\\Data\\20200305_Simple\\PointCloud\\"; // 30 vs 33 : small drift, 33 vs 34 : drift and left right big movement.
   //private static final String DATA_PATH = "C:\\PointCloudData\\Data\\20200601_LidarWalking_DownStairs\\PointCloud\\";  // 12 vs 13 : drift
   private static final String DATA_PATH = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\"; // 3 vs 4 : drift in Y. (Great)
   //private static final String DATA_PATH = "C:\\PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\PointCloud\\"; // 4 vs 5 : point to point can not resolve the drift in Y for this frames.
   private static final int INDEX_FRAME_ONE = 3;
   private static final int INDEX_FRAME_TWO = 4;
   private static final int NUMBER_OF_POINTS_TO_VISUALIZE = 2000;

   private static final boolean VISUALIZE_OCTREE = false;

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
      scs.addYoVariableRegistry(registry);
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
                                                                       frameForSourcePoints.getOriginalPointCloudToSensorPose(),
                                                                       frameForSourcePoints.getInitialSensorPoseToWorld());

      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frameForSourcePoints.getInitialSensorPoseToWorld());
      correctedSensorPoseToWorld.multiply(icpTransformer);

      Point3D[] correctedData = new Point3D[frameForSourcePoints.getOriginalPointCloudToSensorPose().length];
      for (int i = 0; i < correctedData.length; i++)
      {
         correctedData[i] = new Point3D(frameForSourcePoints.getOriginalPointCloudToSensorPose()[i]);
         icpTransformer.transform(correctedData[i]);
      }

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
         icpTransformer.set(convertTransform(optimizer.getOptimalParameter().getData()));
         correctedSensorPoseToWorld.set(frameForSourcePoints.getInitialSensorPoseToWorld());
         correctedSensorPoseToWorld.multiply(icpTransformer);
         for (int i = 0; i < correctedData.length; i++)
         {
            correctedData[i].set(frameForSourcePoints.getOriginalPointCloudToSensorPose()[i]);
            icpTransformer.transform(correctedData[i]);
         }

         frame2.updateOptimizedCorrection(icpTransformer);
         frameForSourcePoints.updateOptimizedCorrection(icpTransformer);

         // update viz.
         frame1GraphicsManager.updateGraphics();
         frame2GraphicsManager.updateGraphics();
         sourcePointsFrameGraphicsManager.updateGraphics();

         // update yo variables.   
         optimizerQuality.set(optimizer.getQuality());
         numberOfCorrespondingPoints.set(optimizer.getNumberOfCoorespondingPoints());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ICPBasedPointCloudDriftCorrectionVisualizer();
   }

   private RigidBodyTransform convertTransform(double... transformParameters)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(transformParameters[0], transformParameters[1], transformParameters[2]);
      transform.appendRollRotation(transformParameters[3]);
      transform.appendPitchRotation(transformParameters[4]);
      transform.appendYawRotation(transformParameters[5]);

      return transform;
   }

   private LevenbergMarquardtParameterOptimizer createOptimizer(NormalOcTree map, Point3DReadOnly[] sourcePointsToSensorPose,
                                                                RigidBodyTransformReadOnly sensorPoseToWorld)
   {
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, sourcePointsToSensorPose.length);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = convertTransform(inputParameter.getData());
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(sensorPoseToWorld);
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Point3D[] correctedData = new Point3D[sourcePointsToSensorPose.length];
            for (int i = 0; i < sourcePointsToSensorPose.length; i++)
            {
               correctedData[i] = new Point3D(sourcePointsToSensorPose[i]);
               correctedSensorPoseToWorld.transform(correctedData[i]);
            }

            DenseMatrix64F errorSpace = new DenseMatrix64F(correctedData.length, 1);
            for (int i = 0; i < correctedData.length; i++)
            {
               double distance = computeClosestDistance(correctedData[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Point3D point)
         {
            double linearDistance = SLAMTools.computeDistanceToNormalOctree(map, point);
            double surfelDistance = SLAMTools.computePerpendicularDistanceToNormalOctree(map, point);
            return Math.min(linearDistance, surfelDistance);
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.0001);
      purterbationVector.set(1, 0.0001);
      purterbationVector.set(2, 0.0001);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      return optimizer;
   }

   private void setupTest()
   {
      // load data.
      File pointCloudFile = new File(DATA_PATH);
      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      frame1 = new SLAMFrame(messages.get(INDEX_FRAME_ONE));
      frame2 = new SLAMFrame(frame1, messages.get(INDEX_FRAME_TWO));

      // octree.
      Point3D dummySensorLocation = new Point3D();
      octreeMap = SLAMTools.computeOctreeData(frame1.getPointCloud(), dummySensorLocation, OCTREE_RESOLUTION);

      // define source points.
      double windowMargin = 0.05;
      ConvexPolygon2D windowForMap = SLAMTools.computeMapConvexHullInSensorFrame(octreeMap, frame2.getSensorPose());

      Point3DReadOnly[] newPointCloud = frame2.getPointCloud();
      Point3DReadOnly[] newPointCloudToSensorPose = frame2.getOriginalPointCloudToSensorPose();
      boolean[] isInPreviousView = new boolean[newPointCloudToSensorPose.length];
      int numberOfPointsInWindow = 0;
      for (int i = 0; i < newPointCloudToSensorPose.length; i++)
      {
         Point3DReadOnly point = newPointCloudToSensorPose[i];
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
      for (int i = 0; i < newPointCloudToSensorPose.length; i++)
      {
         if (isInPreviousView[i])
         {
            pointsInPreviousWindow[indexOfPointsInWindow] = new Point3D(newPointCloud[i]);
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
      dummyMessageForSourcePoints.getSensorPosition().set(frame2.getSensorPose().getTranslation());
      dummyMessageForSourcePoints.getSensorOrientation().set(frame2.getSensorPose().getRotation());
      frameForSourcePoints = new SLAMFrame(frame1, dummyMessageForSourcePoints);
   }
}
