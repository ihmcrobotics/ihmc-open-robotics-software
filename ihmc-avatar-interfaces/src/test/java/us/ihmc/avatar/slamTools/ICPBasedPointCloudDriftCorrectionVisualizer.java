package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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

public class ICPBasedPointCloudDriftCorrectionVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 100.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private final static double OCTREE_RESOLUTION = 0.02;
   private NormalOcTree octreeMap;
   private final static int NUMBER_OF_SOURCE_POINT = 500;
   private Point3D[] sourcePoints = new Point3D[NUMBER_OF_SOURCE_POINT];

   private SLAMFrame frame1;
   private SLAMFrame frame2;
   private SLAMFrame frameForSourcePoints;

   private final SLAMFrameYoGraphicsManager frame1GraphicsManager;
   private final SLAMFrameYoGraphicsManager frame2GraphicsManager;
   private final SLAMFrameYoGraphicsManager sourcePointsFrameGraphicsManager;

   private final YoDouble optimizerQuality;

   private final AppearanceDefinition octreeMapColor = YoAppearance.AliceBlue();
   private final AppearanceDefinition frame1Appearance = YoAppearance.Blue();
   private final AppearanceDefinition frame2Appearance = YoAppearance.Green();
   private final AppearanceDefinition sourcePointsAppearance = YoAppearance.Red();

   public ICPBasedPointCloudDriftCorrectionVisualizer()
   {
      optimizerQuality = new YoDouble("optimizerQuality", registry);

      // Define frames .
      setupTest();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      frame1GraphicsManager = new SLAMFrameYoGraphicsManager("Frame1_", frame1, frame1Appearance, registry, yoGraphicsListRegistry);
      frame2GraphicsManager = new SLAMFrameYoGraphicsManager("Frame2_", frame2, frame2Appearance, registry, yoGraphicsListRegistry);
      sourcePointsFrameGraphicsManager = new SLAMFrameYoGraphicsManager("SourcePointFrame_",
                                                                        frameForSourcePoints,
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
      scs.addStaticLinkGraphics(octreeGraphics);

      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      // define optimizer.
      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // do ICP.
         //icpTransformer.setTranslationZ(t / 10);
         frame2.updateOptimizedCorrection(icpTransformer);

         // update viz.
         frame1GraphicsManager.updateGraphics();
         frame2GraphicsManager.updateGraphics();
         sourcePointsFrameGraphicsManager.updateGraphics();

         // update yo variables.   

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

   private void transformPointCloud(Point3D[] pointCloud, RigidBodyTransform transform)
   {
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         transform.transform(point);
      }
   }

   private LevenbergMarquardtParameterOptimizer createOptimizer(NormalOcTree map, Point3D[] sourcePoints)
   {
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, sourcePoints.length);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            Point3D[] transformedData = new Point3D[sourcePoints.length];
            for (int i = 0; i < sourcePoints.length; i++)
               transformedData[i] = new Point3D(sourcePoints[i]);
            RigidBodyTransform transform = convertTransform(inputParameter.getData());
            transformPointCloud(transformedData, transform);

            DenseMatrix64F errorSpace = new DenseMatrix64F(transformedData.length, 1);
            for (int i = 0; i < transformedData.length; i++)
            {
               double distance = computeClosestDistance(transformedData[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Point3D point)
         {
            double distance = SLAMTools.computeDistanceToNormalOctree(map, point, 10);
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
      optimizer.setCorrespondenceThreshold(0.01);

      return optimizer;
   }

   private void setupTest()
   {
      // load data.
      String stereoPath = "C:\\\\PointCloudData\\Data\\20200305_Simple\\PointCloud\\"; // 30 vs 33 : small drift, 33 vs 34 : drift and left right big movement.
      File pointCloudFile = new File(stereoPath);
      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      frame1 = new SLAMFrame(messages.get(30));
      frame2 = new SLAMFrame(frame1, messages.get(33));

      // octree.
      Point3D dummySensorLocation = new Point3D();
      octreeMap = SLAMTools.computeOctreeData(frame1.getPointCloud(), dummySensorLocation, OCTREE_RESOLUTION);

      // define source points.
      double windowMargin = 0.1;
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

      StereoVisionPointCloudMessage dummyMessageForSourcePoints = PointCloudCompression.compressPointCloud(0612L,
                                                                                                           pointsInPreviousWindow,
                                                                                                           colors,
                                                                                                           pointsInPreviousWindow.length,
                                                                                                           0.005,
                                                                                                           null);
      frameForSourcePoints = new SLAMFrame(frame1, dummyMessageForSourcePoints);
   }
}
