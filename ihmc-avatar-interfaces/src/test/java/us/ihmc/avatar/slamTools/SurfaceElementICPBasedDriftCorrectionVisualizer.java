package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.List;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SurfaceElementICPBasedDriftCorrectionVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 100.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private static final DriftCase DRIFT_CASE = DriftCase.YDrift;

   private static final String DATA_PATH = DRIFT_CASE.getFilePath();

   private static final int NUMBER_OF_POINTS_TO_VISUALIZE = 2000;

   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(true);
   private final static double OCTREE_RESOLUTION = 0.02;
   private NormalOcTree octreeMap;

   private final SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(OCTREE_RESOLUTION);
   private SLAMFrame frame1;
   private SLAMFrame frame2;

   private final SLAMFrameYoGraphicsManager frame1GraphicsManager;
   private final SLAMFrameYoGraphicsManager frame2GraphicsManager;

   private final YoDouble optimizerQuality;
   private final YoDouble optimizerPureQuality;
   private final YoDouble optimizerTranslationalEffort;
   private final YoDouble optimizerRotationalEffort;
   private final YoDouble optimizerTranslationalEffortDiff;
   private final YoDouble optimizerRotationalEffortDiff;
   private final YoInteger numberOfCorrespondingPoints;
   private final YoInteger numberOfSourcePoints;

   private static final int NUMBER_OF_STEADY = 3;
   private boolean isSteady = false;
   private int steadyIterations = 0;
   private final YoBoolean terminalCondition;

   private final YoBoolean qualityCondition;
   private final YoBoolean translationalCondition;
   private final YoBoolean rotationalCondition;

   private final AppearanceDefinition octreeMapColor = YoAppearance.Coral();
   private final AppearanceDefinition frame1Appearance = YoAppearance.Blue();
   private final AppearanceDefinition frame2Appearance = YoAppearance.Green();
   private final AppearanceDefinition frameSurfelAppearance = YoAppearance.Red();

   public SurfaceElementICPBasedDriftCorrectionVisualizer()
   {
      optimizerQuality = new YoDouble("optimizerQuality", registry);
      optimizerPureQuality = new YoDouble("optimizerPureQuality", registry);
      optimizerTranslationalEffort = new YoDouble("optimizerTranslationalEffort", registry);
      optimizerRotationalEffort = new YoDouble("optimizerRotationalEffort", registry);
      optimizerTranslationalEffortDiff = new YoDouble("optimizerTranslationalEffortDiff", registry);
      optimizerRotationalEffortDiff = new YoDouble("optimizerRotationalEffortDiff", registry);
      numberOfCorrespondingPoints = new YoInteger("numberOfCorrespondingPoints", registry);
      numberOfSourcePoints = new YoInteger("numberOfSourcePoints", registry);

      terminalCondition = new YoBoolean("terminalCondition", registry);
      qualityCondition = new YoBoolean("qualityCondition", registry);
      translationalCondition = new YoBoolean("translationalCondition", registry);
      rotationalCondition = new YoBoolean("rotationalCondition", registry);

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
                                                             frameSurfelAppearance,
                                                             registry,
                                                             yoGraphicsListRegistry,
                                                             false);
      new OctreeYoGraphicsManager("Octree_", octreeMap, octreeMapColor, registry, yoGraphicsListRegistry, false);

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

      // define optimizer.
      LevenbergMarquardtParameterOptimizer optimizer = createOptimizer(octreeMap, frame2);

      RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform();
      RigidBodyTransform correctedLocalPoseInWorld = new RigidBodyTransform(frame2.getUncorrectedLocalPoseInWorld());
      correctedLocalPoseInWorld.multiply(driftCorrectionTransform);

      List<Point3D> correctedData = frame2.getPointCloudInLocalFrame().stream().map(Point3D::new).collect(Collectors.toList());
      correctedData.forEach(driftCorrectionTransform::transform);

      frame1GraphicsManager.updateGraphics();
      frame2GraphicsManager.updateGraphics();
      scs.tickAndUpdate();

      RigidBodyTransform previousDriftCorrectionTransform = new RigidBodyTransform(driftCorrectionTransform);
      for (double t = 1.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // do ICP.
         optimizer.iterate();

         // get parameter.
         DMatrixRMaj optimalParameter = optimizer.getOptimalParameter();
         driftCorrectionTransform.set(inputFunction.apply(optimalParameter));
         correctedLocalPoseInWorld.set(frame2.getUncorrectedLocalPoseInWorld());
         correctedLocalPoseInWorld.multiply(driftCorrectionTransform);
         correctedData = frame2.getPointCloudInLocalFrame().stream().map(Point3D::new).collect(Collectors.toList());
         correctedData.forEach(driftCorrectionTransform::transform);

         frame2.updateOptimizedCorrection(driftCorrectionTransform);
         double quality = optimizer.getQuality();
         double pureQuality = optimizer.getPureQuality();
         double translationalEffort = driftCorrectionTransform.getTranslation().lengthSquared();
         double rotationalEffort = driftCorrectionTransform.getRotation().distance(new RotationMatrix());
         //double translationalEffortDiff = new Point3D(previousIcpTransformer.getTranslation()).distance(new Point3D(icpTransformer.getTranslation()));
         double translationalEffortDiff = Math.abs(translationalEffort - optimizerTranslationalEffort.getDoubleValue());
         double rotationalEffortDiff = new Quaternion(previousDriftCorrectionTransform.getRotation()).distance(new Quaternion(driftCorrectionTransform.getRotation()));

         // update viz.
         frame1GraphicsManager.updateGraphics();
         frame2GraphicsManager.updateGraphics();

         // update terminal conditions
         boolean isSteadyQuality = false;
         boolean isSteadyTranslational = false;
         boolean isSteadyRotational = false;

         if (Math.abs(quality - optimizerQuality.getDoubleValue()) < 0.001)
         {
            isSteadyQuality = true;
         }
         if (translationalEffortDiff < 0.001)
         {
            isSteadyTranslational = true;
         }
         if (rotationalEffortDiff < 0.005)
         {
            isSteadyRotational = true;
         }
         qualityCondition.set(isSteadyQuality);
         translationalCondition.set(isSteadyTranslational);
         rotationalCondition.set(isSteadyRotational);

         if (isSteadyQuality && isSteadyTranslational && isSteadyRotational)
         {
            isSteady = true;
         }
         else
         {
            isSteady = false;
         }

         // update yo variables.   
         optimizerQuality.set(quality);
         optimizerPureQuality.set(pureQuality);
         optimizerTranslationalEffortDiff.set(translationalEffortDiff);
         optimizerRotationalEffortDiff.set(rotationalEffortDiff);
         optimizerTranslationalEffort.set(translationalEffort);
         optimizerRotationalEffort.set(rotationalEffort);
         numberOfCorrespondingPoints.set(optimizer.getNumberOfCorrespondingPoints());

         previousDriftCorrectionTransform.set(driftCorrectionTransform);
         if (isSteady)
         {
            steadyIterations++;
         }
         else
         {
            steadyIterations = 0;
         }
         if (steadyIterations >= NUMBER_OF_STEADY)
         {
            terminalCondition.set(true);
         }
         else
         {
            terminalCondition.set(false);
         }

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new SurfaceElementICPBasedDriftCorrectionVisualizer();
   }

   private LevenbergMarquardtParameterOptimizer createOptimizer(NormalOcTree map, SLAMFrame frame)
   {
      int numberOfSurfel = frame.getNumberOfSurfaceElements();
      OutputCalculator outputCalculator = new OutputCalculator()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform(inputFunction.apply(inputParameter));
            RigidBodyTransform correctedLocalPoseInWorld = new RigidBodyTransform(frame.getUncorrectedLocalPoseInWorld());
            correctedLocalPoseInWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfelInWorld = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfelInWorld[i] = new Plane3D();
               correctedSurfelInWorld[i].set(frame.getSurfaceElementsInLocalFrame().get(i));

               correctedLocalPoseInWorld.transform(correctedSurfelInWorld[i].getPoint());
               correctedLocalPoseInWorld.transform(correctedSurfelInWorld[i].getNormal());
            }

            DMatrixRMaj errorSpace = new DMatrixRMaj(correctedSurfelInWorld.length, 1);
            for (int i = 0; i < correctedSurfelInWorld.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfelInWorld[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeBoundedPerpendicularDistancePointToNormalOctree(map, surfel.getPoint(), map.getResolution() * 1.1);
         }
      };
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction, outputCalculator, 6, numberOfSurfel);
      DMatrixRMaj purterbationVector = new DMatrixRMaj(6, 1);
      purterbationVector.set(0, map.getResolution() * 0.002);
      purterbationVector.set(1, map.getResolution() * 0.002);
      purterbationVector.set(2, map.getResolution() * 0.002);
      purterbationVector.set(3, 0.00001);
      purterbationVector.set(4, 0.00001);
      purterbationVector.set(5, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(map.getResolution() * 1.5);

      return optimizer;
   }

   private void setupTest()
   {
      // load data.
      File pointCloudFile = new File(DATA_PATH);
      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      slam.addKeyFrame(messages.get(0), true);
      octreeMap = slam.getMapOcTree();
      octreeMap.updateNormals();

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);

      frame1 = new SLAMFrame(messages.get(0), normalEstimationParameters);
      frame2 = new SLAMFrame(frame1, messages.get(1), normalEstimationParameters);
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.0;
      int minimumNumberOfHits = 1;
      int maxNumberOfSurfels = Integer.MAX_VALUE;
      frame2.registerSurfaceElements(octreeMap, windowMargin, surfaceElementResolution, minimumNumberOfHits, true, maxNumberOfSurfels);
      numberOfSourcePoints.set(frame2.getSurfaceElements().size());
   }
}
