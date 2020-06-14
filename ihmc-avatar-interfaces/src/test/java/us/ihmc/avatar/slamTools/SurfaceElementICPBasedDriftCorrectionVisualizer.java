package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SurfaceElementICPBasedDriftCorrectionVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 100.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private static final DriftCase DRIFT_CASE = DriftCase.MultiplePlanes;

   private static final String DATA_PATH = "C:\\" + DRIFT_CASE.getFilePath();
   private static final int INDEX_FRAME_ONE = DRIFT_CASE.getPreviousFrameIndex();
   private static final int INDEX_FRAME_TWO = DRIFT_CASE.getNewFrameIndex();

   private static final int NUMBER_OF_POINTS_TO_VISUALIZE = 2000;

   private final static double OCTREE_RESOLUTION = 0.02;
   private NormalOcTree octreeMap;

   private final SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(OCTREE_RESOLUTION);
   private SLAMFrame frame1;
   private SLAMFrame frame2;

   private final SLAMFrameYoGraphicsManager frame1GraphicsManager;
   private final SLAMFrameYoGraphicsManager frame2GraphicsManager;

   private final YoDouble optimizerQuality;
   private final YoDouble optimizerTranslationalEffort;
   private final YoDouble optimizerRotationalEffort;
   private final YoInteger numberOfCorrespondingPoints;

   private final AppearanceDefinition octreeMapColor = YoAppearance.Coral();
   private final AppearanceDefinition frame1Appearance = YoAppearance.Blue();
   private final AppearanceDefinition frame2Appearance = YoAppearance.Green();
   private final AppearanceDefinition frameSurfelAppearance = YoAppearance.Red();

   public SurfaceElementICPBasedDriftCorrectionVisualizer()
   {
      optimizerQuality = new YoDouble("optimizerQuality", registry);
      optimizerTranslationalEffort = new YoDouble("optimizerTranslationalEffort", registry);
      optimizerRotationalEffort = new YoDouble("optimizerRotationalEffort", registry);
      numberOfCorrespondingPoints = new YoInteger("numberOfCorrespondingPoints", registry);

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
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setGroundVisible(false);

      // define optimizer.
      LevenbergMarquardtParameterOptimizer optimizer = createOptimizer(octreeMap, frame2);

      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame2.getInitialSensorPoseToWorld());
      correctedSensorPoseToWorld.multiply(icpTransformer);

      Point3D[] correctedData = new Point3D[frame2.getOriginalPointCloudToSensorPose().length];
      for (int i = 0; i < correctedData.length; i++)
      {
         correctedData[i] = new Point3D(frame2.getOriginalPointCloudToSensorPose()[i]);
         icpTransformer.transform(correctedData[i]);
      }

      frame1GraphicsManager.updateGraphics();
      frame2GraphicsManager.updateGraphics();
      scs.tickAndUpdate();

      for (double t = 1.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // do ICP.
         optimizer.iterate();

         // get parameter.
         DenseMatrix64F optimalParameter = optimizer.getOptimalParameter();
         icpTransformer.set(convertTransform(optimalParameter.getData()));
         correctedSensorPoseToWorld.set(frame2.getInitialSensorPoseToWorld());
         correctedSensorPoseToWorld.multiply(icpTransformer);
         for (int i = 0; i < correctedData.length; i++)
         {
            correctedData[i].set(frame2.getOriginalPointCloudToSensorPose()[i]);
            icpTransformer.transform(correctedData[i]);
         }

         frame2.updateOptimizedCorrection(icpTransformer);

         // update viz.
         frame1GraphicsManager.updateGraphics();
         frame2GraphicsManager.updateGraphics();

         // update yo variables.   
         optimizerQuality.set(optimizer.getQuality());
         double translationalEffort = 0.0, rotationalEffort = 0.0;
         for (int i = 0; i < 3; i++)
         {
            translationalEffort = translationalEffort + optimalParameter.get(i) * optimalParameter.get(i);
         }
         for (int i = 3; i < 6; i++)
         {
            rotationalEffort = rotationalEffort + optimalParameter.get(i) * optimalParameter.get(i);
         }
         optimizer.getOptimalParameter();
         optimizerTranslationalEffort.set(translationalEffort);
         optimizerRotationalEffort.set(rotationalEffort);
         numberOfCorrespondingPoints.set(optimizer.getNumberOfCoorespondingPoints());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new SurfaceElementICPBasedDriftCorrectionVisualizer();
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

   private LevenbergMarquardtParameterOptimizer createOptimizer(NormalOcTree map, SLAMFrame frame)
   {
      int numberOfSurfel = frame.getSurfaceElementsToSensor().size();
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, numberOfSurfel);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = convertTransform(inputParameter.getData());
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame.getOriginalSensorPose());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame.getSurfaceElementsToSensor().get(i));

               correctedSensorPoseToWorld.transform(correctedSurfel[i].getPoint());
               correctedSensorPoseToWorld.transform(correctedSurfel[i].getNormal());
            }

            DenseMatrix64F errorSpace = new DenseMatrix64F(correctedSurfel.length, 1);
            for (int i = 0; i < correctedSurfel.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfel[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeSurfaceElementBoundedDistanceToNormalOctree(map,
                                                                                surfel,
                                                                                frame.getFrameMap().getResolution(),
                                                                                frame.getFrameMap().getResolution());
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.0005);
      purterbationVector.set(1, 0.0005);
      purterbationVector.set(2, 0.0005);
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

      slam.addKeyFrame(messages.get(INDEX_FRAME_ONE));
      octreeMap = slam.getOctree();
      octreeMap.updateNormals();

      frame1 = new SLAMFrame(messages.get(INDEX_FRAME_ONE));
      frame2 = new SLAMFrame(frame1, messages.get(INDEX_FRAME_TWO));
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.0;
      int minimumNumberOfHits = 1;
      frame2.registerSurfaceElements(octreeMap, windowMargin, surfaceElementResolution, minimumNumberOfHits, true);
   }
}
