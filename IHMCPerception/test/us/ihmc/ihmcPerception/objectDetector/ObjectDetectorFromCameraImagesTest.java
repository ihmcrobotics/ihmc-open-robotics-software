package us.ihmc.ihmcPerception.objectDetector;

import java.awt.image.BufferedImage;

import org.junit.Test;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.camera.RenderedSceneHandler;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.CameraMount;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.FloatingObjectBoxRobot;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.thread.ThreadTools;

public class ObjectDetectorFromCameraImagesTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @ContinuousIntegrationTest(estimatedDuration = 5.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 300000)
   public void testUsingSimulationConstructionSet() throws Exception
   {
      double fieldOfView = 0.81;

      final Robot simpleRobotWithCamera = createCameraRobot(fieldOfView);

      final FloatingObjectBoxRobot floatingObjectBoxRobot = new FloatingObjectBoxRobot("/valve/red-valve.jpg");
      floatingObjectBoxRobot.setPosition(6.0, 0.0, 2.0);
      floatingObjectBoxRobot.setYawPitchRoll(0.0, -Math.PI / 2.0, 0.0);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RigidBodyTransform transformFromReportedToFiducialFrame = new RigidBodyTransform();
      transformFromReportedToFiducialFrame.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);

      final ObjectDetectorFromCameraImages detector = new ObjectDetectorFromCameraImages(transformFromReportedToFiducialFrame, simpleRobotWithCamera.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);

      detector.setFieldOfView(fieldOfView, fieldOfView);

      SimulationConstructionSet scsForDetecting = new SimulationConstructionSet(new Robot[] { simpleRobotWithCamera, floatingObjectBoxRobot});
      scsForDetecting.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      CameraConfiguration cameraConfiguration = new CameraConfiguration("cameraMount");
      cameraConfiguration.setCameraMount("cameraMount");
      //      cameraConfiguration.setCameraFieldOfView(fieldOfView);
      scsForDetecting.setupCamera(cameraConfiguration);
      scsForDetecting.selectCamera("cameraMount");

      scsForDetecting.setDT(0.01, 1);
      scsForDetecting.setSimulateNoFasterThanRealTime(true);

      scsForDetecting.setSynchronizeGraphicsAndCamerasWhileSimulating(true);

      int width = 800;
      int height = 800;
      RenderedSceneHandler videoDataServer = new RenderedSceneHandler()
      {
         @Override
         public void updateImage(RobotSide robotSide, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters)
         {
            FloatingJoint cameraJoint = (FloatingJoint) simpleRobotWithCamera.getRootJoints().get(0);

            Point3D cameraPositionInWorld = new Point3D();
            Quaternion cameraOrientationInWorldXForward = new Quaternion();

            cameraJoint.getPosition(cameraPositionInWorld);
            cameraJoint.getRotationToWorld(cameraOrientationInWorldXForward);

            //            System.out.println("Received image.");
            //            System.out.println("intrinsicParameters.width = " + intrinsicParameters.width);
            //            System.out.println("intrinsicParameters.height = " + intrinsicParameters.height);
            //            System.out.println("intrinsicParameters.fx = " + intrinsicParameters.fx);
            //            System.out.println("intrinsicParameters.fy = " + intrinsicParameters.fy);
            //            System.out.println("intrinsicParameters.skew = " + intrinsicParameters.skew);
            //            System.out.println("intrinsicParameters.cx = " + intrinsicParameters.cx);
            //            System.out.println("intrinsicParameters.cy = " + intrinsicParameters.cy);
            //            System.out.println("intrinsicParameters.t1 = " + intrinsicParameters.t1);
            //            System.out.println("intrinsicParameters.t2 = " + intrinsicParameters.t2);
            //            System.out.println("intrinsicParameters.radial.length = " + intrinsicParameters.radial.length);
            //            System.out.println("intrinsicParameters.radial[0] = " + intrinsicParameters.radial[0]);
            //            System.out.println("intrinsicParameters.radial[1] = " + intrinsicParameters.radial[1]);

            detector.detect(bufferedImage, cameraPositionInWorld, cameraOrientationInWorldXForward);
         }

         @Override
         public boolean isReadyForNewData()
         {
            return true;
         }

         @Override
         public void close()
         {
         }
      };

      TimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
      int framesPerSecond = 10;

      scsForDetecting.startStreamingVideoData(cameraConfiguration, width, height, videoDataServer, timestampProvider, framesPerSecond);

      GoalOrientedTestConductor testConductor = new GoalOrientedTestConductor(scsForDetecting, simulationTestingParameters);

      BooleanYoVariable objectTargetIDHasBeenLocated = (BooleanYoVariable) scsForDetecting.getVariable("objectTargetIDHasBeenLocated");

      DoubleYoVariable objectReportedPoseWorldFrameX = (DoubleYoVariable) scsForDetecting.getVariable("objectReportedPoseWorldFrameX");
      DoubleYoVariable objectReportedPoseWorldFrameY = (DoubleYoVariable) scsForDetecting.getVariable("objectReportedPoseWorldFrameY");
      DoubleYoVariable objectReportedPoseWorldFrameZ = (DoubleYoVariable) scsForDetecting.getVariable("objectReportedPoseWorldFrameZ");

      DoubleYoVariable q_object_x = (DoubleYoVariable) scsForDetecting.getVariable("q_object_x");
      DoubleYoVariable q_object_y = (DoubleYoVariable) scsForDetecting.getVariable("q_object_y");
      DoubleYoVariable q_object_z = (DoubleYoVariable) scsForDetecting.getVariable("q_object_z");

      final DoubleYoVariable time = simpleRobotWithCamera.getYoTime();

      time.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            double t = time.getDoubleValue();
            double ampX = 0.05;
            double ampY = 0.07;
            double ampZ = 0.13;

            double freqX = 0.23;
            double freqY = 0.19;
            double freqZ = 0.37;

            double wX = ampX * Math.sin(2.0 * Math.PI * freqX * t);
            double wY = ampY * Math.sin(2.0 * Math.PI * freqY * t);
            double wZ = ampZ * Math.sin(2.0 * Math.PI * freqZ * t);

            double vX = ampX * Math.sin(2.0 * Math.PI * freqX * t);
            double vY = ampY * Math.sin(2.0 * Math.PI * freqY * t);
            double vZ = ampZ * Math.sin(2.0 * Math.PI * freqZ * t);

            Vector3D linearVelocityInWorld = new Vector3D(vX, vY, vZ);
            floatingObjectBoxRobot.setLinearVelocity(linearVelocityInWorld);

            Vector3D angularVelocityInBody = new Vector3D(wX, wY, wZ);
            floatingObjectBoxRobot.setAngularVelocity(angularVelocityInBody);
         }
      });

      testConductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(time, 6.0));

      testConductor.addSustainGoal(YoVariableTestGoal.booleanEquals(objectTargetIDHasBeenLocated, true));

      double okTrackingDeltaPositionX = 0.6;
      double okTrackingDeltaPositionYZ = 0.6;
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(objectReportedPoseWorldFrameX, q_object_x, okTrackingDeltaPositionX));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(objectReportedPoseWorldFrameY, q_object_y, okTrackingDeltaPositionYZ));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(objectReportedPoseWorldFrameZ, q_object_z, okTrackingDeltaPositionYZ));

      ThreadTools.sleep(2000L);
      testConductor.simulate();

      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }

      testConductor.concludeTesting();

   }

   private Robot createCameraRobot(double fieldOfView)
   {
      final Robot simpleRobotWithCamera = new Robot("SimpleRobotWithCamera");
      FloatingJoint cameraJoint = new FloatingJoint("camera", "camera", new Vector3D(), simpleRobotWithCamera);
      Link cameraLink = new Link("camera");
      cameraLink.setMassAndRadiiOfGyration(1.0, 0.1, 0.1, 0.1);
      Graphics3DObject cameraLinkGraphics = new Graphics3DObject();
      cameraLinkGraphics.translate(-0.251, 0.0, 0.0);
      cameraLinkGraphics.addCoordinateSystem(0.25);
      cameraLink.setLinkGraphics(cameraLinkGraphics);
      cameraJoint.setLink(cameraLink);

      RigidBodyTransform cameraOffsetTransform = new RigidBodyTransform();

      double clipDistanceNear = 0.01;
      double clipDistanceFar = 10.0;

      CameraMount cameraMount = new CameraMount("cameraMount", cameraOffsetTransform, fieldOfView, clipDistanceNear, clipDistanceFar, simpleRobotWithCamera);
      cameraJoint.addCameraMount(cameraMount);

      simpleRobotWithCamera.addRootJoint(cameraJoint);
      simpleRobotWithCamera.setGravity(0.0);
      cameraJoint.setPosition(0.0, 0.0, 2.0);
      return simpleRobotWithCamera;
   }


}
