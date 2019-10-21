package us.ihmc.ihmcPerception.fiducialDetector;

import java.awt.image.BufferedImage;

import org.junit.jupiter.api.Test;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoDataServerImageCallback;
import us.ihmc.communication.producers.VideoSource;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.Fiducial;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.FloatingFiducialBoxRobot;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.CameraMount;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.commons.thread.ThreadTools;

@Disabled
public class FiducialDetectorFromCameraImagesTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @Test
   public void testUsingSimulationConstructionSet()
   {
      double fieldOfView = 0.81;

      final Robot simpleRobotWithCamera = createCameraRobot(fieldOfView);

      final FloatingFiducialBoxRobot floatingFiducialBoxRobot = new FloatingFiducialBoxRobot(Fiducial.FIDUCIAL50,"3");
      floatingFiducialBoxRobot.setPosition(6.0, 0.0, 2.0);
      floatingFiducialBoxRobot.setYawPitchRoll(0.0, -Math.PI / 2.0, 0.0);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RigidBodyTransform transformFromReportedToFiducialFrame = new RigidBodyTransform();
      transformFromReportedToFiducialFrame.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);

      final FiducialDetectorFromCameraImages detector = new FiducialDetectorFromCameraImages(transformFromReportedToFiducialFrame, simpleRobotWithCamera.getRobotsYoVariableRegistry(), yoGraphicsListRegistry,"test");
      detector.setTargetIDToLocate(50L);

      detector.setFieldOfView(fieldOfView, fieldOfView);

      SimulationConstructionSet scsForDetecting = new SimulationConstructionSet(new Robot[] { simpleRobotWithCamera, floatingFiducialBoxRobot});
      scsForDetecting.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.translate(10.0, 0.0, 0.0);
      staticLinkGraphics.addCube(0.01, 10.0, 10.0, YoAppearance.Orange());
      scsForDetecting.addStaticLinkGraphics(staticLinkGraphics);
      
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
      VideoDataServer videoDataServer = new VideoDataServer()
      {
         @Override
         public void onFrame(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters)
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

            int height = bufferedImage.getHeight();
            int width = bufferedImage.getWidth();

            double fx = (width / 2.0) / Math.tan(Math.toRadians(80.0)/ 2.0);
            double fy = (height / 2.0) / Math.tan(Math.toRadians(45.0) / 2.0);

            intrinsicParameters.width = width;
            intrinsicParameters.height = height;
            intrinsicParameters.cx = width / 2.0;
            intrinsicParameters.cy = height / 2.0;
            intrinsicParameters.fx = fx;
            intrinsicParameters.fy = fy;

           
            detector.detect(bufferedImage, cameraPositionInWorld, cameraOrientationInWorldXForward,intrinsicParameters);
         }

         @Override
         public boolean isConnected()
         {
            return true;
         }
      };

      TimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
      int framesPerSecond = 10;

      scsForDetecting.startStreamingVideoData(cameraConfiguration, width, height, new VideoDataServerImageCallback(videoDataServer), timestampProvider, framesPerSecond);

      GoalOrientedTestConductor testConductor = new GoalOrientedTestConductor(scsForDetecting, simulationTestingParameters);

      YoBoolean fiducialTargetIDHasBeenLocated = (YoBoolean) scsForDetecting.getVariable("fiducialTargetIDHasBeenLocated");
      YoBoolean fiducialTargetIDHasBeenLocatedFiltered = (YoBoolean) scsForDetecting.getVariable("fiducialTargetIDHasBeenLocatedFiltered");

      YoDouble fiducialReportedPoseWorldFrameX = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameX");
      YoDouble fiducialReportedPoseWorldFrameY = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameY");
      YoDouble fiducialReportedPoseWorldFrameZ = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameZ");

      YoDouble fiducialReportedPoseWorldFrameQS = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameQS");
      YoDouble fiducialReportedPoseWorldFrameQX = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameQX");
      YoDouble fiducialReportedPoseWorldFrameQY = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameQY");
      YoDouble fiducialReportedPoseWorldFrameQZ = (YoDouble) scsForDetecting.getVariable("fiducialReportedPoseWorldFrameQZ");

      YoDouble q_qrCode_x = (YoDouble) scsForDetecting.getVariable("q_qrCode_x");
      YoDouble q_qrCode_y = (YoDouble) scsForDetecting.getVariable("q_qrCode_y");
      YoDouble q_qrCode_z = (YoDouble) scsForDetecting.getVariable("q_qrCode_z");

      YoDouble q_qrCode_qs = (YoDouble) scsForDetecting.getVariable("q_qrCode_qs");
      YoDouble q_qrCode_qx = (YoDouble) scsForDetecting.getVariable("q_qrCode_qx");
      YoDouble q_qrCode_qy = (YoDouble) scsForDetecting.getVariable("q_qrCode_qy");
      YoDouble q_qrCode_qz = (YoDouble) scsForDetecting.getVariable("q_qrCode_qz");

      final YoDouble time = simpleRobotWithCamera.getYoTime();

      time.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
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
            floatingFiducialBoxRobot.setLinearVelocity(linearVelocityInWorld);

            Vector3D angularVelocityInBody = new Vector3D(wX, wY, wZ);
            floatingFiducialBoxRobot.setAngularVelocity(angularVelocityInBody);
         }
      });

      testConductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(time, 6.0));

      testConductor.addSustainGoal(YoVariableTestGoal.or(YoVariableTestGoal.doubleLessThan(time, 0.1), YoVariableTestGoal.booleanEquals(fiducialTargetIDHasBeenLocatedFiltered, true)));

      double okTrackingDeltaPositionX = 0.05;
      double okTrackingDeltaPositionYZ = 0.05;
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameX, q_qrCode_x, okTrackingDeltaPositionX));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameY, q_qrCode_y, okTrackingDeltaPositionYZ));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameZ, q_qrCode_z, okTrackingDeltaPositionYZ));

      double okTrackingDeltaQuaternion = 0.08;
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameQS, q_qrCode_qs, okTrackingDeltaQuaternion));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameQX, q_qrCode_qx, okTrackingDeltaQuaternion));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameQY, q_qrCode_qy, okTrackingDeltaQuaternion));
      testConductor.addSustainGoal(YoVariableTestGoal.variablesEqual(fiducialReportedPoseWorldFrameQZ, q_qrCode_qz, okTrackingDeltaQuaternion));

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
