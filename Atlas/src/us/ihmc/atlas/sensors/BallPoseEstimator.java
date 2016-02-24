package us.ihmc.atlas.sensors;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;

import javax.vecmath.Point3f;

import org.apache.commons.lang3.tuple.ImmutablePair;

import boofcv.struct.calib.IntrinsicParameters;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModelFactory;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.darpaRoboticsChallenge.sensors.DetectedObjectId;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.ihmcPerception.chessboardDetection.OpenCVChessboardPoseEstimator;
import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiver;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.sensorData.CameraData;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.tools.io.printing.PrintTools;

public class BallPoseEstimator {
	private final static boolean DEBUG = false;
	private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

	ExecutorService executorService = Executors.newFixedThreadPool(2);

	PacketCommunicator communicator;
	SDFFullHumanoidRobotModelFactory modelFactory;
	PointCloudDataReceiver pointCloudDataReceiver;

	public BallPoseEstimator(PacketCommunicator packetCommunicator, PointCloudDataReceiver pointCloudDataReceiver,
			SDFFullHumanoidRobotModelFactory modelFactory, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
			boolean runningOnRealRobot) {
		this.communicator = packetCommunicator;
		this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
		this.modelFactory = modelFactory;
		this.pointCloudDataReceiver = pointCloudDataReceiver;
		startBallDetector();
	}

	private void startBallDetector() {
		final FullHumanoidRobotModel fullRobotModel = modelFactory.createFullRobotModel();
		final int pointDropFactor = 4;
		final float searchRadius = 2.0f;
		executorService.submit(new Runnable() {

			@Override
			public void run() {
				Thread.currentThread().setPriority(Thread.MIN_PRIORITY);

				while (true) {

					Point3f[] fullPoints = pointCloudDataReceiver.getDecayingPointCloudPoints();

					// get head
					robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
					RigidBodyTransform headToWorld = fullRobotModel.getHead().getBodyFixedFrame()
							.getTransformToWorldFrame();
					Point3f head = new Point3f();
					headToWorld.transform(head);

					// filter points
					ArrayList<Point3D_F64> pointsNearBy = new ArrayList<Point3D_F64>();
					int counter = 0;
					for (Point3f tmpPoint : fullPoints) {
						if (!Double.isNaN(tmpPoint.z) & counter % pointDropFactor == 0
								&& tmpPoint.distance(head) < searchRadius)
							pointsNearBy.add(new Point3D_F64(tmpPoint.x, tmpPoint.y, tmpPoint.z));
						counter++;
					}
					PrintTools.debug(DEBUG, this, "Points around center " + pointsNearBy.size());

					// find plane
					ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(30, 1.57, 0.010,
							CloudShapeTypes.SPHERE);
					configRansac.minimumPoints = 15;
					PointCloudShapeFinder findSpheres = FactoryPointCloudShape
							.ransacSingleAll(new ConfigSurfaceNormals(20, 0.10), configRansac);

					PrintStream out = System.out;
					System.setOut(new PrintStream(new OutputStream() {
						@Override
						public void write(int b) throws IOException {
						}
					}));
					try {
						findSpheres.process(pointsNearBy, null);
					} finally {
						System.setOut(out);
					}

					// sort large to small
					List<Shape> spheres = findSpheres.getFound();
					Collections.sort(spheres, new Comparator<Shape>() {

						@Override
						public int compare(Shape o1, Shape o2) {
							return -Integer.compare(o1.points.size(), o2.points.size());
						};
					});

					if (spheres.size() > 0)
						PrintTools.debug(DEBUG, this, "spheres.size() " + spheres.size());

					for (Shape sphere : spheres) {
						Sphere3D_F64 sphereParams = (Sphere3D_F64) sphere.getParameters();
						PrintTools.debug(DEBUG, this,
								"sphere radius" + sphereParams.getRadius() + " center " + sphereParams.getCenter());

						if (sphereParams.getRadius() < 0.254)// soccer ball -
																// 0.127) 4 inch
																// balls
						{
							RigidBodyTransform t = new RigidBodyTransform();
							t.setTranslation(sphereParams.getCenter().x, sphereParams.getCenter().y,
									sphereParams.getCenter().z);
							communicator.send(new DetectedObjectPacket(t, DetectedObjectId.BALL.ordinal()));
						}

					}

				}
			}

		});

	}

}
