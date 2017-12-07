package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import jxl.read.biff.File;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

public class CollaborativeBehavior extends AbstractBehavior {

	private FullHumanoidRobotModel fullHumanoidModel;
	private WalkingControllerParameters walkingControllerParameters;
	private HumanoidReferenceFrames referenceFrames;
	private DRCRobotSensorInformation robotSensorInfo;
	private int cameraID;
	private String cameraName;
	private ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>(20);
	private boolean testImage = false;
	
	public CollaborativeBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames,
			FullHumanoidRobotModel fullHumanoidRobotModel, DRCRobotSensorInformation robotSensorInfo, WalkingControllerParameters walkingControllerParameters, YoGraphicsListRegistry graphicsListRegistry) {
		super(communicationBridge);
		this.attachNetworkListeningQueue(cameraData, VideoPacket.class);
		this.fullHumanoidModel = fullHumanoidRobotModel;
		this.walkingControllerParameters = walkingControllerParameters;
		this.referenceFrames = referenceFrames;
		this.robotSensorInfo = robotSensorInfo;
		DRCRobotCameraParameters[] robotCameraParameters = robotSensorInfo.getCameraParameters();
		this.cameraName = robotCameraParameters[0].getSensorNameInSdf();
		//System.out.println(cameraName);
	}

	@Override
	public void doControl() 
	{		
		if(cameraData.isNewPacketAvailable())
		{
			VideoPacket vidPack = cameraData.getLatestPacket();
			//System.out.println(vidPack.videoSource.toString());
			if(!testImage)
			{
				try{
					InputStream in = new ByteArrayInputStream(vidPack.getData()); 
					ImageIO.write(ImageIO.read(in),"png", new java.io.File("testImage"));					
				}catch (IOException e)
				{
					
				}
				testImage = true;
			}			
		}
		else
		{
			//System.out.println("Nothing Works");
		}
	}

	@Override
	public void onBehaviorEntered() {
		
	}

	@Override
	public void onBehaviorAborted() {
	}

	@Override
	public void onBehaviorPaused() {

	}

	@Override
	public void onBehaviorResumed() {

	}

	@Override
	public void onBehaviorExited() {

	}

	@Override
	public boolean isDone() {
		return false;
	}

}
