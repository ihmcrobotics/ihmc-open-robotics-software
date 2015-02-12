package us.ihmc.humanoidBehaviors.behaviors;

import java.awt.FlowLayout;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepPathPlanPacket;
import us.ihmc.communication.packets.walking.FootstepPlanRequestPacket;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.FootstepStatus.Status;
import us.ihmc.communication.packets.walking.PauseCommand;
import us.ihmc.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.VideoStreamer;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class ReceiveImageBehavior extends BehaviorInterface implements VideoStreamer{

	private LongYoVariable counter = new LongYoVariable("counter", registry);
	
	final static boolean SHOW_LOCAL_UI = false;
	JFrame frame=null; 
	ImageIcon icon=null; 
	private final ConcurrentListeningQueue<VideoPacket> inputListeningQueue = new ConcurrentListeningQueue<VideoPacket>();
	private CompressedVideoDataClient compressedVideoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);


	public ReceiveImageBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
	{
		super(outgoingCommunicationBridge);
		this.attachNetworkProcessorListeningQueue(inputListeningQueue, VideoPacket.class);
		
		if(SHOW_LOCAL_UI)
		   setupUI();
	}
	
	private void setupUI()
	{
	   frame = new JFrame();
	   frame.setLayout(new FlowLayout());
	   icon = new ImageIcon();
	   frame.add(new JLabel(icon));
	   frame.pack();
	   frame.setVisible(true);
	}

	@Override
	public void doControl() {
	   VideoPacket packet;
	   while((packet=inputListeningQueue.getNewestPacket())!= null)
	   {
	      if(!isPaused())
	         compressedVideoDataClient.consumeObject(packet.data, packet.position, packet.orientation, packet.fieldOfView);
	   }
	}	
	
   @Override
   public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, double fov)
   {
      counter.increment();
      if(frame!=null)
      {
         icon.setImage(bufferedImage);
         frame.invalidate();
         frame.repaint();
         frame.setTitle(counter.toString());
         frame.pack();
      }
   }
	


	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
	{
	}

	@Override
	protected void passReceivedControllerObjectToChildBehaviors(Object object)
	{
	}

	@Override
	public void stop()
	{
	   defaultStop();
	}

	@Override
	public void enableActions()
	{

	}

	@Override
	public void pause()
	{
	   defaultPause();
	}

	@Override
	public void resume()
	{
	   defaultResume();
	}

	@Override
	public boolean isDone()
	{
	   return defaultIsDone();
	}

	@Override
	public void finalize()
	{
	   defaultFinalize();
	}

	
   @Override
   public boolean hasInputBeenSet()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void initialize()
   {
      defaultFinalize();
   }
}
