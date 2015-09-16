package us.ihmc.humanoidBehaviors.behaviors;

import java.awt.FlowLayout;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import boofcv.struct.calib.IntrinsicParameters;

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
	         compressedVideoDataClient.consumeObject(packet.data, packet.position, packet.orientation, packet.getIntrinsicParameters());
	   }
	}	
	
   @Override
   public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters)
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
	public void doPostBehaviorCleanup()
	{
	   defaultPostBehaviorCleanup();
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
      defaultPostBehaviorCleanup();
   }
}
