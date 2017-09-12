package us.ihmc.avatar.ros;

import java.net.URI;
import java.net.URISyntaxException;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import us.ihmc.tools.inputDevices.keyboard.linux.RepeatingReleasedEventsFixer;
import us.ihmc.utilities.ros.RosTools;

public class RosCameraBenchmarker extends AbstractNodeMain
{
	private Subscriber<sensor_msgs.CompressedImage> cameraSubscriber;
	private String topicName;
	
	private long initialTime;
	private double framesReceived = 0.0;
	
	public RosCameraBenchmarker(String topicName)
	{
		new RepeatingReleasedEventsFixer().install();
		
		this.topicName = topicName;
	}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("darpaRoboticsChallenge/RosCameraBenchmarker" + topicName);
	}
	
	public void onStart(ConnectedNode connectedNode)
	{
		setupSubscriber(connectedNode);
		
		setupCameraSubscriberListener();
	}
	
	private void setupCameraSubscriberListener()
	{
		cameraSubscriber.addMessageListener(new MessageListener<sensor_msgs.CompressedImage>()
		{
			public void onNewMessage(sensor_msgs.CompressedImage message)
			{
				if (framesReceived == 0.0)
					initialTime = System.currentTimeMillis();
				else
				{
					System.out.println("FPS: "+(framesReceived*1000.0)/(System.currentTimeMillis()-initialTime));
				}
				++framesReceived;
			}
			
		});
	}
	
	private void setupSubscriber(ConnectedNode connectedNode)
	{
		cameraSubscriber = connectedNode.newSubscriber(topicName, sensor_msgs.CompressedImage._TYPE);
	}
	
	public static void main(String[] args) throws URISyntaxException
	{	
	   String MASTER = "http://localhost:11311";
	   String TOPIC_NAME = "/camera/image_raw/compressed";
		URI master = new URI(MASTER);
		
		try
		{
			RosCameraBenchmarker externalCamera = new RosCameraBenchmarker(TOPIC_NAME);
			NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
			NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(externalCamera, nodeConfiguration);
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}
}
