package us.ihmc.darpaRoboticsChallenge;

import java.awt.Dimension;
import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;
import java.net.URI;
import java.net.URISyntaxException;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

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

public class ExternalCameraFeed extends AbstractNodeMain
{
	public static final boolean COLOR_IMAGE = true;
	
	private boolean showRecordingButtons;
	
	private Subscriber<sensor_msgs.CompressedImage> cameraSubscriber;
	private BufferedImage cameraImage;
	private String cameraName, topicName;
	private JFrame cameraFrame, recordButtonFrame;
	private JPanel cameraPanel, recordButtonPanel;
	private JButton startRecordingButton, stopRecordingButton;

	private ColorSpace colorSpace;
	private ColorModel colorModel;
	
	private ConnectedNode connectedNode;
	
	public ExternalCameraFeed(String cameraName, String topicName)
	{
	   new ExternalCameraFeed(cameraName, topicName, false);
	}
	
	public ExternalCameraFeed(String cameraName, String topicName, boolean showRecordingButtons)
	{
		new RepeatingReleasedEventsFixer().install();
		
		this.cameraName = cameraName;
		this.topicName = topicName;
		
		colorSpace = (COLOR_IMAGE ? ColorSpace.getInstance(ColorSpace.CS_sRGB) : ColorSpace.getInstance(ColorSpace.CS_GRAY));
		colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
		
		this.showRecordingButtons = showRecordingButtons;
	}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("darpaRoboticsChallenge/ExternalCameraFeed/" + cameraName);
	}
	
	public void onStart(ConnectedNode connectedNode)
	{
		this.connectedNode = connectedNode;
		
		setupJFrame();
		
		setupSubscriber(connectedNode);
		
		setupCameraSubscriberListener();
	}
	
	private void setupCameraSubscriberListener()
	{
	   cameraSubscriber.addMessageListener(new MessageListener<sensor_msgs.CompressedImage>()
		{

			public void onNewMessage(sensor_msgs.CompressedImage message)
			{
				if (cameraFrame.isVisible())
				{
					cameraImage = RosTools.bufferedImageFromRosMessageJpeg(colorModel, message);
					cameraPanel.getGraphics().drawImage(cameraImage.getScaledInstance(cameraImage.getWidth(), cameraImage.getHeight(), 0), 0, 0, null);
				}
			}
			
		});
	}
	
	private void setupSubscriber(ConnectedNode connectedNode)
	{
		cameraSubscriber = connectedNode.newSubscriber(topicName, sensor_msgs.CompressedImage._TYPE);
	}
	
	private void setupJFrame()
	{
		setupCameraFrame();
		
		setupRecordButtonFrame();
	}


   private void setupCameraFrame()
   {
      cameraFrame = new JFrame(topicName);
		cameraFrame.setMinimumSize(new Dimension(800, 800));
		
		cameraPanel = new JPanel();
		cameraFrame.add(cameraPanel);
		
		cameraFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		cameraFrame.setVisible(true);
   }

   private void setupRecordButtonFrame()
   {
      recordButtonFrame = new JFrame("Record");
      recordButtonFrame.setMinimumSize(new Dimension(300, 40));
      
      recordButtonPanel = new JPanel();
      
      setupRecordButtonPanel();
      
      recordButtonFrame.add(recordButtonPanel);
      recordButtonFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      
      recordButtonFrame.setVisible(showRecordingButtons);
   }

   private void setupRecordButtonPanel()
   {
      startRecordingButton = new JButton("Start Recording");
      stopRecordingButton = new JButton("Stop Recording");
      
      stopRecordingButton.setEnabled(false);
      
      recordButtonPanel.add(startRecordingButton);
      recordButtonPanel.add(stopRecordingButton);
      
      startRecordingButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            
            startRecordingButton.setEnabled(false);
            stopRecordingButton.setEnabled(true);
            
         }  
      });
      
      stopRecordingButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            stopRecordingButton.setEnabled(false);
            startRecordingButton.setEnabled(true);
         }  
      });
   }
   
	public static void main(String[] args) throws URISyntaxException
	{
		try
		{
		   if (args.length < 3)
		      throw new IllegalArgumentException("Insufficient arguments provided. Please provide IP of MASTER, camera name, and topic name.");
		   else 
		   {
		      URI master = new URI(args[0]);
		      ExternalCameraFeed externalCamera;
		      if (args.length == 3)
		         externalCamera = new ExternalCameraFeed(args[1], args[2]);
		      else
		         externalCamera = new ExternalCameraFeed(args[1], args[2], args[3].equals("true"));
		      NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
		      NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		      nodeMainExecutor.execute(externalCamera, nodeConfiguration);
		   }
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}
}