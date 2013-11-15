package us.ihmc.darpaRoboticsChallenge.handControl.sandia;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;

import javax.swing.ButtonGroup;
import javax.swing.DefaultBoundedRangeModel;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSlider;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import osrf_msgs.JointCommands;
import us.ihmc.darpaRoboticsChallenge.handControl.sandia.SandiaHandModel.SandiaFingerName;
import us.ihmc.utilities.ros.RosTools;

public class SandiaHandManualControlUI extends AbstractNodeMain implements ActionListener
{
   private static final String MASTER_URI = "http://localhost:11311";

   private static final int sliderBounds = 20;
   private static final int sliderMin = -sliderBounds * 3;
   private static final int sliderMax = sliderBounds * 3;

   private Publisher<osrf_msgs.JointCommands> indexFingerJointPublisher, middleFingerJointPublisher, ringFingerJointPublisher, thumbJointPublisher;
   private osrf_msgs.JointCommands indexFingerJointCommand, middleFingerJointCommand, ringFingerJointCommand, thumbJointCommand;

   private HashMap<SandiaFingerName, Publisher<osrf_msgs.JointCommands>> fingerPublishers = new HashMap<SandiaFingerName, Publisher<osrf_msgs.JointCommands>>();
   private HashMap<SandiaFingerName, osrf_msgs.JointCommands> jointCommands = new HashMap<SandiaFingerName, osrf_msgs.JointCommands>();

   private ConnectedNode connectedNode;

   private JFrame frame = new JFrame("Sandia Hand Manual Controle");

   private GridBagConstraints c;

   private JPanel panel, handSelectionPanel, fingerSelectionPanel, sliderPanel, actionButtonPanel;

   private JLabel handLabel, fingerLabel, baseJointLabel, firstJointLabel, secondJointLabel;

   private ButtonGroup leftOrRight;
   private JRadioButton leftRadioButton, rightRadioButton;

   private JComboBox fingerComboBox;

   private JSlider baseJointSlider, firstJointSlider, secondJointSlider;

   private JButton sendCommandButton, resetButton;

   public SandiaHandManualControlUI()
   {
      //      Butts
   }

   public GraphName getDefaultNodeName()
   {
      return GraphName.of("darpaRoboticsChallenge/SandiaHandManualControlUI");
   }

   public void onStart(ConnectedNode connectedNode)
   {
      this.connectedNode = connectedNode;

      setupFrame();

      setupPublishers();

      setupJointCommandMessages();

      setupFingerMaps();
   }

   private void setupFingerMaps()
   {
      fingerPublishers.put(SandiaFingerName.INDEX, indexFingerJointPublisher);
      fingerPublishers.put(SandiaFingerName.MIDDLE, middleFingerJointPublisher);
      fingerPublishers.put(SandiaFingerName.RING, ringFingerJointPublisher);
      fingerPublishers.put(SandiaFingerName.THUMB, thumbJointPublisher);

      jointCommands.put(SandiaFingerName.INDEX, indexFingerJointCommand);
      jointCommands.put(SandiaFingerName.MIDDLE, middleFingerJointCommand);
      jointCommands.put(SandiaFingerName.RING, ringFingerJointCommand);
      jointCommands.put(SandiaFingerName.THUMB, thumbJointCommand);
   }

   private void setupFrame()
   {
      c = new GridBagConstraints();

      setupPanel();

      frame.add(panel);
      frame.setVisible(true);
      frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
      frame.pack();
   }

   private void setupPublishers()
   {
      indexFingerJointPublisher = connectedNode.newPublisher("/finger_0/joint_commands", osrf_msgs.JointCommands._TYPE);
      middleFingerJointPublisher = connectedNode.newPublisher("/finger_1/joint_commands", osrf_msgs.JointCommands._TYPE);
      ringFingerJointPublisher = connectedNode.newPublisher("/finger_2/joint_commands", osrf_msgs.JointCommands._TYPE);
      thumbJointPublisher = connectedNode.newPublisher("/finger_3/joint_commands", osrf_msgs.JointCommands._TYPE);
   }

   private void setupJointCommandMessages()
   {
      indexFingerJointCommand = indexFingerJointPublisher.newMessage();

      middleFingerJointCommand = middleFingerJointPublisher.newMessage();

      ringFingerJointCommand = ringFingerJointPublisher.newMessage();

      thumbJointCommand = thumbJointPublisher.newMessage();
   }

   private void setupPanel()
   {
      panel = new JPanel(new GridBagLayout());

      setuphandSelectionPanel();
      setupfingerSelectionPanel();
      setupSliderPanel();
      setupActionButtonPanel();

      c.ipadx = 100;
      c.ipady = 20;

      c.gridx = 0;
      c.gridy = 0;
      panel.add(handSelectionPanel, c);

      c.gridy = 1;
      panel.add(fingerSelectionPanel, c);

      c.gridy = 2;
      panel.add(sliderPanel, c);

      c.gridy = 3;
      panel.add(actionButtonPanel, c);
   }

   private void setuphandSelectionPanel()
   {
      handSelectionPanel = new JPanel(new GridBagLayout());

      handLabel = new JLabel("Select Hand:");

      leftRadioButton = new JRadioButton("Left");
      rightRadioButton = new JRadioButton("Right");
      rightRadioButton.setSelected(true);

      leftOrRight = new ButtonGroup();
      leftOrRight.add(leftRadioButton);
      leftOrRight.add(rightRadioButton);

      c.gridx = 0;
      c.gridy = 0;
      handSelectionPanel.add(handLabel, c);

      c.gridx = 1;
      handSelectionPanel.add(leftRadioButton, c);

      c.gridx = 2;
      handSelectionPanel.add(rightRadioButton, c);
   }

   private void setupfingerSelectionPanel()
   {
      fingerSelectionPanel = new JPanel(new GridBagLayout());

      fingerLabel = new JLabel("Select Finger:");

      fingerComboBox = new JComboBox(SandiaFingerName.values());
      fingerComboBox.setSelectedIndex(0);

      c.gridx = 0;
      c.gridy = 0;
      fingerSelectionPanel.add(fingerLabel, c);

      c.gridx = 1;
      fingerSelectionPanel.add(fingerComboBox, c);

   }

   private void setupSliderPanel()
   {
      sliderPanel = new JPanel(new GridBagLayout());

      baseJointLabel = new JLabel("Base Joint:");
      baseJointSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, sliderMin, sliderMax));

      firstJointLabel = new JLabel("First Joint:");
      firstJointSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, sliderMin, sliderMax));

      secondJointLabel = new JLabel("Second Joint:");
      secondJointSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, sliderMin, sliderMax));

      c.ipady = 10;

      c.gridx = 0;
      c.gridy = 0;
      sliderPanel.add(baseJointLabel, c);
      c.gridx = 1;
      sliderPanel.add(baseJointSlider, c);

      c.gridx = 0;
      c.gridy = 1;
      sliderPanel.add(firstJointLabel, c);
      c.gridx = 1;
      sliderPanel.add(firstJointSlider, c);

      c.gridx = 0;
      c.gridy = 2;
      sliderPanel.add(secondJointLabel, c);
      c.gridx = 1;
      sliderPanel.add(secondJointSlider, c);
   }

   private void setupActionButtonPanel()
   {
      actionButtonPanel = new JPanel(new GridBagLayout());

      sendCommandButton = new JButton("Send");
      sendCommandButton.addActionListener(this);

      resetButton = new JButton("Reset");
      resetButton.addActionListener(this);

      c.gridx = 0;
      c.gridy = 0;
      actionButtonPanel.add(sendCommandButton, c);

      c.gridx = 1;
      actionButtonPanel.add(resetButton, c);
   }

   public void actionPerformed(ActionEvent event)
   {
      if (((JButton) event.getSource()).getText().contains("Send"))
      {
         double baseJointPosition = (double) baseJointSlider.getValue() / (double) sliderBounds;
         double firstJointPosition = (double) firstJointSlider.getValue() / (double) sliderBounds;
         double secondJointPosition = (double) secondJointSlider.getValue() / (double) sliderBounds;

         double[] position = new double[] { baseJointPosition, firstJointPosition, secondJointPosition };

         osrf_msgs.JointCommands tempJointCommand = jointCommands.get(fingerComboBox.getSelectedItem());
         Publisher<JointCommands> tempJointPublisher = fingerPublishers.get(fingerComboBox.getSelectedItem());

         tempJointCommand.setPosition(position);
         tempJointPublisher.publish(tempJointCommand);
      }

      if (((JButton) event.getSource()).getText().contains("Reset"))
      {
         baseJointSlider.setValue(0);
         firstJointSlider.setValue(0);
         secondJointSlider.setValue(0);
      }

   }

   public static void main(String[] args) throws URISyntaxException
   {
      URI master;
      if (args.length > 0)
         master = new URI(args[0]);
      else
         master = new URI(MASTER_URI);

      SandiaHandManualControlUI manualControlUI = new SandiaHandManualControlUI();
      NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
      NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(manualControlUI, nodeConfiguration);
   }

}
