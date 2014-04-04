package us.ihmc.darpaRoboticsChallenge.handControl.sandia;

import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.DefaultBoundedRangeModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
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

   private JFrame frame = new JFrame("Sandia Hand Manual Control");

   private GridBagConstraints c;

   private JPanel panel, leftPanel, middlePanel, rightPanel, handSelectionPanel, controlTypeSelectionPanel, fingerSelectionPanel,
                  sliderPanel, actionButtonPanel, graspTypePanel, graspControlPanel;

   private JLabel handLabel, controlTypeLabel, graspSelectionLabel, fingerLabel, baseJointLabel, firstJointLabel, secondJointLabel, graspControlLabel;

   private ButtonGroup leftOrRight, controlType, graspType;
   private JRadioButton leftRadioButton, rightRadioButton, fullHandRadioButton, individualFingerRadioButton, cylindricalGraspRadioButton,
                        sphericalGraspRadioButton, prismaticGraspRadioButton;

   private JCheckBox thumbCheckBox, indexCheckBox, middleCheckBox, ringCheckBox;

   private JSlider baseJointSlider, firstJointSlider, secondJointSlider, graspControlSlider;

   private JButton sendCommandButton, resetButton;

   public SandiaHandManualControlUI()
   {
      // Butts
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

      addPanels();

      frame.add(panel);
      frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
      frame.pack();
      frame.setVisible(true);
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

   private void addPanels()
   {
      panel = new JPanel(new GridBagLayout());

      setupLeftPanel();
      setupMiddlePanel();
      setupRightPanel();

      setupActionButtonPanel();

      c.insets = new Insets(5, 5, 5, 5);
      c.fill = GridBagConstraints.BOTH;
      c.ipadx = 50;
      c.ipady = 20;

      c.gridx = 0;
      c.gridy = 0;
      panel.add(leftPanel, c);
      
      ++c.gridx;
      panel.add(middlePanel, c);
      
      ++c.gridx;
      panel.add(rightPanel, c);
   }

   private void setupLeftPanel()
   {
      leftPanel = new JPanel(new GridBagLayout());
      leftPanel.setBorder(BorderFactory.createEtchedBorder());

      setupHandSelectionPanel();
      setupControlTypeSelectionPanel();

      c.gridx = 0;
      c.gridy = 0;
      leftPanel.add(handSelectionPanel, c);

      c.gridy++;
      leftPanel.add(controlTypeSelectionPanel, c);
   }

   private void setupMiddlePanel()
   {
      middlePanel = new JPanel(new GridBagLayout());
      middlePanel.setBorder(BorderFactory.createEtchedBorder());

      setupFingerSelectionPanel();
      setupSliderPanel();
      
      c.gridx = 0;
      c.gridy = 0;
      middlePanel.add(fingerSelectionPanel, c);
      
      ++c.gridy;
      middlePanel.add(sliderPanel, c);
   }

   private void setupRightPanel()
   {
	   rightPanel = new JPanel(new GridBagLayout());
	   rightPanel.setBorder(BorderFactory.createEtchedBorder());
	   
	   setupGraspSelectionPanel();
	   setupGraspControlPanel();
	   
	   c.gridx = 0;
	   c.gridy = 0;
	   rightPanel.add(graspTypePanel, c);
	   
	   ++c.gridy;
	   rightPanel.add(graspControlPanel, c);
   }

   private void setupHandSelectionPanel()
   {
      handSelectionPanel = new JPanel(new GridBagLayout());

      handLabel = new JLabel("Select Hand:");

      leftRadioButton = new JRadioButton("Left");
      rightRadioButton = new JRadioButton("Right");
      leftRadioButton.setSelected(true);

      leftOrRight = new ButtonGroup();
      leftOrRight.add(leftRadioButton);
      leftOrRight.add(rightRadioButton);

      c.gridx = 0;
      c.gridy = 0;
      handSelectionPanel.add(handLabel, c);

      ++c.gridy;
      handSelectionPanel.add(leftRadioButton, c);

      ++c.gridy;
      handSelectionPanel.add(rightRadioButton, c);
   }

   private void setupControlTypeSelectionPanel()
   {
      controlTypeSelectionPanel = new JPanel(new GridBagLayout());

      controlTypeLabel = new JLabel("Select Control Type:");

      individualFingerRadioButton = new JRadioButton("Individual Fingers");
      fullHandRadioButton = new JRadioButton("Full Hand");
      individualFingerRadioButton.setSelected(true);

      controlType = new ButtonGroup();
      controlType.add(individualFingerRadioButton);
      controlType.add(fullHandRadioButton);

      c.gridx = 0;
      c.gridy = 0;
      controlTypeSelectionPanel.add(controlTypeLabel, c);

      ++c.gridy;
      controlTypeSelectionPanel.add(individualFingerRadioButton, c);

      ++c.gridy;
      controlTypeSelectionPanel.add(fullHandRadioButton, c);
   }

   private void setupFingerSelectionPanel()
   {
      fingerSelectionPanel = new JPanel(new GridBagLayout());

      fingerLabel = new JLabel("Select Fingers:");
      
      thumbCheckBox = new JCheckBox("Thumb");
      indexCheckBox = new JCheckBox("Index");
      middleCheckBox = new JCheckBox("Middle");
      ringCheckBox = new JCheckBox("Ring");
      
      JPanel leftFingerCheckBoxPanel = new JPanel(new GridBagLayout());
      JPanel rightFingerCheckBoxPanel = new JPanel(new GridBagLayout());
      
      c.anchor = GridBagConstraints.WEST;
      c.gridx = 0;
      c.gridy = 0;
      leftFingerCheckBoxPanel.add(thumbCheckBox, c);
      thumbCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      rightFingerCheckBoxPanel.add(middleCheckBox, c);
      middleCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      
      ++c.gridy;
      leftFingerCheckBoxPanel.add(indexCheckBox, c);
      indexCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      rightFingerCheckBoxPanel.add(ringCheckBox, c);
      ringCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      
      JPanel fingerCheckBoxPanel = new JPanel(new GridBagLayout());
      
      c.gridx = 0;
      c.gridy = 0;
      fingerCheckBoxPanel.add(leftFingerCheckBoxPanel, c);
      
      ++c.gridx;
      fingerCheckBoxPanel.add(rightFingerCheckBoxPanel, c);
      
      c.gridx = 0;
      c.gridy = 0;
      c.anchor = GridBagConstraints.CENTER;
      fingerSelectionPanel.add(fingerLabel, c);
      
      ++c.gridy;
      fingerSelectionPanel.add(fingerCheckBoxPanel, c);
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
   
   private void setupGraspSelectionPanel()
   {
	   graspTypePanel = new JPanel(new GridBagLayout());
	   
	   graspSelectionLabel = new JLabel("Select Grasp Type:");
	   
	   cylindricalGraspRadioButton = new JRadioButton("Cylindrical");
	   sphericalGraspRadioButton = new JRadioButton("Spherical");
	   prismaticGraspRadioButton = new JRadioButton("Prismatic");
	   cylindricalGraspRadioButton.setSelected(true);
	   
	   graspType = new ButtonGroup();
	   graspType.add(cylindricalGraspRadioButton);
	   graspType.add(sphericalGraspRadioButton);
	   graspType.add(prismaticGraspRadioButton);
	   
	   JPanel graspSelectionPanel = new JPanel(new GridBagLayout());
	   
	   c.anchor = GridBagConstraints.WEST;
	   c.gridx = 0;
	   c.gridy = 0;
	   graspSelectionPanel.add(cylindricalGraspRadioButton, c);
	   
	   ++c.gridy;
	   graspSelectionPanel.add(sphericalGraspRadioButton, c);
	   
	   ++c.gridy;
	   graspSelectionPanel.add(prismaticGraspRadioButton, c);
	   
	   c.anchor = GridBagConstraints.CENTER;
	   c.gridx = 0;
	   c.gridy = 0;
	   graspTypePanel.add(graspSelectionLabel, c);
	   
	   ++c.gridy;
	   graspTypePanel.add(graspSelectionPanel, c);
   }
   
   private void setupGraspControlPanel()
   {
	   graspControlPanel = new JPanel(new GridBagLayout());
	   
	   graspControlLabel = new JLabel("Grasp Control");
	   graspControlSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, 0, 10));
	   
	   c.anchor = GridBagConstraints.CENTER;
	   c.gridx = 0;
	   c.gridy = 0;
	   graspControlPanel.add(graspControlLabel, c);
	   
	   ++c.gridx;
	   graspControlPanel.add(graspControlSlider, c);
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

         double[] position = new double[] {baseJointPosition, firstJointPosition, secondJointPosition};

//         osrf_msgs.JointCommands tempJointCommand = jointCommands.get(fingerComboBox.getSelectedItem());
//         Publisher<JointCommands> tempJointPublisher = fingerPublishers.get(fingerComboBox.getSelectedItem());

//         tempJointCommand.setPosition(position);
//         tempJointPublisher.publish(tempJointCommand);
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
//    URI master;
//    if (args.length > 0)
//       master = new URI(args[0]);
//    else
//       master = new URI(MASTER_URI);
//
//    NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
//    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
//    nodeMainExecutor.execute(new SandiaHandManualControlUI(), nodeConfiguration);

      new SandiaHandManualControlUI().setupFrame();
   }

}
