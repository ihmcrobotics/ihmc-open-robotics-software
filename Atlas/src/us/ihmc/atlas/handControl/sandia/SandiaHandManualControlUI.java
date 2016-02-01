package us.ihmc.atlas.handControl.sandia;

import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;

import javax.swing.BorderFactory;
import javax.swing.DefaultBoundedRangeModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import osrf_msgs.JointCommands;
import us.ihmc.atlas.handControl.sandia.SandiaHandModel.SandiaFingerName;
import us.ihmc.utilities.ros.RosTools;

public class SandiaHandManualControlUI extends AbstractNodeMain
{
   private static final String MASTER_URI = "http://localhost:11311";

   private static final int joinSliderBounds = 20;
   private static final int jointSliderMin = -joinSliderBounds * 3;
   private static final int jointSliderMax = joinSliderBounds * 3;
   private static final int graspSliderMax = 10;
   private static final int graspSliderMin = 0;

   private Publisher<osrf_msgs.JointCommands> indexFingerJointPublisher, middleFingerJointPublisher, ringFingerJointPublisher, thumbJointPublisher;
   private osrf_msgs.JointCommands indexFingerJointCommand, middleFingerJointCommand, ringFingerJointCommand, thumbJointCommand;

   private Publisher<sandia_hand_msgs.SimpleGrasp> simpleGraspPublisher;
   private sandia_hand_msgs.SimpleGrasp simpleGraspCommand;

   private HashMap<SandiaFingerName, Publisher<osrf_msgs.JointCommands>> fingerPublishers = new HashMap<SandiaFingerName, Publisher<osrf_msgs.JointCommands>>();
   private HashMap<SandiaFingerName, osrf_msgs.JointCommands> jointCommands = new HashMap<SandiaFingerName, osrf_msgs.JointCommands>();

   private ConnectedNode connectedNode;

   private JFrame frame = new JFrame("Sandia Hand Manual Control");

   private GridBagConstraints c;

   private JPanel panel, leftPanel, rightPanel, fingerSelectionPanel, sliderPanel, actionButtonPanel, graspTypePanel, graspControlPanel;

   private JLabel graspSelectionLabel, fingerLabel, baseJointLabel, firstJointLabel, secondJointLabel, graspControlLabel;

   private JComboBox<GraspTypes> graspTypeComboBox;

   private JCheckBox thumbCheckBox, indexCheckBox, middleCheckBox, ringCheckBox;
   private HashMap<SandiaFingerName, JCheckBox> fingerCheckBoxes = new HashMap<SandiaFingerName, JCheckBox>();

   private JSlider baseJointSlider, firstJointSlider, secondJointSlider, graspControlSlider;

   private JButton resetButton;

   enum GraspTypes
   {
      CYLINDRICAL, SPHERICAL, PRISMATIC, NUMBER_ONE, PEACE, ROCK_N_ROLL;
   }

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
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.pack();
      frame.setResizable(false);
      frame.setVisible(true);
   }

   private void setupPublishers()
   {
      indexFingerJointPublisher = connectedNode.newPublisher("/finger_0/joint_commands", osrf_msgs.JointCommands._TYPE);
      middleFingerJointPublisher = connectedNode.newPublisher("/finger_1/joint_commands", osrf_msgs.JointCommands._TYPE);
      ringFingerJointPublisher = connectedNode.newPublisher("/finger_2/joint_commands", osrf_msgs.JointCommands._TYPE);
      thumbJointPublisher = connectedNode.newPublisher("/finger_3/joint_commands", osrf_msgs.JointCommands._TYPE);

      simpleGraspPublisher = connectedNode.newPublisher("/simple_grasp", sandia_hand_msgs.SimpleGrasp._TYPE);
   }

   private void setupJointCommandMessages()
   {
      indexFingerJointCommand = indexFingerJointPublisher.newMessage();
      middleFingerJointCommand = middleFingerJointPublisher.newMessage();
      ringFingerJointCommand = ringFingerJointPublisher.newMessage();
      thumbJointCommand = thumbJointPublisher.newMessage();

      simpleGraspCommand = simpleGraspPublisher.newMessage();
   }

   private void addPanels()
   {
      panel = new JPanel(new GridBagLayout());

      setupLeftPanel();
      setupRightPanel();

      setupActionButtonPanel();

      c.insets = new Insets(5, 5, 5, 5);
      c.fill = GridBagConstraints.BOTH;
      c.ipadx = 50;

      JPanel controlBoardPanel = new JPanel(new GridBagLayout());

      c.gridx = 0;
      c.gridy = 0;
      controlBoardPanel.add(leftPanel, c);

      ++c.gridx;
      controlBoardPanel.add(rightPanel, c);

      c.gridx = 0;
      c.gridy = 0;
      panel.add(controlBoardPanel, c);

      ++c.gridy;
      panel.add(actionButtonPanel, c);
   }

   private void setupLeftPanel()
   {
      leftPanel = new JPanel(new GridBagLayout());
      leftPanel.setBorder(BorderFactory.createEtchedBorder());

      setupFingerSelectionPanel();
      setupSliderPanel();

      c.gridx = 0;
      c.gridy = 0;
      leftPanel.add(fingerSelectionPanel, c);

      ++c.gridy;
      leftPanel.add(sliderPanel, c);
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
      fingerCheckBoxes.put(SandiaFingerName.THUMB, thumbCheckBox);
      rightFingerCheckBoxPanel.add(middleCheckBox, c);
      middleCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      fingerCheckBoxes.put(SandiaFingerName.MIDDLE, middleCheckBox);

      ++c.gridy;
      leftFingerCheckBoxPanel.add(indexCheckBox, c);
      indexCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      fingerCheckBoxes.put(SandiaFingerName.INDEX, indexCheckBox);
      rightFingerCheckBoxPanel.add(ringCheckBox, c);
      ringCheckBox.setAlignmentX(Component.LEFT_ALIGNMENT);
      fingerCheckBoxes.put(SandiaFingerName.RING, ringCheckBox);

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
      baseJointSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, jointSliderMin, jointSliderMax));
      baseJointSlider.addMouseListener(new FingerJointSliderMouseListener());

      firstJointLabel = new JLabel("First Joint:");
      firstJointSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, jointSliderMin, jointSliderMax));
      firstJointSlider.addMouseListener(new FingerJointSliderMouseListener());

      secondJointLabel = new JLabel("Second Joint:");
      secondJointSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, jointSliderMin, jointSliderMax));
      secondJointSlider.addMouseListener(new FingerJointSliderMouseListener());

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

      graspTypeComboBox = new JComboBox<GraspTypes>(GraspTypes.values());
      graspTypeComboBox.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
        	 graspControlSlider.setValue(0);
        	 
        	 simpleGraspCommand.setName(graspTypeComboBox.getSelectedItem().toString().toLowerCase());
             simpleGraspCommand.setClosedAmount(0.0);

             simpleGraspPublisher.publish(simpleGraspCommand);
         }
      });

      JPanel graspSelectionPanel = new JPanel(new GridBagLayout());

      c.anchor = GridBagConstraints.CENTER;
      c.gridx = 0;
      c.gridy = 0;
      graspSelectionPanel.add(graspSelectionLabel, c);

      ++c.gridx;
      graspSelectionPanel.add(graspTypeComboBox, c);

      c.gridx = 0;
      c.gridy = 0;
      c.ipady = 10;
      graspTypePanel.add(graspSelectionLabel, c);

      ++c.gridy;
      graspTypePanel.add(graspSelectionPanel, c);
   }

   private void setupGraspControlPanel()
   {
      graspControlPanel = new JPanel(new GridBagLayout());

      graspControlLabel = new JLabel("Grasp Control:");
      graspControlSlider = new JSlider(new DefaultBoundedRangeModel(0, 1, graspSliderMin, graspSliderMax));
      graspControlSlider.addMouseListener(new GraspControlSliderMouseListener());

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

      resetButton = new JButton("Reset");
      resetButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent arg0)
         {
            baseJointSlider.setValue(0);
            firstJointSlider.setValue(0);
            secondJointSlider.setValue(0);
            graspControlSlider.setValue(0);

            graspTypeComboBox.setSelectedIndex(0);

            for (SandiaFingerName fingerName : SandiaFingerName.values())
            {
               fingerCheckBoxes.get(fingerName).setSelected(false);
            }

            simpleGraspCommand.setName(GraspTypes.CYLINDRICAL.toString().toLowerCase());
            simpleGraspCommand.setClosedAmount(0.0);

            simpleGraspPublisher.publish(simpleGraspCommand);
         }
      });

      c.gridx = 0;
      c.gridy = 0;
      actionButtonPanel.add(resetButton, c);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      URI master;
      if (args.length > 0)
         master = new URI(args[0]);
      else
         master = new URI(MASTER_URI);

      NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
      NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(new SandiaHandManualControlUI(), nodeConfiguration);
   }

   class FingerJointSliderMouseListener implements MouseListener
   {
      @Override
      public void mouseClicked(MouseEvent arg0)
      {
      }

      @Override
      public void mouseEntered(MouseEvent arg0)
      {
      }

      @Override
      public void mouseExited(MouseEvent arg0)
      {
      }

      @Override
      public void mousePressed(MouseEvent arg0)
      {
      }

      @Override
      public void mouseReleased(MouseEvent arg0)
      {
         double baseJointPosition = (double) baseJointSlider.getValue() / (double) joinSliderBounds;
         double firstJointPosition = (double) firstJointSlider.getValue() / (double) joinSliderBounds;
         double secondJointPosition = (double) secondJointSlider.getValue() / (double) joinSliderBounds;

         double[] position = new double[] {baseJointPosition, firstJointPosition, secondJointPosition};

         for (SandiaFingerName fingerName : SandiaFingerName.values())
         {
            if (fingerCheckBoxes.get(fingerName).isSelected())
            {
               osrf_msgs.JointCommands tempJointCommand = jointCommands.get(fingerName);
               Publisher<JointCommands> tempJointPublisher = fingerPublishers.get(fingerName);

               tempJointCommand.setPosition(position);
               tempJointPublisher.publish(tempJointCommand);
            }
         }
      }
   }


   class GraspControlSliderMouseListener implements MouseListener
   {
      @Override
      public void mouseClicked(MouseEvent arg0)
      {
      }

      @Override
      public void mouseEntered(MouseEvent arg0)
      {
      }

      @Override
      public void mouseExited(MouseEvent arg0)
      {
      }

      @Override
      public void mousePressed(MouseEvent arg0)
      {
      }

      @Override
      public void mouseReleased(MouseEvent arg0)
      {
         simpleGraspCommand.setName(graspTypeComboBox.getSelectedItem().toString().toLowerCase());
         simpleGraspCommand.setClosedAmount((double) graspControlSlider.getValue() / (double) graspSliderMax);

         simpleGraspPublisher.publish(simpleGraspCommand);

      }

   }
}
