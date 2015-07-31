package us.ihmc.atlas.processManagement;

import java.awt.BorderLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

import javax.swing.AbstractButton;
import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.border.EmptyBorder;
import javax.swing.border.EtchedBorder;
import javax.swing.text.AttributeSet;
import javax.swing.text.BadLocationException;
import javax.swing.text.PlainDocument;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.atlas.AtlasOperatorUserInterface;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.tcpServer.DisconnectedException;
import us.ihmc.communication.net.tcpServer.ReconnectingTCPClient;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.utilities.processManagement.UnsignedByteTools;
import us.ihmc.utilities.processManagement.ExitListener;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public class DRCEnterpriseCloudDispatcherFrontend implements Runnable
{
   private static final String BLUE_TEAM_ACTION_COMMAND = "blue";
   private static final String RED_TEAM_ACTION_COMMAND = "red";
   private static final boolean ENABLE_CONSOLE_OUTPUT = false;

   private ReconnectingTCPClient netProcClient;
   private final byte[] netProcBuffer;

   private ReconnectingTCPClient controllerClient;
   private final byte[] controllerBuffer;

   private static String netProcMachineIpAddress = "10.6.12.44";
   private static String controllerMachineIpAddress = "192.168.130.112";

   private JFrame frame;
   private JPanel netProcPanel;
   private JPanel controllerPanel;
   private JPanel selectControllerPanel;

   private JButton netProcStartButton, netProcStopButton, netProcRestartButton;
   private final JLabel netProcRunningStatusLabel =
      new JLabel("<html><body>Running: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>", JLabel.CENTER);
   private final JLabel netProcConnectedStatusLabel =
      new JLabel("<html><body>Connected: <span style=\"color:red;font-style:italic;\">Disconnected</span></body></html>", JLabel.CENTER);

   private JButton controllerStartButton, controllerStopButton;
   private final JLabel controllerRunningStatusLabel =
      new JLabel("<html><body>Running: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>", JLabel.CENTER);
   private final JLabel controllerConnectedStatusLabel =
      new JLabel("<html><body>Connected: <span style=\"color:red;font-style:italic;\">Disconnected</span></body></html>", JLabel.CENTER);

   private ButtonGroup selectControllerRadioButtonGroup, selectRobotModelRadioButtonGroup;
   private final JTextArea netProcConsole, controllerConsole;
   private JScrollPane robotModelScrollPane;

   private final JavaProcessSpawner localSpawner = new JavaProcessSpawner(true);
   private JButton spawnUIButton;
   private JButton spawnBehaviorModuleButton;
   private JPanel handIPPanel;
   private JTextField leftHandField;
   private JTextField rightHandField;

   public DRCEnterpriseCloudDispatcherFrontend()
   {
      setupNetProcSocket();

      setupControllerSocket();

      netProcBuffer = netProcClient.getBuffer();
      controllerBuffer = controllerClient.getBuffer();
      netProcConsole = new JTextArea();
      controllerConsole = new JTextArea();
   }

   private void requestControllerStream()
   {
      try
      {
         controllerClient.write(new byte[] {UnsignedByteTools.fromInt(0x22)});
      }
      catch (DisconnectedException e)
      {
         controllerClient.reset();
      }
   }

   private void requestNetProcStream()
   {
      try
      {
         netProcClient.write(new byte[] {UnsignedByteTools.fromInt(0x22)});
      }
      catch (DisconnectedException e)
      {
         netProcClient.reset();
      }
   }

   private void setupNetProcSocket()
   {
      netProcClient = new ReconnectingTCPClient(netProcMachineIpAddress, NetworkPorts.NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT.getPort());
      netProcClient.attachStateListener(new NetStateListener()
      {
         public void connected()
         {
            netProcConnectedStatusLabel.setText("<html><body>Connected: <span style=\"color:green;font-style:italic;\">Connected</span></body></html>");
            netProcNotRunningButtonConfiguration();
            repaintFrame();
         }

         public void disconnected()
         {
            netProcConnectedStatusLabel.setText("<html><body>Connected: <span style=\"color:red;font-style:italic;\">Disconnected</span></body></html>");
            netProcRunningStatusLabel.setText("<html><body>Running: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>");
            netProcDisableAll();
            repaintFrame();
         }
      });
   }

   private void setupControllerSocket()
   {
      controllerClient = new ReconnectingTCPClient(controllerMachineIpAddress, NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT.getPort());
      controllerClient.attachStateListener(new NetStateListener()
      {
         public void connected()
         {
            controllerConnectedStatusLabel.setText("<html><body>Connected: <span style=\"color:green;font-style:italic;\">Connected</span></body></html>");
            controllerNotRunningButtonConfiguration();
            repaintFrame();
         }

         public void disconnected()
         {
            controllerConnectedStatusLabel.setText("<html><body>Connected: <span style=\"color:red;font-style:italic;\">Disconnected</span></body></html>");
            controllerRunningStatusLabel.setText("<html><body>Running: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>");
            controllerDisableAll();
            repaintFrame();
         }
      });
   }

   private void repaintFrame()
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            frame.validate();
            frame.repaint();
         }
      });
   }

   public void run()
   {
      new Thread(new Runnable()
      {
         public void run()
         {
            netProcClient.connect();
            netProcNotRunningButtonConfiguration();
         }
      }).start();

      new Thread(new Runnable()
      {
         public void run()
         {
            controllerClient.connect();
            controllerNotRunningButtonConfiguration();
         }
      }).start();

      new Thread(new Runnable()
      {
         public void run()
         {
            while (true)
            {
               processNetProcSocket();
            }
         }
      }).start();

      new Thread(new Runnable()
      {
         public void run()
         {
            while (true)
            {
               processControllerSocket();
            }
         }
      }).start();
   }

   private void processNetProcSocket()
   {
      try
      {
         netProcClient.read(1);

         switch (UnsignedByteTools.toInt(netProcBuffer[0]))
         {
            case 0x00 :
               netProcRunningButtonConfiguration();
               netProcRunningStatusLabel.setText("<html><body>Network Processor Status: <span style=\"color:green;font-style:italic;"
                                                 + "\">Running</span></body></html>");
               repaintFrame();

               break;

            case 0x11 :
               netProcNotRunningButtonConfiguration();
               netProcRunningStatusLabel.setText(
                   "<html><body>Network Processor Status: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>");
               repaintFrame();

               break;

            case 0x22 :
               startReceivingNetProcConsoleText();

               break;

            default :
               System.err.println("Invalid status: " + Integer.toHexString(UnsignedByteTools.toInt(netProcBuffer[0])));

               break;
         }
      }
      catch (DisconnectedException e)
      {
         netProcClient.reset();
      }

      netProcClient.reset();
   }

   private void processControllerSocket()
   {
      try
      {
         controllerClient.read(1);

         switch (UnsignedByteTools.toInt(controllerBuffer[0]))
         {
            case 0x00 :
               controllerRunningButtonConfiguration();
               controllerRunningStatusLabel.setText("<html><body>Controller Status: <span style=\"color:green;font-style:italic;"
                       + "\">Running</span></body></html>");
               repaintFrame();

               break;

            case 0x10 :
               controllerDisableAll();
               controllerRunningStatusLabel.setText("<html><body>Controller Status: <span style=\"color:gray;font-style:italic;"
                       + "\">Restarting</span></body></html>");
               repaintFrame();

               break;

            case 0x11 :
               controllerNotRunningButtonConfiguration();
               controllerRunningStatusLabel.setText(
                   "<html><body>Controller Status: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>");
               repaintFrame();

               break;

            case 0x22 :
               startReceivingControllerConsoleText();

               break;

            default :
               System.err.println("Invalid status: " + Integer.toHexString(UnsignedByteTools.toInt(controllerBuffer[0])));

               break;
         }
      }
      catch (DisconnectedException e)
      {
         controllerClient.reset();
      }

      controllerClient.reset();
   }

   private void netProcDisableAll()
   {
      netProcStartButton.setEnabled(false);
      netProcStopButton.setEnabled(false);
      netProcRestartButton.setEnabled(false);
   }

   private void netProcNotRunningButtonConfiguration()
   {
      netProcStartButton.setEnabled(true);
      netProcStopButton.setEnabled(false);
      netProcRestartButton.setEnabled(false);
   }

   private void netProcRunningButtonConfiguration()
   {
      netProcStartButton.setEnabled(false);
      netProcStopButton.setEnabled(true);
      netProcRestartButton.setEnabled(true);
   }

   private void controllerDisableAll()
   {
      controllerStartButton.setEnabled(false);
      controllerStopButton.setEnabled(false);
   }

   private void controllerNotRunningButtonConfiguration()
   {
      controllerStartButton.setEnabled(true);
      controllerStopButton.setEnabled(false);
   }

   private void controllerRunningButtonConfiguration()
   {
      controllerStartButton.setEnabled(false);
      controllerStopButton.setEnabled(true);
   }

   private void initAndStartSwingGui()
   {
      try
      {
         UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
      }
      catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException e1)
      {
         e1.printStackTrace();
      }

      frame = new JFrame("DRC Networking Dispatcher");
      frame.setSize(1310, 345);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

//    frame.setAlwaysOnTop(true);
      frame.setLocationRelativeTo(null);

      JPanel mainPanel = new JPanel(new GridBagLayout());

      // mainPanel.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));

      GridBagConstraints c = new GridBagConstraints();

      JPanel controlsPanel = new JPanel(new GridLayout(1, 5));
      ((GridLayout) controlsPanel.getLayout()).setHgap(5);

      setupNetProcPanel();

      setupControllerPanel();

      setupSelectControllerPanel();

      setupSelectRobotModelPanel();

      setupHandIPPanel();

      controlsPanel.add(controllerPanel);
      controlsPanel.add(netProcPanel);
      controlsPanel.add(selectControllerPanel);
      controlsPanel.add(robotModelScrollPane);
      controlsPanel.add(handIPPanel);

      c.gridx = 0;
      c.gridy = 0;
      c.insets = new Insets(10, 10, 5, 10);
      c.weighty = 0.9;
      c.fill = GridBagConstraints.HORIZONTAL;
      c.ipadx = 180;
      c.ipady = 200;
      mainPanel.add(controlsPanel, c);

      JPanel panel = new JPanel();
      spawnUIButton = new JButton("Start Operator UI");
      spawnUIButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (verifyEverythingIsConfigured())
               JOptionPane.showMessageDialog(frame, "Please complete the Controller/Robot Model configuration!", "Bad Deploy Configuration",
                                             JOptionPane.ERROR_MESSAGE);
            else
            {
               String[] javaArgs = new String[] {"-Xms4096m", "-Xmx4096m"};
               String[] programArgs = new String[] {"-m", selectRobotModelRadioButtonGroup.getSelection().getActionCommand(), "--realRobot"};

               localSpawner.spawn(AtlasOperatorUserInterface.class, javaArgs, programArgs, new ExitListener()
               {
                  @Override
                  public void exited(int statusValue)
                  {
                     spawnUIButton.setEnabled(true);
                  }
               });
               spawnUIButton.setEnabled(false);
            }
         }
      });
      panel.add(spawnUIButton);

      spawnBehaviorModuleButton = new JButton("[DEBUG]: Spawn Local Behavior Module");
      spawnBehaviorModuleButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            localSpawner.spawn(IHMCHumanoidBehaviorManager.class, new String[]{}, new String[]{}, new ExitListener()
            {
               @Override
               public void exited(int statusValue)
               {
                  spawnBehaviorModuleButton.setEnabled(true);
               }
            });
            spawnBehaviorModuleButton.setEnabled(false);
         }
      });

      panel.add(spawnBehaviorModuleButton);

      c.gridy++;
      mainPanel.add(panel, c);

      if (ENABLE_CONSOLE_OUTPUT)
      {
         frame.setSize(1100, 660);
         frame.setLocationRelativeTo(null);

         JTabbedPane consoleTabs = new JTabbedPane();

         JPanel controllerConsolePanel = new JPanel(new BorderLayout());
         controllerConsolePanel.setBorder(BorderFactory.createEtchedBorder(EtchedBorder.LOWERED));
         controllerConsole.setEditable(false);
         controllerConsolePanel.add(controllerConsole, BorderLayout.CENTER);

         JPanel netProcConsolePanel = new JPanel(new BorderLayout());
         netProcConsolePanel.setBorder(BorderFactory.createEtchedBorder(EtchedBorder.LOWERED));
         netProcConsole.setEditable(false);
         netProcConsolePanel.add(netProcConsole, BorderLayout.CENTER);

         consoleTabs.addTab("Controller", null, controllerConsolePanel, "View Output from Controller process");
         consoleTabs.addTab("Network Processor", null, netProcConsolePanel, "View Output from Network Processor process");

         c.gridy++;
         c.insets = new Insets(5, 10, 10, 10);
         c.ipady = 400;
         c.weighty = 0.1;
         mainPanel.add(consoleTabs, c);
      }

      frame.getContentPane().add(mainPanel);
      frame.setResizable(false);
      frame.setVisible(true);
   }

   private boolean isStringInteger(String str)
   {
      try
      {
         Integer.parseInt(str);
      }
      catch (NumberFormatException e)
      {
         return false;
      }

      // only got here if we didn't return false
      return true;
   }

   private void setupHandIPPanel()
   {
      handIPPanel = new JPanel(new BorderLayout());
      handIPPanel.setBorder(BorderFactory.createEtchedBorder());

      handIPPanel.add(new JLabel("<html><body><h2>Hand Configuration</h2></body></html>", JLabel.CENTER), BorderLayout.NORTH);

      JLabel leftHandLabel = new JLabel("Left Hand Number:");
      JLabel rightHandLabel = new JLabel("Right Hand Number:");

      leftHandField = new JTextField(2);
      leftHandField.setDocument(new PlainDocument()
      {
         @Override
         public void insertString(int offs, String str, AttributeSet a) throws BadLocationException
         {
            if ((getLength() + str.length() <= 2) && isStringInteger(str))
               super.insertString(offs, str, a);
         }
      });
      leftHandField.setText("40");

      rightHandField = new JTextField(2);
      rightHandField.setDocument(new PlainDocument()
      {
         @Override
         public void insertString(int offs, String str, AttributeSet a) throws BadLocationException
         {
            if ((getLength() + str.length() <= 2) && isStringInteger(str))
               super.insertString(offs, str, a);
         }
      });
      rightHandField.setText("35");

      JPanel leftPanel = new JPanel();
      leftPanel.setBorder(new EmptyBorder(40, 0, 0, 0));
      leftPanel.add(leftHandLabel);
      leftPanel.add(leftHandField);
      handIPPanel.add(leftPanel, BorderLayout.CENTER);

      JPanel rightPanel = new JPanel();
      rightPanel.setBorder(new EmptyBorder(0, 0, 80, 0));
      rightPanel.add(rightHandLabel);
      rightPanel.add(rightHandField);
      handIPPanel.add(rightPanel, BorderLayout.SOUTH);
   }

   private void setupNetProcPanel()
   {
      netProcPanel = new JPanel(new GridLayout(6, 1));
      netProcPanel.setBorder(BorderFactory.createEtchedBorder());

      netProcStartButton = new JButton("Start Network Processor");
      netProcStartButton.setEnabled(false);
      netProcStartButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent actionEvent)
         {
            try
            {
               if (verifyEverythingIsConfigured())
                  JOptionPane.showMessageDialog(frame, "Please complete the Controller/Robot Model configuration!", "Bad Deploy Configuration",
                                                JOptionPane.ERROR_MESSAGE);
               else
               {
            	  String modelAsString = selectRobotModelRadioButtonGroup.getSelection().getActionCommand();
            	  int leftHandIP = Integer.parseInt(leftHandField.getText());
                 int rightHandIP = Integer.parseInt(rightHandField.getText());
                  netProcClient.write(new byte[] {UnsignedByteTools.fromInt(0x00), UnsignedByteTools.fromInt(AtlasRobotModelFactory.getOrdinalOfModel(modelAsString)),
                        UnsignedByteTools.fromInt(leftHandIP), UnsignedByteTools.fromInt(rightHandIP)});
               }
            }
            catch (DisconnectedException e)
            {
               netProcClient.reset();
            }
         }
      });

      netProcStopButton = new JButton("Stop Network Processor");
      netProcStopButton.setEnabled(false);
      netProcStopButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent actionEvent)
         {
            try
            {
               netProcClient.write(new byte[] {UnsignedByteTools.fromInt(0x10)});
            }
            catch (DisconnectedException e)
            {
               netProcClient.reset();
            }
         }
      });

      netProcRestartButton = new JButton("Restart Network Processor");
      netProcRestartButton.setEnabled(false);
      netProcRestartButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent actionEvent)
         {
            try
            {
               netProcClient.write(new byte[] {UnsignedByteTools.fromInt(0x11)});
            }
            catch (DisconnectedException e)
            {
               netProcClient.reset();
            }
         }
      });

      netProcPanel.add(new JLabel("<html><body><h2>Network Processor</h2></body></html>", JLabel.CENTER));
      netProcPanel.add(netProcConnectedStatusLabel);
      netProcPanel.add(netProcRunningStatusLabel);
      netProcPanel.add(netProcStartButton);
      netProcPanel.add(netProcStopButton);
      netProcPanel.add(netProcRestartButton);
   }

   private void setupControllerPanel()
   {
      controllerPanel = new JPanel(new GridLayout(6, 1));
      controllerPanel.setBorder(BorderFactory.createEtchedBorder());

      controllerStartButton = new JButton("Start Controller");
      controllerStartButton.setEnabled(false);
      controllerStartButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent actionEvent)
         {
            try
            {
               if (verifyEverythingIsConfigured())
                  JOptionPane.showMessageDialog(frame, "Please complete the Controller/Robot Model configuration!", "Bad Deploy Configuration",
                                                JOptionPane.ERROR_MESSAGE);
               else if (selectControllerRadioButtonGroup.getSelection().getActionCommand().contains(BLUE_TEAM_ACTION_COMMAND))
               {
            	  String modelAsString = selectRobotModelRadioButtonGroup.getSelection().getActionCommand();
                  controllerClient.write(new byte[] {UnsignedByteTools.fromInt(0x00), UnsignedByteTools.fromInt(AtlasRobotModelFactory.getOrdinalOfModel(modelAsString))});
               }
               else if (selectControllerRadioButtonGroup.getSelection().getActionCommand().contains(RED_TEAM_ACTION_COMMAND))
               {
            	   String modelAsString = selectRobotModelRadioButtonGroup.getSelection().getActionCommand();
                  controllerClient.write(new byte[] {UnsignedByteTools.fromInt(0x01), UnsignedByteTools.fromInt(AtlasRobotModelFactory.getOrdinalOfModel(modelAsString))});
               }
            }
            catch (DisconnectedException e)
            {
               controllerClient.reset();
            }
         }
      });

      controllerStopButton = new JButton("Stop Controller");
      controllerStopButton.setEnabled(false);
      controllerStopButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent actionEvent)
         {
            try
            {
               controllerClient.write(new byte[] {UnsignedByteTools.fromInt(0x10)});
            }
            catch (DisconnectedException e)
            {
               controllerClient.reset();
            }
         }
      });

      controllerPanel.add(new JLabel("<html><body><h2>Controller</h2></body></html>", JLabel.CENTER));
      controllerPanel.add(controllerConnectedStatusLabel);
      controllerPanel.add(controllerRunningStatusLabel);
      controllerPanel.add(controllerStartButton);
      controllerPanel.add(controllerStopButton);
   }

   private boolean verifyEverythingIsConfigured()
   {
      return (selectControllerRadioButtonGroup.getSelection() == null) || (selectRobotModelRadioButtonGroup.getSelection() == null)
             || (leftHandField.getText().isEmpty()) || (rightHandField.getText().isEmpty());
   }

   private void setupSelectRobotModelPanel()
   {
      JPanel selectRobotModelPanel = new JPanel(new GridLayout(AtlasRobotModelFactory.getAvailableRobotModels().length + 1, 1));
      robotModelScrollPane = new JScrollPane(selectRobotModelPanel);

      selectRobotModelRadioButtonGroup = new ButtonGroup();

      selectRobotModelPanel.add(new JLabel("<html><body style=\"margin-left: 32px;\"><h2>Select Robot Model</h2></body></html>"));

      for (String st : AtlasRobotModelFactory.getAvailableRobotModels())
      {
         JRadioButton nextButton = new JRadioButton(st);
         nextButton.setActionCommand(st);
         nextButton.setHorizontalAlignment(AbstractButton.LEADING);
         nextButton.setHorizontalTextPosition(AbstractButton.TRAILING);
         selectRobotModelRadioButtonGroup.add(nextButton);
         selectRobotModelPanel.add(nextButton);
      }
   }

   private void setupSelectControllerPanel()
   {
      selectControllerPanel = new JPanel(new GridLayout(4, 1));
      selectControllerPanel.setBorder(BorderFactory.createEtchedBorder());

      selectControllerRadioButtonGroup = new ButtonGroup();

      JRadioButton atlasBDIControllerRadioButton = new JRadioButton("Atlas BDI Controller");
      atlasBDIControllerRadioButton.setActionCommand(RED_TEAM_ACTION_COMMAND);
      atlasBDIControllerRadioButton.setBorder(BorderFactory.createEmptyBorder(0, 35, 0, -35));
      atlasBDIControllerRadioButton.setHorizontalAlignment(AbstractButton.LEADING);
      atlasBDIControllerRadioButton.setHorizontalTextPosition(AbstractButton.TRAILING);

      JRadioButton atlasControllerFactoryRadioButton = new JRadioButton("Atlas Controller Factory");
      atlasControllerFactoryRadioButton.setActionCommand(BLUE_TEAM_ACTION_COMMAND);
      atlasControllerFactoryRadioButton.setBorder(BorderFactory.createEmptyBorder(0, 35, 0, -35));
      atlasControllerFactoryRadioButton.setHorizontalAlignment(AbstractButton.LEADING);
      atlasControllerFactoryRadioButton.setHorizontalTextPosition(AbstractButton.TRAILING);

      selectControllerRadioButtonGroup.add(atlasBDIControllerRadioButton);
      selectControllerRadioButtonGroup.add(atlasControllerFactoryRadioButton);

      selectControllerPanel.add(new JLabel("<html><body><h2>Select Controller</h2></body></html>", JLabel.CENTER));
      selectControllerPanel.add(atlasBDIControllerRadioButton);
      selectControllerPanel.add(atlasControllerFactoryRadioButton);
   }

   private void startReceivingControllerConsoleText()
   {
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            Socket clientSocket = null;
            try
            {
               clientSocket = new Socket(controllerMachineIpAddress, NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT.getPort());
               clientSocket.setTcpNoDelay(true);

               DataInputStream inputStream = new DataInputStream(clientSocket.getInputStream());
               final BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));

               while (true)
               {
                  final String line;
                  if (reader.ready() && (line = reader.readLine()) != null)
                  {
                     SwingUtilities.invokeLater(new Runnable()
                     {
                        @Override
                        public void run()
                        {
                           controllerConsole.append(line);
                        }
                     });
                  }
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();

               try
               {
                  clientSocket.close();
               }
               catch (IOException e1)
               {
                  e1.printStackTrace();
               }
            }
         }
      }).start();
   }

   private void startReceivingNetProcConsoleText()
   {
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            Socket clientSocket = null;
            try
            {
               clientSocket = new Socket(netProcMachineIpAddress, NetworkPorts.NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT.getPort());
               clientSocket.setTcpNoDelay(true);

               DataInputStream inputStream = new DataInputStream(clientSocket.getInputStream());
               final BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));

               while (true)
               {
                  final String line;
                  if (reader.ready() && (line = reader.readLine()) != null)
                  {
                     SwingUtilities.invokeLater(new Runnable()
                     {
                        @Override
                        public void run()
                        {
                           netProcConsole.append(line);
                        }
                     });
                  }
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();

               try
               {
                  clientSocket.close();
               }
               catch (IOException e1)
               {
                  e1.printStackTrace();
               }
            }
         }
      }).start();
   }

   public static void main(String[] args) throws JSAPException
   {
      if (ENABLE_CONSOLE_OUTPUT)
      {
         System.err.println("WARNING: Console Output is being piped over TCP. Do not use this in competition; this will chew through bandwidth.");
      }

      JSAP jsap = new JSAP();

      FlaggedOption netProcIPFlag =
         new FlaggedOption("net-proc-ip").setLongFlag("net-proc-ip").setShortFlag('n').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption controllerIPFlag =
         new FlaggedOption("scs-ip").setLongFlag("scs-ip").setShortFlag('s').setRequired(false).setStringParser(JSAP.STRING_PARSER);

      jsap.registerParameter(netProcIPFlag);
      jsap.registerParameter(controllerIPFlag);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
         if (config.getString(netProcIPFlag.getID()) != null)
         {
            netProcMachineIpAddress = config.getString(netProcIPFlag.getID());
         }

         if (config.getString(controllerIPFlag.getID()) != null)
         {
            controllerMachineIpAddress = config.getString(controllerIPFlag.getID());
         }
      }

      final DRCEnterpriseCloudDispatcherFrontend frontend = new DRCEnterpriseCloudDispatcherFrontend();

      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            frontend.initAndStartSwingGui();
            new Thread(frontend).start();
         }
      });
   }
}
