package us.ihmc.darpaRoboticsChallenge.processManagement;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.fixedPointRepresentation.UnsignedByteTools;
import us.ihmc.utilities.gui.IHMCSwingTools;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.tcpServer.DisconnectedException;
import us.ihmc.utilities.net.tcpServer.ReconnectingTCPClient;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class DRCEnterpriseCloudDispatcherFrontend implements Runnable
{
   private ReconnectingTCPClient netProcClient;
   private final byte[] netProcBuffer;

   private ReconnectingTCPClient controllerClient;
   private final byte[] controllerBuffer;

   private static String netProcMachineIpAddress = DRCConfigParameters.NET_PROC_MACHINE_IP_ADDRESS;
   private static String controllerMachineIpAddress = DRCConfigParameters.SCS_MACHINE_IP_ADDRESS;

   private JFrame frame;
   private JPanel netProcPanel, controllerPanel;
   
   private JButton netProcStartButton, netProcStopButton, netProcRestartButton;
   private final JLabel netProcRunningStatusLabel =
      new JLabel("<html><body>Running: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>", JLabel.CENTER);
   private final JLabel netProcConnectedStatusLabel =
      new JLabel("<html><body>Connected: <span style=\"color:red;font-style:italic;\">Disconnected</span></body></html>", JLabel.CENTER);

   private JButton controllerStartButton, controllerStopButton, controllerRestartButton;
   private final JLabel controllerRunningStatusLabel =
         new JLabel("<html><body>Running: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>", JLabel.CENTER);
   private final JLabel controllerConnectedStatusLabel =
         new JLabel("<html><body>Connected: <span style=\"color:red;font-style:italic;\">Disconnected</span></body></html>", JLabel.CENTER);

   public DRCEnterpriseCloudDispatcherFrontend()
   {
      setupNetProcSocket();

      setupControllerSocket();

      netProcBuffer = netProcClient.getBuffer();
      controllerBuffer = controllerClient.getBuffer();
   }

   private void setupNetProcSocket()
   {
      netProcClient = new ReconnectingTCPClient(netProcMachineIpAddress, DRCConfigParameters.NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT);
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
      controllerClient = new ReconnectingTCPClient(controllerMachineIpAddress, DRCConfigParameters.CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT);
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
      netProcClient.connect();
      controllerClient.connect();

      netProcNotRunningButtonConfiguration();

      controllerNotRunningButtonConfiguration();

      while (true)
      {
         processNetProcSocket();

         processControllerSocket();
      }
   }

   private void processNetProcSocket()
   {
      try
      {
         netProcClient.read(1);

         switch (netProcBuffer[0])
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

         switch (controllerBuffer[0])
         {
            case 0x00 :
               controllerRunningButtonConfiguration();
               controllerRunningStatusLabel.setText("<html><body>Controller Status: <span style=\"color:green;font-style:italic;"
                     + "\">Running</span></body></html>");
               repaintFrame();

               break;

            case 0x10 :
               controllerDisableAll();
               controllerRunningStatusLabel.setText("<html><body>Controller Status: <span style=\"color:yellow;font-style:italic;"
                     + "\">Restarting</span></body></html>");
               repaintFrame();
               break;

            case 0x11 :
               controllerNotRunningButtonConfiguration();
               controllerRunningStatusLabel.setText(
                     "<html><body>Controller Status: <span style=\"color:red;font-style:italic;\">Not Running</span></body></html>");
               repaintFrame();

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
      controllerRestartButton.setEnabled(false);
   }

   private void controllerNotRunningButtonConfiguration()
   {
      controllerStartButton.setEnabled(true);
      controllerStopButton.setEnabled(false);
      controllerRestartButton.setEnabled(false);
   }

   private void controllerRunningButtonConfiguration()
   {
      controllerStartButton.setEnabled(false);
      controllerStopButton.setEnabled(true);
      controllerRestartButton.setEnabled(true);
   }

   private void initAndStartSwingGui()
   {
      IHMCSwingTools.setNativeLookAndFeel();

      frame = new JFrame("Network Processor Dispatcher");
      frame.setSize(650, 230);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setResizable(false);
      frame.setAlwaysOnTop(true);
      frame.setLocationRelativeTo(null);

      JPanel panel = new JPanel(new GridLayout(1,2));
      ((GridLayout) panel.getLayout()).setHgap(5);
      panel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

      setupNetProcPanel();

      setupControllerPanel();

      panel.add(netProcPanel);
      panel.add(controllerPanel);

      frame.getContentPane().add(panel);
      frame.setVisible(true);
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
               netProcClient.write(new byte[]{UnsignedByteTools.fromInt(0x00)});
            } catch (DisconnectedException e)
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
               netProcClient.write(new byte[]{UnsignedByteTools.fromInt(0x10)});
            } catch (DisconnectedException e)
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
               netProcClient.write(new byte[]{UnsignedByteTools.fromInt(0x11)});
            } catch (DisconnectedException e)
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
               controllerClient.write(new byte[]{UnsignedByteTools.fromInt(0x00)});
            } catch (DisconnectedException e)
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
               controllerClient.write(new byte[]{UnsignedByteTools.fromInt(0x10)});
            } catch (DisconnectedException e)
            {
               controllerClient.reset();
            }
         }
      });

      controllerRestartButton = new JButton("Restart Controller");
      controllerRestartButton.setEnabled(false);
      controllerRestartButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent actionEvent)
         {
            try
            {
               controllerClient.write(new byte[]{UnsignedByteTools.fromInt(0x11)});
            } catch (DisconnectedException e)
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
      controllerPanel.add(controllerRestartButton);
   }

   public static void main(String[] args) throws JSAPException
   {
      JSAP jsap = new JSAP();

      FlaggedOption netProcIPFlag =
         new FlaggedOption("net-proc-ip").setLongFlag("net-proc-ip").setShortFlag('n').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption controllerIPFlag = new FlaggedOption("scs-ip").setLongFlag("scs-ip").setShortFlag('s').setRequired(false).setStringParser(JSAP.STRING_PARSER);

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
