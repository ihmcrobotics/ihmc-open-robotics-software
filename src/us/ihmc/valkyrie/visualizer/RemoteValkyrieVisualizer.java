package us.ihmc.valkyrie.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieNetworkParameters;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class RemoteValkyrieVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;
   public static final String defaultHost = ValkyrieNetworkParameters.CONTROL_COMPUTER_HOST;
   public static final int defaultPort = Integer.MAX_VALUE;
   
   private ValkyrieSliderBoardType valkyrieSliderBoardType;
   
   private SCSVisualizer scsVisualizer;
   private ValkyrieRobotModel valkyrieRobotModel;
   private String host;
   private int port;

   public RemoteValkyrieVisualizer(String[] networkArguments, ValkyrieSliderBoardType valkyrieSliderBoardType)
   {
      this.valkyrieSliderBoardType = valkyrieSliderBoardType;
      
      parseNetworkArguments(networkArguments);
      
      System.out.println("Connecting to host " + host);
      valkyrieRobotModel = new ValkyrieRobotModel(true, false);

      scsVisualizer = new SCSVisualizer(valkyrieRobotModel.createSdfRobot(false), BUFFER_SIZE);
      scsVisualizer.addSCSVisualizerStateListener(this);

      YoVariableClient client = new YoVariableClient(host, scsVisualizer, "remote", false);
      client.start();
   }
   
   public void parseNetworkArguments(String[] networkArguments)
   {
      JSAP jsap = new JSAP();

      FlaggedOption hostOption = new FlaggedOption("host").setStringParser(JSAP.STRING_PARSER).setRequired(false).setLongFlag("host").setShortFlag('L')
            .setDefault(defaultHost);
      FlaggedOption portOption = new FlaggedOption("port").setStringParser(JSAP.INTEGER_PARSER).setRequired(false).setLongFlag("port").setShortFlag('p')
            .setDefault(String.valueOf(defaultPort));

      try
      {
         jsap.registerParameter(hostOption);
         jsap.registerParameter(portOption);
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }

      JSAPResult config = jsap.parse(networkArguments);

      if (!config.success())
      {
         System.err.println();
         System.err.println("Usage: java " + RemoteValkyrieVisualizer.class.getName());
         System.err.println("                " + jsap.getUsage());
         System.err.println();
         System.exit(1);
      }
      
      host = config.getString("host");
      port = config.getInt("port");
   }

   @Override
   public void starting(final SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      RobonetRegisterPanel registerPanel = new RobonetRegisterPanel(registry);
      scs.addExtraJpanel(registerPanel, "Registers");
      scs.attachPlaybackListener(registerPanel);

      RobonetRegisterModifierPanel modifierPanel = new RobonetRegisterModifierPanel(registry);
      scs.addExtraJpanel(modifierPanel, "Change Control Modes");
      scs.attachPlaybackListener(modifierPanel);

      JButton showRegisterViewer = new JButton("Show registers");
      showRegisterViewer.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            scs.getStandardSimulationGUI().selectPanel("Registers");
         }
      });

      JButton showControlModePanel = new JButton("Change Control Modes");
      showControlModePanel.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            scs.getStandardSimulationGUI().selectPanel("Change Control Modes");
         }
      });

      scs.addButton(showRegisterViewer);
      scs.addButton(showControlModePanel);
      
      new ValkyrieSliderBoard(scs, registry, valkyrieRobotModel, valkyrieSliderBoardType);
   }
   
   public static void main(String[] args)
   {
      new RemoteValkyrieWalkingVisualizer(args);
   }
}
