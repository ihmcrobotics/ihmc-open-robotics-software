package us.ihmc.valkyrie.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class RemoteValkyrieVisualizer implements SCSVisualizerStateListener
{
   public static final int BUFFER_SIZE = 16384;
   
   private ValkyrieSliderBoardType valkyrieSliderBoardType;
   
   private final SCSVisualizer scsVisualizer;
   private final ValkyrieRobotModel valkyrieRobotModel;
   private String host;

   public RemoteValkyrieVisualizer(ValkyrieSliderBoardType valkyrieSliderBoardType)
   {
      this.valkyrieSliderBoardType = valkyrieSliderBoardType;
            
      host = NetworkParameters.getHost(NetworkParameterKeys.logger);
      
      System.out.println("Connecting to host " + host);
      valkyrieRobotModel = new ValkyrieRobotModel(true, false);

      scsVisualizer = new SCSVisualizer(BUFFER_SIZE);
      scsVisualizer.addSCSVisualizerStateListener(this);

      YoVariableClient client = new YoVariableClient(scsVisualizer, "remote", false);
      client.start();
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
      new RemoteValkyrieWalkingVisualizer();
   }
}
