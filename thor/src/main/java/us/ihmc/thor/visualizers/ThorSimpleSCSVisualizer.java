package us.ihmc.thor.visualizers;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToggleButton;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataVisualizer.visualizer.JointUpdater;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class ThorSimpleSCSVisualizer extends SCSVisualizer
{

   private static final int DISPLAY_ONE_IN_N_PACKETS = 6;

   protected YoVariableRegistry registry;
   protected SimulationConstructionSet scs;

   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<>();
   private volatile boolean recording = true;
   private YoVariableClient yoVariableClient;
   private ArrayList<SCSVisualizerStateListener> stateListeners = new ArrayList<>();

   private int displayOneInNPackets = DISPLAY_ONE_IN_N_PACKETS;

   private JPanel buttonPanel = null;
   private GridBagConstraints cons;


   public ThorSimpleSCSVisualizer(int bufferSize)
   {
      this(bufferSize, true);
   }

   public ThorSimpleSCSVisualizer(int bufferSize, boolean showGUI)
   {
      this(bufferSize, showGUI, false);
   }

   public ThorSimpleSCSVisualizer(int bufferSize, boolean showGUI, boolean hideViewport)
   {
      super(bufferSize, showGUI);
      addSCSVisualizerStateListener(this);
      createButtonPanel();
   }

   @Override public void disconnected()
   {
      System.out.println("DISCONNECTED. SLIDERS NOW ENABLED");
      scs.setScrollGraphsEnabled(true);
   }

   @Override public void setYoVariableClient(final YoVariableClient client)
   {
      this.yoVariableClient = client;
   }

   private void disconnect(final JButton disconnectButton)
   {
      disconnectButton.setEnabled(false);
      yoVariableClient.requestStop();
   }

   private void createButtonPanel()
   {
      buttonPanel = new JPanel();
      buttonPanel.setLayout(new GridBagLayout());
      cons = new GridBagConstraints();
      cons.fill = GridBagConstraints.HORIZONTAL;
      cons.weightx = 1;
      cons.gridx = 0;
   }


   @Override public boolean changesVariables()
   {
      return true;
   }

   @Override public boolean updateYoVariables()
   {
      return true;
   }

   public void closeAndDispose()
   {
      scs.closeAndDispose();
   }

   public DataBuffer getDataBuffer()
   {
      return scs.getDataBuffer();
   }

   @Override public void exitActionPerformed()
   {
      recording = false;
      if (yoVariableClient != null)
      {
         yoVariableClient.requestStop();
      }
   }

   @Override public int getDisplayOneInNPackets()
   {
      return displayOneInNPackets;
   }


   public void setDisplayOneInNPackets(int val)
   {
      displayOneInNPackets = val;
   }

   public void updateGraphsLessFrequently(boolean enable, int numberOfTicksBeforeUpdatingGraphs)
   {
      scs.setFastSimulate(enable, numberOfTicksBeforeUpdatingGraphs);
   }

   public void addSCSVisualizerStateListener(SCSVisualizerStateListener stateListener)
   {
      stateListeners.add(stateListener);
   }

   @Override public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
   }

   public static void main(String[] args)
   {
      SCSVisualizer visualizer = new ThorSimpleSCSVisualizer(32169, true);
      visualizer.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(visualizer);
      client.start();
   }

   @Override public void clearLog(String guid)
   {
   }

   @Override public synchronized boolean executeVariableChangedListeners()
   {
      return recording;
   }
}
