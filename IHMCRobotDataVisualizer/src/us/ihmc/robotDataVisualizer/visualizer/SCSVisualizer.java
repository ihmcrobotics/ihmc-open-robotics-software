package us.ihmc.robotDataVisualizer.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JToggleButton;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

public class SCSVisualizer implements YoVariablesUpdatedListener, ExitActionListener, SCSVisualizerStateListener
{
   protected YoVariableRegistry registry;
   protected SimulationConstructionSet scs;

   
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<>();
   private volatile boolean recording = true;
   private YoVariableClient yoVariableClient;
   private ArrayList<SCSVisualizerStateListener> stateListeners = new ArrayList<>();

   private int displayOneInNPackets = 1;

   private final TObjectDoubleHashMap<String> buttons = new TObjectDoubleHashMap<>();

   private final JButton disconnectButton = new JButton("Disconnect");
   private final JButton clearLogButton = new JButton("Clear log");

   private final DecimalFormat delayFormat = new DecimalFormat("0000");
   private final JLabel delayValue = new JLabel();


   private volatile long lastTimestamp;

   private int bufferSize;
   private boolean showGUI;
   private boolean hideViewport;
   private boolean showOverheadView;

   private YoGraphicsListRegistry yoGraphicsListRegistry;

   public SCSVisualizer(int bufferSize)
   {
      this(bufferSize, true);
   }

   public SCSVisualizer(int bufferSize, boolean showGUI)
   {
      this(bufferSize, showGUI, false);
   }

   public SCSVisualizer(int bufferSize, boolean showGUI, boolean hideViewport)
   {
      this.bufferSize = bufferSize;
      this.showGUI = showGUI;
      this.hideViewport = hideViewport;
      addSCSVisualizerStateListener(this);
   }

   @Override
   public void receivedTimestampAndData(long timestamp, ByteBuffer buffer)
   {
      long delay = Conversions.nanosecondsToMilliseconds(lastTimestamp - timestamp);
      delayValue.setText(delayFormat.format(delay));

      if (recording)
      {
         for (int i = 0; i < jointUpdaters.size(); i++)
         {
            jointUpdaters.get(i).update();
         }
         scs.setTime(Conversions.nanosecondsToSeconds(timestamp));
         scs.tickAndUpdate();
      }
   }

   @Override
   public void disconnected()
   {
      System.out.println("DISCONNECTED. SLIDERS NOW ENABLED");
      scs.setScrollGraphsEnabled(true);
   }

   @Override
   public void setYoVariableClient(final YoVariableClient client)
   {

      this.yoVariableClient = client;
   }

   private void disconnect(final JButton disconnectButton)
   {
      disconnectButton.setEnabled(false);
      yoVariableClient.requestStop();
   }

   public void addButton(String yoVariableName, double newValue)
   {
      buttons.put(yoVariableName, newValue);
   }

   @Override
   public boolean changesVariables()
   {
      return true;
   }

   @Override
   public void receiveTimedOut()
   {
      System.out.println("Connection lost, closing client.");
      yoVariableClient.disconnected();
   }

   @Override
   public boolean updateYoVariables()
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

   public void addButtonToSimulationConstructionSetGUI(AbstractButton button)
   {
      scs.addButton(button);
   }

   @Override
   public void exitActionPerformed()
   {
      recording = false;
      if (yoVariableClient != null)
      {
         yoVariableClient.requestStop();
      }
   }

   @Override
   public int getDisplayOneInNPackets()
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

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {
      this.showOverheadView = showOverheadView;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   @Override
   public final void start(LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      Robot robot = new Robot("DummyRobot");
      if (handshake.getModelLoaderClass() != null)
      {
         SDFModelLoader modelLoader = new SDFModelLoader();
         modelLoader.load(handshake.getModelName(), handshake.getModel(), handshake.getResourceDirectories(), handshake.getResourceZip(), null);
         robot = new FloatingRootJointRobot(modelLoader.createRobot());
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(this.bufferSize);

      this.scs = new SimulationConstructionSet(robot, parameters);
      if (hideViewport) { scs.hideViewport(); }
      this.registry = scs.getRootRegistry();
      scs.setScrollGraphsEnabled(false);
      scs.setGroundVisible(false);
      scs.attachExitActionListener(this);
      scs.attachPlaybackListener(createYoGraphicsUpdater());
      scs.setRunName(yoVariableClient.getServerName());
      //scs.setFastSimulate(true, 50);

      scs.addButton(disconnectButton);
      disconnectButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            disconnect(disconnectButton);
         }

      });
      scs.addButton(clearLogButton);
      clearLogButton.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (yoVariableClient != null)
            {
               yoVariableClient.sendClearLogRequest();
            }
         }
      });

      scs.addJLabel(new JLabel("Delay: "));
      scs.addJLabel(delayValue);
      scs.addJLabel(new JLabel("ms"));

      YoVariableRegistry yoVariableRegistry = handshakeParser.getRootRegistry();
      this.registry.addChild(yoVariableRegistry);

      List<JointState> jointStates = handshakeParser.getJointStates();
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);

      yoGraphicsListRegistry = handshakeParser.getYoGraphicsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);

      for (int i = 0; i < stateListeners.size(); i++)
      {
         SCSVisualizerStateListener stateListener = stateListeners.get(i);
         stateListener.starting(scs, robot, this.registry);
      }

      final JToggleButton record = new JToggleButton("Pause recording");
      scs.addButton(record);
      record.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            if(record.isSelected())
            {
               synchronized (this)
               {
                  yoVariableClient.setSendingVariableChanges(false);
                  recording = false;
                  record.setText("Resume recording");
                  scs.setScrollGraphsEnabled(true);
               }
            }
            else
            {
               synchronized (this)
               {
                  scs.gotoOutPointNow();
                  recording = true;
                  record.setText("Pause recording");
                  scs.setScrollGraphsEnabled(false);
                  yoVariableClient.setSendingVariableChanges(true);
               }
            }
         }
      });

      for (String yoVariableName : buttons.keySet())
      {
         final YoVariable<?> var = registry.getVariable(yoVariableName);
         final JButton button = new JButton(yoVariableName);
         final double newValue = buttons.get(yoVariableName);
         scs.addButton(button);
         button.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               var.setValueFromDouble(newValue);
            }
         });
      }

      new Thread(scs).start();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
   }

   public static void main(String[] args)
   {
      SCSVisualizer visualizer = new SCSVisualizer(32169, true);
      visualizer.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(visualizer, "remote");
      client.start();
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
      lastTimestamp = timestamp;
   }

   @Override
   public void clearLog(String guid)
   {
   }

   @Override
   public synchronized boolean executeVariableChangedListeners()
   {
      return recording;
   }

   private PlaybackListener createYoGraphicsUpdater()
   {
      return new PlaybackListener()
      {
         @Override
         public void indexChanged(int newIndex, double newTime)
         {
            updateYoGraphics();
         }

         @Override
         public void stop()
         {
         }

         @Override
         public void play(double realTimeRate)
         {
         }
      };
   }

   private void updateYoGraphics()
   {
      if (yoGraphicsListRegistry == null)
         return;

      List<YoGraphicsList> yoGraphicsLists = yoGraphicsListRegistry.getYoGraphicsLists();
      for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
      {
         ArrayList<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();
         for (YoGraphic yoGraphic : yoGraphics)
            yoGraphic.update();
      }
      yoGraphicsListRegistry.update();
   }
}
