package us.ihmc.robotDataVisualizer.visualizer;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JTextField;

import org.apache.commons.lang3.StringUtils;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.robotics.robotDescription.modelLoaders.LogModelLoader;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.AdditionalPanelTools;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Main entry point for the visualizer. To make a custom visualizer for your robot, do NOT copy,
 * instead extend.
 *
 * @author jesper
 */
public class SCSVisualizer implements YoVariablesUpdatedListener, ExitActionListener, SCSVisualizerStateListener
{
   private final static String DISCONNECT_DISCONNECT = "Disconnect";
   private final static String DISCONNECT_RECONNECT = "Reconnect";

   private YoRegistry registry;
   private SimulationConstructionSet scs;

   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<>();
   private YoVariableClientInterface yoVariableClientInterface;
   private ArrayList<SCSVisualizerStateListener> stateListeners = new ArrayList<>();

   private int variableUpdateRate = 0;

   private final TObjectDoubleHashMap<String> buttons = new TObjectDoubleHashMap<>();

   private final Object disconnectLock = new Object();
   private final JButton disconnectButton = new JButton("Waiting for connection...");
   private final JButton clearLogButton = new JButton("<html><center>Clear<br>log</center></html>");
   private final JTextField updateRateField = new JTextField("", 6);

   private final DecimalFormat delayFormat = new DecimalFormat("0000");
   private final JLabel delayValue = new JLabel();

   private volatile long lastTimestamp;

   private int bufferSize;
   private boolean showGUI;
   private boolean hideViewport;
   private boolean showOverheadView;

   private YoGraphicsListRegistry yoGraphicsListRegistry;

   private final LoggerStatusVisualizer loggerStatusVisualizer = new LoggerStatusVisualizer();

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
      this.disconnectButton.setEnabled(false);
      addSCSVisualizerStateListener(this);
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
      long delay = Conversions.nanosecondsToMilliseconds(lastTimestamp - timestamp);
      delayValue.setText("<html>Delay:<br>" + delayFormat.format(delay) + " ms</html>");

      for (int i = 0; i < jointUpdaters.size(); i++)
      {
         jointUpdaters.get(i).update();
      }
      scs.setTime(Conversions.nanosecondsToSeconds(timestamp));
      updateLocalVariables();
      scs.tickAndUpdate();
   }

   @Override
   public void disconnected()
   {
      synchronized (disconnectLock)
      {
         LogTools.info("Disconnected. Sliders now enabled.");
         disconnectButton.setText(DISCONNECT_RECONNECT);
         disconnectButton.setEnabled(true);
         scs.setScrollGraphsEnabled(true);
      }
   }

   @Override
   public void connected()
   {
      synchronized (disconnectLock)
      {
         setVariableUpdateRate(variableUpdateRate);

         scs.setInPoint();
         disconnectButton.setText(DISCONNECT_DISCONNECT);
         disconnectButton.setEnabled(true);
      }
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
   public boolean updateYoVariables()
   {
      return true;
   }

   public void closeAndDispose()
   {
      scs.closeAndDispose();
   }

   public YoBuffer getDataBuffer()
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
      if (yoVariableClientInterface != null)
      {
         yoVariableClientInterface.stop();
      }
   }

   /**
    * On connect, the variable update rate is send to the server. The server will limit the amount of
    * data to approximately the desired rate, depending on the update rate of the underlying threads.
    * Set to zero to update variables as fast as the server produces them. Note: If the server does not
    * send data at a constant period or does not set the timestamp, setting the variable update rate
    * will result in no data being received.
    *
    * @param updateRateInMilliseconds Update rate in milliseconds. Set to zero to disable rate limiting
    */
   public void setVariableUpdateRate(int updateRateInMilliseconds)
   {
      synchronized (disconnectLock)
      {
         if (yoVariableClientInterface != null)
         {
            if (yoVariableClientInterface.isConnected())
            {
               yoVariableClientInterface.setVariableUpdateRate(updateRateInMilliseconds);
            }
         }

         variableUpdateRate = updateRateInMilliseconds;

         updateRateField.setText(String.valueOf(updateRateInMilliseconds));
      }

   }

   private void setVariableUpdateRate(String text)
   {
      int newUpdateRate;
      try
      {
         newUpdateRate = MathTools.clamp(Integer.parseInt(text.trim()), 0, 99999);
      }
      catch (NumberFormatException e)
      {
         newUpdateRate = 0;
      }
      setVariableUpdateRate(newUpdateRate);
   }

   @Deprecated
   /**
    * This functionality has been replaced with setVariableUpdateRate() For backwards compatibility,
    * this function will print a big fat warning and set the variable update rate to
    * displayOneInNPackets * 1 ms
    *
    * @param displayOneInNPackets
    */
   public void setDisplayOneInNPackets(int displayOneInNPackets)
   {
      LogTools.warn("setDisplayOneInNPackets is DEPRECATED. Setting the variable update rate to " + displayOneInNPackets + " ms instead");
      setVariableUpdateRate(displayOneInNPackets);
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
   public final void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake, YoVariableHandshakeParser handshakeParser,
                           DebugRegistry debugRegistry)
   {
      this.yoVariableClientInterface = yoVariableClientInterface;

      Robot robot = new Robot("DummyRobot");
      if (handshake.getModelLoaderClass() != null)
      {
         LogModelLoader modelLoader;
         try
         {
            modelLoader = (LogModelLoader) Class.forName(handshake.getModelLoaderClass()).newInstance();
         }
         catch (Exception e)
         {
            LogTools.error("Could not instantiate LogModelLoader: {}. Defaulting to SDFModelLoader.", handshake.getModelLoaderClass());
            modelLoader = new SDFModelLoader();
         }
         modelLoader.load(handshake.getModelName(), handshake.getModel(), handshake.getResourceDirectories(), handshake.getResourceZip());
         robot = new RobotFromDescription(modelLoader.createRobot());
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(this.bufferSize);

      this.scs = new SimulationConstructionSet(robot, parameters);
      if (hideViewport)
      {
         scs.hideViewport();
      }
      this.registry = scs.getRootRegistry();
      scs.setScrollGraphsEnabled(false);
      scs.setGroundVisible(false);
      scs.attachExitActionListener(this);
      scs.attachPlaybackListener(createYoGraphicsUpdater());
      scs.setRunName(yoVariableClientInterface.getServerName());

      scs.addButton(disconnectButton);
      disconnectButton.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            synchronized (disconnectLock)
            {
               if (disconnectButton.getText().equals(DISCONNECT_DISCONNECT))
               {
                  disconnectButton.setEnabled(false);
                  yoVariableClientInterface.disconnect();
               }
               else
               {
                  try
                  {
                     LogTools.info("Reconnecting. Disabling sliders.");
                     scs.gotoOutPointNow();
                     scs.setScrollGraphsEnabled(false);
                     scs.tickAndReadFromBuffer(0);

                     if (yoVariableClientInterface.reconnect())
                     {
                        disconnectButton.setEnabled(false);
                     }
                     else
                     {
                        JOptionPane.showMessageDialog(scs.getJFrame(),
                                                      "Cannot reconnect. No matching sessions found.",
                                                      "Cannot reconnect",
                                                      JOptionPane.ERROR_MESSAGE);
                        LogTools.warn("Reconnect failed. Enabling sliders.");
                        scs.setScrollGraphsEnabled(true);

                     }
                  }
                  catch (IOException reconnectError)
                  {
                     reconnectError.printStackTrace();
                  }
               }
            }
         }

      });

      scs.addJLabel(new JLabel(" Rate: "));
      scs.addTextField(updateRateField);
      scs.addJLabel(new JLabel("ms "));

      updateRateField.addActionListener((e) -> setVariableUpdateRate(updateRateField.getText()));

      scs.addButton(clearLogButton);
      clearLogButton.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (yoVariableClientInterface != null)
            {
               yoVariableClientInterface.sendClearLogRequest();
            }
         }
      });

      scs.addJLabel(delayValue);

      loggerStatusVisualizer.addToSimulationConstructionSet(scs);

      YoRegistry yoVariableRegistry = handshakeParser.getRootRegistry();
      this.registry.addChild(yoVariableRegistry);
      this.registry.addChild(debugRegistry.getYoRegistry());
      scs.setParameterRootPath(yoVariableRegistry);

      List<JointState> jointStates = handshakeParser.getJointStates();
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);

      yoGraphicsListRegistry = handshakeParser.getYoGraphicsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      for (int i = 0; i < stateListeners.size(); i++)
      {
         SCSVisualizerStateListener stateListener = stateListeners.get(i);
         stateListener.starting(scs, robot, this.registry);
      }

      for (String yoVariableName : buttons.keySet())
      {
         final YoVariable var = registry.findVariable(yoVariableName);
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

      AdditionalPanelTools.setupFrameView(scs, handshakeParser.getFrameIndexMap()::getReferenceFrame, createFrameFilter());

      new Thread(scs, "SCSVisualizer").start();
   }

   public static Predicate<YoVariable> createFrameFilter()
   {
      return v -> v instanceof YoLong && StringUtils.containsIgnoreCase(v.getName(), "frame");
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoRegistry registry)
   {
      scs.setFastSimulate(true);
   }

   /**
    * Overridable method to update local variables. Needs to return in less than your visualization dt.
    */
   public void updateLocalVariables()
   {

   }

   public static void main(String[] args)
   {
      SCSVisualizer visualizer = new SCSVisualizer(32169, true);
      visualizer.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(visualizer);
      client.startWithHostSelector();
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
      lastTimestamp = timestamp;
   }

   private PlaybackListener createYoGraphicsUpdater()
   {
      return new PlaybackListener()
      {
         @Override
         public void indexChanged(int newIndex)
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
         List<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();
         for (YoGraphic yoGraphic : yoGraphics)
            yoGraphic.update();
      }
      yoGraphicsListRegistry.update();
   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {
      loggerStatusVisualizer.updateStatus(command, argument);
   }
}
