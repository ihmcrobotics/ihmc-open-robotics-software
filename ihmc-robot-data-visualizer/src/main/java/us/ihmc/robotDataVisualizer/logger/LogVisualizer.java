package us.ihmc.robotDataVisualizer.logger;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataVisualizer.logger.converters.LogFormatUpdater;
import us.ihmc.robotDataVisualizer.logger.util.FileSelectionDialog;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.AdditionalPanelTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class LogVisualizer
{
   private static final int DEFAULT_BUFFER_SIZE = 8000;

   private static final boolean PRINT_OUT_YOVARIABLE_NAMES = false;

   private final SimulationConstructionSet scs;
   private YoVariableLogPlaybackRobot robot;
   private SimulationOverheadPlotter plotter;

   public LogVisualizer() throws IOException
   {
      this(DEFAULT_BUFFER_SIZE);
   }

   public LogVisualizer(int bufferSize) throws IOException
   {
      this(bufferSize, false, null);
   }

   public LogVisualizer(int bufferSize, boolean showOverheadView, File logFile) throws IOException
   {
      if (logFile == null)
      {
         logFile = FileSelectionDialog.loadDirectoryWithFileNamed(YoVariableLoggerListener.propertyFile);
      }

      if (logFile != null)
      {
         System.out.println("loading log from folder:" + logFile);

         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setCreateGUI(true);
         parameters.setDataBufferSize(bufferSize);
         scs = new SimulationConstructionSet(parameters);

         scs.setFastSimulate(true, 50);
         readLogFile(logFile, showOverheadView);

         if (PRINT_OUT_YOVARIABLE_NAMES)
            printOutYoVariableNames();
      }
      else
      {
         scs = null;
      }

   }

   private void printOutYoVariableNames()
   {
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      ArrayList<YoVariable<?>> allVariablesIncludingDescendants = rootRegistry.getAllVariablesIncludingDescendants();

      for (YoVariable<?> yoVariable : allVariablesIncludingDescendants)
      {
         System.out.println(yoVariable.getName());
      }
   }

   private void readLogFile(File selectedFile, boolean showOverheadView) throws IOException
   {
      LogPropertiesReader logProperties = new LogPropertiesReader(new File(selectedFile, YoVariableLoggerListener.propertyFile));
      LogFormatUpdater.updateLogs(selectedFile, logProperties);

      File handshake = new File(selectedFile, logProperties.getVariables().getHandshakeAsString());
      if (!handshake.exists() || handshake.isDirectory())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getHandshakeAsString());
      }

      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();

      YoVariableHandshakeParser parser = YoVariableHandshakeParser.create(logProperties.getVariables().getHandshakeFileType());
      parser.parseFrom(handshakeData);

      System.out.println("This log contains " + parser.getNumberOfVariables() + " YoVariables");

      GeneralizedSDFRobotModel generalizedSDFRobotModel = null;
      List<JointState> jointStates = parser.getJointStates();

      if (!logProperties.getModel().getLoaderAsString().isEmpty())
      {
         SDFModelLoader loader = new SDFModelLoader();
         String modelName = logProperties.getModel().getNameAsString();
         String[] resourceDirectories = logProperties.getModel().getResourceDirectoriesList().toStringArray();

         File model = new File(selectedFile, logProperties.getModel().getPathAsString());
         DataInputStream modelStream = new DataInputStream(new FileInputStream(model));
         byte[] modelData = new byte[(int) model.length()];
         modelStream.readFully(modelData);
         modelStream.close();

         File resourceBundle = new File(selectedFile, logProperties.getModel().getResourceBundleAsString());
         DataInputStream resourceStream = new DataInputStream(new FileInputStream(resourceBundle));
         byte[] resourceData = new byte[(int) resourceBundle.length()];
         resourceStream.readFully(resourceData);
         resourceStream.close();

         loader.load(modelName, modelData, resourceDirectories, resourceData, null);

         try
         {
            generalizedSDFRobotModel = loader.createJaxbSDFLoader().getGeneralizedSDFRobotModel(modelName);
         }
         catch (Exception e)
         {
            LogTools.warn("Robot model not available.");
         }
      }
      else if (jointStates.size() != 0)
      {
         throw new RuntimeException("No model available for log, but jointstates are defined.");
      }

      boolean useCollisionMeshes = false;

      RobotDescription robotDescription;

      if (generalizedSDFRobotModel != null)
      {
         RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
         robotDescription = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, null, null, useCollisionMeshes);
      }
      else
      {
         robotDescription = new RobotDescription("NullRobot");
         JointDescription rootJoint = new FloatingJointDescription("RootJoint");
         rootJoint.setLink(new LinkDescription("RootLink"));
         robotDescription.addRootJoint(rootJoint);
      }

      robot = new YoVariableLogPlaybackRobot(selectedFile, robotDescription, jointStates, parser.getYoVariablesList(), logProperties, scs);
      scs.setTimeVariableName(robot.getRobotsYoVariableRegistry().getName() + ".robotTime");

      double dt = parser.getDt();
      System.out.println(getClass().getSimpleName() + ": dt set to " + dt);
      scs.setDT(dt, 1);
      scs.setPlaybackDesiredFrameRate(0.04);

      YoGraphicsListRegistry yoGraphicsListRegistry = parser.getYoGraphicsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      scs.attachPlaybackListener(createYoGraphicsUpdater(yoGraphicsListRegistry));
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.getRootRegistry().addChild(parser.getRootRegistry());
      scs.setParameterRootPath(parser.getRootRegistry());
      scs.setGroundVisible(false);

      MultiVideoDataPlayer players = null;
      try
      {
         players = new MultiVideoDataPlayer(selectedFile, logProperties, robot.getTimestamp());

         scs.attachPlaybackListener(players);
         scs.attachSimulationRewoundListener(players);

      }
      catch (Exception e)
      {
         System.err.println("Couldn't load video file!");
         e.printStackTrace();
      }

      scs.getJFrame().setTitle(this.getClass().getSimpleName() + " - " + selectedFile);
      YoVariableLogVisualizerGUI gui = new YoVariableLogVisualizerGUI(selectedFile, logProperties, players, parser, robot, scs);
      scs.getStandardSimulationGUI().addJComponentToMainPanel(gui, BorderLayout.SOUTH);

      AdditionalPanelTools.setupFrameView(scs, parser.getFrameIndexMap()::getReferenceFrame, SCSVisualizer.createFrameFilter());

      //      ErrorPanel errorPanel = new ErrorPanel(scs.getRootRegistry());
      //      scs.getStandardSimulationGUI().addJComponentToMainPanel(errorPanel,  BorderLayout.EAST);

      setupReadEveryNTicksTextField();
   }

   private void setupReadEveryNTicksTextField()
   {
      JLabel label = new JLabel("ReadEveryNTicks");
      scs.addJLabel(label);

      String everyNTicksString = Integer.toString(robot.getReadEveryNTicks());
      final JTextField textField = new JTextField(everyNTicksString, 3);
      textField.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            try
            {
               int readEveryNTicks = Integer.parseInt(textField.getText());
               robot.setReadEveryNTicks(readEveryNTicks);
            }
            catch (NumberFormatException exception)
            {
            }

            setReadEveryNTicksTextFieldToCurrentValue(textField);
            textField.getParent().requestFocus();
         }
      });

      textField.addFocusListener(new FocusListener()
      {
         @Override
         public void focusLost(FocusEvent arg0)
         {
            setReadEveryNTicksTextFieldToCurrentValue(textField);
         }

         @Override
         public void focusGained(FocusEvent arg0)
         {
            setReadEveryNTicksTextFieldToCurrentValue(textField);
         }
      });

      scs.addTextField(textField);
   }

   private void setReadEveryNTicksTextFieldToCurrentValue(final JTextField textField)
   {
      String everyNTicksString = Integer.toString(robot.getReadEveryNTicks());
      textField.setText(everyNTicksString);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public FloatingRootJointRobot getSDFRobot()
   {
      return robot;
   }

   public Plotter getPlotter()
   {
      return plotter.getPlotter();
   }

   public void run()
   {
      new Thread(scs, "IHMC-SCSLogVisualizer").start();
   }

   public void addLogPlaybackListener(YoVariableLogPlaybackListener listener)
   {
      listener.setYoVariableRegistry(scs.getRootRegistry());
      robot.addLogPlaybackListener(listener);
   }

   private PlaybackListener createYoGraphicsUpdater(final YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      return new PlaybackListener()
      {
         @Override
         public void notifyOfIndexChange(int newIndex)
         {
            updateYoGraphics(yoGraphicsListRegistry);
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

   private void updateYoGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
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

   /**
    * Valid arguments are:
    * <ul>
    * <li>SCS Var group file path, example: <tt>--varGroupFile="/home/user/varGroupExample.xml"</tt>
    * <li>Setting custom buffer size: example: <tt>-b16000</tt> or <tt>--bufferSize=16000</tt>
    * </ul>
    * 
    * @param args
    * @throws Exception
    */
   public static void main(String[] args) throws Exception
   {
      JSAP jsap = new JSAP();

      FlaggedOption varGroupFileFlag = new FlaggedOption("varGroupFile").setLongFlag("varGroupFile").setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption bufferSizeFlag = new FlaggedOption("buffserSize").setLongFlag("bufferSize").setShortFlag('b').setRequired(false)
                                                                     .setStringParser(JSAP.STRING_PARSER);
      jsap.registerParameter(varGroupFileFlag);
      jsap.registerParameter(bufferSizeFlag);

      JSAPResult config = jsap.parse(args);

      int bufferSize = DEFAULT_BUFFER_SIZE;
      List<VarGroup> varGroups = null;

      if (config.success())
      {
         String bufferSizeString = config.getString(bufferSizeFlag.getID());
         if (bufferSizeString != null)
         {
            bufferSize = Integer.parseInt(bufferSizeString);
            LogTools.info("Setting custom buffer size: " + bufferSize);
         }

         String varGroupFilePath = config.getString("varGroupFile");
         if (varGroupFilePath != null)
         {
            File varGroupFile = new File(varGroupFilePath);
            varGroups = loadVarGroups(varGroupFile);
            LogTools.info("Loaded VarGroup file: " + varGroupFile);
         }
      }

      LogVisualizer visualizer = new LogVisualizer(bufferSize);
      if (visualizer.scs == null)
         return;
      if (varGroups != null && !varGroups.isEmpty())
         varGroups.forEach(visualizer.scs.getVarGroupList()::addVarGroup);

      visualizer.run();
   }

   private static List<VarGroup> loadVarGroups(File file) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(XMLVarGroupCollection.class);
      Unmarshaller unmarshaller = context.createUnmarshaller();
      XMLVarGroupCollection loaded = (XMLVarGroupCollection) unmarshaller.unmarshal(file);
      return loaded.toVarGroup();

   }
}
