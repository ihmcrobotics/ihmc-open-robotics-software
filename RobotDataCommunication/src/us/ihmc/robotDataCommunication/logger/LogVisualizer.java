package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import javax.swing.JLabel;
import javax.swing.JTextField;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.logger.util.FileSelectionDialog;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.plotting.SimulationOverheadPlotter;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class LogVisualizer
{
   private final SDFJointNameMap jointNameMap;
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;
   protected final SimulationConstructionSet scs;
   private YoVariableLogPlaybackRobot robot;
   private SimulationOverheadPlotter plotter;

   public LogVisualizer() throws IOException
   {
      this(null, null);
   }
   
   public LogVisualizer(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap jointNameMap) throws IOException
   {
      this(generalizedSDFRobotModel, jointNameMap, 32768, false, null);
   }

   public LogVisualizer(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap jointNameMap, int bufferSize, boolean showOverheadView,
         File logFile) throws IOException
   {
      this.generalizedSDFRobotModel = generalizedSDFRobotModel;
      this.jointNameMap = jointNameMap;

      if (logFile == null)
      {
         logFile = FileSelectionDialog.loadDirectoryWithFileNamed("robotData.log");
      }

      if (logFile != null)
      {
         System.out.println("loading log from folder:" + logFile);
         scs = new SimulationConstructionSet(true, bufferSize);
         scs.setFastSimulate(true, 50);
         readLogFile(logFile, showOverheadView);
      }
      else
      {
         scs = null;
      }

   }

   private void readLogFile(File selectedFile, boolean showOverheadView) throws IOException
   {
      LogPropertiesReader logProperties = new LogPropertiesReader(new File(selectedFile, YoVariableLoggerListener.propertyFile));
      File handshake = new File(selectedFile, logProperties.getHandshakeFile());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getHandshakeFile());
      }

      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();

      YoVariableHandshakeParser parser = new YoVariableHandshakeParser("logged", true);
      parser.parseFrom(handshakeData);

      GeneralizedSDFRobotModel generalizedSDFRobotModel;
      if (logProperties.getModelLoaderClass() != null)
      {
         SDFModelLoader loader = new SDFModelLoader();
         String modelName = logProperties.getModelName();
         String[] resourceDirectories = logProperties.getModelResourceDirectories();

         File model = new File(selectedFile, logProperties.getModelPath());
         DataInputStream modelStream = new DataInputStream(new FileInputStream(model));
         byte[] modelData = new byte[(int) model.length()];
         modelStream.readFully(modelData);
         modelStream.close();

         File resourceBundle = new File(selectedFile, logProperties.getModelResourceBundlePath());
         DataInputStream resourceStream = new DataInputStream(new FileInputStream(resourceBundle));
         byte[] resourceData = new byte[(int) resourceBundle.length()];
         resourceStream.readFully(resourceData);
         resourceStream.close();

         loader.load(modelName, modelData, resourceDirectories, resourceData);

         generalizedSDFRobotModel = loader.createJaxbSDFLoader().getGeneralizedSDFRobotModel(modelName);
      }
      else
      {
         generalizedSDFRobotModel = this.generalizedSDFRobotModel;
      }


      robot = new YoVariableLogPlaybackRobot(selectedFile, generalizedSDFRobotModel, jointNameMap, parser.getJointStates(), parser.getYoVariablesList(), logProperties ,scs);
      scs.setTimeVariableName(robot.getRobotsYoVariableRegistry().getName() + ".robotTime");

      double dt = parser.getDt();
      System.out.println(getClass().getSimpleName() + ": dt set to " + dt);
      scs.setDT(dt, 1);
      scs.setPlaybackDesiredFrameRate(0.04);

      YoGraphicsListRegistry yoGraphicsListRegistry = parser.getDynamicGraphicObjectsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      plotter = VisualizerUtils.createOverheadPlotter(scs, showOverheadView, yoGraphicsListRegistry);
      scs.getRootRegistry().addChild(parser.getRootRegistry());
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

      YoVariableLogCropper yoVariableLogCropper = new YoVariableLogCropper(players, selectedFile, logProperties);

      scs.getJFrame().setTitle(this.getClass().getSimpleName() + " - " + selectedFile);
      YoVariableLogVisualizerGUI gui = new YoVariableLogVisualizerGUI(selectedFile, logProperties, players, robot, yoVariableLogCropper, scs);
      scs.getStandardSimulationGUI().addJComponentToMainPanel(gui, BorderLayout.SOUTH);

      setupReadEveryNTicksTextField();
   }

   private void setupReadEveryNTicksTextField()
   {
      JLabel label = new JLabel("ReadEveryNTicks");
      scs.addJLable(label);

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

   public SDFRobot getSDFRobot()
   {
      return robot;
   }

   public Plotter getPlotter()
   {
      return plotter.getPlotter();
   }

   public void run()
   {
      new Thread(scs).start();
   }
   
   public static void main(String[] args) throws IOException
   {
      LogVisualizer visualizer = new LogVisualizer();
      visualizer.run();
   }
}
