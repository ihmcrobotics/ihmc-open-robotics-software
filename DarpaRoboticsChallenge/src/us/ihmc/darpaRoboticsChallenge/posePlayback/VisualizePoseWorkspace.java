package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModelFactory;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.ThreadTools;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;
import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class VisualizePoseWorkspace
{
   private static final String ipAddress = "localhost"; //DRCConfigParameters.CLOUD_MINION5_IP;

   private final PosePlaybackAllJointsController posePlaybackController;
   private final PosePlaybackSender posePlaybackSender;
   private PosePlaybackRobotPoseSequence posePlaybackRobotPoseSequence;

   private final PosePlaybackSmoothPoseInterpolator interpolator;
   private final YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");

   private final BooleanYoVariable plotBalls = new BooleanYoVariable("plotBalls", registry);
   private DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   
   private final BagOfBalls balls = new BagOfBalls(500, 0.01, YoAppearance.AliceBlue(), registry, dynamicGraphicObjectsListRegistry);
   
   public VisualizePoseWorkspace(DRCRobotModel robotModel) throws IOException
   {
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      SDFRobot sdfRobot = loader.createRobot(jointMap, false);

      interpolator = new PosePlaybackSmoothPoseInterpolator(registry);

      posePlaybackController = new PosePlaybackAllJointsController(registry);
      posePlaybackSender = new PosePlaybackSender(posePlaybackController, ipAddress);
      posePlaybackRobotPoseSequence = new PosePlaybackRobotPoseSequence();

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.addYoVariableRegistry(registry);
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      SDFFullRobotModel fullRobotModelForSlider = loader.createFullRobotModel(jointMap);
      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs, sdfRobot, fullRobotModelForSlider, dynamicGraphicObjectsListRegistry);

      CaptureSnapshotListener captureSnapshotListener = new CaptureSnapshotListener(sdfRobot, scs);
      sliderBoard.addCaptureSnapshotListener(captureSnapshotListener);

      SaveSequenceListener saveSequenceListener = new SaveSequenceListener();
      sliderBoard.addSaveSequenceRequestedListener(saveSequenceListener);

      LoadSequenceListener loadSequenceListener = new LoadSequenceListener(sdfRobot, scs);
      sliderBoard.addLoadSequenceRequestedListener(loadSequenceListener);

  
      
      
      scs.startOnAThread();

      try
      {
         posePlaybackSender.connect();
         posePlaybackSender.waitUntilConnected();
      }
      catch (Exception e)
      {
         System.err.println("Didn't connect to posePlaybackSender!");
      }

   }

   private class CaptureSnapshotListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;
      private final SimulationConstructionSet scs;
      private PosePlaybackRobotPose previousPose;

      public CaptureSnapshotListener(SDFRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
         //System.out.println("This is what's getting printed now:" + sdfRobot.getOneDoFJoints());
      }


      public void variableChanged(YoVariable yoVariable)
      {
         PosePlaybackRobotPose pose = new PosePlaybackRobotPose(sdfRobot);

         if (previousPose != null)
         {
            if (pose.epsilonEquals(previousPose, 1e-3, 1.0))
            {
               return;
            }
         }

         System.out.println("Adding pose to sequence list: " + pose);
         posePlaybackRobotPoseSequence.addPose(pose);  
         
//         FramePoint location = new FramePoint(ReferenceFrame.getWorldFrame(), Math.random(), Math.random(), Math.random());
//         balls.setBall(location);

         double dt = 0.01;
         double morphTime = 1.0;
         for (double time = 0.0; time < morphTime; time = time + dt)
         {
            double morphPercentage = time / morphTime;
            PosePlaybackRobotPose morphedPose;

            if (previousPose == null)
            {
               morphedPose = pose;
            }
            else
            {
               morphedPose = PosePlaybackRobotPose.morph(previousPose, pose, morphPercentage);
            }

            posePlaybackController.setPlaybackPose(morphedPose);
            scs.setTime(time);
            scs.tickAndUpdate();

            try
            {
               if (posePlaybackSender.isConnected())
                  posePlaybackSender.writeData();
               ThreadTools.sleep((long) (dt * 1000));
            }
            catch (IOException e)
            {
            }
         }

         previousPose = pose;
      }

   }


   private class LoadSequenceListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;
      private final SimulationConstructionSet scs;
      private PosePlaybackRobotPose previousPose;

      public LoadSequenceListener(SDFRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         System.out.println("Load Sequence Listener");

         JFileChooser chooser = new JFileChooser(new File("PoseSequences"));
         int approveOption = chooser.showOpenDialog(null);

         if (approveOption != JFileChooser.APPROVE_OPTION)
         {
            System.err.println("Can not load selected file :" + chooser.getName());

            return;
         }

         File selectedFile = chooser.getSelectedFile();

         PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
         sequence.appendFromFile(selectedFile);

         double startTime = 0.0;
         double time = startTime;
         double dt = DRCConfigParameters.CONTROL_DT;

         interpolator.startSequencePlayback(sequence, startTime);

         while (!interpolator.isDone())
         {
            time = time + dt;

            PosePlaybackRobotPose morphedPose = interpolator.getPose(time);

            posePlaybackController.setPlaybackPose(morphedPose);
            scs.setTime(time);
            scs.tickAndUpdate();

            try
            {
               if (posePlaybackSender.isConnected())
                  posePlaybackSender.writeData();
               ThreadTools.sleep((long) (dt * 1000));
            }
            catch (IOException e)
            {
            }
         }
      }
   }


   private class SaveSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         System.out.println("saving file");
         posePlaybackRobotPoseSequence.promptWriteToFile();
      }
   }


   public static void main(String[] args) throws IOException
   {
      // Flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + DRCRobotModelFactory.robotModelsToString());
      
      DRCRobotModel model;
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            model = DRCRobotModelFactory.CreateDRCRobotModel(config.getString("robotModel"));
         }
         else
         {
            System.out.println("Enter a robot model.");
            return;
         }
      }
      catch (Exception e)
      {
         System.out.println("Robot model not found");
         e.printStackTrace();
         return;
      }
      
      new VisualizePoseWorkspace(model);

   }

}
