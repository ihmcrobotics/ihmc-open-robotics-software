package us.ihmc.avatar.posePlayback;

import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class VisualizePoseWorkspace
{
   private final FullHumanoidRobotModel fullRobotModelForSlider;
   
   private PlaybackPoseSequence posePlaybackRobotPoseSequence;

   private final PlaybackPoseInterpolator interpolator;
   private final YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");
   private final double controlDT;
   
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   
   public VisualizePoseWorkspace(DRCRobotModel robotModel) throws IOException
   {
      this.controlDT = robotModel.getControllerDT();
      
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      interpolator = new PlaybackPoseInterpolator(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      fullRobotModelForSlider = robotModel.createFullRobotModel();
      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs, sdfRobot, fullRobotModelForSlider, yoGraphicsListRegistry);

      posePlaybackRobotPoseSequence = new PlaybackPoseSequence(fullRobotModelForSlider);
      
      CaptureSnapshotListener captureSnapshotListener = new CaptureSnapshotListener(sdfRobot, scs);
      sliderBoard.addCaptureSnapshotListener(captureSnapshotListener);

      SaveSequenceListener saveSequenceListener = new SaveSequenceListener();
      sliderBoard.addSaveSequenceRequestedListener(saveSequenceListener);

      LoadSequenceListener loadSequenceListener = new LoadSequenceListener(fullRobotModelForSlider, sdfRobot, scs);
      sliderBoard.addLoadSequenceRequestedListener(loadSequenceListener);

  
      
      
      scs.startOnAThread();
   }

   private class CaptureSnapshotListener implements VariableChangedListener
   {
      private final HumanoidFloatingRootJointRobot sdfRobot;
      private final SimulationConstructionSet scs;
      private PlaybackPose previousPose;

      public CaptureSnapshotListener(HumanoidFloatingRootJointRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
         //System.out.println("This is what's getting printed now:" + sdfRobot.getOneDoFJoints());
      }


      public void variableChanged(YoVariable yoVariable)
      {
         PlaybackPose pose = new PlaybackPose(fullRobotModelForSlider, sdfRobot);

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
            scs.setTime(time);
            scs.tickAndUpdate();
         }

         previousPose = pose;
      }

   }


   private class LoadSequenceListener implements VariableChangedListener
   {
      private final SimulationConstructionSet scs;

      public LoadSequenceListener(FullRobotModel fullRobotModel, FloatingRootJointRobot sdfRobot, SimulationConstructionSet scs)
      {
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

         PlaybackPoseSequence sequence = new PlaybackPoseSequence(fullRobotModelForSlider);
         PlaybackPoseSequenceReader.appendFromFile(sequence, selectedFile);

         double startTime = 0.0;
         double time = startTime;
         double dt = controlDT;

         interpolator.startSequencePlayback(sequence, startTime);

         while (!interpolator.isDone())
         {
            time = time + dt;

            scs.setTime(time);
            scs.tickAndUpdate();
         }
      }
   }


   private class SaveSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         System.out.println("saving file");
         PlaybackPoseSequenceWriter.promptWriteToFile(posePlaybackRobotPoseSequence);
      }
   }

}
