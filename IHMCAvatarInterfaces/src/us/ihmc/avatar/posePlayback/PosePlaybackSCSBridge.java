package us.ihmc.avatar.posePlayback;

import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.vecmath.Point3d;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;

public class PosePlaybackSCSBridge
{

   private static final boolean promptForTimeDelay = false;

   private PlaybackPoseSequence posePlaybackRobotPoseSequence;
   private PlaybackPoseSequence lastLoadedPoseSequence;
   
   private int frameByframePoseNumber;
   private double frameByframeTime;
   private boolean playOnlyOnePose = false;

   private final PlaybackPoseInterpolator interpolator;
   private final YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");

   // private final BooleanYoVariable plotBalls = new BooleanYoVariable("plotBalls", registry);
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassPosition2d = new YoFramePoint("centerOfMass2d", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint leftAnklePosition = new YoFramePoint("leftAnklePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint rightAnklePosition = new YoFramePoint("rightAnklePosition", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePoint> anklePositions = new SideDependentList<YoFramePoint>(leftAnklePosition, rightAnklePosition);

   private PlaybackPose previousPose;

   private final SideDependentList<YoGraphicCoordinateSystem> feetCoordinateSystems = new SideDependentList<YoGraphicCoordinateSystem>();

   private final YoFramePoint leftWristPosition = new YoFramePoint("leftWristPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint rightWristPosition = new YoFramePoint("rightWristPosition", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePoint> wristPositions = new SideDependentList<YoFramePoint>(leftWristPosition, rightWristPosition);

   private final SideDependentList<YoGraphicCoordinateSystem> handCoordinateSystems = new SideDependentList<>();

   private final PoseSequenceEditorGUI poseSequenceEditorGUI;

   private final HumanoidFloatingRootJointRobot sdfRobot;
   private final FullRobotModel fullRobotModel;
   private final SimulationConstructionSet scs;
   
   private final double controlDT;

   // private final BagOfBalls balls = new BagOfBalls(500, 0.01, YoAppearance.AliceBlue(), registry, yoGraphicsListRegistry);

   public PosePlaybackSCSBridge(HumanoidFloatingRootJointRobot sdfRobot, FullHumanoidRobotModel fullRobotModel, FullHumanoidRobotModel fullRobotModelForSlider, double controlDT) throws IOException
   {
      this.sdfRobot = sdfRobot;
      this.fullRobotModel = fullRobotModel;
      this.controlDT = controlDT;
      
      interpolator = new PlaybackPoseInterpolator(registry);
      
      posePlaybackRobotPoseSequence = new PlaybackPoseSequence(fullRobotModel);

      SDFPerfectSimulatedSensorReader reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, null);
      ModularRobotController controller = new ModularRobotController("Reader");
      controller.setRawSensorReader(reader);

      scs = new SimulationConstructionSet(sdfRobot);
      scs.setDT(controlDT, 1);
      scs.addYoVariableRegistry(registry);

      String listName = getClass().getSimpleName();
      yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicPosition("centerOfMass", centerOfMassPosition, 0.03, YoAppearance.Gold()));
      yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicPosition("centerOfMass2d", centerOfMassPosition2d, 0.03, YoAppearance.Gold()));
      
      SideDependentList<AppearanceDefinition> appearance = new SideDependentList<>(YoAppearance.Red(), YoAppearance.Green());
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicPosition(sidePrefix + "AnkleViz", anklePositions.get(robotSide), 0.05, appearance.get(robotSide)));
         yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicPosition(sidePrefix + "WristViz", wristPositions.get(robotSide), 0.05, appearance.get(robotSide)));
         
         YoGraphicCoordinateSystem footCoordinateSystem = new YoGraphicCoordinateSystem(sidePrefix + "Foot", "", registry, 0.25);
         yoGraphicsListRegistry.registerYoGraphic(listName, footCoordinateSystem);
         feetCoordinateSystems.put(robotSide, footCoordinateSystem);

         YoGraphicCoordinateSystem handCoordinateSystem = new YoGraphicCoordinateSystem(sidePrefix + "Hand", "", registry, 0.25);
         yoGraphicsListRegistry.registerYoGraphic(listName, handCoordinateSystem);
         handCoordinateSystems.put(robotSide, handCoordinateSystem);
      }
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      
      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs, sdfRobot, fullRobotModelForSlider, yoGraphicsListRegistry);

      sliderBoard.addCaptureSnapshotListener(new CaptureSnapshotListener(fullRobotModel, controller));
      sliderBoard.addSaveSequenceRequestedListener(new SaveSequenceListener());
      sliderBoard.addLoadSequenceRequestedListener(new LoadSequenceFromFileListener(fullRobotModel));
      sliderBoard.addClearSequenceRequestedListener(new ClearSequenceListener());
      sliderBoard.addLoadFrameByFrameSequenceRequestedListener(new LoadFrameByFrameSequenceListener());
      sliderBoard.addLoadLastSequenceRequestedListener(new LoadLastSequenceListener());
      sliderBoard.addPlayPoseFromFrameByFrameSequenceRequestedListener(new PlayPoseFromFrameByFrameSequenceListener());
      sliderBoard.addResetToBasePoseRequestedListener(new ResetToBasePoseListener());

      poseSequenceEditorGUI = new PoseSequenceEditorGUI(registry, sdfRobot, fullRobotModelForSlider, sliderBoard);
      poseSequenceEditorGUI.setVisible(true);

      scs.startOnAThread();

      CenterOfMassGraphicUpdater centerOfMassGraphicUpdater = new CenterOfMassGraphicUpdater();
      Thread thread = new Thread(centerOfMassGraphicUpdater);
      thread.start();
   }

   private class CenterOfMassGraphicUpdater implements Runnable
   {
      private final Point3d comPoint = new Point3d();
      private final Point3d comPoint2d = new Point3d();

      public void run()
      {
         while (true)
         {
            sdfRobot.update();

            sdfRobot.computeCenterOfMass(comPoint);
            centerOfMassPosition.set(comPoint);

            //            System.out.println(comPoint);
            comPoint2d.set(comPoint.getX(), comPoint.getY(), 0.0);
            centerOfMassPosition2d.set(comPoint2d);

            ThreadTools.sleep(100);
         }
      }
   }

   private class CaptureSnapshotListener implements VariableChangedListener
   {
      private final FullHumanoidRobotModel fullRobotModel;

      private final ModularRobotController controller;

      public CaptureSnapshotListener(FullHumanoidRobotModel fullRobotModel, ModularRobotController controller)
      {
         this.fullRobotModel = fullRobotModel;
         this.controller = controller;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         PlaybackPose pose = new PlaybackPose(fullRobotModel, sdfRobot);

//         if (previousPose != null)
//         {
//            if (pose.epsilonEquals(previousPose, 1e-3, 1.0))
//            {
//               return;
//            }
//         }

         visualizeAppendages();

         System.out.println("Adding pose to sequence list: " + pose);
         posePlaybackRobotPoseSequence.addPose(pose);

         //       FramePoint location = new FramePoint(ReferenceFrame.getWorldFrame(), Math.random(), Math.random(), Math.random());
         //       balls.setBall(location);

         double dt = 0.01;
         double morphTime = 1.0;
         PlaybackPose morphedPose = pose;
         for (double time = 0.0; time < morphTime; time = time + dt)
         {
            double morphPercentage = time / morphTime;
            

            if (previousPose == null)
            {
               morphedPose = pose;
            }
            else
            {
               morphedPose = PlaybackPose.morph(previousPose, pose, morphPercentage);
            }

            if (promptForTimeDelay)
            {
               String requestedPlaybackDelay = JOptionPane.showInputDialog("Playback delay before this transition in milliseconds?");
               double playBackDelayPoseTransition = Double.parseDouble(requestedPlaybackDelay);
               pose.setPlaybackDelayBeforePose(playBackDelayPoseTransition);
            }
            
            scs.setTime(time);
            scs.tickAndUpdate();
         }
         morphedPose.setRobotAtPose(sdfRobot);

         previousPose = pose;
      }

      private void visualizeAppendages()
      {
         sdfRobot.update();
         controller.doControl();
         fullRobotModel.updateFrames();

         for (RobotSide robotSide : RobotSide.values())
         {
            ReferenceFrame ankleFrame = fullRobotModel.getFoot(robotSide).getParentJoint().getFrameAfterJoint();
            FramePoint anklePosition = new FramePoint(ankleFrame);
            anklePosition.changeFrame(ReferenceFrame.getWorldFrame());
            anklePositions.get(robotSide).set(anklePosition);

            ReferenceFrame footFrame = fullRobotModel.getFoot(robotSide).getBodyFixedFrame();
            feetCoordinateSystems.get(robotSide).setToReferenceFrame(footFrame);

            ReferenceFrame wristFrame = fullRobotModel.getHand(robotSide).getParentJoint().getFrameAfterJoint();
            FramePoint wristPosition = new FramePoint(wristFrame);
            wristPosition.changeFrame(ReferenceFrame.getWorldFrame());
            wristPositions.get(robotSide).set(wristPosition);

            ReferenceFrame handFrame = fullRobotModel.getHand(robotSide).getBodyFixedFrame();

            FramePose palmPose = new FramePose(handFrame);
            FramePoint palmPositionWithRespectToHandFrame = new FramePoint(handFrame, 0.0, robotSide.negateIfRightSide(0.08), -0.04);
            double yaw = 0.0;
            double pitch = 0.0;
            double roll = robotSide.negateIfLeftSide(0.4);
            FrameOrientation palmOrientationWithRespectToHandFrame = new FrameOrientation(handFrame, yaw, pitch, roll);

            palmPose.setPosition(palmPositionWithRespectToHandFrame);
            palmPose.setOrientation(palmOrientationWithRespectToHandFrame);

            PoseReferenceFrame palmFrame = new PoseReferenceFrame("palmFrame", palmPose);
            palmFrame.update();

            handCoordinateSystems.get(robotSide).setToReferenceFrame(palmFrame);
         }
      }

   }

   private class LoadSequenceFromFileListener implements VariableChangedListener
   {
      private final FullRobotModel fullRobotModel;
      
      public LoadSequenceFromFileListener(FullRobotModel fullRobotModel)
      { 
         this.fullRobotModel = fullRobotModel;
      }
      
      public void variableChanged(YoVariable yoVariable)
      {
         if (!((BooleanYoVariable) yoVariable).getBooleanValue())
            return;

         System.out.println("Load Sequence");

         boolean selectedFileSuccessful = initPlaybackFromFile(fullRobotModel, new PlaybackPoseSequence(fullRobotModel));
         if (!selectedFileSuccessful)
            return;

         playOnlyOnePose = false;

         playLoadedSequence();
      }
   }

   private class LoadLastSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         if (!((BooleanYoVariable) yoVariable).getBooleanValue())
            return;

         System.out.println("Load Last Sequence");

         initPlayback(lastLoadedPoseSequence);
         
         playOnlyOnePose = false;

         playLoadedSequence();
      }
   }

   private class LoadFrameByFrameSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         if (!((BooleanYoVariable) yoVariable).getBooleanValue())
            return;

         System.out.println("Load Sequence for Frame by Frame Play Back");

         boolean selectedFileSuccessful = initPlaybackFromFile(fullRobotModel, posePlaybackRobotPoseSequence);
         if (!selectedFileSuccessful)
            return;

         playOnlyOnePose = true;

         poseSequenceEditorGUI.setSequence(posePlaybackRobotPoseSequence);
         poseSequenceEditorGUI.setVisible(true);
      }
   }

   private class PlayPoseFromFrameByFrameSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         //         if(!((BooleanYoVariable) yoVariable).getBooleanValue())
         //            return;

         playLoadedSequence();
      }
   }

   private class SaveSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         if (((BooleanYoVariable) yoVariable).getBooleanValue())
         {
            System.out.println("saving file");
            PlaybackPoseSequenceWriter.promptWriteToFile(posePlaybackRobotPoseSequence);
         }
      }
   }

   private class ClearSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         posePlaybackRobotPoseSequence.clear();
         System.out.println("Clearing Sequence");
      }
   }

   private class ResetToBasePoseListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         if (sdfRobot != null && previousPose != null)
         {
            previousPose.setRobotAtPose(sdfRobot);
         }
      }
   }

   private boolean initPlaybackFromFile(FullRobotModel fullRobotModel, PlaybackPoseSequence sequence)
   {
      boolean successful = true;
      JFileChooser chooser = new JFileChooser(new File("PoseSequences"));
      int approveOption = chooser.showOpenDialog(null);

      if (approveOption != JFileChooser.APPROVE_OPTION)
      {
         if (chooser.getName() != null)
            System.err.println("Can not load selected file :" + chooser.getName());

         return successful = false;
      }

      File selectedFile = chooser.getSelectedFile();

      sequence.clear();
      PlaybackPoseSequenceReader.appendFromFile(sequence, selectedFile);

      initPlayback(sequence);

      return successful;
   }

   private void initPlayback(PlaybackPoseSequence sequence)
   {
      double startTime = 0.0;
      frameByframeTime = startTime;

      interpolator.startSequencePlayback(sequence, startTime);
      interpolator.setTimeDelayAfterPose(0.0);

      frameByframePoseNumber = 0;
      
      lastLoadedPoseSequence=sequence;
   }

   private void playLoadedSequence()
   {
      PlaybackPose morphedPose = interpolator.getPose(frameByframeTime);
      while (!interpolator.isDone())
      {
         frameByframeTime = frameByframeTime + controlDT;

         morphedPose = interpolator.getPose(frameByframeTime);
         previousPose = morphedPose;

         scs.setTime(frameByframeTime);
         scs.tickAndUpdate();
         //morphedPose.setRobotAtPose(sdfRobot);//don't update scs while playing back and connected to gazebo to avoid slider actuation delays

         {
            System.out.println("pose #: " + frameByframePoseNumber++ + " \t pausing for " + interpolator.getNextTransitionTimeDelay());
            if (playOnlyOnePose)
            {
               morphedPose.setRobotAtPose(sdfRobot);// make sure scs ends in last pose
               return;
            }
         }
         ThreadTools.sleep((long) (controlDT * 1000));
      }
      morphedPose.setRobotAtPose(sdfRobot);// make sure scs ends in last pose

      System.out.println("End of Play back");
   }

}
