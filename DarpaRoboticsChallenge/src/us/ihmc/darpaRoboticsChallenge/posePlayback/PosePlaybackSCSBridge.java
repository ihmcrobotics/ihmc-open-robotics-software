package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTask;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTaskName;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class PosePlaybackSCSBridge
{
   private static final String ipAddress = DRCConfigParameters.CLOUD_MINION2_IP;
   private static final double controlDT = 0.005;
   
   private static final boolean promptForTimeDelay = false;

   private final PosePlaybackAllJointsController posePlaybackController;
   private final PosePlaybackSender posePlaybackSender;
   private PosePlaybackRobotPoseSequence posePlaybackRobotPoseSequence;
   
   private int frameByframePoseNumber;
   private double frameByframeTime;

   private final PosePlaybackSmoothPoseInterpolator interpolator;
   private final YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");

// private final BooleanYoVariable plotBalls = new BooleanYoVariable("plotBalls", registry);
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassPosition2d = new YoFramePoint("centerOfMass2d", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint leftAnklePosition = new YoFramePoint("leftAnklePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint rightAnklePosition = new YoFramePoint("rightAnklePosition", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePoint> anklePositions = new SideDependentList<YoFramePoint>(leftAnklePosition, rightAnklePosition);
   
   private final EnumYoVariable leftPalmPoseClassification = new EnumYoVariable("leftPalmPose", "", registry, PalmPoseClassification.class, true);
   private final EnumYoVariable rightPalmPoseClassification = new EnumYoVariable("rightPalmPose", "", registry, PalmPoseClassification.class, true);
   private final SideDependentList<EnumYoVariable> palmPoseClassifications = new SideDependentList<EnumYoVariable>(leftPalmPoseClassification, rightPalmPoseClassification);

   private PosePlaybackRobotPose previousPose;

   private final SideDependentList<DynamicGraphicCoordinateSystem> feetCoordinateSystems;

   private final YoFramePoint leftWristPosition = new YoFramePoint("leftWristPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint rightWristPosition = new YoFramePoint("rightWristPosition", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePoint> wristPositions = new SideDependentList<YoFramePoint>(leftWristPosition, rightWristPosition);

   private final SideDependentList<DynamicGraphicCoordinateSystem> handCoordinateSystems;

   private final PoseSequenceEditorGUI poseSequenceEditorGUI;

// private final BagOfBalls balls = new BagOfBalls(500, 0.01, YoAppearance.AliceBlue(), registry, dynamicGraphicObjectsListRegistry);

   public PosePlaybackSCSBridge() throws IOException
   {
      interpolator = new PosePlaybackSmoothPoseInterpolator(registry);

      posePlaybackController = new PosePlaybackAllJointsController(registry);
      posePlaybackSender = new PosePlaybackSender(posePlaybackController, ipAddress);
      posePlaybackRobotPoseSequence = new PosePlaybackRobotPoseSequence();

      VRCTask vrcTask = new VRCTask(VRCTaskName.ONLY_VEHICLE);
      SDFFullRobotModel fullRobotModel = vrcTask.getFullRobotModelFactory().create();

      
      SDFRobot sdfRobot = vrcTask.getRobot();
      ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel, vrcTask.getJointMap(), vrcTask.getJointMap().getAnkleHeight());
      SDFPerfectSimulatedSensorReader reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
      ModularRobotController controller = new ModularRobotController("Reader");
      controller.setRawSensorReader(reader);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.setDT(controlDT, 1);
      scs.addYoVariableRegistry(registry);

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList(getClass().getSimpleName());
      DynamicGraphicPosition centerOfMassViz = new DynamicGraphicPosition("centerOfMass", centerOfMassPosition, 0.03, YoAppearance.Gold());
      DynamicGraphicPosition centerOfMass2dViz = new DynamicGraphicPosition("centerOfMass2d", centerOfMassPosition2d, 0.03, YoAppearance.Gold());

      DynamicGraphicPosition leftAnkleViz = new DynamicGraphicPosition("leftAnkleViz", leftAnklePosition, 0.05, YoAppearance.Red());
      DynamicGraphicPosition rightAnkleViz = new DynamicGraphicPosition("rightAnkleViz", rightAnklePosition, 0.05, YoAppearance.Green());
      
      DynamicGraphicPosition leftWristViz = new DynamicGraphicPosition("leftWristViz", leftWristPosition, 0.05, YoAppearance.Red());
      DynamicGraphicPosition rightWristViz = new DynamicGraphicPosition("rightWristViz", rightWristPosition, 0.05, YoAppearance.Green());

      DynamicGraphicCoordinateSystem leftFootCoordinateSystem = new DynamicGraphicCoordinateSystem("leftFoot", "", registry, 0.25);
      DynamicGraphicCoordinateSystem rightFootCoordinateSystem = new DynamicGraphicCoordinateSystem("rightFoot", "", registry, 0.25);
      feetCoordinateSystems = new SideDependentList<DynamicGraphicCoordinateSystem>(leftFootCoordinateSystem, rightFootCoordinateSystem);
      
      DynamicGraphicCoordinateSystem leftHandCoordinateSystem = new DynamicGraphicCoordinateSystem("leftHand", "", registry, 0.25);
      DynamicGraphicCoordinateSystem rightHandCoordinateSystem = new DynamicGraphicCoordinateSystem("rightHand", "", registry, 0.25);
      handCoordinateSystems = new SideDependentList<DynamicGraphicCoordinateSystem>(leftHandCoordinateSystem, rightHandCoordinateSystem);
      
      
      
      dynamicGraphicObjectsList.add(centerOfMassViz);
      dynamicGraphicObjectsList.add(centerOfMass2dViz);

      dynamicGraphicObjectsList.add(leftAnkleViz);
      dynamicGraphicObjectsList.add(rightAnkleViz);
      
      dynamicGraphicObjectsList.add(leftFootCoordinateSystem);
      dynamicGraphicObjectsList.add(rightFootCoordinateSystem);

      
      dynamicGraphicObjectsList.add(leftWristViz);
      dynamicGraphicObjectsList.add(rightWristViz);
      
      dynamicGraphicObjectsList.add(leftHandCoordinateSystem);
      dynamicGraphicObjectsList.add(rightHandCoordinateSystem);

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      
      SDFFullRobotModel fullRobotModelForSlider = vrcTask.getFullRobotModelFactory().create();
      ReferenceFrames referenceFramesForSlider = new ReferenceFrames(fullRobotModelForSlider, vrcTask.getJointMap(), vrcTask.getJointMap().getAnkleHeight());
      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs, sdfRobot, referenceFramesForSlider, fullRobotModelForSlider, dynamicGraphicObjectsListRegistry);
      
      CaptureSnapshotListener captureSnapshotListener = new CaptureSnapshotListener(sdfRobot, referenceFrames, fullRobotModel, controller, scs);
      sliderBoard.addCaptureSnapshotListener(captureSnapshotListener);

      SaveSequenceListener saveSequenceListener = new SaveSequenceListener();
      sliderBoard.addSaveSequenceRequestedListener(saveSequenceListener);

      LoadSequenceListener loadSequenceListener = new LoadSequenceListener(sdfRobot, scs);
      sliderBoard.addLoadSequenceRequestedListener(loadSequenceListener);

      ClearSequenceListener clearSequenceListener = new ClearSequenceListener();
      sliderBoard.addClearSequenceRequestedListener(clearSequenceListener);
      
      LoadFrameByFrameSequenceListener loadFrameByFrameSequenceListener = new LoadFrameByFrameSequenceListener(sdfRobot, scs);
      sliderBoard.addLoadFrameByFrameSequenceRequestedListener(loadFrameByFrameSequenceListener);

      PlayPoseFromFrameByFrameSequenceListener playPoseFromFrameByFrameSequenceListener = new PlayPoseFromFrameByFrameSequenceListener(sdfRobot, scs);
      sliderBoard.addPlayPoseFromFrameByFrameSequenceRequestedListener(playPoseFromFrameByFrameSequenceListener);
      
      ResetToBasePoseListener resetToBasePoseListener = new ResetToBasePoseListener(sdfRobot);
      sliderBoard.addResetToBasePoseRequestedListener(resetToBasePoseListener);
      
      poseSequenceEditorGUI = new PoseSequenceEditorGUI(registry,posePlaybackController,sdfRobot, sliderBoard);
      poseSequenceEditorGUI.setVisible(true);
      
      scs.startOnAThread();

      CenterOfMassGraphicUpdater centerOfMassGraphicUpdater = new CenterOfMassGraphicUpdater(sdfRobot);
      Thread thread = new Thread(centerOfMassGraphicUpdater);
      thread.start();

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

   private class CenterOfMassGraphicUpdater implements Runnable
   {
      private final SDFRobot sdfRobot;

      private final Point3d comPoint = new Point3d();
      private final Point3d comPoint2d = new Point3d();


      CenterOfMassGraphicUpdater(SDFRobot sdfRobot)
      {
         this.sdfRobot = sdfRobot;
      }

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
      private final SDFRobot sdfRobot;
      private final FullRobotModel fullRobotModel;

      private final SimulationConstructionSet scs;
      private final ReferenceFrames referenceFrames;
      private final ModularRobotController controller;

      public CaptureSnapshotListener(SDFRobot sdfRobot, ReferenceFrames referenceFrames, SDFFullRobotModel fullRobotModel,
            ModularRobotController controller,
            SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.referenceFrames = referenceFrames;
         this.fullRobotModel = fullRobotModel;
         this.controller = controller;
         this.scs = scs;
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

         visualizeAppendages();
         
         System.out.println("Adding pose to sequence list: " + pose);
         posePlaybackRobotPoseSequence.addPose(pose);

//       FramePoint location = new FramePoint(ReferenceFrame.getWorldFrame(), Math.random(), Math.random(), Math.random());
//       balls.setBall(location);

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
            
            if(promptForTimeDelay)
            {
               String requestedPlaybackDelay = JOptionPane.showInputDialog("Playback delay before this transition in milliseconds?");
               double playBackDelayPoseTransition = (requestedPlaybackDelay == "") ? PosePlaybackAtlasDefaultParameters.defaultPlaybackTransitionDelayMillis : Double
                     .parseDouble(requestedPlaybackDelay);
               pose.setPlaybackDelayBeforePose(playBackDelayPoseTransition);
            }

            morphedPose.setRobotAtPose(sdfRobot);
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

      private void visualizeAppendages()
      {
         sdfRobot.update();
         controller.doControl();
         referenceFrames.updateFrames();
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

            PalmPoseClassification classifcation = PalmPoseClassifier.getClassification(robotSide, handFrame, fullRobotModel.getChest().getBodyFixedFrame());
            palmPoseClassifications.get(robotSide).set(classifcation);

//            PoseReferenceFrame toDisplay = PalmPoseClassifier.getPoseReferenceFrame(RobotSide.RIGHT, PalmPoseClassification.PALM_IN);
//            handCoordinateSystems.get(robotSide).setToReferenceFrame(toDisplay);
         }
      }

   }


   private class LoadSequenceListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;
      private final SimulationConstructionSet scs;
 
      public LoadSequenceListener(SDFRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         if(!((BooleanYoVariable) yoVariable).getBooleanValue())
            return;
         
         System.out.println("Load Sequence");

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

         interpolator.startSequencePlayback(sequence, startTime);
         interpolator.setTimeDelayAfterPose(0.0);
         int poseNumber = 0;

         PosePlaybackRobotPose morphedPose = interpolator.getPose(time);
         while (!interpolator.isDone())
         {
            time = time + controlDT;

            morphedPose = interpolator.getPose(time);

            posePlaybackController.setPlaybackPose(morphedPose);
            scs.setTime(time);
            scs.tickAndUpdate();
            //morphedPose.setRobotAtPose(sdfRobot);//don't update scs while playing back and connected to gazebo to avoid slider actuation delays

            try
            {
               if (posePlaybackSender.isConnected())
               {
                  posePlaybackSender.writeData();
               }
               else
                  morphedPose.setRobotAtPose(sdfRobot);// playing back to scs only if not connected to gazebo to see what's happening
            }
            catch (IOException e)
            {
            }
            
            if(interpolator.didLastPoseIncrementSequence() || (poseNumber == 0))
            {
               System.out.println("pose #: " + poseNumber++ + " \t pausing for " + interpolator.getNextTransitionTimeDelay());
            }
            ThreadTools.sleep((long) (controlDT * 1000));
         }
         morphedPose.setRobotAtPose(sdfRobot);// make sure scs ends in last pose
         
         System.out.println("End of Play back");
      }
   }

   private class LoadFrameByFrameSequenceListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;
      private final SimulationConstructionSet scs;

      public LoadFrameByFrameSequenceListener(SDFRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         if(!((BooleanYoVariable) yoVariable).getBooleanValue())
            return;
         
         System.out.println("Load Sequence for Frame by Frame Play Back");

         JFileChooser chooser = new JFileChooser(new File("PoseSequences"));
         int approveOption = chooser.showOpenDialog(null);

         if (approveOption != JFileChooser.APPROVE_OPTION)
         {
            System.err.println("Can not load selected file :" + chooser.getName());

            return;
         }

         File selectedFile = chooser.getSelectedFile();

         System.out.println("    Clearing sequence for Frame by Frame Play Back");
         posePlaybackRobotPoseSequence.clear();
         posePlaybackRobotPoseSequence.appendFromFile(selectedFile);

         double startTime = 0.0;
         frameByframeTime = startTime;

         interpolator.startSequencePlayback(posePlaybackRobotPoseSequence, startTime);
         interpolator.setTimeDelayAfterPose(0.0);
         frameByframePoseNumber = 0;
         
         poseSequenceEditorGUI.setSequence(posePlaybackRobotPoseSequence);
         poseSequenceEditorGUI.setVisible(true);
      }
    }
   
   private class ResetToBasePoseListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;
      
      public ResetToBasePoseListener(SDFRobot sdfRobot)
      {
         this.sdfRobot = sdfRobot;
      }
      
      public void variableChanged(YoVariable yoVariable)
      {
         if(sdfRobot!=null)
         {
            previousPose.setRobotAtPose(sdfRobot);
            posePlaybackController.setPlaybackPose(previousPose);
         }
      }
   }

   private class PlayPoseFromFrameByFrameSequenceListener implements VariableChangedListener
   {
      private final SDFRobot sdfRobot;
      private final SimulationConstructionSet scs;

      public PlayPoseFromFrameByFrameSequenceListener(SDFRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         //         if(!((BooleanYoVariable) yoVariable).getBooleanValue())
         //            return;
         
         while (!interpolator.isDone())
         {
            frameByframeTime = frameByframeTime + controlDT;

            PosePlaybackRobotPose morphedPose = interpolator.getPose(frameByframeTime);

            posePlaybackController.setPlaybackPose(morphedPose);
            scs.setTime(frameByframeTime);
            scs.tickAndUpdate();
            morphedPose.setRobotAtPose(sdfRobot);

            try
            {
               if (posePlaybackSender.isConnected())
               {
                  posePlaybackSender.writeData();
               }
               if(interpolator.didLastPoseIncrementSequence() || (frameByframePoseNumber == 0))
               {
                  System.out.println("pose #: " + frameByframePoseNumber++ + " \t pausing for " + interpolator.getNextTransitionTimeDelay());
                  ThreadTools.sleep((long) interpolator.getTransitionTimeDelay());//3000 worked for standing up, 2000 failed at about 33 in standcde, 1000 failed at about 23 in standcde
                  return;
               }
               ThreadTools.sleep((long) (controlDT * 1000));
            }
            catch (IOException e)
            {
            }
         }
         
         System.out.println("End of Play back");
      }
    }

   private class SaveSequenceListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable yoVariable)
      {
         if(((BooleanYoVariable) yoVariable).getBooleanValue())
         {
            System.out.println("saving file");
            posePlaybackRobotPoseSequence.promptWriteToFile();            
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

   public static void main(String[] args) throws IOException
   {
      new PosePlaybackSCSBridge();
   }

}
