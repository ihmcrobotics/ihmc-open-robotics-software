package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.configuration.LocalCloudMachines;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModelFactory;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;
import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class PosePlaybackSCSBridge
{
   private static final String ipAddress = LocalCloudMachines.CLOUDMINION_2.getIp();
   private static final double controlDT = DRCConfigParameters.CONTROL_DT;

   private static final boolean promptForTimeDelay = false;

   private final PosePlaybackAllJointsController posePlaybackController;
   private final PosePlaybackSender posePlaybackSender;
   private PosePlaybackRobotPoseSequence posePlaybackRobotPoseSequence;
   private PosePlaybackRobotPoseSequence lastLoadedPoseSequence;
   
   private int frameByframePoseNumber;
   private double frameByframeTime;
   private boolean playOnlyOnePose = false;

   private final PosePlaybackSmoothPoseInterpolator interpolator;
   private final YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");

   // private final BooleanYoVariable plotBalls = new BooleanYoVariable("plotBalls", registry);
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassPosition2d = new YoFramePoint("centerOfMass2d", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint leftAnklePosition = new YoFramePoint("leftAnklePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint rightAnklePosition = new YoFramePoint("rightAnklePosition", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePoint> anklePositions = new SideDependentList<YoFramePoint>(leftAnklePosition, rightAnklePosition);

   private final EnumYoVariable<PalmPoseClassification> leftPalmPoseClassification = new EnumYoVariable<PalmPoseClassification>("leftPalmPose", "", registry, PalmPoseClassification.class, true);
   private final EnumYoVariable<PalmPoseClassification> rightPalmPoseClassification = new EnumYoVariable<PalmPoseClassification>("rightPalmPose", "", registry, PalmPoseClassification.class, true);
   private final SideDependentList<EnumYoVariable<PalmPoseClassification>> palmPoseClassifications = new SideDependentList<EnumYoVariable<PalmPoseClassification>>(leftPalmPoseClassification, rightPalmPoseClassification);

   private PlaybackPose previousPose;

   private final SideDependentList<DynamicGraphicCoordinateSystem> feetCoordinateSystems = new SideDependentList<DynamicGraphicCoordinateSystem>();

   private final YoFramePoint leftWristPosition = new YoFramePoint("leftWristPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint rightWristPosition = new YoFramePoint("rightWristPosition", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePoint> wristPositions = new SideDependentList<YoFramePoint>(leftWristPosition, rightWristPosition);

   private final SideDependentList<DynamicGraphicCoordinateSystem> handCoordinateSystems = new SideDependentList<>();

   private final PoseSequenceEditorGUI poseSequenceEditorGUI;

   private final SDFRobot sdfRobot;
   private final SDFFullRobotModel fullRobotModel;
   private final SimulationConstructionSet scs;

   // private final BagOfBalls balls = new BagOfBalls(500, 0.01, YoAppearance.AliceBlue(), registry, dynamicGraphicObjectsListRegistry);

   public PosePlaybackSCSBridge(DRCRobotModel robotModel) throws IOException
   {
      interpolator = new PosePlaybackSmoothPoseInterpolator(registry);

      DRCRobotJointMap jointMap = robotModel.getJointMap();
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      sdfRobot = loader.createRobot(jointMap, false);
      fullRobotModel = loader.createFullRobotModel(jointMap);
      
      posePlaybackController = new PosePlaybackAllJointsController(fullRobotModel, registry);
      posePlaybackSender = new PosePlaybackSender(posePlaybackController, ipAddress);
      posePlaybackRobotPoseSequence = new PosePlaybackRobotPoseSequence(fullRobotModel);

      SDFPerfectSimulatedSensorReader reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, null);
      ModularRobotController controller = new ModularRobotController("Reader");
      controller.setRawSensorReader(reader);

      scs = new SimulationConstructionSet(sdfRobot);
      scs.setDT(controlDT, 1);
      scs.addYoVariableRegistry(registry);

      String listName = getClass().getSimpleName();
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, centerOfMassPosition.createDynamicGraphicPosition("centerOfMass", 0.03, YoAppearance.Gold()));
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, centerOfMassPosition2d.createDynamicGraphicPosition("centerOfMass2d", 0.03, YoAppearance.Gold()));
      
      SideDependentList<AppearanceDefinition> appearance = new SideDependentList<>(YoAppearance.Red(), YoAppearance.Green());
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, anklePositions.get(robotSide).createDynamicGraphicPosition(sidePrefix + "AnkleViz", 0.05, appearance.get(robotSide)));
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, wristPositions.get(robotSide).createDynamicGraphicPosition(sidePrefix + "WristViz", 0.05, appearance.get(robotSide)));
         
         DynamicGraphicCoordinateSystem footCoordinateSystem = new DynamicGraphicCoordinateSystem(sidePrefix + "Foot", "", registry, 0.25);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, footCoordinateSystem);
         feetCoordinateSystems.put(robotSide, footCoordinateSystem);

         DynamicGraphicCoordinateSystem handCoordinateSystem = new DynamicGraphicCoordinateSystem(sidePrefix + "Hand", "", registry, 0.25);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, handCoordinateSystem);
         handCoordinateSystems.put(robotSide, handCoordinateSystem);
      }
      
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      SDFFullRobotModel fullRobotModelForSlider = loader.createFullRobotModel(jointMap);
      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs, sdfRobot, fullRobotModelForSlider, dynamicGraphicObjectsListRegistry);

      sliderBoard.addCaptureSnapshotListener(new CaptureSnapshotListener(fullRobotModel, controller));
      sliderBoard.addSaveSequenceRequestedListener(new SaveSequenceListener());
      sliderBoard.addLoadSequenceRequestedListener(new LoadSequenceFromFileListener(fullRobotModel));
      sliderBoard.addClearSequenceRequestedListener(new ClearSequenceListener());
      sliderBoard.addLoadFrameByFrameSequenceRequestedListener(new LoadFrameByFrameSequenceListener());
      sliderBoard.addLoadLastSequenceRequestedListener(new LoadLastSequenceListener());
      sliderBoard.addPlayPoseFromFrameByFrameSequenceRequestedListener(new PlayPoseFromFrameByFrameSequenceListener());
      sliderBoard.addResetToBasePoseRequestedListener(new ResetToBasePoseListener());

      poseSequenceEditorGUI = new PoseSequenceEditorGUI(registry, posePlaybackController, sdfRobot, fullRobotModelForSlider, sliderBoard);
      poseSequenceEditorGUI.setVisible(true);

      scs.startOnAThread();

      CenterOfMassGraphicUpdater centerOfMassGraphicUpdater = new CenterOfMassGraphicUpdater();
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
      private final FullRobotModel fullRobotModel;

      private final ModularRobotController controller;

      public CaptureSnapshotListener(SDFFullRobotModel fullRobotModel, ModularRobotController controller)
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
               double playBackDelayPoseTransition = (requestedPlaybackDelay == "") ? PosePlaybackAtlasDefaultParameters.defaultPlaybackTransitionDelayMillis
                     : Double.parseDouble(requestedPlaybackDelay);
               pose.setPlaybackDelayBeforePose(playBackDelayPoseTransition);
            }

            
            posePlaybackController.setPlaybackPose(morphedPose);
            scs.setTime(time);
            scs.tickAndUpdate();

            try
            {
               if (posePlaybackSender.isConnected())
                  posePlaybackSender.writeData();
               else
                  morphedPose.setRobotAtPose(sdfRobot);
               ThreadTools.sleep((long) (dt * 1000));
            }
            catch (IOException e)
            {
            }
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

            PalmPoseClassification classifcation = PalmPoseClassifier.getClassification(robotSide, handFrame, fullRobotModel.getChest().getBodyFixedFrame());
            palmPoseClassifications.get(robotSide).set(classifcation);

            //            PoseReferenceFrame toDisplay = PalmPoseClassifier.getPoseReferenceFrame(RobotSide.RIGHT, PalmPoseClassification.PALM_IN);
            //            handCoordinateSystems.get(robotSide).setToReferenceFrame(toDisplay);
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

         boolean selectedFileSuccessful = initPlaybackFromFile(fullRobotModel, new PosePlaybackRobotPoseSequence(fullRobotModel));
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
            PosePlaybackRobotPoseSequence.promptWriteToFile(posePlaybackRobotPoseSequence);
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
            posePlaybackController.setPlaybackPose(previousPose);
         }
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
      
      new PosePlaybackSCSBridge(model);
   }

   private boolean initPlaybackFromFile(FullRobotModel fullRobotModel, PosePlaybackRobotPoseSequence sequence)
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
      PosePlaybackRobotPoseSequence.appendFromFile(sequence, selectedFile);

      initPlayback(sequence);

      return successful;
   }

   private void initPlayback(PosePlaybackRobotPoseSequence sequence)
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

         posePlaybackController.setPlaybackPose(morphedPose);
         scs.setTime(frameByframeTime);
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

         if (interpolator.didLastPoseIncrementSequence() || (frameByframePoseNumber == 0))
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
