package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.File;
import java.io.IOException;

import javax.swing.JFileChooser;
import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTask;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTaskName;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class PosePlaybackSCSBridge
{
   private static final String ipAddress = DRCConfigParameters.CLOUD_MINION5_IP;

   private final PosePlaybackAllJointsController posePlaybackController;
   private final PosePlaybackSender posePlaybackSender;
   private PosePlaybackRobotPoseSequence posePlaybackRobotPoseSequence;

   private final PosePlaybackSmoothPoseInterpolator interpolator;
   private final YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");

//   private final BooleanYoVariable plotBalls = new BooleanYoVariable("plotBalls", registry);
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassPosition2d = new YoFramePoint("centerOfMass2d", ReferenceFrame.getWorldFrame(), registry);
   
//   private final BagOfBalls balls = new BagOfBalls(500, 0.01, YoAppearance.AliceBlue(), registry, dynamicGraphicObjectsListRegistry);
   
   public PosePlaybackSCSBridge() throws IOException
   {
      interpolator = new PosePlaybackSmoothPoseInterpolator(registry);

      posePlaybackController = new PosePlaybackAllJointsController(registry);
      posePlaybackSender = new PosePlaybackSender(posePlaybackController, ipAddress);
      posePlaybackRobotPoseSequence = new PosePlaybackRobotPoseSequence();

      VRCTask vrcTask = new VRCTask(VRCTaskName.ONLY_VEHICLE);
      SDFRobot sdfRobot = vrcTask.getRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.addYoVariableRegistry(registry);
      
      DynamicGraphicPosition centerOfMassViz = new DynamicGraphicPosition("centerOfMass", centerOfMassPosition, 0.03, YoAppearance.Gold());
      DynamicGraphicPosition centerOfMass2dViz = new DynamicGraphicPosition("centerOfMass2d", centerOfMassPosition2d, 0.03, YoAppearance.Gold());
      
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(getClass().getSimpleName(), centerOfMassViz);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(getClass().getSimpleName(), centerOfMass2dViz);
      
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs);

      CaptureSnapshotListener captureSnapshotListener = new CaptureSnapshotListener(sdfRobot, scs);
      sliderBoard.addCaptureSnapshotListener(captureSnapshotListener);

      SaveSequenceListener saveSequenceListener = new SaveSequenceListener();
      sliderBoard.addSaveSequenceRequestedListener(saveSequenceListener);

      LoadSequenceListener loadSequenceListener = new LoadSequenceListener(sdfRobot, scs);
      sliderBoard.addLoadSequenceRequestedListener(loadSequenceListener);

      sliderBoard.addClearSequenceRequestedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable yoVariable)
         {
            posePlaybackRobotPoseSequence.clear();
            System.out.println("Clearing Sequence");
         }
      });

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
         while(true)
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
      private final SimulationConstructionSet scs;
      private PosePlaybackRobotPose previousPose;

      public CaptureSnapshotListener(SDFRobot sdfRobot, SimulationConstructionSet scs)
      {
         this.sdfRobot = sdfRobot;
         this.scs = scs;
      }


      public void variableChanged(YoVariable yoVariable)
      {
         PosePlaybackRobotPose pose = new PosePlaybackRobotPose(sdfRobot);

         if (previousPose != null)
         {
            if (pose.epsilonEquals(previousPose, 1e-3))
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
         if(!((BooleanYoVariable) yoVariable).getBooleanValue())
            return;
         
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
         double dt = 0.005;

         interpolator.startSequencePlayback(sequence, startTime);
         int poseNumber = 0;

         while (!interpolator.isDone())
         {
            time = time + dt;

            PosePlaybackRobotPose morphedPose = interpolator.getPose(time);

            posePlaybackController.setPlaybackPose(morphedPose);
            scs.setTime(time);
            scs.tickAndUpdate();
            morphedPose.setRobotAtPose(sdfRobot);
            
            try
            {
               if (posePlaybackSender.isConnected())
               {
                  posePlaybackSender.writeData();
                  if(interpolator.didLastPoseIncrementSequence())
                  {
                     ThreadTools.sleep(3000);
                     System.out.println("pose #: " + poseNumber++);
                  }
               }
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
         if(((BooleanYoVariable) yoVariable).getBooleanValue())
         {
            System.out.println("saving file");
            posePlaybackRobotPoseSequence.promptWriteToFile();            
         }
      }
   }


   public static void main(String[] args) throws IOException
   {
      new PosePlaybackSCSBridge();
   }

}
