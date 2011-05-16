package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.VisualizeFramesController;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ZUpFrame;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.RobotController;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;

public class DesiredFootstepVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredFootstepVisualizer");

   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final SideDependentList<FrameConvexPolygon2d> feetPolygonsInFootFrame = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<YoFrameConvexPolygon2d> feetPolygonsInWorld = new SideDependentList<YoFrameConvexPolygon2d>();
   private final SideDependentList<PoseReferenceFrame> footFrames;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;


   public DesiredFootstepVisualizer(DesiredFootstepCalculator desiredFootstepCalculator, SideDependentList<ConvexPolygon2d> feetPolygonsInFootFrame,
                                    YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<PoseReferenceFrame> footFrames, SideDependentList<ReferenceFrame> ankleZUpFrames)
   {
      this.desiredFootstepCalculator = desiredFootstepCalculator;

      this.footFrames = footFrames;
      this.ankleZUpFrames = ankleZUpFrames;

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FeetPolygons");
      ArtifactList artifactList = new ArtifactList("FeetPolygons");
      SideDependentList<Color> footColors = new SideDependentList<Color>(Color.pink, Color.blue);
      for (RobotSide robotSide : RobotSide.values())
      {
         FrameConvexPolygon2d frameFootPolygonInFootVisualizationFrame = new FrameConvexPolygon2d(ankleZUpFrames.get(robotSide), feetPolygonsInFootFrame.get(robotSide));
         this.feetPolygonsInFootFrame.put(robotSide, frameFootPolygonInFootVisualizationFrame);

         int maxNumberOfVertices = frameFootPolygonInFootVisualizationFrame.getNumberOfVertices();
         String footName = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";
         YoFrameConvexPolygon2d yoFrameFootPolygonInWorld = new YoFrameConvexPolygon2d(footName, "", ReferenceFrame.getWorldFrame(), maxNumberOfVertices,
                                                               registry);
         this.feetPolygonsInWorld.put(robotSide, yoFrameFootPolygonInWorld);

         DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact(footName, yoFrameFootPolygonInWorld,
                                                                              footColors.get(robotSide), false);
         artifactList.add(dynamicGraphicYoPolygonArtifact);
      }

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      parentRegistry.addChild(registry);
   }

   private static SimulationConstructionSet createSCSAndAttachVisualizer(YoVariableRegistry registryToAddToRobot,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, ArrayList<RobotController> robotControllers)
   {
      Robot robot = new Robot("DesiredFootstepVisualizerRobot");
      for (RobotController robotController : robotControllers)
      {
         robot.setController(robotController);
      }
      
      robot.getRobotsYoVariableRegistry().addChild(registryToAddToRobot);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

//    // Create and attach plotter as listener
      SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();
      simulationOverheadPlotter.setDrawHistory(false);

//    simulationOverheadPlotter.setXVariableToTrack(processedSensors.getYoBodyPositionInWorld().getYoX());
//    simulationOverheadPlotter.setYVariableToTrack(processedSensors.getYoBodyPositionInWorld().getYoY());

      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, "Plotter");
      JPanel simulationOverheadPlotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(simulationOverheadPlotterKeyJPanel);

      scs.addExtraJpanel(scrollPane, "Plotter Legend");

      dynamicGraphicObjectsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());


      Thread thread = new Thread(scs);
      thread.start();

      return scs;
   }

   public Footstep takeAndVisualizeAStep(RobotSide swingLegSide)
   {
      RobotSide supportLegSide = swingLegSide.getOppositeSide();

      desiredFootstepCalculator.initializeDesiredFootstep(supportLegSide);
      Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportLegSide);

//    System.out.println("desiredFootstep = " + desiredFootstep);

      FramePose pose = desiredFootstep.getFootstepPose();
      pose = pose.changeFrameCopy(ReferenceFrame.getWorldFrame());
      footFrames.get(swingLegSide).updatePose(pose);

      updateFrames();

      setFeetPolygonsInWorld();

      return desiredFootstep;
   }


   private void setFeetPolygonsInWorld()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         FrameConvexPolygon2d footPolygon = feetPolygonsInFootFrame.get(robotSide);
         FrameConvexPolygon2d footPolygonInWorld = footPolygon.changeFrameCopy(ReferenceFrame.getWorldFrame());

         feetPolygonsInWorld.get(robotSide).setFrameConvexPolygon2d(footPolygonInWorld);
      }
   }

   private void updateFrames()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         footFrames.get(robotSide).update();
         ankleZUpFrames.get(robotSide).update();
      }
   }

   public static void main(String[] args)
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      ArrayList<ReferenceFrame> framesToVisualize = new ArrayList<ReferenceFrame>();
      
      SideDependentList<PoseReferenceFrame> feetPoseReferenceFrames = new SideDependentList<PoseReferenceFrame>();
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>();
      for (RobotSide robotSide : RobotSide.values())
      {
         String robotSideName = robotSide.getCamelCaseNameForStartOfExpression();
         PoseReferenceFrame footFrame = new PoseReferenceFrame(robotSideName + "Foot", ReferenceFrame.getWorldFrame());
         feetPoseReferenceFrames.put(robotSide, footFrame);
         
         ZUpFrame ankleZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), footFrame, robotSideName + "AnkleZUp");
         ankleZUpFrames.put(robotSide, ankleZUpFrame);
         
         framesToVisualize.add(ankleZUpFrame);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         feetPoseReferenceFrames.get(robotSide).update();
         ankleZUpFrames.get(robotSide).update();
      }

      VisualizeFramesController visualizeFramesController = new VisualizeFramesController(framesToVisualize, dynamicGraphicObjectsListRegistry, 1.0);

      double desiredHeadingFinal = 0.0;
      double controlDT = 0.1;
      int ticksPerStep = 10;

      DesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(desiredHeadingFinal, controlDT, parentRegistry);

      ManualDesiredVelocityControlModule desiredVelocityControlModule = new ManualDesiredVelocityControlModule(parentRegistry);

      HeadingAndVelocityBasedDesiredFootstepCalculator desiredFootstepCalculator = new HeadingAndVelocityBasedDesiredFootstepCalculator(ankleZUpFrames,
                                                                                      desiredHeadingControlModule, desiredVelocityControlModule,
                                                                                      parentRegistry);
      desiredFootstepCalculator.setUpParametersForR2();

      double footWidth = 0.15;
      double footForward = 0.25;
      double footBackward = 0.05;

      double[][] leftPointList = new double[][]
      {
         {footForward, footWidth / 2.0}, {-footBackward, footWidth / 2.0}, {-footBackward, -footWidth / 2.0}, {footForward, -footWidth / 2.0}
      };
      double[][] rightPointList = new double[][]
      {
         {footForward, footWidth / 2.0}, {-footBackward, footWidth / 2.0}, {-footBackward, -footWidth / 2.0}, {footForward, -footWidth / 2.0}
      };

      ConvexPolygon2d leftFootInFootFrame = new ConvexPolygon2d(leftPointList);
      ConvexPolygon2d rightFootInFootFrame = new ConvexPolygon2d(rightPointList);

      SideDependentList<ConvexPolygon2d> feetPolygonsInFootFrame = new SideDependentList<ConvexPolygon2d>(leftFootInFootFrame, rightFootInFootFrame);
      HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(controlDT, desiredHeadingControlModule,
                                                                                 desiredVelocityControlModule, parentRegistry);


      DesiredFootstepVisualizer visualizer = new DesiredFootstepVisualizer(desiredFootstepCalculator, feetPolygonsInFootFrame, parentRegistry,
                                                dynamicGraphicObjectsListRegistry, feetPoseReferenceFrames, ankleZUpFrames);

      ArrayList<RobotController> robotControllers = new ArrayList<RobotController>();
      robotControllers.add(visualizeFramesController);
      SimulationConstructionSet scs = DesiredFootstepVisualizer.createSCSAndAttachVisualizer(parentRegistry, dynamicGraphicObjectsListRegistry, robotControllers);
      scs.setDT(controlDT, 1);

      RobotSide swingLegSide = RobotSide.LEFT;
      int numberOfSteps = 100;

      double time = scs.getTime();

      desiredHeadingControlModule.updateDesiredHeadingFrame();

      for (int i = 0; i < numberOfSteps; i++)
      {
         for (int j = 0; j < ticksPerStep; j++)
         {
            headingAndVelocityEvaluationScript.update(time);
            desiredHeadingControlModule.updateDesiredHeadingFrame();

            scs.setTime(time);
            boolean doSleep = false;
            if (doSleep)
               sleep(controlDT);
            time = time + controlDT;
            scs.doControl();
            scs.tickAndUpdate();

         }

         Footstep footstep = visualizer.takeAndVisualizeAStep(swingLegSide);

         PoseReferenceFrame footToMoveFrame = feetPoseReferenceFrames.get(swingLegSide);

         FramePose poseToMoveTo = footstep.getFootstepPoseCopy();
         poseToMoveTo.changeFrame(ReferenceFrame.getWorldFrame());

         footToMoveFrame.updatePose(poseToMoveTo);
         feetPoseReferenceFrames.get(swingLegSide).update();

         swingLegSide = swingLegSide.getOppositeSide();
      }
   }


   private static void sleep(double sleepSeconds)
   {
      try
      {
         Thread.sleep((long) (sleepSeconds * 1000));
      }
      catch (InterruptedException e)
      {
      }
   }
}
