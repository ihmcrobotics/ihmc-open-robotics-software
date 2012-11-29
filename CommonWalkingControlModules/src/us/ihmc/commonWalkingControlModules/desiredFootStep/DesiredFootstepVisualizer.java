package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.Color;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleBipedFoot;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.VisualizeFramesController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ZUpFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DesiredFootstepVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredFootstepVisualizer");

   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final SideDependentList<YoFrameConvexPolygon2d> feetPolygonsInWorld = new SideDependentList<YoFrameConvexPolygon2d>();
   private final DoubleYoVariable minZ = new DoubleYoVariable("minZ", registry);
   private final SideDependentList<PoseReferenceFrame> footFrames;
   private final SideDependentList<ReferenceFrame> soleFrames;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final SideDependentList<BipedFootInterface> bipedFeet;


   public DesiredFootstepVisualizer(DesiredFootstepCalculator desiredFootstepCalculator, SideDependentList<BipedFootInterface> bipedFeet,
                                    YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<PoseReferenceFrame> footFrames, SideDependentList<ReferenceFrame> soleFrames,
                                    SideDependentList<ReferenceFrame> ankleZUpFrames)
   {
      this.desiredFootstepCalculator = desiredFootstepCalculator;

      this.footFrames = footFrames;
      this.soleFrames = soleFrames;
      this.ankleZUpFrames = ankleZUpFrames;
      this.bipedFeet = bipedFeet;

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FeetPolygons");
      ArtifactList artifactList = new ArtifactList("FeetPolygons");
      SideDependentList<Color> footColors = new SideDependentList<Color>(Color.pink, Color.blue);
      for (RobotSide robotSide : RobotSide.values())
      {
         int maxNumberOfVertices = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getNumberOfVertices();
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

      FramePose pose = desiredFootstep.getPose();
      pose = pose.changeFrameCopy(ReferenceFrame.getWorldFrame());
      footFrames.get(swingLegSide).updatePose(pose);

      updateFrames();

      setFeetPolygonsInWorld();

      computeMinZ(swingLegSide);

      return desiredFootstep;
   }

   private void hideSwingLeg(RobotSide swingLegSide)
   {
      feetPolygonsInWorld.get(swingLegSide).hide();
   }


   private void setFeetPolygonsInWorld()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         FrameConvexPolygon2d footPolygon = bipedFeet.get(robotSide).getFlatFootPolygonInAnkleZUp();
         FrameConvexPolygon2d footPolygonInWorld = footPolygon.changeFrameCopy(ReferenceFrame.getWorldFrame());

         feetPolygonsInWorld.get(robotSide).setFrameConvexPolygon2d(footPolygonInWorld);
      }
   }

   private void computeMinZ(RobotSide swingLegSide)
   {
      BipedFootInterface bipedFoot = bipedFeet.get(swingLegSide);
      Transform3D footToWorldTransform = footFrames.get(swingLegSide).getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      FramePoint minZPoint = DesiredFootstepCalculatorTools.computeMinZPointInFrame(footToWorldTransform, bipedFoot, ReferenceFrame.getWorldFrame());
      this.minZ.set(minZPoint.getZ());
   }

   private void updateFrames()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         footFrames.get(robotSide).update();
         soleFrames.get(robotSide).update();
         ankleZUpFrames.get(robotSide).update();
      }
   }

   public static void main(String[] args)
   {
      double HEADING_VIZ_Z = 0.03;
      double VELOCITY_VIZ_Z = 0.06;

      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      BagOfBalls leftBagOfBalls = new BagOfBalls(1000, 0.05, "leftBalls", YoAppearance.Red(), parentRegistry, dynamicGraphicObjectsListRegistry);
      BagOfBalls rightBagOfBalls = new BagOfBalls(1000, 0.05, "rightBalls", YoAppearance.Blue(), parentRegistry, dynamicGraphicObjectsListRegistry);
      SideDependentList<BagOfBalls> bagsOfBalls = new SideDependentList<BagOfBalls>(leftBagOfBalls, rightBagOfBalls);


      // Visualizers for the HeadingAndVelocityEvaluationScript:
      YoFramePoint position = new YoFramePoint("position", "", ReferenceFrame.getWorldFrame(), parentRegistry);
      YoFrameVector velocity = new YoFrameVector("velocity", "", ReferenceFrame.getWorldFrame(), parentRegistry);
      YoFrameVector heading = new YoFrameVector("heading", "", ReferenceFrame.getWorldFrame(), parentRegistry);

      DynamicGraphicVector velocityVector = new DynamicGraphicVector("velocity", position, velocity, YoAppearance.Yellow());
      DynamicGraphicVector headingVector = new DynamicGraphicVector("heading", position, heading, YoAppearance.Blue());

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("velocityVector", velocityVector);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("headingVector", headingVector);

      BagOfBalls bagOfBalls = new BagOfBalls(1200, 0.03, YoAppearance.Red(), parentRegistry, dynamicGraphicObjectsListRegistry);




      ArrayList<ReferenceFrame> framesToVisualize = new ArrayList<ReferenceFrame>();

      SideDependentList<PoseReferenceFrame> feetPoseReferenceFrames = new SideDependentList<PoseReferenceFrame>();
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>();
      SideDependentList<BipedFootInterface> bipedFeet = new SideDependentList<BipedFootInterface>();

      double footWidth = 0.15;
      double footForward = 0.25;
      double footBackward = 0.05;
      double footHeight = 0.05;
      for (RobotSide robotSide : RobotSide.values())
      {
         String robotSideName = robotSide.getCamelCaseNameForStartOfExpression();
         PoseReferenceFrame footFrame = new PoseReferenceFrame(robotSideName + "Foot", ReferenceFrame.getWorldFrame());
         feetPoseReferenceFrames.put(robotSide, footFrame);

         ReferenceFrame soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(robotSideName + "Sole", footFrame,
                                       new Vector3d(0.0, 0.0, -footHeight));
         soleFrames.put(robotSide, soleFrame);

         ZUpFrame ankleZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), footFrame, robotSideName + "AnkleZUp");
         ankleZUpFrames.put(robotSide, ankleZUpFrame);

         BipedFootInterface foot = new SimpleBipedFoot(footFrame, ankleZUpFrame, soleFrame, robotSide, footForward, footBackward, footWidth / 2.0,
                                      footWidth / 2.0, parentRegistry);
         bipedFeet.put(robotSide, foot);
         framesToVisualize.add(footFrame);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         feetPoseReferenceFrames.get(robotSide).update();
         ankleZUpFrames.get(robotSide).update();
      }

      VisualizeFramesController visualizeFramesController = new VisualizeFramesController(framesToVisualize, dynamicGraphicObjectsListRegistry, 1.0);

      double desiredHeadingFinal = 0.0;
      double controlDT = 0.1;
      int ticksPerStep = 7;
      int ticksPerDoubleSupport = 2;

      SimpleDesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(desiredHeadingFinal, controlDT, parentRegistry);

      ManualDesiredVelocityControlModule desiredVelocityControlModule = new ManualDesiredVelocityControlModule(parentRegistry);

//    SimpleDesiredFootstepCalculator desiredFootstepCalculator = new SimpleDesiredFootstepCalculator(ankleZUpFrames,
//                                                                                    desiredHeadingControlModule,
//                                                                                    parentRegistry);


      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator = new ComponentBasedDesiredFootstepCalculator(ankleZUpFrames, bipedFeet,
                                                                             desiredHeadingControlModule, desiredVelocityControlModule, parentRegistry);
      desiredFootstepCalculator.setInPlaceWidth(0.4);
      desiredFootstepCalculator.setWalkingForwardWidth(0.25);

      desiredFootstepCalculator.setMaxStepLength(0.6);

      desiredFootstepCalculator.setSidestepMaxWidth(0.4);
      desiredFootstepCalculator.setSidestepMinWidth(0.15);

      desiredFootstepCalculator.setMinStepWidth(0.25);
      desiredFootstepCalculator.setMaxStepWidth(0.5);

      desiredFootstepCalculator.setStepPitch(0.0); // -0.25);

//    HeadingAndVelocityBasedDesiredFootstepCalculator desiredFootstepCalculator = new HeadingAndVelocityBasedDesiredFootstepCalculator(ankleZUpFrames,
//          desiredHeadingControlModule, desiredVelocityControlModule,
//          parentRegistry);


      boolean cycleThroughAllEvents = true;
      HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(cycleThroughAllEvents, controlDT,
                                                                                 desiredHeadingControlModule, desiredVelocityControlModule, parentRegistry);


      DesiredFootstepVisualizer visualizer = new DesiredFootstepVisualizer(desiredFootstepCalculator, bipedFeet, parentRegistry,
                                                dynamicGraphicObjectsListRegistry, feetPoseReferenceFrames, soleFrames, ankleZUpFrames);

      ArrayList<RobotController> robotControllers = new ArrayList<RobotController>();
      robotControllers.add(visualizeFramesController);
      SimulationConstructionSet scs = DesiredFootstepVisualizer.createSCSAndAttachVisualizer(parentRegistry, dynamicGraphicObjectsListRegistry,
                                         robotControllers);
      scs.setDT(controlDT, 1);

      RobotSide swingLegSide = RobotSide.LEFT;
      int numberOfSteps = 100;

      double time = scs.getTime();

      desiredHeadingControlModule.updateDesiredHeadingFrame();

      for (int i = 0; i < numberOfSteps; i++)
      {
         visualizer.hideSwingLeg(swingLegSide);

         for (int j = 0; j < ticksPerStep; j++)
         {
            headingAndVelocityEvaluationScript.update(time);

//          desiredHeadingControlModule.updateDesiredHeadingFrame();

            FrameVector2d desiredHeading = desiredHeadingControlModule.getDesiredHeading();
            FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();

            heading.set(desiredHeading.getX(), desiredHeading.getY(), HEADING_VIZ_Z);
            velocity.set(desiredVelocity.getX(), desiredVelocity.getY(), VELOCITY_VIZ_Z);

            position.add(desiredVelocity.getX() * controlDT, desiredVelocity.getY() * controlDT, 0.0);

            FramePoint location = new FramePoint(ReferenceFrame.getWorldFrame());
            location.set(position.getX(), position.getY(), 0.0);

            bagOfBalls.setBall(location);


            scs.setTime(time);
            boolean doSleep = false;
            if (doSleep)
               sleep(controlDT);
            time = time + controlDT;
            scs.doControl();
            scs.tickAndUpdate();

         }

         Footstep footstep = visualizer.takeAndVisualizeAStep(swingLegSide);

         bagsOfBalls.get(swingLegSide).setBall(footstep.getPositionInFrame(ReferenceFrame.getWorldFrame()));

         PoseReferenceFrame footToMoveFrame = feetPoseReferenceFrames.get(swingLegSide);

         FramePose poseToMoveTo = new FramePose(ReferenceFrame.getWorldFrame());
         footstep.getPose(poseToMoveTo);
         poseToMoveTo.changeFrame(ReferenceFrame.getWorldFrame());

         footToMoveFrame.updatePose(poseToMoveTo);
         feetPoseReferenceFrames.get(swingLegSide).update();


         for (int j = 0; j < ticksPerDoubleSupport; j++)
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
