package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.Color;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.Robot;
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
   private final SideDependentList<YoFrameConvexPolygon2d> feetPolygonsInWorld;
   private final SideDependentList<FrameConvexPolygon2d> feetPolygonsInFootFrame;
   private final SideDependentList<PoseReferenceFrame> footPoseFramesForVisualizing;
   
   
   public DesiredFootstepVisualizer(DesiredFootstepCalculator desiredFootstepCalculator, SideDependentList<ConvexPolygon2d> feetPolygonsInFootFrame, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.desiredFootstepCalculator = desiredFootstepCalculator;
 
      PoseReferenceFrame leftFootPoseFramesForVisualizing = new PoseReferenceFrame("leftFootPose", ReferenceFrame.getWorldFrame()) ;
      PoseReferenceFrame rightFootPoseFramesForVisualizing = new PoseReferenceFrame("rightFootPose", ReferenceFrame.getWorldFrame()) ;
      footPoseFramesForVisualizing = new SideDependentList<PoseReferenceFrame>(leftFootPoseFramesForVisualizing, rightFootPoseFramesForVisualizing);
      
      FrameConvexPolygon2d leftFootPolygonInFoot = new FrameConvexPolygon2d(footPoseFramesForVisualizing.get(RobotSide.LEFT), feetPolygonsInFootFrame.get(RobotSide.LEFT));
      FrameConvexPolygon2d rightFootPolygonInFoot = new FrameConvexPolygon2d(footPoseFramesForVisualizing.get(RobotSide.RIGHT), feetPolygonsInFootFrame.get(RobotSide.RIGHT));
      this.feetPolygonsInFootFrame = new SideDependentList<FrameConvexPolygon2d>(leftFootPolygonInFoot, rightFootPolygonInFoot);
            
      int maxNumberOfVertices = feetPolygonsInFootFrame.get(RobotSide.LEFT).getNumberOfVertices();
      
      YoFrameConvexPolygon2d leftFootPolygonInWorld = new YoFrameConvexPolygon2d("leftFoot", "", ReferenceFrame.getWorldFrame(), maxNumberOfVertices, registry);
      YoFrameConvexPolygon2d rightFootPolygonInWorld = new YoFrameConvexPolygon2d("rightFoot", "", ReferenceFrame.getWorldFrame(), maxNumberOfVertices, registry);
  
      feetPolygonsInWorld = new SideDependentList<YoFrameConvexPolygon2d>(leftFootPolygonInWorld, rightFootPolygonInWorld);
   

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("FeetPolygons");
      ArtifactList artifactList = new ArtifactList("FeetPolygons");

      DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact("leftFoot", leftFootPolygonInWorld, Color.pink, false);
      artifactList.add(dynamicGraphicYoPolygonArtifact);

      dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact("rightFoot", rightFootPolygonInWorld, Color.blue, false);
      artifactList.add(dynamicGraphicYoPolygonArtifact);      
      
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);   
      
      parentRegistry.addChild(registry);
   }
   
   private static SimulationConstructionSet createSCSAndAttachVisualizer(YoVariableRegistry registryToAddToRobot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      Robot robot = new Robot("Robot");
      robot.getRobotsYoVariableRegistry().addChild(registryToAddToRobot);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
   
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
      
//    // Create and attach plotter as listener
      SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();
      simulationOverheadPlotter.setDrawHistory(false);
//      simulationOverheadPlotter.setXVariableToTrack(processedSensors.getYoBodyPositionInWorld().getYoX());
//      simulationOverheadPlotter.setYVariableToTrack(processedSensors.getYoBodyPositionInWorld().getYoY());

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
   
//      System.out.println("desiredFootstep = " + desiredFootstep);
      
      FramePose pose = desiredFootstep.getFootstepPose();      
      pose = pose.changeFrameCopy(ReferenceFrame.getWorldFrame());
      
//      System.out.println("pose = " + pose);
      
      Orientation orientation = pose.getOrientation();
      double[] yawPitchRoll = orientation.getYawPitchRoll();
      yawPitchRoll[1] = 0.0;
      yawPitchRoll[2] = 0.0;
      orientation.setYawPitchRoll(yawPitchRoll);
      pose.setOrientation(orientation);
      
      footPoseFramesForVisualizing.get(swingLegSide).updatePose(pose);
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
         footPoseFramesForVisualizing.get(robotSide).update();
      }
   }
  
   public static void main(String[] args)
   {
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");

      PoseReferenceFrame leftFootReferenceFrame = new PoseReferenceFrame("leftFootReferenceFrame", ReferenceFrame.getWorldFrame());
      PoseReferenceFrame rightFootReferenceFrame = new PoseReferenceFrame("rightFootReferenceFrame", ReferenceFrame.getWorldFrame());
      SideDependentList<PoseReferenceFrame> feetPoseReferenceFrames = new SideDependentList<PoseReferenceFrame>(leftFootReferenceFrame, rightFootReferenceFrame);
     
      SideDependentList<ReferenceFrame> feetReferenceFrames = new SideDependentList<ReferenceFrame>(leftFootReferenceFrame, rightFootReferenceFrame);
//      DesiredFootstepCalculatorForTesting desiredFootstepCalculator = new DesiredFootstepCalculatorForTesting(feetReferenceFrames);
      
      
      double desiredHeadingFinal = 0.0;
      double controlDT = 4.0;
      DesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(desiredHeadingFinal, controlDT, parentRegistry);
      
      
      SimpleDesiredFootstepCalculator desiredFootstepCalculator = new SimpleDesiredFootstepCalculator(feetReferenceFrames, desiredHeadingControlModule, parentRegistry);
//      desiredFootstepCalculator();
      
//      AdjustableDesiredFootstepCalculator desiredFootstepCalculator = new AdjustableDesiredFootstepCalculator(feetReferenceFrames, desiredHeadingControlModule, parentRegistry);
//      desiredFootstepCalculator.setupParametersForR2();
      
      
      double footWidth = 0.15;
      double footForward = 0.25;
      double footBackward = 0.05;
      
      double[][] leftPointList = new double[][]{{footForward, footWidth/2.0}, {-footBackward, footWidth/2.0}, {-footBackward, -footWidth/2.0}, {footForward, -footWidth/2.0}};
      double[][] rightPointList = new double[][]{{footForward, footWidth/2.0}, {-footBackward, footWidth/2.0}, {-footBackward, -footWidth/2.0}, {footForward, -footWidth/2.0}};
      
      ConvexPolygon2d leftFootInFootFrame = new ConvexPolygon2d(leftPointList);
      ConvexPolygon2d rightFootInFootFrame = new ConvexPolygon2d(rightPointList);
      
      SideDependentList<ConvexPolygon2d> feetPolygonsInFootFrame = new SideDependentList<ConvexPolygon2d>(leftFootInFootFrame, rightFootInFootFrame);
      
      
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      
      DesiredFootstepVisualizer visualizer = new DesiredFootstepVisualizer(desiredFootstepCalculator, feetPolygonsInFootFrame, parentRegistry, dynamicGraphicObjectsListRegistry);
      
 
      SimulationConstructionSet scs = DesiredFootstepVisualizer.createSCSAndAttachVisualizer(parentRegistry, dynamicGraphicObjectsListRegistry);
      scs.setDT(controlDT, 1);
      
      RobotSide swingLegSide = RobotSide.LEFT;
      int numberOfSteps = 20000;
      
      for (int i=0; i<numberOfSteps; i++)
      {
         desiredHeadingControlModule.updateDesiredHeadingFrame();
         Footstep footstep = visualizer.takeAndVisualizeAStep(swingLegSide);
         scs.tickAndUpdate();
         
         sleep(controlDT);
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
