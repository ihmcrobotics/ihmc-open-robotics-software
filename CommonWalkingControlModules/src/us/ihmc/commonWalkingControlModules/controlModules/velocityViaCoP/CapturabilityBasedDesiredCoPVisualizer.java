package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import java.awt.Color;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class CapturabilityBasedDesiredCoPVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("CapturabilityBasedDesiredCoPVisualizer");
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final YoFramePoint desiredCoP = new YoFramePoint("desiredCoP", "", world, registry);
   private final YoFramePoint desiredCapturePoint = new YoFramePoint("desiredCapturePoint", "", world, registry);
   private final YoFrameLineSegment2d guideLine = new YoFrameLineSegment2d("guideLine", "", world, registry);
   private final YoFramePoint desiredCMP = new YoFramePoint("desiredCMP", "", world, registry);
   private final YoFramePoint pseudoCMP = new YoFramePoint("pseudoCMP", "", world, registry);
   private final YoFramePoint finalDesiredCapturePoint = new YoFramePoint("finalDesiredCapturePoint", "", world, registry);
   private final YoFramePoint centerOfMass = new YoFramePoint("centerOfMass", world, registry);
   
   public CapturabilityBasedDesiredCoPVisualizer(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicList = new YoGraphicsList("CapturabilityBasedDesiredCoPVisualizer");
      ArtifactList artifactList = new ArtifactList("CapturabilityBasedDesiredCoPVisualizer");

      if (yoGraphicsListRegistry != null)
      {
         addFinalDesiredCapturePointViz(yoGraphicList, artifactList);
         addDesiredCoPViz(yoGraphicList, artifactList);
         addDesiredCapturePointViz(yoGraphicList, artifactList);
         addDesiredCMPViz(yoGraphicList, artifactList);
         addCenterOfMassViz(yoGraphicList, artifactList);
         addGuideLineViz(artifactList);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
      desiredCMP.setToNaN();
      pseudoCMP.setToNaN();
      centerOfMass.setToNaN();
      
      parentRegistry.addChild(registry);
   }

   private void addDesiredCoPViz(YoGraphicsList yoGraphicList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCoPViz = new YoGraphicPosition("Desired Center of Pressure", desiredCoP, 0.012, YoAppearance.Gray(),
                                                GraphicType.CROSS);
      yoGraphicList.add(desiredCoPViz);
      artifactList.add(desiredCoPViz.createArtifact());
   }

   private void addDesiredCMPViz(YoGraphicsList yoGraphicList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", desiredCMP, 0.012, YoAppearance.Red(),
                                                GraphicType.CROSS);
      yoGraphicList.add(desiredCMPViz);
      artifactList.add(desiredCMPViz.createArtifact());
      
      YoGraphicPosition pseudoCMPViz = new YoGraphicPosition("Pseudo CMP", pseudoCMP, 0.012, YoAppearance.Purple(),
            GraphicType.CROSS);
      yoGraphicList.add(pseudoCMPViz);
      artifactList.add(pseudoCMPViz.createArtifact());
   }

   private void addDesiredCapturePointViz(YoGraphicsList yoGraphicList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", desiredCapturePoint, 0.01, YoAppearance.Yellow(),
                                                         GraphicType.ROTATED_CROSS);
      yoGraphicList.add(desiredCapturePointViz);
      artifactList.add(desiredCapturePointViz.createArtifact());
   }

   private void addFinalDesiredCapturePointViz(YoGraphicsList yoGraphicList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", finalDesiredCapturePoint, 0.01, YoAppearance.Beige(),
                                                         GraphicType.ROTATED_CROSS);
      yoGraphicList.add(desiredCapturePointViz);
      artifactList.add(desiredCapturePointViz.createArtifact());
   }

   private void addGuideLineViz(ArtifactList artifactList)
   {
      YoArtifactLineSegment2d guideLineArtifact = new YoArtifactLineSegment2d("Guide Line", guideLine, Color.RED);
      artifactList.add(guideLineArtifact);
   }

   private void addCenterOfMassViz(YoGraphicsList yoGraphicList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Center Of Mass", centerOfMass, 0.006, YoAppearance.Black(), GraphicType.CROSS);
      yoGraphicList.add(desiredCapturePointViz);
      artifactList.add(desiredCapturePointViz.createArtifact());
   }

   public void setDesiredCoP(FramePoint2d desiredCoP2d)
   {
      FramePoint desiredCoP = desiredCoP2d.toFramePoint();
      desiredCoP.changeFrame(world);
      this.desiredCoP.set(desiredCoP);
   }

   private final FramePoint tempPoint = new FramePoint();
   
   public void setDesiredCapturePoint(FramePoint2d desiredCapturePoint2d)
   {
      tempPoint.setIncludingFrame(desiredCapturePoint2d.getReferenceFrame(), desiredCapturePoint2d.getX(), desiredCapturePoint2d.getY(), 0.0);
      tempPoint.changeFrame(world);
      this.desiredCapturePoint.set(tempPoint);
      hideGuideLine();
   }

   public void setFinalDesiredCapturePoint(FramePoint2d finalDesiredCapturePoint2d)
   {
      tempPoint.setIncludingFrame(finalDesiredCapturePoint2d.getReferenceFrame(), finalDesiredCapturePoint2d.getX(), finalDesiredCapturePoint2d.getY(), 0.0);
      tempPoint.changeFrame(world);
      this.finalDesiredCapturePoint.set(tempPoint);
      hideGuideLine();
   }

   public void setGuideLine(FrameLineSegment2d guideLine)
   {
      FrameLineSegment2d guideLineInWorld = new FrameLineSegment2d(guideLine);
      guideLineInWorld.changeFrame(world);
      this.guideLine.setFrameLineSegment2d(guideLineInWorld);
      hideDesiredCapturePoint();
   }

   private void hideDesiredCapturePoint()
   {
      desiredCapturePoint.setToNaN();
   }

   private void hideGuideLine()
   {
      guideLine.setFrameLineSegment2d(null);
   }

   public void setDesiredCMP(FramePoint2d desiredCMP)
   {
      desiredCMP.changeFrame(this.desiredCMP.getReferenceFrame());
      this.desiredCMP.set(desiredCMP.getX(), desiredCMP.getY(), 0.0);
   }

   public void setPseudoCMP(FramePoint pseudoCMP)
   {
      pseudoCMP.changeFrame(this.pseudoCMP.getReferenceFrame());
      this.pseudoCMP.set(pseudoCMP);
   }

   public void setCenterOfMass(FramePoint centerOfMass)
   {
      centerOfMass.changeFrame(this.centerOfMass.getReferenceFrame());
      this.centerOfMass.set(centerOfMass);
   }
}
