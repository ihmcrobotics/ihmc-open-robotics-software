package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;


public class LungingBasedDesiredCoPCMPVisualizer extends CapturabilityBasedDesiredCoPVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("LungingBasedDesiredCoPVisualizer");
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   
   private final YoFramePoint desiredCMP = new YoFramePoint("desiredCMP", "", world, registry);

   public LungingBasedDesiredCoPCMPVisualizer(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(parentRegistry, yoGraphicsListRegistry);
      
      YoGraphicsList yoGraphicList = new YoGraphicsList("LungingBasedDesiredCoPVisualizer");
      ArtifactList artifactList = new ArtifactList("LungingBasedDesiredCoPVisualizer");
      if (yoGraphicsListRegistry != null)
      {
         addDesiredCMPViz(yoGraphicList, artifactList);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
      
      parentRegistry.addChild(registry);
   }

   private void addDesiredCMPViz(YoGraphicsList yoGraphicList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired Centriodal Moment Pivot", desiredCMP, 0.012, YoAppearance.White(),
            YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      yoGraphicList.add(desiredCMPViz);
      artifactList.add(desiredCMPViz.createArtifact());
   }
   
   public void setDesiredCMP(FramePoint2d desiredCMP2d)
   {
      FramePoint desiredCMP = desiredCMP2d.toFramePoint();
      desiredCMP.changeFrame(world);
      this.desiredCMP.set(desiredCMP);
   }
}
