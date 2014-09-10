package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class LungingBasedDesiredCoPCMPVisualizer extends CapturabilityBasedDesiredCoPVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("LungingBasedDesiredCoPVisualizer");
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   
   private final YoFramePoint desiredCMP = new YoFramePoint("desiredCMP", "", world, registry);

   public LungingBasedDesiredCoPCMPVisualizer(YoVariableRegistry parentRegistry, YoGraphicsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(parentRegistry, dynamicGraphicObjectsListRegistry);
      
      YoGraphicsList dynamicGraphicObjectList = new YoGraphicsList("LungingBasedDesiredCoPVisualizer");
      ArtifactList artifactList = new ArtifactList("LungingBasedDesiredCoPVisualizer");
      if (dynamicGraphicObjectsListRegistry != null)
      {
         addDesiredCMPViz(dynamicGraphicObjectList, artifactList);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectList);
         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
      
      parentRegistry.addChild(registry);
   }

   private void addDesiredCMPViz(YoGraphicsList dynamicGraphicObjectList, ArtifactList artifactList)
   {
      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired Centriodal Moment Pivot", desiredCMP, 0.012, YoAppearance.White(),
            YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      dynamicGraphicObjectList.add(desiredCMPViz);
      artifactList.add(desiredCMPViz.createArtifact());
   }
   
   public void setDesiredCMP(FramePoint2d desiredCMP2d)
   {
      FramePoint desiredCMP = desiredCMP2d.toFramePoint();
      desiredCMP.changeFrame(world);
      this.desiredCMP.set(desiredCMP);
   }
}
