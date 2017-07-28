package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/*
 * This visualizer show state of the CTTaskNodeTree.
 * It shows the time reaching.
 * And the current new node validity is also shown by color (Green and Red). 
 */

public class TreeStateVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private double currentNormalizedTime;   
   private double treeReachingTime = 0.0;
   
   private boolean currentCTTaskNodeValidity;
   
   private final YoFramePose yoCurrentStartTime;
   private final YoFramePose yoCurrentEndTime;
   private final YoFramePose yoCurrentTime;
   
   private final YoFramePose yoTreeStartTime;
   private final YoFramePose yoTreeEndTime;   
   private final YoFramePose yoTreeTime;
   
   /*
    * find sphere thing
    */
   private final YoGraphicEllipsoid currentStartTimeViz;
   
   public TreeStateVisualizer(String name, String graphicsListName, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      yoCurrentStartTime = new YoFramePose(name + "currentStartTime", worldFrame, registry);
      yoCurrentEndTime = new YoFramePose(name + "currentEndTime", worldFrame, registry);
      yoCurrentTime = new YoFramePose(name + "currentTime", worldFrame, registry);
      
      yoTreeStartTime = new YoFramePose(name + "treeStartTime", worldFrame, registry);
      yoTreeEndTime = new YoFramePose(name + "treeEndTime", worldFrame, registry);
      yoTreeTime = new YoFramePose(name + "treeTime", worldFrame, registry);
      
      
      FramePose currentStartTime = new FramePose(worldFrame, new Pose3D(new Point3D(1.0, 0.0, 0.0), new Quaternion()));
      yoCurrentStartTime.setAndMatchFrame(currentStartTime);
      
      currentStartTimeViz = new YoGraphicEllipsoid("currentTime", yoCurrentStartTime.getPosition(), yoCurrentStartTime.getOrientation(), YoAppearance.LightBlue(), new Vector3D());
      currentStartTimeViz.setRadii(new Vector3D(1.0, 1.0, 1.0));
      
      yoGraphicsListRegistry.registerYoGraphic(graphicsListName, currentStartTimeViz);
   }
   
   public void setCurrentNormalizedTime(double value)
   {
      currentNormalizedTime = value;
      if(currentNormalizedTime > treeReachingTime)
         treeReachingTime = value;
   }
   
   public void setCurrentCTTaskNodeValidity(boolean value)
   {
      currentCTTaskNodeValidity = value;
   }
   
   public void updateVisualizer()
   {
      currentStartTimeViz.update();
   }
}
