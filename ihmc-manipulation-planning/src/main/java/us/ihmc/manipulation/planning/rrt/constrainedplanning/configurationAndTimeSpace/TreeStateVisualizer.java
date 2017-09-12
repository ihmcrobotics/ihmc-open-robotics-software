package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
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

   private static Point3D pointCurrentTimeLineOrigin = new Point3D(0.0, 1.0, 1.0);
   private Point3D pointCurrentTime = new Point3D(0.0, 0.0, 1.0);
   private Point3D pointTreeReachingTime = new Point3D(0.0, 0.0, 1.0);

   private final YoFramePoint yoFramePointCurrentTimeLineOrigin;
   private final YoFramePoint yoFramePointCurrentTime;
   private final YoFramePoint yoFramePointCurrentTimeInvalid;
   private final YoFramePoint yoFramePointTreeReachingTime;

   /*
    * find sphere thing
    */
   private final YoGraphicCylinder currentTimeLineViz;
   private final YoGraphicPosition currentTimeViz;
   private final YoGraphicPosition currentTimeInvalidViz;
   private final YoGraphicPosition treeReachingTimeViz;
   
   public TreeStateVisualizer(String name, String graphicsListName, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("TreeStateVisualizerGraphicsList");

      /*
       * set currentTimeLine.
       */
      yoFramePointCurrentTimeLineOrigin = new YoFramePoint(name + "currentTimeOrigin", worldFrame, registry);
      yoFramePointCurrentTimeLineOrigin.setPoint(pointCurrentTimeLineOrigin);

      YoFrameVector currentTimeLineVector = new YoFrameVector("currentTimeLineVector", worldFrame, registry);
      currentTimeLineVector.setVector(new Vector3D(0.0, 1.0, 0.0));

      currentTimeLineViz = new YoGraphicCylinder("currentTimeLine", yoFramePointCurrentTimeLineOrigin, currentTimeLineVector, YoAppearance.LightBlue(), 0.03);

      yoGraphicsList.add(currentTimeLineViz);

      /*
       * set currentTime.
       */
      yoFramePointCurrentTime = new YoFramePoint(name + "currentTime", worldFrame, registry);
      pointCurrentTime.set(pointCurrentTimeLineOrigin);
      yoFramePointCurrentTime.setPoint(pointCurrentTime);
      
      yoFramePointCurrentTimeInvalid = new YoFramePoint(name + "currentTimeInvalid", worldFrame, registry);
      pointCurrentTime.set(pointCurrentTimeLineOrigin);
      yoFramePointCurrentTimeInvalid.setPoint(pointCurrentTime);

      currentTimeViz = new YoGraphicPosition("currentTime", yoFramePointCurrentTime, 0.06, YoAppearance.Blue(), GraphicType.BALL);
      yoGraphicsList.add(currentTimeViz);
      
      currentTimeInvalidViz = new YoGraphicPosition("currentTimeInvalid", yoFramePointCurrentTimeInvalid, 0.06, YoAppearance.Red(), GraphicType.BALL);
      yoGraphicsList.add(currentTimeInvalidViz);

      /*
       * set treeReachingTime.
       */
      yoFramePointTreeReachingTime = new YoFramePoint(name + "treeReachingTime", worldFrame, registry);
      pointCurrentTime.set(pointCurrentTimeLineOrigin);
      yoFramePointTreeReachingTime.setPoint(pointCurrentTime);

      treeReachingTimeViz = new YoGraphicPosition("treeReachingTime", yoFramePointTreeReachingTime, 0.05, YoAppearance.Black(), GraphicType.BALL);
      yoGraphicsList.add(treeReachingTimeViz);
      
      /*
       * register YoGraphicsList
       */
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   public void setCurrentNormalizedTime(double value)
   {
      currentNormalizedTime = value;
      if (currentNormalizedTime > treeReachingTime)
         treeReachingTime = value;
   }

   public void setCurrentCTTaskNodeValidity(boolean value)
   {
      currentCTTaskNodeValidity = value;
   }

   public void updateVisualizer()
   {
      if(currentCTTaskNodeValidity)
      {
         pointCurrentTime.set(pointCurrentTimeLineOrigin);
         pointCurrentTime.add(new Vector3D(0.0, currentNormalizedTime, 0.0));
         yoFramePointCurrentTime.setPoint(pointCurrentTime);
         
         pointCurrentTime.setToNaN();
         yoFramePointCurrentTimeInvalid.setPoint(pointCurrentTime);
      }
      else
      {
         pointCurrentTime.set(pointCurrentTimeLineOrigin);
         pointCurrentTime.add(new Vector3D(0.0, currentNormalizedTime, 0.0));
         yoFramePointCurrentTimeInvalid.setPoint(pointCurrentTime);
         
         pointCurrentTime.setToNaN();
         yoFramePointCurrentTime.setPoint(pointCurrentTime);  
      }
      
      pointTreeReachingTime.set(pointCurrentTimeLineOrigin);
      pointTreeReachingTime.add(new Vector3D(0.0, treeReachingTime, 0.0));
      yoFramePointTreeReachingTime.setPoint(pointTreeReachingTime);

      currentTimeInvalidViz.update();
      treeReachingTimeViz.update();
      currentTimeLineViz.update();
      currentTimeViz.update();
   }
}