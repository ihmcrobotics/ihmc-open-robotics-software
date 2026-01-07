package us.ihmc.manipulation.planning.rrt.configurationAndTimeSpace;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;

/*
 * This visualizer show state of the CTTaskNodeTree.
 * It shows the time reaching.
 * And the current new node validity is also shown by color (Green and Red). 
 */

public class TreeStateVisualizer implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private double currentNormalizedTime;
   private double treeReachingTime = 0.0;

   private boolean currentCTTaskNodeValidity;

   private static Point3D pointCurrentTimeLineOrigin = new Point3D(0.0, 1.0, 1.0);
   private Point3D pointCurrentTime = new Point3D(0.0, 0.0, 1.0);
   private Point3D pointTreeReachingTime = new Point3D(0.0, 0.0, 1.0);

   private final YoFramePoint3D yoFramePointCurrentTime;
   private final YoFramePoint3D yoFramePointCurrentTimeInvalid;
   private final YoFramePoint3D yoFramePointTreeReachingTime;


   private final YoGraphicGroupDefinition graphicGroupDefinition = new YoGraphicGroupDefinition(getClass().getSimpleName());
   
   public TreeStateVisualizer(String name, YoRegistry registry)
   {



      /*
       * set currentTime.
       */
      yoFramePointCurrentTime = new YoFramePoint3D(name + "currentTime", worldFrame, registry);
      pointCurrentTime.set(pointCurrentTimeLineOrigin);
      yoFramePointCurrentTime.set(pointCurrentTime);
      
      yoFramePointCurrentTimeInvalid = new YoFramePoint3D(name + "currentTimeInvalid", worldFrame, registry);
      pointCurrentTime.set(pointCurrentTimeLineOrigin);
      yoFramePointCurrentTimeInvalid.set(pointCurrentTime);

      graphicGroupDefinition.addChild(newYoGraphicPoint3D("currentTime", yoFramePointCurrentTime, 0.12, ColorDefinitions.Blue()));

      graphicGroupDefinition.addChild(newYoGraphicPoint3D("currentTimeInvalid", yoFramePointCurrentTimeInvalid, 0.12, ColorDefinitions.Red()));

      /*
       * set treeReachingTime.
       */
      yoFramePointTreeReachingTime = new YoFramePoint3D(name + "treeReachingTime", worldFrame, registry);
      pointCurrentTime.set(pointCurrentTimeLineOrigin);
      yoFramePointTreeReachingTime.set(pointCurrentTime);

      graphicGroupDefinition.addChild(newYoGraphicPoint3D("treeReachingTime", yoFramePointTreeReachingTime, 0.1, ColorDefinitions.Black()));
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
         yoFramePointCurrentTime.set(pointCurrentTime);
         
         pointCurrentTime.setToNaN();
         yoFramePointCurrentTimeInvalid.set(pointCurrentTime);
      }
      else
      {
         pointCurrentTime.set(pointCurrentTimeLineOrigin);
         pointCurrentTime.add(new Vector3D(0.0, currentNormalizedTime, 0.0));
         yoFramePointCurrentTimeInvalid.set(pointCurrentTime);
         
         pointCurrentTime.setToNaN();
         yoFramePointCurrentTime.set(pointCurrentTime);  
      }
      
      pointTreeReachingTime.set(pointCurrentTimeLineOrigin);
      pointTreeReachingTime.add(new Vector3D(0.0, treeReachingTime, 0.0));
      yoFramePointTreeReachingTime.set(pointTreeReachingTime);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return graphicGroupDefinition;
   }
}