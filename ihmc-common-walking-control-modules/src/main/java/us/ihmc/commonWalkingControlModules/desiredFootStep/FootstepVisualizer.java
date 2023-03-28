package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootstepVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePose3D yoFootstepPose;
   private final YoFrameConvexPolygon2D yoFoothold;

   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   private final RobotSide robotSide;
   private final List<Point2D> defaultContactPointsInSoleFrame = new ArrayList<>();

   private final YoGraphicCoordinateSystem poseViz;
   private final YoGraphicPolygon footholdViz;

   private final String name;

   private final Color footstepColor;

   public FootstepVisualizer(String name,
                             String graphicListName,
                             RobotSide robotSide,
                             ContactablePlaneBody contactableFoot,
                             Color footstepColor,
                             YoGraphicsListRegistry yoGraphicsListRegistry,
                             YoRegistry registry)
   {
      this(name, graphicListName, robotSide, contactableFoot.getContactPoints2d(), footstepColor, yoGraphicsListRegistry, registry);
   }

   public FootstepVisualizer(String name,
                             String graphicListName,
                             RobotSide robotSide,
                             List<? extends Point2DReadOnly> footPolygon,
                             Color footstepColor,
                             YoGraphicsListRegistry yoGraphicsListRegistry,
                             YoRegistry registry)
   {
      this.name = name;
      this.robotSide = robotSide;
      this.footstepColor = footstepColor;
      yoFootstepPose = new YoFramePose3D(name + "Pose", worldFrame, registry);
      yoFoothold = new YoFrameConvexPolygon2D(name + "Foothold", "", worldFrame, footPolygon.size(), registry);

      double coordinateSystemSize = 0.2;
      double footholdScale = 1.0;
      AppearanceDefinition footstepApp = new YoAppearanceRGBColor(footstepColor, 0.0);
      poseViz = new YoGraphicCoordinateSystem(name + "Pose", yoFootstepPose, coordinateSystemSize, footstepApp);
      footholdViz = new YoGraphicPolygon(name + "Foothold", yoFoothold, yoFootstepPose, footholdScale, footstepApp);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, poseViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, footholdViz);

      for (int i = 0; i < footPolygon.size(); i++)
         defaultContactPointsInSoleFrame.add(new Point2D(footPolygon.get(i)));
   }

   public void update(Footstep footstep)
   {
      FramePose3DReadOnly footstepPose = footstep.getFootstepPose();
      List<? extends Point2DReadOnly> predictedContactPoints = footstep.getPredictedContactPoints();
      update(footstepPose, predictedContactPoints);
   }

   public void update(FramePose3DReadOnly footstepPose)
   {
      update(footstepPose, null);
   }

   public void update(FramePose3DReadOnly footstepPose, List<? extends Point2DReadOnly> predictedContactPoints)
   {
      yoFootstepPose.setMatchingFrame(footstepPose.getPosition(), footstepPose.getOrientation());

      List<? extends Point2DReadOnly> contactPointsToVisualize;
      if (predictedContactPoints == null || predictedContactPoints.isEmpty())
         contactPointsToVisualize = defaultContactPointsInSoleFrame;
      else
         contactPointsToVisualize = predictedContactPoints;

      foothold.clear();
      for (int i = 0; i < contactPointsToVisualize.size(); i++)
         foothold.addVertex(contactPointsToVisualize.get(i));
      foothold.update();

      yoFoothold.set(foothold);

      poseViz.update();
      footholdViz.update();
   }

   public void hide()
   {
      yoFootstepPose.setToNaN();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public static List<Point2D> createRectangularFootPolygon(double footWidth, double footLength)
   {
      return createTrapezoidalFootPolygon(footWidth, footWidth, footLength);
   }

   public static List<Point2D> createTrapezoidalFootPolygon(double toeWidth, double heelWidth, double footLength)
   {
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(-footLength / 2.0, -heelWidth / 2.0));
      contactPoints.add(new Point2D(-footLength / 2.0, heelWidth / 2.0));
      contactPoints.add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
      contactPoints.add(new Point2D(footLength / 2.0, toeWidth / 2.0));
      return contactPoints;
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicListDefinition list = new YoGraphicListDefinition();
      ColorDefinition color = ColorDefinitions.argb(footstepColor.getRGB());
      list.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name + "Pose", yoFootstepPose, 0.2, color));
      list.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicPolygonExtruded3DDefinition(name + "Foothold", yoFootstepPose, yoFoothold, 0.01, color));
      return list;
   }
}
