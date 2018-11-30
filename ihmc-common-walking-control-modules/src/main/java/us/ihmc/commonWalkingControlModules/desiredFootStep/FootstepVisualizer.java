package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class FootstepVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int maxNumberOfContactPoints = 6;

   private final YoFramePose3D yoFootstepPose;
   private final YoFrameConvexPolygon2D yoFoothold;

   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   private final RobotSide robotSide;
   private final List<Point2D> defaultContactPointsInSoleFrame = new ArrayList<>();

   private final YoGraphicCoordinateSystem poseViz;
   private final YoGraphicPolygon footholdViz;

   public FootstepVisualizer(String name, String graphicListName, RobotSide robotSide, ContactablePlaneBody contactableFoot, AppearanceDefinition footstepColor,
                             YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this(name, graphicListName, robotSide, contactableFoot.getContactPoints2d(), footstepColor, yoGraphicsListRegistry, registry);
   }

   public FootstepVisualizer(String name, String graphicListName, RobotSide robotSide, List<? extends Point2DReadOnly> footPolygon,
                             AppearanceDefinition footstepColor, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.robotSide = robotSide;
      yoFootstepPose = new YoFramePose3D(name + "Pose", worldFrame, registry);
      yoFoothold = new YoFrameConvexPolygon2D(name + "Foothold", "", worldFrame, maxNumberOfContactPoints, registry);

      double coordinateSystemSize = 0.2;
      double footholdScale = 1.0;
      poseViz = new YoGraphicCoordinateSystem(name + "Pose", yoFootstepPose, coordinateSystemSize, footstepColor);
      footholdViz = new YoGraphicPolygon(name + "Foothold", yoFoothold, yoFootstepPose, footholdScale, footstepColor);
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
}
