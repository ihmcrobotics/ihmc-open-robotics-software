package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SingleFootstepVisualizer
{
   private static final SideDependentList<AppearanceDefinition> footPolygonAppearances = new SideDependentList<AppearanceDefinition>(YoAppearance.Purple(),
         YoAppearance.Green());
   
   private static SideDependentList<Integer> indices = new SideDependentList<Integer>(0, 0);

   private final YoFramePose soleFramePose;
   private final YoFramePoint[] yoContactPoints;
   private final YoFrameConvexPolygon2d footPolygon;
   private final YoGraphicPolygon footPolygonViz;
   private final RobotSide robotSide;

   public SingleFootstepVisualizer(RobotSide robotSide, int maxContactPoints, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Integer index = indices.get(robotSide);
      String namePrefix = robotSide.getLowerCaseName() + "Foot" + index;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix);
      this.robotSide = robotSide;

      ArrayList<Point2D> polyPoints = new ArrayList<Point2D>();
      yoContactPoints = new YoFramePoint[maxContactPoints];

      for (int i = 0; i < maxContactPoints; i++)
      {
         yoContactPoints[i] = new YoFramePoint(namePrefix + "ContactPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoContactPoints[i].set(0.0, 0.0, -1.0);

         YoGraphicPosition baseControlPointViz = new YoGraphicPosition(namePrefix + "Point" + i, yoContactPoints[i], 0.01, YoAppearance.Blue());
         yoGraphicsList.add(baseControlPointViz);

         polyPoints.add(new Point2D());
      }

      footPolygon = new YoFrameConvexPolygon2d(namePrefix + "yoPolygon", "", ReferenceFrame.getWorldFrame(), maxContactPoints, registry);
      footPolygon.setConvexPolygon2d(new ConvexPolygon2d(polyPoints));

      soleFramePose = new YoFramePose(namePrefix + "polygonPose", "", ReferenceFrame.getWorldFrame(), registry);
      soleFramePose.setXYZ(0.0, 0.0, -1.0);

      footPolygonViz = new YoGraphicPolygon(namePrefix + "graphicPolygon", footPolygon, soleFramePose, 1.0, footPolygonAppearances.get(robotSide));

      yoGraphicsList.add(footPolygonViz);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(footPolygonViz);
      }

      index++;
      indices.set(robotSide, index);
   }

   public void visualizeFootstep(Footstep footstep, ContactablePlaneBody bipedFoot)
   {
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      
      if (robotSide != footstep.getRobotSide())
         throw new RuntimeException("Wrong Robot Side!");

      if ((predictedContactPoints == null) || (predictedContactPoints.isEmpty()))
      {
         predictedContactPoints = new ArrayList<Point2D>();

         List<FramePoint2d> contactPointsFromContactablePlaneBody = bipedFoot.getContactPoints2d();
         for (int i=0; i<contactPointsFromContactablePlaneBody.size(); i++)
         {
            FramePoint2d point = contactPointsFromContactablePlaneBody.get(i);
            predictedContactPoints.add(point.getPointCopy());
         }
      }
      
      ReferenceFrame soleReferenceFrame = footstep.getSoleReferenceFrame();
      double increaseZSlightlyToSeeBetter = 0.001;
      FramePose soleFramePose = new FramePose(soleReferenceFrame, new Point3D(0.0, 0.0, increaseZSlightlyToSeeBetter), new AxisAngle());
      soleFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      
      this.soleFramePose.set(soleFramePose);
      
      for (int i=0; i<predictedContactPoints.size(); i++)
      {
         Point2D contactPoint = predictedContactPoints.get(i);
         
         FramePoint pointInWorld = new FramePoint(soleReferenceFrame, contactPoint.getX(), contactPoint.getY(), 0.0);
         pointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         
         yoContactPoints[i].set(pointInWorld.getPoint());
      }

      footPolygon.setConvexPolygon2d(new ConvexPolygon2d(predictedContactPoints));
      footPolygonViz.update();
   }
}
