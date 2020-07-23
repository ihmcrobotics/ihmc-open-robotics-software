package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SingleFootstepVisualizer
{
   private static final SideDependentList<AppearanceDefinition> footPolygonAppearances = new SideDependentList<AppearanceDefinition>(YoAppearance.Purple(),
         YoAppearance.Green());
   
   private static SideDependentList<Integer> indices = new SideDependentList<Integer>(0, 0);

   private final YoFramePoseUsingYawPitchRoll soleFramePose;
   private final YoFramePoint3D[] yoContactPoints;
   private final YoFrameConvexPolygon2D footPolygon;
   private final YoGraphicPolygon footPolygonViz;
   private final RobotSide robotSide;

   public SingleFootstepVisualizer(RobotSide robotSide, int maxContactPoints, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Integer index = indices.get(robotSide);
      String namePrefix = robotSide.getLowerCaseName() + "Foot" + index;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix);
      this.robotSide = robotSide;

      ArrayList<Point2D> polyPoints = new ArrayList<Point2D>();
      yoContactPoints = new YoFramePoint3D[maxContactPoints];

      for (int i = 0; i < maxContactPoints; i++)
      {
         yoContactPoints[i] = new YoFramePoint3D(namePrefix + "ContactPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoContactPoints[i].set(0.0, 0.0, -1.0);

         YoGraphicPosition baseControlPointViz = new YoGraphicPosition(namePrefix + "Point" + i, yoContactPoints[i], 0.01, YoAppearance.Blue());
         yoGraphicsList.add(baseControlPointViz);

         polyPoints.add(new Point2D());
      }

      footPolygon = new YoFrameConvexPolygon2D(namePrefix + "yoPolygon", "", ReferenceFrame.getWorldFrame(), maxContactPoints, registry);
      footPolygon.set(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(polyPoints)));

      soleFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "polygonPose", "", ReferenceFrame.getWorldFrame(), registry);
      soleFramePose.setPosition(0.0, 0.0, -1.0);

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

         List<FramePoint2D> contactPointsFromContactablePlaneBody = bipedFoot.getContactPoints2d();
         for (int i=0; i<contactPointsFromContactablePlaneBody.size(); i++)
         {
            FramePoint2D point = contactPointsFromContactablePlaneBody.get(i);
            predictedContactPoints.add(new Point2D(point));
         }
      }
      
      ReferenceFrame soleReferenceFrame = footstep.getSoleReferenceFrame();
      double increaseZSlightlyToSeeBetter = 0.001;
      FramePose3D soleFramePose = new FramePose3D(soleReferenceFrame, new Point3D(0.0, 0.0, increaseZSlightlyToSeeBetter), new AxisAngle());
      soleFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      
      this.soleFramePose.set(soleFramePose);
      
      for (int i=0; i<predictedContactPoints.size(); i++)
      {
         Point2D contactPoint = predictedContactPoints.get(i);
         
         FramePoint3D pointInWorld = new FramePoint3D(soleReferenceFrame, contactPoint.getX(), contactPoint.getY(), 0.0);
         pointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         
         yoContactPoints[i].set(pointInWorld);
      }

      footPolygon.set(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(predictedContactPoints)));
      footPolygonViz.update();
   }
}
