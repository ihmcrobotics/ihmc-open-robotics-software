package us.ihmc.footstepPlanning.baselinePlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class TimedFootstepPlanVisualization
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FramePose3D tempPose = new FramePose3D();
   private final SideDependentList<List<YoFramePoseUsingYawPitchRoll>> yoFootstepPoses = new SideDependentList<>();
   private final List<SimpleTimedFootstep> footstepPlan;
   private double previewTime = 0;
   private int numberOfValidSteps = 0;

   public TimedFootstepPlanVisualization(List<SimpleTimedFootstep> footstepPlan, int numberOfValidSteps, SideDependentList<ConvexPolygon2D> footPolygons,
                                         YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.footstepPlan = footstepPlan;
      this.numberOfValidSteps = numberOfValidSteps;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Create yo footstep polygons.
      SideDependentList<YoFrameConvexPolygon2D> yoFootPolygon = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         yoFootPolygon.put(robotSide, new YoFrameConvexPolygon2D(robotSide.getLowerCaseName() + "FootPolygon", "", worldFrame, 6, registry));
         yoFootPolygon.get(robotSide).set(footPolygons.get(robotSide));
      }

      // Create foothold graphics (add a left polygon and right polygon for each possible step).
      for (RobotSide robotSide : RobotSide.values)
      {
         yoFootstepPoses.put(robotSide, new ArrayList<>());
         for (int i = 0; i < footstepPlan.size(); i++)
         {
            yoFootstepPoses.get(robotSide).add(new YoFramePoseUsingYawPitchRoll(robotSide + "Footstep" + i + "Pose", worldFrame, registry));
            yoFootstepPoses.get(robotSide).get(i).setZ(-100); // hide the graphic
            AppearanceDefinition appearance = (robotSide == RobotSide.LEFT) ? YoAppearance.Magenta() : YoAppearance.Gold();
            graphicsListRegistry.registerYoGraphic("footstep",
                                                   new YoGraphicPolygon(robotSide + "footstep" + i, yoFootPolygon.get(robotSide), yoFootstepPoses.get(robotSide).get(i), 1.0, appearance));
         }
      }

      parentRegistry.addChild(registry);
   }

   public void setPreviewTime(double previewTime)
   {
      this.previewTime = previewTime;
   }

   public void setNumberOfValidSteps(int numberOfValidSteps)
   {
      this.numberOfValidSteps = numberOfValidSteps;
   }

   public void update(double currentTime)
   {
      // Update planned footstep polygons
      for (RobotSide robotSide : RobotSide.values)
      {
         for (int i = 0; i < yoFootstepPoses.get(robotSide).size(); i++)
         {
            tempPose.getPosition().set(0, 0, -100);
            yoFootstepPoses.get(robotSide).get(i).set(tempPose);
         }
      }
      for (int i = 0; i < numberOfValidSteps; i++)
      {
         if (footstepPlan.get(i).getTimeInterval().getEndTime() < currentTime + previewTime)
         {
            RobotSide robotSide = footstepPlan.get(i).getRobotSide();
            footstepPlan.get(i).getSoleFramePose(tempPose);
            yoFootstepPoses.get(robotSide).get(i).set(tempPose);
         }
      }
   }
}