package us.ihmc.footstepPlanning.bodyPath;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

import com.google.common.util.concurrent.AtomicDouble;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerBasedBodyPathTest
{
   private final AtomicDouble xValue = new AtomicDouble();
   private final AtomicDouble yValue = new AtomicDouble();
   private final AtomicDouble yawValue = new AtomicDouble();

   public ControllerBasedBodyPathTest() throws JoystickNotFoundException
   {
      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      Component forward = joystick.findComponent(XBoxOneMapping.LEFT_STICK_Y.getIdentifier());
      Component sideway = joystick.findComponent(XBoxOneMapping.LEFT_STICK_X.getIdentifier());
      Component turn = joystick.findComponent(XBoxOneMapping.RIGHT_STICK_X.getIdentifier());
      joystick.addJoystickEventListener(new JoystickEventListener()
      {
         @Override
         public void processEvent(Event event)
         {
            if (event.getComponent().equals(forward))
            {
               xValue.set(-event.getValue());
            }
            if (event.getComponent().equals(sideway))
            {
               yValue.set(-event.getValue());
            }
            if (event.getComponent().equals(turn))
            {
               yawValue.set(-event.getValue());
            }
         }
      });

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      Robot dummy = new Robot("Dummy");
      dummy.setController(new Controller(graphicsListRegistry));

      SimulationConstructionSet scs = new SimulationConstructionSet(dummy);
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(graphics3DObject);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setCameraPosition(-0.01, 0.0, 10.0);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.startOnAThread();
      scs.play();
   }

   private class Controller extends SimpleRobotController
   {
      private final YoDouble x = new YoDouble("x", registry);
      private final YoDouble y = new YoDouble("y", registry);

      private static final int numberOfPoints = 5;
      private final List<YoFramePoint3D> points = new ArrayList<>();

      private final List<Point3D> waypoints = new ArrayList<>();
      private final WaypointDefinedBodyPathPlanHolder bodyPath = new WaypointDefinedBodyPathPlanHolder();

      private final DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      private final FootstepPlanningModule footstepPlanningModule = new FootstepPlanningModule("controllerBasedBodyPathTest");
      private final FootstepPlannerRequest request = new FootstepPlannerRequest();

      private static final int stepsPerSide = 10;
      private static final int steps = 15;
      private final SideDependentList<List<YoFramePoseUsingYawPitchRoll>> yoSteps = new SideDependentList<>();

      private final YoPolynomial xPoly = new YoPolynomial("xPoly", 4, registry);
      private final YoPolynomial yPoly = new YoPolynomial("yPoly", 4, registry);

      public Controller(YoGraphicsListRegistry graphicsListRegistry)
      {
         for (int i = 0; i < numberOfPoints; i++)
         {
            YoFramePoint3D yoPoint = new YoFramePoint3D("Position" + i, ReferenceFrame.getWorldFrame(), registry);
            YoGraphicPosition position = new YoGraphicPosition("Position" + i, yoPoint, 0.02, YoAppearance.Blue());
            points.add(yoPoint);
            graphicsListRegistry.registerYoGraphic("BodyPath", position);
         }

         YoFrameConvexPolygon2D yoDefaultFootPolygon = new YoFrameConvexPolygon2D("DefaultFootPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
         yoDefaultFootPolygon.set(PlannerTools.createDefaultFootPolygon());

         for (RobotSide side : RobotSide.values)
         {
            AppearanceDefinition appearance = side == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
            ArrayList<YoFramePoseUsingYawPitchRoll> poses = new ArrayList<>();
            for (int i = 0; i < stepsPerSide; i++)
            {
               YoFramePoseUsingYawPitchRoll yoFootstepPose = new YoFramePoseUsingYawPitchRoll("footPose" + side.getCamelCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + side.getCamelCaseName() + i, yoDefaultFootPolygon, yoFootstepPose, 1.0,
                                                                   appearance);
               poses.add(yoFootstepPose);
               yoFootstepPose.setToNaN();
               graphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
            yoSteps.put(side, poses);
         }

         request.setTimeout(1.0);
      }

      @Override
      public void doControl()
      {
         double newX = Math.max(0.0, xValue.get());
         double newY = MathTools.clamp(yValue.get(), 0.5);

         if (newX != x.getDoubleValue() || newY != y.getDoubleValue())
         {
            x.set(newX);
            y.set(newY);

            double deadband = 0.2;
            boolean smallTranslation = Math.sqrt(newX * newX + newY * newY) < deadband;
            if (smallTranslation)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  for (int i = 0; i < stepsPerSide; i++)
                  {
                     yoSteps.get(robotSide).get(i).setToNaN();
                  }
               }
               for (int i = 0; i < numberOfPoints; i++)
               {
                  points.get(i).setToNaN();
               }
               return;
            }

            waypoints.clear();
            xPoly.setQuadratic(0.0, 1.0, 0.0, 0.2, x.getDoubleValue());
            yPoly.setQuadratic(0.0, 1.0, 0.0, 0.0, y.getDoubleValue());
            for (int i = 0; i < numberOfPoints; i++)
            {
               double percent = (double) i / (double) (numberOfPoints - 1);
               xPoly.compute(percent);
               yPoly.compute(percent);
               Point3D point2d = new Point3D(xPoly.getValue(), yPoly.getValue(), 0.0);
               waypoints.add(point2d);
            }

            bodyPath.setWaypoints(waypoints);

            Pose3D pose = new Pose3D();
            for (int i = 0; i < numberOfPoints; i++)
            {
               double percent = (double) i / (double) (numberOfPoints - 1);
               bodyPath.getPointAlongPath(percent, pose);
               points.get(i).set(pose.getX(), pose.getY(), 0.0);
            }

            Pose3D startPose = new Pose3D();
            bodyPath.getPointAlongPath(0.0, startPose);
            Pose3D finalPose = new Pose3D();
            bodyPath.getPointAlongPath(1.0, finalPose);

            FramePose3D initialMidFootPose = new FramePose3D();
            initialMidFootPose.setX(startPose.getX());
            initialMidFootPose.setY(startPose.getY());
            initialMidFootPose.getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
            PoseReferenceFrame midFootFrame = new PoseReferenceFrame("InitialMidFootFrame", initialMidFootPose);

            RobotSide initialStanceFootSide = newY > 0.0 ? RobotSide.RIGHT : RobotSide.LEFT;
            FramePose3D midFootPose = new FramePose3D(midFootFrame);
            midFootPose.changeFrame(ReferenceFrame.getWorldFrame());

            FramePose3D goalPose = new FramePose3D();
            goalPose.setX(finalPose.getX());
            goalPose.setY(finalPose.getY());
            goalPose.getOrientation().setYawPitchRoll(finalPose.getYaw(), 0.0, 0.0);

            request.setStartFootPoses(parameters.getIdealFootstepWidth(), midFootPose);
            request.setRequestedInitialStanceSide(initialStanceFootSide);
            request.setGoalFootPoses(parameters.getIdealFootstepWidth(), goalPose);
            FootstepPlannerOutput output = footstepPlanningModule.handleRequest(request);

            if (output.getFootstepPlanningResult().validForExecution())
            {
               FootstepPlan footstepPlan = output.getFootstepPlan();
               int stepsToVisualize = Math.min(footstepPlan.getNumberOfSteps(), steps);
               SideDependentList<MutableInt> counts = new SideDependentList<>(new MutableInt(0), new MutableInt(0));

               for (int i = 0; i < stepsToVisualize; i++)
               {
                  PlannedFootstep footstep = footstepPlan.getFootstep(i);
                  FramePose3D footstepPose = new FramePose3D();
                  footstep.getFootstepPose(footstepPose);

                  RobotSide robotSide = footstep.getRobotSide();
                  MutableInt sideCount = counts.get(robotSide);
                  YoFramePoseUsingYawPitchRoll stepPose = yoSteps.get(robotSide).get(sideCount.intValue());
                  stepPose.set(footstepPose);
                  sideCount.increment();
               }

               for (RobotSide robotSide : RobotSide.values)
               {
                  MutableInt sideCount = counts.get(robotSide);
                  for (int i = sideCount.intValue(); i < stepsPerSide; i++)
                  {
                     yoSteps.get(robotSide).get(i).setToNaN();
                  }
               }
            }
            else
            {
               PrintTools.info("Failed: " + output.getFootstepPlanningResult());
            }
         }
      }
   }

   public static void main(String[] args) throws JoystickNotFoundException
   {
      PrintTools.info("Controller Demo.");
      new ControllerBasedBodyPathTest();
   }
}
