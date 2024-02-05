package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.swing.SwingKnotPoint;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.stream.Collectors;

public class SwingPlannerTools
{
   private static Vector3D infiniteWeight = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private static FrameVector3D zeroVector = new FrameVector3D();

   public static List<EnumMap<Axis3D, List<PolynomialReadOnly>>> computeTrajectories(SwingPlannerParametersBasics swingPlannerParameters,
                                                                                     SwingTrajectoryParameters swingTrajectoryParameters,
                                                                                     PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator,
                                                                                     SideDependentList<? extends Pose3DReadOnly> initialStanceFootPoses,
                                                                                     FootstepPlan footstepPlan)
   {
      List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = new ArrayList<>();

      FramePose3D startOfSwingPose = new FramePose3D();
      FramePose3D endOfSwingPose = new FramePose3D();

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);

         startOfSwingPose.set((i < 2 ? initialStanceFootPoses.get(footstep.getRobotSide()) : footstepPlan.getFootstep(i - 2).getFootstepPose()));
         endOfSwingPose.set(footstep.getFootstepPose());

         EnumMap<Axis3D, List<PolynomialReadOnly>> swingTrajectory = computeTrajectory(swingPlannerParameters, swingTrajectoryParameters,
                                                                                       positionTrajectoryGenerator,
                                                                                       startOfSwingPose,
                                                                                       endOfSwingPose,
                                                                                       footstepPlan.getFootstep(i));
         swingTrajectories.add(swingTrajectory);
      }

      return swingTrajectories;
   }

   public static EnumMap<Axis3D, List<PolynomialReadOnly>> computeTrajectory(SwingPlannerParametersBasics swingPlannerParameters,
                                                                             SwingTrajectoryParameters swingTrajectoryParameters,
                                                                             PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator,
                                                                             FramePose3D startOfSwingPose,
                                                                             FramePose3D endOfSwingPose,
                                                                             PlannedFootstep footstep)
   {

      boolean defaultTrajectoryUsed = footstep.getCustomWaypointPositions().isEmpty();
      int totalNumberOfWaypoints = defaultTrajectoryUsed ? 2 : footstep.getCustomWaypointPositions().size();

      /*
      * Compute default trajectories
      * */
      if (defaultTrajectoryUsed)
      {
         List<FramePoint3D> defaultWaypoints = new ArrayList<>();
         double[] defaultWaypointProportions = new double[] {0.15, 0.85};
         double defaultSwingHeightFromStanceFoot = swingTrajectoryParameters.getDefaultSwingHeight();

         for (int i = 0; i < 2; i++)
         {
            FramePoint3D waypoint = new FramePoint3D();
            waypoint.interpolate(startOfSwingPose.getPosition(), endOfSwingPose.getPosition(), defaultWaypointProportions[i]);
            waypoint.addZ(defaultSwingHeightFromStanceFoot);
            defaultWaypoints.add(waypoint);
         }

         double zDifference = Math.abs(startOfSwingPose.getZ() - endOfSwingPose.getZ());
         boolean obstacleClearance = zDifference > swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown();
         if (obstacleClearance)
         {
            double maxStepZ = Math.max(startOfSwingPose.getZ(), endOfSwingPose.getZ());
            for (int i = 0; i < 2; i++)
            {
               defaultWaypoints.get(i).setZ(maxStepZ + defaultSwingHeightFromStanceFoot);
            }
         }

         positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
         positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
         positionTrajectoryGenerator.setWaypoints(defaultWaypoints);
         positionTrajectoryGenerator.initialize();

         for (int i = 0; i < 30; i++)
         {
            boolean isDone = positionTrajectoryGenerator.doOptimizationUpdate();
            if (isDone)
               break;
         }
      }
      else
      {
         /* Compute the custom trajectories */
         positionTrajectoryGenerator.reset();
         positionTrajectoryGenerator.setEndpointConditions(startOfSwingPose.getPosition(), zeroVector, endOfSwingPose.getPosition(), zeroVector);
         positionTrajectoryGenerator.setEndpointWeights(infiniteWeight, infiniteWeight, infiniteWeight, infiniteWeight);
         positionTrajectoryGenerator.setWaypoints(footstep.getCustomWaypointPositions()
                                                          .stream()
                                                          .map(p -> new FramePoint3D(ReferenceFrame.getWorldFrame(), p))
                                                          .collect(Collectors.toList()));
         positionTrajectoryGenerator.initialize();
         positionTrajectoryGenerator.setShouldVisualize(false);

         for (int i = 0; i < 30; i++)
         {
            boolean isDone = positionTrajectoryGenerator.doOptimizationUpdate();
            if (isDone)
               break;
         }
      }

      return copySwingTrajectories(positionTrajectoryGenerator.getTrajectories(), totalNumberOfWaypoints + 1);
   }

   public static EnumMap<Axis3D, List<PolynomialReadOnly>> copySwingTrajectories(EnumMap<Axis3D, ArrayList<YoPolynomial>> trajectories, int trajectoriesToCopy)
   {
      EnumMap<Axis3D, List<PolynomialReadOnly>> copy = new EnumMap<>(Axis3D.class);
      trajectories.keySet().forEach(axis ->
                                    {
                                       List<PolynomialReadOnly> listCopy = new ArrayList<>();
                                       for (int i = 0; i < trajectoriesToCopy; i++)
                                       {
                                          PolynomialReadOnly polynomialReadOnly = trajectories.get(axis).get(i);
                                          double duration = polynomialReadOnly.getTimeInterval().getDuration();
                                          if (Double.isNaN(duration) || duration < 1e-4)
                                             continue;

                                          Polynomial polynomialCopy = new Polynomial(polynomialReadOnly.getNumberOfCoefficients());
                                          polynomialCopy.set(polynomialReadOnly);
                                          listCopy.add(polynomialCopy);
                                       }
                                       copy.put(axis, listCopy);
                                    });

      return copy;
   }

   public static EnumMap<Axis3D, List<PolynomialReadOnly>> copySwingTrajectories(EnumMap<Axis3D, ArrayList<YoPolynomial>> trajectories)
   {
      return copySwingTrajectories(trajectories, trajectories.get(Axis3D.X).size());
   }
}
