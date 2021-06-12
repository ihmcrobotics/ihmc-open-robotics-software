package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class PushRecoveryCoPTrajectoryGenerator extends YoSaveableModule<PushRecoveryState>
{
   private final static double continuityDuration = 0.1;
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FrameConvexPolygon2D combinedPolygon = new FrameConvexPolygon2D();

   private final SideDependentList<FrameConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final PoseReferenceFrame stepFrame = new PoseReferenceFrame("StepFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame stanceFrame = new PoseReferenceFrame("StanceFrame", ReferenceFrame.getWorldFrame());

   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();

   public PushRecoveryCoPTrajectoryGenerator(ConvexPolygon2DReadOnly defaultSupportPolygon, YoRegistry parentRegistry)
   {
      super(PushRecoveryCoPTrajectoryGenerator.class, parentRegistry);

      this.defaultSupportPolygon.set(defaultSupportPolygon);

   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   public List<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   private final FrameConvexPolygon2DBasics nextPolygon = new FrameConvexPolygon2D();
   private final FrameLine2D intersectionLine = new FrameLine2D();

   private final FramePoint2D firstIntersection = new FramePoint2D();
   private final FramePoint2D secondIntersection = new FramePoint2D();

   private final FramePoint2D stanceCMP = new FramePoint2D();
   private final FramePoint3D nextCMP = new FramePoint3D();
   private final FramePoint2D startICP = new FramePoint2D();

   @Override
   public void compute(PushRecoveryState state)
   {
      contactStateProviders.clear();

      if (state.getNumberOfFootsteps() > 0)
         computeWithSteps(state);
      else
         computeTransferToStanding(state);
   }

   private void computeTransferToStanding(PushRecoveryState state)
   {
      combinedPolygon.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2DReadOnly supportPolygon = state.getFootPolygonInSole(robotSide);
         for (int vertexId = 0; vertexId < supportPolygon.getNumberOfVertices(); vertexId++)
            combinedPolygon.addVertexMatchingFrame(supportPolygon.getVertex(vertexId), false);
      }
      combinedPolygon.update();

      startICP.setIncludingFrame(state.getIcpAtStartOfState());
      startICP.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      stanceCMP.setToZero(ReferenceFrame.getWorldFrame());

      if (combinedPolygon.isPointInside(startICP))
      {
         stanceCMP.set(startICP);
      }
      else
      {
         combinedPolygon.orthogonalProjection(startICP, stanceCMP);
      }

      FramePoint3DReadOnly leftFootPosition = state.getFootPose(RobotSide.LEFT).getPosition();
      FramePoint3DReadOnly rightFootPosition = state.getFootPose(RobotSide.RIGHT).getPosition();
      double percentageAlongFoot = EuclidGeometryTools.percentageAlongLineSegment2D(stanceCMP.getX(),
                                                                                    stanceCMP.getY(),
                                                                                    leftFootPosition.getX(),
                                                                                    leftFootPosition.getY(),
                                                                                    rightFootPosition.getX(),
                                                                                    rightFootPosition.getY());
      nextCMP.set(stanceCMP, InterpolationTools.linearInterpolate(leftFootPosition.getZ(), rightFootPosition.getZ(), percentageAlongFoot));

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setContactState(ContactState.IN_CONTACT);
      contactState.getTimeInterval().setInterval(0.0, state.getFinalTransferDuration());
      contactState.setStartECMPPosition(nextCMP);
      contactState.setEndECMPPosition(nextCMP);
      contactState.setLinearECMPVelocity();
   }

   private void computeWithSteps(PushRecoveryState state)
   {
      DynamicPlanningFootstep recoveryFootstep = state.getFootstep(0);

      RobotSide swingSide = recoveryFootstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      FrameConvexPolygon2DBasics swingPolygon = movingPolygonsInSole.get(swingSide);
      FrameConvexPolygon2DBasics stancePolygon = movingPolygonsInSole.get(stanceSide);

      stanceFrame.setPoseAndUpdate(state.getFootPose(stanceSide));
      stepFrame.setPoseAndUpdate(recoveryFootstep.getFootstepPose());

      extractSupportPolygon(recoveryFootstep, stepFrame, nextPolygon, defaultSupportPolygon);
      swingPolygon.setIncludingFrame(nextPolygon);
      swingPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      stancePolygon.setIncludingFrame(state.getFootPolygonInSole(stanceSide));
      stancePolygon.changeFrameAndProjectToXYPlane(stanceFrame);

      stanceCMP.setToZero(stanceFrame);
      intersectionLine.setIncludingFrame(swingPolygon.getCentroid(), state.getIcpAtStartOfState());
      intersectionLine.changeFrameAndProjectToXYPlane(stanceFrame);

      int intersections = stancePolygon.intersectionWithRay(intersectionLine, firstIntersection, secondIntersection);

      if (intersections == 0)
      {
         stancePolygon.getClosestVertex(intersectionLine, stanceCMP);
      }
      else if (intersections == 1)
      {
         stanceCMP.setIncludingFrame(firstIntersection);
      }
      else
      {
         startICP.setIncludingFrame(state.getIcpAtStartOfState());
         startICP.changeFrameAndProjectToXYPlane(stanceFrame);
         if (firstIntersection.distanceSquared(startICP) < secondIntersection.distanceSquared(startICP))
            stanceCMP.set(firstIntersection);
         else
            stanceCMP.set(secondIntersection);
      }

      nextCMP.setIncludingFrame(stanceCMP, 0.0);
      nextCMP.changeFrame(ReferenceFrame.getWorldFrame());

      double currentTime = 0.0;

      SettableContactStateProvider swingContactState = contactStateProviders.add();
      swingContactState.reset();
      swingContactState.setContactState(ContactState.IN_CONTACT);
      swingContactState.getTimeInterval().setInterval(currentTime, currentTime + continuityDuration);
      swingContactState.setStartECMPPosition(nextCMP);
      swingContactState.setEndECMPPosition(nextCMP);
      swingContactState.setLinearECMPVelocity();

      swingContactState = contactStateProviders.add();
      swingContactState.reset();
      swingContactState.setContactState(ContactState.IN_CONTACT);
      swingContactState.getTimeInterval().setInterval(currentTime + continuityDuration, currentTime + state.getTiming(0).getSwingTime());
      swingContactState.setStartECMPPosition(nextCMP);
      swingContactState.setEndECMPPosition(nextCMP);
      swingContactState.setLinearECMPVelocity();

      currentTime += state.getTiming(0).getSwingTime();

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setContactState(ContactState.IN_CONTACT);
      contactState.getTimeInterval().setInterval(currentTime, currentTime + state.getTiming(0).getTransferTime());
      contactState.setStartECMPPosition(nextCMP);
      nextCMP.setIncludingFrame(recoveryFootstep.getFootstepPose().getPosition());

      contactState.setEndECMPPosition(nextCMP);
      contactState.setLinearECMPVelocity();

      currentTime += state.getTiming(0).getTransferTime();

      for (int i = 1; i < state.getNumberOfFootsteps(); i++)
      {
         recoveryFootstep = state.getFootstep(i);

         contactState = contactStateProviders.add();
         contactState.reset();
         contactState.setContactState(ContactState.IN_CONTACT);
         contactState.getTimeInterval().setInterval(currentTime, currentTime + state.getTiming(i).getSwingTime());
         contactState.setStartECMPPosition(nextCMP);
         contactState.setEndECMPPosition(nextCMP);
         contactState.setLinearECMPVelocity();

         currentTime += state.getTiming(i).getSwingTime();

         contactState = contactStateProviders.add();
         contactState.reset();
         contactState.setContactState(ContactState.IN_CONTACT);
         contactState.getTimeInterval().setInterval(currentTime, currentTime + state.getTiming(i).getTransferTime());
         contactState.setStartECMPPosition(nextCMP);
         nextCMP.setIncludingFrame(recoveryFootstep.getFootstepPose().getPosition());

         contactState.setEndECMPPosition(nextCMP);
         contactState.setLinearECMPVelocity();

         currentTime += state.getTiming(i).getTransferTime();
      }

      contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setContactState(ContactState.IN_CONTACT);
      contactState.getTimeInterval().setInterval(currentTime, currentTime + state.getFinalTransferDuration());
      contactState.setStartECMPPosition(nextCMP);
      contactState.setEndECMPPosition(nextCMP);
      contactState.setLinearECMPVelocity();
   }

   private static void extractSupportPolygon(DynamicPlanningFootstep footstep,
                                      ReferenceFrame stepFrame,
                                      FrameConvexPolygon2DBasics footSupportPolygonToPack,
                                      ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      if (footstep.hasPredictedContactPoints())
      {
         List<? extends Point2DReadOnly> predictedContactPoints = footstep.getPredictedContactPoints();

         footSupportPolygonToPack.clear(stepFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            footSupportPolygonToPack.addVertex(predictedContactPoints.get(i));
         footSupportPolygonToPack.update();
      }
      else
      {
         footSupportPolygonToPack.setIncludingFrame(stepFrame, defaultSupportPolygon);
      }
   }
}
