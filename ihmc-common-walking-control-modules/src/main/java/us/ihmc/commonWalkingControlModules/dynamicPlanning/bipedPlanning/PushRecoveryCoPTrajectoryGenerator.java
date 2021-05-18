package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.function.Supplier;

public class PushRecoveryCoPTrajectoryGenerator extends YoSaveableModule<PushRecoveryState>
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

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

   @Override
   public void compute(PushRecoveryState state)
   {
      contactStateProviders.clear();

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
      stancePolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      intersectionLine.set(swingPolygon.getCentroid(), state.getIcpAtStartOfState());

      int intersections = stancePolygon.intersectionWithRay(intersectionLine, firstIntersection, secondIntersection);

      if (intersections == 0)
      {
         stancePolygon.getClosestVertex(intersectionLine, stanceCMP);
      }
      else if (intersections == 1)
      {
         stanceCMP.set(firstIntersection);
      }
      else
      {
         if (firstIntersection.distanceSquared(state.getIcpAtStartOfState()) < secondIntersection.distanceSquared(state.getIcpAtStartOfState()))
            stanceCMP.set(firstIntersection);
         else
            stanceCMP.set(secondIntersection);
      }

      SettableContactStateProvider swingContactState = contactStateProviders.add();
      swingContactState.setContactState(ContactState.IN_CONTACT);
      swingContactState.getTimeInterval().setInterval(0.0, state.getTiming(0).getSwingTime());
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
