package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class SupportSequence
{
   private static final int INITIAL_CAPACITY = 50;
   private static final double UNSET_TIME = -1.0;

   private static final double stepLengthToDoToeOff = 0.05;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<? extends ReferenceFrame> soleFrames;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final RecyclingArrayList<ConvexPolygon2D> supportPolygons = new RecyclingArrayList<>(INITIAL_CAPACITY, ConvexPolygon2D.class);
   private final TDoubleArrayList supportInitialTimes = new TDoubleArrayList(INITIAL_CAPACITY, UNSET_TIME);

   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSole = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());
   private final SideDependentList<FramePose3D> footPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final BipedSupportPolygons bipedSupportPolygons;
   private final FramePose3D tempPose = new FramePose3D();

   private final SideDependentList<ConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());

   private final SideDependentList<RecyclingArrayList<FrameConvexPolygon2DBasics>> footSupportSequences = new SideDependentList<>();
   private final SideDependentList<TDoubleArrayList> footSupportInitialTimes = new SideDependentList<>();

   private final List<YoDouble> polygonStartTimes = new ArrayList<>();
   private final List<ConvexPolygon2DBasics> vizPolygons = new ArrayList<>();

   /**
    * Each step gets one of these frames. It is located at the sole frame of the foot when it is in full support.
    */
   private final SideDependentList<RecyclingArrayList<PoseReferenceFrame>> stepFrames = new SideDependentList<>();

   public SupportSequence(ConvexPolygon2DReadOnly defaultSupportPolygon,
                          SideDependentList<? extends ReferenceFrame> soleFrames,
                          SideDependentList<? extends ReferenceFrame> soleZUpFrames)
   {
      this(defaultSupportPolygon, soleFrames, soleZUpFrames, null, null, null);
   }

   public SupportSequence(ConvexPolygon2DReadOnly defaultSupportPolygon,
                          SideDependentList<? extends ReferenceFrame> soleFrames,
                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                          BipedSupportPolygons bipedSupportPolygons,
                          YoRegistry parentRegistry,
                          YoGraphicsListRegistry graphicRegistry)
   {
      if (graphicRegistry != null)
      {
         for (int i = 0; i < 10; i++)
         {
            YoFrameConvexPolygon2D yoPolygon = new YoFrameConvexPolygon2D("SupportPolygon" + i, ReferenceFrame.getWorldFrame(), 10, registry);
            graphicRegistry.registerArtifact(getClass().getSimpleName(), new YoArtifactPolygon("SupportPolygon" + i, yoPolygon, Color.GRAY, false));
            vizPolygons.add(yoPolygon);
            polygonStartTimes.add(new YoDouble("SupportPolygonStartTime" + i, registry));
         }
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         footSupportSequences.put(robotSide, new RecyclingArrayList<>(INITIAL_CAPACITY, FrameConvexPolygon2D::new));
         footSupportInitialTimes.put(robotSide, new TDoubleArrayList(INITIAL_CAPACITY, UNSET_TIME));

         stepFrames.put(robotSide, new RecyclingArrayList<>(3, new Supplier<PoseReferenceFrame>()
         {
            private int frameIndexCounter = 0;

            @Override
            public PoseReferenceFrame get()
            {
               return new PoseReferenceFrame(robotSide.getLowerCaseName() + "StepFrame" + frameIndexCounter++, ReferenceFrame.getWorldFrame());
            }
         }));
      }

      this.defaultSupportPolygon.set(defaultSupportPolygon);
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.soleFrames = soleFrames;
      this.soleZUpFrames = soleZUpFrames;
   }

   /**
    * Get the list of upcoming support polygons with the first polygon in the list being the current support.
    *
    * @return the list of upcoming support polygons.
    */
   public List<? extends ConvexPolygon2DReadOnly> getSupportPolygons()
   {
      return supportPolygons;
   }

   /**
    * Get the times at which the respective support polygons obtained with {@link #getSupportPolygons()} will be active.
    * The first value in this list should always be 0.0. The time is relative to the start time of the support sequence
    *
    * @return the times at which the support polygon will change.
    */
   public TDoubleList getSupportTimes()
   {
      return supportInitialTimes;
   }

   /**
    * Updates the current foot poses and the footholds of the robot to match the actual sole poses and footholds.
    */
   public void initializeStance()
   {
      for (RobotSide robotSide : RobotSide.values)
         initializeStance(robotSide);
   }

   /**
    * For the given side updates the current foot pose and the foothold of the robot to match the actual sole pose and
    * foothold.
    *
    * @param robotSide to be updated.
    */
   public void initializeStance(RobotSide robotSide)
   {
      if (bipedSupportPolygons == null)
         footPolygonsInSole.get(robotSide).set(defaultSupportPolygon);
      else
         footPolygonsInSole.get(robotSide).set(bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide));
      footPoses.get(robotSide).setToZero(soleFrames.get(robotSide));
      footPoses.get(robotSide).changeFrame(soleZUpFrames.get(robotSide));
   }

   public void changeFootFrame(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      footPoses.get(robotSide).changeFrame(referenceFrame);
   }

   /**
    * Updates the support sequence. This can be called every tick to prevent the support state from becoming invalid in
    * case the robot would drift.
    */
   public void update()
   {
      update(Collections.emptyList(), Collections.emptyList());
   }

   /**
    * Updates the support sequence with the given footstep parameters. This can be called every tick. This method will
    * retain all contact state since the sequence was started and update all contact switches in the future.
    *
    * @param footsteps to be added to the sequence.
    * @param footstepTimings respective timings.
    */
   public void update(List<Footstep> footsteps, List<FootstepTiming> footstepTimings)
   {
      reset();

      // Add initial support states of the feet and set the moving polygons
      for (RobotSide robotSide : RobotSide.values)
      {
         // Record the initial step frames. In case there is a step touching down this frame will be updated.
         PoseReferenceFrame stepFrame = stepFrames.get(robotSide).add();
         tempPose.setIncludingFrame(footPoses.get(robotSide));
         tempPose.changeFrame(stepFrame.getParent());
         stepFrame.setPoseAndUpdate(tempPose);

         movingPolygonsInSole.get(robotSide).set(footPolygonsInSole.get(robotSide));
         FrameConvexPolygon2DBasics initialFootSupport = footSupportSequences.get(robotSide).add();
         initialFootSupport.setIncludingFrame(stepFrame, movingPolygonsInSole.get(robotSide));
         footSupportInitialTimes.get(robotSide).add(0.0);
      }

      // Assemble the individual foot support trajectories for regular walking
      double stepStartTime = 0.0;
      for (int stepIndex = 0; stepIndex < footsteps.size(); stepIndex++)
      {
         FootstepTiming footstepTiming = footstepTimings.get(stepIndex);
         Footstep footstep = footsteps.get(stepIndex);
         RobotSide stepSide = footstep.getRobotSide();

         // Add swing - no support for foot
         addToeOffPolygon(footstep, footstepTiming, stepStartTime);
         footSupportSequences.get(stepSide).add().clearAndUpdate();
         footSupportInitialTimes.get(stepSide).add(stepStartTime + footstepTiming.getTransferTime());

         // Update the moving polygon and sole frame to reflect that the step was taken.
         extractSupportPolygon(footstep, movingPolygonsInSole.get(stepSide), defaultSupportPolygon);

         // Add touchdown polygon
         addTouchdownPolygon(footstep, footstepTiming, stepStartTime);

         stepStartTime += footstepTiming.getStepTime();
      }

      // Convert the foot support trajectories to a full support trajectory
      int lIndex = 0;
      int rIndex = 0;
      FrameConvexPolygon2DReadOnly lPolygon = footSupportSequences.get(RobotSide.LEFT).get(lIndex);
      FrameConvexPolygon2DReadOnly rPolygon = footSupportSequences.get(RobotSide.RIGHT).get(rIndex);
      combinePolygons(supportPolygons.add(), lPolygon, rPolygon);
      supportInitialTimes.add(0.0);

      while (true)
      {
         double lNextTime;
         double rNextTime;
         if (footSupportInitialTimes.get(RobotSide.LEFT).size() == lIndex + 1)
            lNextTime = Double.POSITIVE_INFINITY;
         else
            lNextTime = footSupportInitialTimes.get(RobotSide.LEFT).get(lIndex + 1);
         if (footSupportInitialTimes.get(RobotSide.RIGHT).size() == rIndex + 1)
            rNextTime = Double.POSITIVE_INFINITY;
         else
            rNextTime = footSupportInitialTimes.get(RobotSide.RIGHT).get(rIndex + 1);

         if (Double.isInfinite(rNextTime) && Double.isInfinite(lNextTime))
            break;

         if (Precision.equals(lNextTime, rNextTime))
         {
            rIndex++;
            lIndex++;
            rPolygon = footSupportSequences.get(RobotSide.RIGHT).get(rIndex);
            lPolygon = footSupportSequences.get(RobotSide.LEFT).get(lIndex);
            supportInitialTimes.add(footSupportInitialTimes.get(RobotSide.LEFT).get(lIndex));
         }
         else if (lNextTime > rNextTime)
         {
            rIndex++;
            rPolygon = footSupportSequences.get(RobotSide.RIGHT).get(rIndex);
            supportInitialTimes.add(footSupportInitialTimes.get(RobotSide.RIGHT).get(rIndex));
         }
         else
         {
            lIndex++;
            lPolygon = footSupportSequences.get(RobotSide.LEFT).get(lIndex);
            supportInitialTimes.add(footSupportInitialTimes.get(RobotSide.LEFT).get(lIndex));
         }

         combinePolygons(supportPolygons.add(), lPolygon, rPolygon);
      }

      updateViz();
   }

   private void addTouchdownPolygon(Footstep footstep, FootstepTiming footstepTiming, double stepStartTime)
   {
      RobotSide stepSide = footstep.getRobotSide();
      TDoubleArrayList footInitialTimes = footSupportInitialTimes.get(stepSide);
      RecyclingArrayList<FrameConvexPolygon2DBasics> footSupports = footSupportSequences.get(stepSide);

      // Record the step frame as the sole frame when the foot is in full contact.
      PoseReferenceFrame stepFrame = stepFrames.get(stepSide).add();
      stepFrame.setPoseAndUpdate(footstep.getFootstepPose());

      // Check if the footstep contains a partial foothold touchdown
      //      boolean doPartialFootholdTouchdown = checkForTouchdown(footstep, footstepTiming);

      // Compute the touchdown polygon in step sole frame
      FrameConvexPolygon2DBasics touchdownPolygon = footSupports.add();

      touchdownPolygon.setIncludingFrame(stepFrame, movingPolygonsInSole.get(stepSide));
      footInitialTimes.add(stepStartTime + footstepTiming.getStepTime());
   }

   private void addToeOffPolygon(Footstep footstep, FootstepTiming footstepTiming, double stepStartTime)
   {
      RobotSide stepSide = footstep.getRobotSide();
      PoseReferenceFrame stepFrame = last(stepFrames.get(stepSide));

      if (shouldDoToeOff(last(stepFrames.get(stepSide.getOppositeSide())), stepFrame))
      {
         FrameConvexPolygon2DBasics liftoffPolygon = footSupportSequences.get(stepSide).add();
         computeToePolygon(liftoffPolygon, movingPolygonsInSole.get(stepSide), stepFrame);

         // TODO: This timing is a heuristic. Make this tunable.
         footSupportInitialTimes.get(stepSide).add(stepStartTime + footstepTiming.getTransferTime() / 2.0);
      }
   }

   private FrameConvexPolygon2DBasics insertAt(RobotSide robotSide, double initialTime)
   {
      TDoubleArrayList initialTimes = footSupportInitialTimes.get(robotSide);
      RecyclingArrayList<FrameConvexPolygon2DBasics> polygons = footSupportSequences.get(robotSide);

      if (initialTime > last(initialTimes))
      {
         initialTimes.add(initialTime);
         return polygons.add();
      }
      else if (Precision.equals(initialTime, last(initialTimes)))
      {
         return last(polygons);
      }
      else if (initialTime <= 0.0)
      {
         return polygons.get(0);
      }

      // If we end up here that suggests that step timings for a single side are overlapping.
      // E.g. a liftoff might be starting before a previous touchdown is finished.
      System.out.println(initialTimes);
      throw new RuntimeException("Tried to add " + initialTime);
   }

   private final FrameConvexPolygon2D framePolygonA = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D framePolygonB = new FrameConvexPolygon2D();

   private void combinePolygons(ConvexPolygon2DBasics result, FrameConvexPolygon2DReadOnly polygonA, FrameConvexPolygon2DReadOnly polygonB)
   {
      framePolygonA.setIncludingFrame(polygonA);
      framePolygonA.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      framePolygonB.setIncludingFrame(polygonB);
      framePolygonB.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      result.clear();
      result.addVertices(framePolygonA);
      result.addVertices(framePolygonB);
      result.update();
   }

   private void computeToePolygon(FrameConvexPolygon2DBasics toePolygon, ConvexPolygon2DReadOnly fullPolygonInSole, ReferenceFrame soleFrame)
   {
      double maxX = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < fullPolygonInSole.getNumberOfVertices(); i++)
      {
         double x = fullPolygonInSole.getVertex(i).getX();
         maxX = Math.max(maxX, x);
      }

      toePolygon.clear(soleFrame);
      for (int i = 0; i < fullPolygonInSole.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = fullPolygonInSole.getVertex(i);
         if (Precision.equals(vertex.getX(), maxX, 0.01))
            toePolygon.addVertex(vertex);
      }
      toePolygon.update();
   }

   private void computeHeelPolygon(FrameConvexPolygon2DBasics heelPolygon, ConvexPolygon2DReadOnly fullPolygonInSole, ReferenceFrame soleFrame)
   {
      double minX = Double.POSITIVE_INFINITY;
      for (int i = 0; i < fullPolygonInSole.getNumberOfVertices(); i++)
      {
         double x = fullPolygonInSole.getVertex(i).getX();
         minX = Math.min(minX, x);
      }

      heelPolygon.clear(soleFrame);
      for (int i = 0; i < fullPolygonInSole.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = fullPolygonInSole.getVertex(i);
         if (Precision.equals(vertex.getX(), minX, 0.01))
            heelPolygon.addVertex(vertex);
      }
      heelPolygon.update();
   }

   private final Quaternion orientationError = new Quaternion();
   private final Quaternion rotation = new Quaternion();
   private final Vector3DReadOnly zAxis = new Vector3D(0.0, 0.0, 1.0);

   /**
    * This method projects the sole frame onto the x-y plane of the foothold pose. It preserves the heel location of the
    * sole and attempts to avoid modifying the yaw of the sole in world.
    *
    * @param solePose to be projected (modified)
    * @param footholdPose defines the x-y plane
    * @param soleToHeel heel location in sole frame
    */
   private void computeAdjustedSole(FixedFramePose3DBasics solePose, FramePose3DReadOnly footholdPose, Tuple2DReadOnly soleToHeel)
   {
      solePose.checkReferenceFrameMatch(footholdPose);

      orientationError.set(solePose.getOrientation());
      orientationError.multiplyConjugateThis(footholdPose.getOrientation());
      EuclidCoreMissingTools.projectRotationOnAxis(orientationError, zAxis, rotation);
      rotation.conjugate();

      solePose.appendTranslation(soleToHeel.getX(), soleToHeel.getY(), 0.0);
      solePose.appendRotation(orientationError);
      solePose.appendRotation(rotation);
      solePose.appendTranslation(-soleToHeel.getX(), -soleToHeel.getY(), 0.0);
   }

   private final FramePoint3D stepLocation = new FramePoint3D();

   private boolean shouldDoToeOff(ReferenceFrame stanceFrame, ReferenceFrame swingFootStartFrame)
   {
      stepLocation.setToZero(swingFootStartFrame);
      stepLocation.changeFrame(stanceFrame);
      return stepLocation.getX() < -stepLengthToDoToeOff;
   }

   private void reset()
   {
      supportPolygons.clear();
      supportInitialTimes.reset();
      for (RobotSide robotSide : RobotSide.values)
      {
         footSupportSequences.get(robotSide).clear();
         footSupportInitialTimes.get(robotSide).reset();
         stepFrames.get(robotSide).clear();
      }
   }

   private void updateViz()
   {
      int max = Math.min(vizPolygons.size(), supportPolygons.size());
      for (int i = 0; i < max; i++)
      {
         vizPolygons.get(i).set(supportPolygons.get(i));
         polygonStartTimes.get(i).set(supportInitialTimes.get(i));
      }
      for (int i = max; i < vizPolygons.size(); i++)
      {
         vizPolygons.get(i).setToNaN();
         polygonStartTimes.get(i).set(UNSET_TIME);
      }
   }

   private static void extractSupportPolygon(Footstep footstep, ConvexPolygon2DBasics footSupportPolygonToPack, ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         footSupportPolygonToPack.clear();
         for (int i = 0; i < predictedContactPoints.size(); i++)
            footSupportPolygonToPack.addVertex(predictedContactPoints.get(i));
         footSupportPolygonToPack.update();
      }
      else
      {
         footSupportPolygonToPack.set(defaultSupportPolygon);
      }
   }

   private static <T> T last(List<T> list)
   {
      return list.get(list.size() - 1);
   }

   private static double last(TDoubleList list)
   {
      return list.get(list.size() - 1);
   }
}