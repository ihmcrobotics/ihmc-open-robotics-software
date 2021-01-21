package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class LinearCoMTrajectoryHandler
{
   private final DMatrixRMaj coefficientArray = new DMatrixRMaj(0, 3);

   private final RecyclingArrayList<CoMTrajectory> comTrajectoryPool = new RecyclingArrayList<>(CoMTrajectory::new);
   private final MultipleSegmentPositionTrajectoryGenerator<CoMTrajectory> comTrajectory;
   private final RecyclingArrayList<Polynomial3DBasics> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Polynomial3D(4));
   private final List<Polynomial3DReadOnly> vrpTrajectories = new ArrayList<>();

   private boolean hasTrajectory = false;

   public LinearCoMTrajectoryHandler(YoRegistry registry)
   {
      comTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("desiredComTrajectory", ReferenceFrame.getWorldFrame(), registry);
   }

   public void clearTrajectory()
   {
      comTrajectory.clear();
      vrpTrajectories.clear();
      comTrajectoryPool.clear();
      vrpTrajectoryPool.clear();
      hasTrajectory = false;
   }

   public boolean hasTrajectory()
   {
      return hasTrajectory;
   }

   public void removeCompletedSegments(double timeToCrop)
   {
      while (comTrajectory.getCurrentNumberOfSegments() > 0 && comTrajectory.getSegment(0).getTimeInterval().getEndTime() <= timeToCrop)
         comTrajectory.removeSegment(0);

      if (comTrajectory.getCurrentNumberOfSegments() < 1)
      {
         hasTrajectory = false;
         return;
      }

      for (int i = 0; i < comTrajectory.getCurrentNumberOfSegments(); i++)
         comTrajectory.getSegment(i).getTimeInterval().shiftInterval(-timeToCrop);
   }

   public MultipleSegmentPositionTrajectoryGenerator<?> getComTrajectory()
   {
      return comTrajectory;
   }

   public List<Polynomial3DReadOnly> getVrpTrajectories()
   {
      return vrpTrajectories;
   }

   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FrameVector3D vrpStartVelocity = new FrameVector3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FrameVector3D vrpEndVelocity = new FrameVector3D();

   public void setLinear(FramePoint3DReadOnly start, FramePoint3DReadOnly end, double omega, double duration)
   {
      clearTrajectory();

      double rateX = (end.getX() - start.getX()) / duration;
      double rateY = (end.getY() - start.getY()) / duration;
      double rateZ = (end.getZ() - start.getZ()) / duration;

      CoMTrajectory comTrajectory = comTrajectoryPool.add();
      comTrajectory.setOmega(omega);
      comTrajectory.getTimeInterval().setInterval(0.0, duration);
      comTrajectory.setFifthCoefficient(ReferenceFrame.getWorldFrame(), rateX, rateY, rateZ);
      comTrajectory.setSixthCoefficient(ReferenceFrame.getWorldFrame(), start.getX(), start.getY(), start.getZ());
      this.comTrajectory.appendSegment(comTrajectory);
      this.comTrajectory.initialize();

      Polynomial3DBasics vrpTrajectory = vrpTrajectoryPool.add();
      vrpTrajectory.setLinear(0.0, duration, start, end);
      this.vrpTrajectories.add(vrpTrajectory);

      hasTrajectory = true;
   }



   public void setCoefficientsFromSolution(double omega, List<? extends ContactStateProvider> contacts, DMatrix1Row xSolution, DMatrix1Row ySolution, DMatrix1Row zSolution)
   {
      if (xSolution.getNumCols() != 1 || ySolution.getNumCols() != 1 || zSolution.getNumCols() != 1)
         throw new IllegalArgumentException("Solution vectors don't match in size");
      if (xSolution.getNumRows() != ySolution.getNumRows() || xSolution.getNumRows() != zSolution.getNumRows())
         throw new IllegalArgumentException("Solution vectors don't match in size");

      int numRows = xSolution.getNumRows();

      coefficientArray.reshape(numRows, 3);

      MatrixTools.setMatrixBlock(coefficientArray, 0, 0, xSolution, 0, 0, numRows, 1, 1.0);
      MatrixTools.setMatrixBlock(coefficientArray, 0, 1, ySolution, 0, 0, numRows, 1, 1.0);
      MatrixTools.setMatrixBlock(coefficientArray, 0, 2, zSolution, 0, 0, numRows, 1, 1.0);

      clearTrajectory();

      int startRow = 0;
      for (int i = 0; i < contacts.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = contacts.get(i).getTimeInterval();
         CoMTrajectory comTrajectory = comTrajectoryPool.add();
         comTrajectory.setCoefficients(coefficientArray, startRow);
         comTrajectory.getTimeInterval().set(timeInterval);
         comTrajectory.setOmega(omega);
         this.comTrajectory.appendSegment(comTrajectory);

         computeVRPBoundaryConditionsFromCoefficients(startRow, coefficientArray, omega, timeInterval.getDuration(), vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         Polynomial3DBasics vrpTrajectory = vrpTrajectoryPool.add();
         vrpTrajectory.setCubic(0.0, timeInterval.getDuration(), vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         this.vrpTrajectories.add(vrpTrajectory);

         startRow += CoMTrajectoryPlannerIndexHandler.polynomialCoefficientsPerSegment;
      }
      comTrajectory.initialize();


      hasTrajectory = true;
   }

   private static void computeVRPBoundaryConditionsFromCoefficients(int startRow,
                                                                    DMatrixRMaj coefficientArray,
                                                                    double omega,
                                                                    double duration,
                                                                    FixedFramePoint3DBasics startPosition,
                                                                    FixedFrameVector3DBasics startVelocity,
                                                                    FixedFramePoint3DBasics endPosition,
                                                                    FixedFrameVector3DBasics endVelocity)
   {
      startPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      startVelocity.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      endPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      endVelocity.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double omega2 = omega * omega;
      double t2 = duration * duration;
      double t3 = duration * t2;

      for (Axis3D axis : Axis3D.values)
      {
         int element = axis.ordinal();
         double startPositionElement = coefficientArray.get(startRow + 5, element) - 2.0 / omega2 * coefficientArray.get(startRow + 3, element);
         double startVelocityElement = coefficientArray.get(startRow + 4, element) - 6.0 / omega2 * coefficientArray.get(startRow + 2, element);
         startPosition.setElement(element, startPositionElement);
         startVelocity.setElement(element, startVelocityElement);

         double endPositionElement = coefficientArray.get(startRow + 2, element) * t3 + coefficientArray.get(startRow + 3, element) * t2 + startVelocityElement * duration + startPositionElement;
         double endVelocityElement = 3.0 * coefficientArray.get(startRow + 2, element) * t2 + 2.0 * coefficientArray.get(startRow + 3, element) * duration + startVelocityElement;

         endPosition.setElement(element, endPositionElement);
         endVelocity.setElement(element, endVelocityElement);
      }
   }

   public void computeCoMPosition(int segment, double timeInSegment, FixedFramePoint3DBasics comPositionToPack)
   {
      comTrajectoryPool.get(segment).computeCoMPosition(timeInSegment, comPositionToPack);
   }

   public void computeCoMVelocity(int segment, double timeInSegment, FixedFrameVector3DBasics comVelocityToPack)
   {
      comTrajectoryPool.get(segment).computeCoMVelocity(timeInSegment, comVelocityToPack);
   }

   public void computeCoMAcceleration(int segment, double timeInSegment, FixedFrameVector3DBasics comAccelerationToPack)
   {
      comTrajectoryPool.get(segment).computeCoMAcceleration(timeInSegment, comAccelerationToPack);
   }

   public void computeVRPVelocity(int segment, double timeInSegment, FixedFrameVector3DBasics vrpVelocityToPack)
   {
      comTrajectoryPool.get(segment).computeVRPVelocity(timeInSegment, vrpVelocityToPack);
   }

   public void compute(int segment,
                       double timeInSegment,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityToPack)
   {
      comTrajectoryPool.get(segment)
                       .compute(timeInSegment,
                              comPositionToPack,
                              comVelocityToPack,
                              comAccelerationToPack,
                              dcmPositionToPack,
                              dcmVelocityToPack,
                              vrpPositionToPack,
                              vrpVelocityToPack);
   }
}
