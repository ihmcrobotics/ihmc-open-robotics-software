package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class ReferenceCMPTrajectoryGenerator
{
   private static final int maxNumberOfFootstepsToConsider = 4;

   private final List<YoDouble> swingDurations;
   private final List<YoDouble> swingFractionsOnHeel;

   private final List<YoDouble> transferDurations;
   private final List<YoDouble> transferFractionsOnHeel;

   private final List<CMPTrajectory> transferCMPTrajectories = new ArrayList<>();
   private final List<CMPTrajectory> swingCMPTrajectories = new ArrayList<>();


   public ReferenceCMPTrajectoryGenerator(String namePrefix, List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> swingFractionsOnHeel,
                                          List<YoDouble> transferFractionsOnHeel, YoVariableRegistry registry)
   {
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingFractionsOnHeel = swingFractionsOnHeel;
      this.transferFractionsOnHeel = transferFractionsOnHeel;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         CMPTrajectory transferCMPTrajectory = new CMPTrajectory(namePrefix + "Transfer + " + i, registry);
         CMPTrajectory swingCMPTrajectory = new CMPTrajectory(namePrefix + "Swing + " + i, registry);
         transferCMPTrajectories.add(transferCMPTrajectory);
         swingCMPTrajectories.add(swingCMPTrajectory);
      }
   }

   public void reset()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         transferCMPTrajectories.get(i).reset();
         swingCMPTrajectories.get(i).reset();
      }
   }

   public void initializeInTransfer(List<CoPTrajectory> transferCoPTrajectories, List<CoPTrajectory> swingCoPTrajectories)
   {

      // todo this needs to combine the angular momentum trajectory with the cop trajectory

      int numberOfSteps = swingCoPTrajectories.size();
      for (int i = 0; i < numberOfSteps; i++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(i);
         CoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(i);

         for (int segmentIndex = 0; segmentIndex < transferCoPTrajectory.getNumberOfSegments(); segmentIndex++)
         {
            YoFramePolynomial3D cmpSegment = transferCMPTrajectory.getNextSegment();
            cmpSegment.set(transferCoPTrajectory.getPolynomials().get(segmentIndex));
         }

         CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(i);
         CoPTrajectory swingCoPTrajectory = swingCoPTrajectories.get(i);
         for (int segmentIndex = 0; segmentIndex < swingCoPTrajectory.getNumberOfSegments(); segmentIndex++)
         {
            YoFramePolynomial3D cmpSegment = swingCMPTrajectory.getNextSegment();
            cmpSegment.set(swingCoPTrajectory.getPolynomials().get(segmentIndex));
         }
      }

      // handle final transfer
      CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      CoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);

      for (int segmentIndex = 0; segmentIndex < transferCoPTrajectory.getNumberOfSegments(); segmentIndex++)
      {
         YoFramePolynomial3D cmpSegment = transferCMPTrajectory.getNextSegment();
         cmpSegment.set(transferCoPTrajectory.getPolynomials().get(segmentIndex));
      }
   }

   public void initializeInSwing(List<CoPTrajectory> transferCoPTrajectories, List<CoPTrajectory> swingCoPTrajectories)
   {
      // todo this needs to combine the angular momentum trajectory with the cop trajectory

      // handle current swing
      CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(0);
      CoPTrajectory swingCoPTrajectory = swingCoPTrajectories.get(0);
      for (int segmentIndex = 0; segmentIndex < swingCoPTrajectory.getNumberOfSegments(); segmentIndex++)
      {
         YoFramePolynomial3D cmpSegment = swingCMPTrajectory.getNextSegment();
         cmpSegment.set(swingCoPTrajectory.getPolynomials().get(segmentIndex));
      }

      int numberOfSteps = swingCoPTrajectories.size();
      for (int i = 1; i < numberOfSteps; i++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(i);
         CoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(i);

         for (int segmentIndex = 0; segmentIndex < transferCoPTrajectory.getNumberOfSegments(); segmentIndex++)
         {
            YoFramePolynomial3D cmpSegment = transferCMPTrajectory.getNextSegment();
            cmpSegment.set(transferCoPTrajectory.getPolynomials().get(segmentIndex));
         }

         swingCMPTrajectory = swingCMPTrajectories.get(i);
         swingCoPTrajectory = swingCoPTrajectories.get(i);
         for (int segmentIndex = 0; segmentIndex < swingCoPTrajectory.getNumberOfSegments(); segmentIndex++)
         {
            YoFramePolynomial3D cmpSegment = swingCMPTrajectory.getNextSegment();
            cmpSegment.set(swingCoPTrajectory.getPolynomials().get(segmentIndex));
         }
      }

      // handle final transfer
      CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      CoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);

      for (int segmentIndex = 0; segmentIndex < transferCoPTrajectory.getNumberOfSegments(); segmentIndex++)
      {
         YoFramePolynomial3D cmpSegment = transferCMPTrajectory.getNextSegment();
         cmpSegment.set(transferCoPTrajectory.getPolynomials().get(segmentIndex));
      }
   }
}
