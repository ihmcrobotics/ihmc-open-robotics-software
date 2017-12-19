package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.TorqueTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPTrajectory;
import us.ihmc.commons.Epsilons;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ReferenceCMPTrajectoryGeneratorTest
{
   private final int numberOfFootsteps = 3;
   private final int numberOfSwingSegments = 3;
   private final int numberOfTransferSegments = 2;
   private final double[] swingSegmentTimes = new double[] {0.0, 0.4, 0.8, 1.0};
   private final double[] transferSegmentTimes = new double[] {0.0, 0.1, 0.2};
   private final double groundReactionForce = 1000.0;
   private final int numberOfCoefficients = 10;
   private String testName = "CMPTest";
   private ReferenceCMPTrajectoryGenerator cmpTrajectoryGenerator;
   private final YoVariableRegistry registry = new YoVariableRegistry(testName + "Registry");
   private List<CoPTrajectory> swingCoPTrajectories = new ArrayList<>();
   private List<CoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private List<AngularMomentumTrajectory> swingAngularMomentumTrajectories = new ArrayList<>();
   private List<AngularMomentumTrajectory> transferAngularMomentumTrajectories = new ArrayList<>();
   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private FrameTrajectory3D cmpSegmentTrajectory = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
   private TorqueTrajectory torqueTrajectory = new TorqueTrajectory(Math.max(numberOfSwingSegments, numberOfTransferSegments), numberOfCoefficients);

   @Before
   public void setupTest()
   {
      clear();
      YoInteger numberOfFootstepsToConsider = new YoInteger(testName + "NumberOfFootstepsToConsider", registry);
      numberOfFootstepsToConsider.set(numberOfFootsteps);
      cmpTrajectoryGenerator = new ReferenceCMPTrajectoryGenerator(testName, numberOfFootsteps, numberOfFootstepsToConsider, registry);
      cmpTrajectoryGenerator.setGroundReaction(groundReactionForce);
      for (int i = 0; i < numberOfFootsteps; i++)
      {
         generateRandomLinearSegmentedTrajectory(swingCoPTrajectories, WalkingTrajectoryType.SWING, swingSegmentTimes, numberOfSwingSegments);
         generateRandomLinearSegmentedTrajectory(transferCoPTrajectories, WalkingTrajectoryType.TRANSFER, transferSegmentTimes, numberOfTransferSegments);
         generateRandomQuinticSegmentedTrajectory(swingAngularMomentumTrajectories, numberOfSwingSegments, swingSegmentTimes, numberOfCoefficients);
         generateRandomQuinticSegmentedTrajectory(transferAngularMomentumTrajectories, numberOfTransferSegments, transferSegmentTimes, numberOfCoefficients);
      }
      generateRandomLinearSegmentedTrajectory(transferCoPTrajectories, WalkingTrajectoryType.TRANSFER, transferSegmentTimes, numberOfTransferSegments);
      generateRandomQuinticSegmentedTrajectory(transferAngularMomentumTrajectories, numberOfTransferSegments, transferSegmentTimes, numberOfCoefficients);
   }

   private void generateRandomQuinticSegmentedTrajectory(List<AngularMomentumTrajectory> trajectoriesList, int numberOfSegments, double[] segmentTimes,
                                                         int numberOfCoefficients)
   {
      AngularMomentumTrajectory angularMomentumTrajectory = new AngularMomentumTrajectory(worldFrame, numberOfSegments, numberOfCoefficients);
      angularMomentumTrajectory.reset();
      for (int j = 0; j < numberOfSegments; j++)
      {
         angularMomentumTrajectory.add().setQuintic(segmentTimes[j], segmentTimes[j + 1], getRandomPoint(), getRandomVector(), getRandomVector(),
                                                    getRandomPoint(), getRandomVector(), getRandomVector());
      }
      trajectoriesList.add(angularMomentumTrajectory);
   }

   private void generateRandomLinearSegmentedTrajectory(List<CoPTrajectory> trajectoriesList, WalkingTrajectoryType trajectoryType, double[] segmentTimes,
                                                        int numberOfSegments)
   {
      CoPTrajectory copTrajectory = new CoPTrajectory(CoPSplineType.LINEAR, numberOfSegments, trajectoryType);
      copTrajectory.reset();
      for (int j = 0; j < numberOfSegments; j++)
      {
         copTrajectory.add().setLinear(segmentTimes[j], segmentTimes[j + 1], getRandomPoint(), getRandomPoint());
      }
      trajectoriesList.add(copTrajectory);
   }

   private void clear()
   {
      registry.clear();
      swingCoPTrajectories.clear();
      transferCoPTrajectories.clear();
      swingAngularMomentumTrajectories.clear();
      transferAngularMomentumTrajectories.clear();
   }

   private FramePoint3D getRandomPoint()
   {
      return new FramePoint3D(worldFrame, Math.random(), Math.random(), Math.random());
   }

   private FrameVector3D getRandomVector()
   {
      return new FrameVector3D(worldFrame, Math.random(), Math.random(), Math.random());
   }

   @After
   public void tearDownTest()
   {

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testDoubleSupportWithoutAngularMomentum()
   {
      cmpTrajectoryGenerator.setNumberOfRegisteredSteps(numberOfFootsteps);
      cmpTrajectoryGenerator.initializeForTransfer(0.0, transferCoPTrajectories, swingCoPTrajectories, null, null);
      checkTransferTrajectoriesForCopy();
      checkSwingTrajectoriesForCopy();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testSingleSupportWithoutAngularMomentum()
   {
      cmpTrajectoryGenerator.setNumberOfRegisteredSteps(numberOfFootsteps);
      cmpTrajectoryGenerator.initializeForTransfer(0.0, transferCoPTrajectories, swingCoPTrajectories, null, null);
      checkTransferTrajectoriesForCopy();
      checkSwingTrajectoriesForCopy();

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testDoubleSupportWithAngularMomentum()
   {
      cmpTrajectoryGenerator.setNumberOfRegisteredSteps(numberOfFootsteps);
      cmpTrajectoryGenerator.initializeForTransfer(0.0, transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories,
                                                   swingAngularMomentumTrajectories);
      checkTransferTrajectoriesForAddition(0);
      checkSwingTrajectoriesForAddition();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testSingleSupportWithAngularMomentum()
   {
      cmpTrajectoryGenerator.setNumberOfRegisteredSteps(numberOfFootsteps);
      cmpTrajectoryGenerator.initializeForSwing(0.0, transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories,
                                                swingAngularMomentumTrajectories);
      checkTransferTrajectoriesForAddition(1);
      checkSwingTrajectoriesForAddition();
   }

   private void checkTransferTrajectoriesForCopy()
   {
      List<? extends CMPTrajectory> transferCMPTrajectories = cmpTrajectoryGenerator.getTransferCMPTrajectories();
      assertTrue("Got : " + transferCMPTrajectories.size() + " should have been: " + (numberOfFootsteps + 1),
                 transferCMPTrajectories.size() == numberOfFootsteps + 1);
      for (int i = 0; i < transferCMPTrajectories.size(); i++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(i);
         assertTrue("Number of segments mismatch for segment " + i + " got: " + transferCMPTrajectory.getNumberOfSegments() + " should have been: "
               + transferCoPTrajectories.get(i).getNumberOfSegments(),
                    transferCMPTrajectory.getNumberOfSegments() == transferCoPTrajectories.get(i).getNumberOfSegments());
         for (int j = 0; j < transferCMPTrajectory.getNumberOfSegments(); j++)
         {
            assertTrue("Got: \n" + transferCMPTrajectories.get(i).getSegment(j).toString() + " should have been: \n"
                  + transferCoPTrajectories.get(i).getSegment(j).toString(),
                       TrajectoryMathTools.epsilonEquals(transferCoPTrajectories.get(i).getSegment(j), transferCMPTrajectories.get(i).getSegment(j),
                                                         Epsilons.ONE_HUNDRED_BILLIONTH));
         }
      }
   }

   private void checkSwingTrajectoriesForCopy()
   {
      List<? extends CMPTrajectory> swingCMPTrajectories = cmpTrajectoryGenerator.getSwingCMPTrajectories();
      assertTrue("Got : " + swingCMPTrajectories.size() + " should have been: " + (numberOfFootsteps), swingCMPTrajectories.size() == numberOfFootsteps);
      for (int i = 0; i < swingCMPTrajectories.size(); i++)
      {
         CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(i);
         assertTrue("Number of segments mismatch for segment " + i + " got: " + swingCMPTrajectory.getNumberOfSegments() + " should have been: "
               + swingCoPTrajectories.get(i).getNumberOfSegments(),
                    swingCMPTrajectory.getNumberOfSegments() == swingCoPTrajectories.get(i).getNumberOfSegments());
         for (int j = 0; j < swingCMPTrajectory.getNumberOfSegments(); j++)
         {
            TrajectoryMathTools.add(cmpSegmentTrajectory, swingCoPTrajectories.get(i).getSegment(j), torqueTrajectory.getSegment(j));
            assertTrue("Got: \n" + swingCMPTrajectories.get(i).getSegment(j).toString() + " should have been: \n"
                  + swingCoPTrajectories.get(i).getSegment(j).toString(),
                       TrajectoryMathTools.epsilonEquals(swingCoPTrajectories.get(i).getSegment(j), swingCMPTrajectories.get(i).getSegment(j),
                                                         Epsilons.ONE_HUNDRED_BILLIONTH));
         }
      }
   }

   private void checkTransferTrajectoriesForAddition(int startSegment)
   {
      List<? extends CMPTrajectory> transferCMPTrajectories = cmpTrajectoryGenerator.getTransferCMPTrajectories();
      assertTrue("Got : " + transferCMPTrajectories.size() + " should have been: " + (numberOfFootsteps + 1),
                 transferCMPTrajectories.size() == numberOfFootsteps + 1);
      for (int i = startSegment; i < transferCMPTrajectories.size(); i++)
      {
         CMPTrajectory transferCMPTrajectory = transferCMPTrajectories.get(i);
         assertTrue("Number of segments mismatch for segment " + i + " got: " + transferCMPTrajectory.getNumberOfSegments() + " should have been: "
               + transferCoPTrajectories.get(i).getNumberOfSegments(),
                    transferCMPTrajectory.getNumberOfSegments() == transferCoPTrajectories.get(i).getNumberOfSegments());
         assertTrue("Number of segments mismatch for segment " + i + " got: " + transferCMPTrajectory.getNumberOfSegments() + " should have been: "
               + transferAngularMomentumTrajectories.get(i).getNumberOfSegments(),
                    transferCMPTrajectory.getNumberOfSegments() == transferAngularMomentumTrajectories.get(i).getNumberOfSegments());
         torqueTrajectory.reset();
         torqueTrajectory.setNext(transferAngularMomentumTrajectories.get(i));
         torqueTrajectory.scale(1.0 / groundReactionForce);
         for (int j = 0; j < transferCMPTrajectory.getNumberOfSegments(); j++)
         {
            TrajectoryMathTools.add(cmpSegmentTrajectory, transferCoPTrajectories.get(i).getSegment(j), torqueTrajectory.getSegment(j));
            assertTrue("Got: \n" + transferCMPTrajectories.get(i).getSegment(j).toString() + " should have been: \n" + cmpSegmentTrajectory.toString(),
                       TrajectoryMathTools.epsilonEquals(cmpSegmentTrajectory, transferCMPTrajectories.get(i).getSegment(j), Epsilons.ONE_HUNDRED_BILLIONTH));
         }
      }
   }

   private void checkSwingTrajectoriesForAddition()
   {
      List<? extends CMPTrajectory> swingCMPTrajectories = cmpTrajectoryGenerator.getSwingCMPTrajectories();
      assertTrue("Got : " + swingCMPTrajectories.size() + " should have been: " + (numberOfFootsteps), swingCMPTrajectories.size() == numberOfFootsteps);
      for (int i = 0; i < swingCMPTrajectories.size(); i++)
      {
         CMPTrajectory swingCMPTrajectory = swingCMPTrajectories.get(i);
         assertTrue("Number of segments mismatch for segment " + i + " got: " + swingCMPTrajectory.getNumberOfSegments() + " should have been: "
               + swingCoPTrajectories.get(i).getNumberOfSegments(),
                    swingCMPTrajectory.getNumberOfSegments() == swingCoPTrajectories.get(i).getNumberOfSegments());
         assertTrue("Number of segments mismatch for segment " + i + " got: " + swingCMPTrajectory.getNumberOfSegments() + " should have been: "
               + swingAngularMomentumTrajectories.get(i).getNumberOfSegments(),
                    swingCMPTrajectory.getNumberOfSegments() == swingAngularMomentumTrajectories.get(i).getNumberOfSegments());
         torqueTrajectory.reset();
         torqueTrajectory.setNext(swingAngularMomentumTrajectories.get(i));
         torqueTrajectory.scale(1.0 / groundReactionForce);
         for (int j = 0; j < swingCMPTrajectory.getNumberOfSegments(); j++)
         {
            TrajectoryMathTools.add(cmpSegmentTrajectory, swingCoPTrajectories.get(i).getSegment(j), torqueTrajectory.getSegment(j));
            assertTrue("Got: \n" + swingCMPTrajectories.get(i).getSegment(j).toString() + " should have been: \n" + cmpSegmentTrajectory.toString(),
                       TrajectoryMathTools.epsilonEquals(cmpSegmentTrajectory, swingCMPTrajectories.get(i).getSegment(j), Epsilons.ONE_HUNDRED_BILLIONTH));
         }
      }
   }
}
