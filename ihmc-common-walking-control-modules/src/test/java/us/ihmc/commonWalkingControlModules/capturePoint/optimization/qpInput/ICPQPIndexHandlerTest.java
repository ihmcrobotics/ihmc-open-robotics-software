package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ICPQPIndexHandlerTest
{
   @Test
   public void testRegisterFootstep()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      indexHandler.registerFootstep();

      Assert.assertTrue(indexHandler.useStepAdjustment());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
   }

   @Test
   public void testSizing()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertFalse(indexHandler.useAngularMomentum());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(2, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(2, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.registerFootstep();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(2, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.setHasCMPFeedbackTask(true);

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.hasCMPFeedbackTask());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(4, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(4, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(0, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(4, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.hasCMPFeedbackTask());
      Assert.assertFalse(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(4, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(4, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(1, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(6, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.hasCMPFeedbackTask());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(2, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(4, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(6, indexHandler.getNumberOfFreeVariables());
      Assert.assertFalse(indexHandler.hasCMPFeedbackTask());
      Assert.assertTrue(indexHandler.useStepAdjustment());

      indexHandler.resetFootsteps();
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      Assert.assertEquals(0, indexHandler.getCoPFeedbackIndex());
      Assert.assertEquals(2, indexHandler.getCMPFeedbackIndex());
      Assert.assertEquals(4, indexHandler.getFootstepIndex(0));
      Assert.assertEquals(4, indexHandler.getFootstepStartIndex());
      Assert.assertEquals(4, indexHandler.getNumberOfFootstepVariables());
      Assert.assertEquals(2, indexHandler.getNumberOfFootstepsToConsider());
      Assert.assertEquals(8, indexHandler.getNumberOfFreeVariables());
      Assert.assertTrue(indexHandler.hasCMPFeedbackTask());
      Assert.assertTrue(indexHandler.useStepAdjustment());
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ICPQPIndexHandler.class, ICPQPIndexHandlerTest.class);
   }
}
