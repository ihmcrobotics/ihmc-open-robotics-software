package us.ihmc.robotics.referenceFrames;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class TranslationReferenceFrameTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testUpdateInMiddleFrame()
   {
      TranslationReferenceFrame frame1 = new TranslationReferenceFrame("frame1", ReferenceFrame.getWorldFrame());
      TranslationReferenceFrame frame2 = new TranslationReferenceFrame("frame2", frame1);
      TranslationReferenceFrame frame3 = new TranslationReferenceFrame("frame3", frame2);

      Vector3D translation1 = new Vector3D(0.1, 0.13, 0.45);
      Vector3D translation2 = new Vector3D(0.7, 0.26, 0.09);
      Vector3D translation3 = new Vector3D(0.04, 0.023, 0.067);

      frame1.updateTranslation(translation1);
      frame2.updateTranslation(translation2);
      frame3.updateTranslation(translation3);

      RigidBodyTransform transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Vector3D totalTranslation = new Vector3D();

      Vector3D expectedTranslation = new Vector3D(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      transformToDesiredFrame.getTranslation(totalTranslation);

      EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, totalTranslation, 1e-7);

      translation2.set(0.33, 0.44, 0.11);
      frame2.updateTranslation(translation2);

      transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      totalTranslation = new Vector3D();

      expectedTranslation = new Vector3D(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      transformToDesiredFrame.getTranslation(totalTranslation);

      EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, totalTranslation, 1e-7);
   }
}
