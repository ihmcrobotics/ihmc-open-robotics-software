package us.ihmc.euclid.referenceFrame;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class TranslationReferenceFrameTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testUpdateInMiddleFrameAtConstruction()
   {
      FrameVector3D translation1 = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.1, 0.13, 0.45);
      TranslationReferenceFrame frame1 = new TranslationReferenceFrame("frame1", ReferenceFrame.getWorldFrame(), translation1);

      FrameVector3D translation2 = new FrameVector3D(frame1, 0.7, 0.26, 0.09);
      TranslationReferenceFrame frame2 = new TranslationReferenceFrame("frame2", frame1, translation2);

      FrameVector3D translation3 = new FrameVector3D(frame2, 0.04, 0.023, 0.067);
      TranslationReferenceFrame frame3 = new TranslationReferenceFrame("frame3", frame2, translation3);

      frame1.setTranslationAndUpdate(translation1);
      frame2.setTranslationAndUpdate(translation2);
      frame3.setTranslationAndUpdate(translation3);

      RigidBodyTransform transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Vector3D totalTranslation = new Vector3D();

      Vector3D expectedTranslation = new Vector3D(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      totalTranslation.set(transformToDesiredFrame.getTranslation());

      EuclidCoreTestTools.assertEquals(expectedTranslation, totalTranslation, 1e-7);

      translation2.set(0.33, 0.44, 0.11);
      frame2.setTranslationAndUpdate(translation2);

      transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      totalTranslation = new Vector3D();

      expectedTranslation = new Vector3D(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      totalTranslation.set(transformToDesiredFrame.getTranslation());

      EuclidCoreTestTools.assertEquals(expectedTranslation, totalTranslation, 1e-7);
   }

   @Test
   public void testUpdateInMiddleFrame()
   {
      TranslationReferenceFrame frame1 = new TranslationReferenceFrame("frame1", ReferenceFrame.getWorldFrame());
      TranslationReferenceFrame frame2 = new TranslationReferenceFrame("frame2", frame1);
      TranslationReferenceFrame frame3 = new TranslationReferenceFrame("frame3", frame2);

      Vector3D translation1 = new Vector3D(0.1, 0.13, 0.45);
      Vector3D translation2 = new Vector3D(0.7, 0.26, 0.09);
      Vector3D translation3 = new Vector3D(0.04, 0.023, 0.067);

      frame1.setTranslationAndUpdate(translation1);
      frame2.setTranslationAndUpdate(translation2);
      frame3.setTranslationAndUpdate(translation3);

      RigidBodyTransform transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Vector3D totalTranslation = new Vector3D();

      Vector3D expectedTranslation = new Vector3D(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      totalTranslation.set(transformToDesiredFrame.getTranslation());

      EuclidCoreTestTools.assertEquals(expectedTranslation, totalTranslation, 1e-7);

      translation2.set(0.33, 0.44, 0.11);
      frame2.setTranslationAndUpdate(translation2);

      transformToDesiredFrame = frame3.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      totalTranslation = new Vector3D();

      expectedTranslation = new Vector3D(translation1);
      expectedTranslation.add(translation2);
      expectedTranslation.add(translation3);

      totalTranslation.set(transformToDesiredFrame.getTranslation());

      EuclidCoreTestTools.assertEquals(expectedTranslation, totalTranslation, 1e-7);
   }
}
