package us.ihmc.robotics.referenceFrames;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import static org.junit.jupiter.api.Assertions.*;

public class ConditionalReferenceFrameTest
{
   @Test
   public void testAnNonExistentParentFrame()
   {
      // Update with a ReferenceFrameLibrary
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", "someParentFrameNameThatDoesNotExist");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(ConditionalReferenceFrame.INVALID_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());

      // Update without a ReferenceFrameLibrary
      conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", "someParentFrameNameThatDoesNotExist");
      conditionalReferenceFrame.update();
      assertEquals(ConditionalReferenceFrame.INVALID_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
   }

   @Test
   public void testAnExistentParentFrame()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame frame0 = new PoseReferenceFrame("frame0", worldFrame);

      // Update with a ReferenceFrameLibrary
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.add(() -> frame0);
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());

      // Update without a ReferenceFrameLibrary
      conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update();
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
   }

   @Test
   public void testChangingFromAnExistentParentFrameToANonExistentParentFrame()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame frame0 = new PoseReferenceFrame("frame0", worldFrame);

      // Update with a ReferenceFrameLibrary
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.add(() -> frame0);
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame1");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(ConditionalReferenceFrame.INVALID_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());

      // Update without a ReferenceFrameLibrary
      conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update();
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame1");
      conditionalReferenceFrame.update();
      assertEquals(ConditionalReferenceFrame.INVALID_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
   }

   @Test
   public void testChangingFromAnExistentParentFrameToANonExistentParentFrameAndBack()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame frame0 = new PoseReferenceFrame("frame0", worldFrame);

      // Update with a ReferenceFrameLibrary
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.add(() -> frame0);
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame1");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(ConditionalReferenceFrame.INVALID_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame0");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());

      // Update without a ReferenceFrameLibrary
      conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update();
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame1");
      conditionalReferenceFrame.update();
      assertEquals(ConditionalReferenceFrame.INVALID_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame0");
      conditionalReferenceFrame.update();
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
   }
}
