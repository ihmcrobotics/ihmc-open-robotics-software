package us.ihmc.robotics.referenceFrames;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import static org.junit.jupiter.api.Assertions.*;

public class ConditionalReferenceFrameTest
{
   @Test
   public void testAnNonExistentParentFrame()
   {
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", "someParentFrameNameThatDoesNotExist");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(ConditionalReferenceFrame.NULL_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
   }

   @Test
   public void testAnExistentParentFrame()
   {
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame frame0 = new PoseReferenceFrame("frame0", worldFrame);
      referenceFrameLibrary.add(()-> frame0);
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
   }

   @Test
   public void testChangingFromAnExistentParentFrameToANonExistentParentFrame()
   {
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame frame0 = new PoseReferenceFrame("frame0", worldFrame);
      referenceFrameLibrary.add(()-> frame0);
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame1");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(ConditionalReferenceFrame.NULL_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
   }

   @Test
   public void testChangingFromAnExistentParentFrameToANonExistentParentFrameAndBack()
   {
      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame frame0 = new PoseReferenceFrame("frame0", worldFrame);
      referenceFrameLibrary.add(()-> frame0);
      ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame("testFrame", frame0.getName());
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame1");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(ConditionalReferenceFrame.NULL_FRAME.getName(), conditionalReferenceFrame.get().getParent().getName());
      conditionalReferenceFrame.setParentFrameName("frame0");
      conditionalReferenceFrame.update(referenceFrameLibrary);
      assertEquals(frame0.getName(), conditionalReferenceFrame.get().getParent().getName());
   }
}
