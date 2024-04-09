package us.ihmc.avatar.sakeGripper;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.*;

public class SakeHandParametersTest
{
   private static final double EPSILON = 0.001;

   private static final double POSITION_LOWER_LIMIT = 2.4467;
   private static final double POSITION_UPPER_LIMIT = 6.2817;

   @Test
   public void testDenormalizeHandOpenAngle()
   {
      double openAngle = denormalizeHandOpenAngle(1.0);
      assertEquals(Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES), openAngle, EPSILON);

      double closedAngle = denormalizeHandOpenAngle(0.0);
      assertEquals(0.0, closedAngle, EPSILON);

      double inBetweenAngle = denormalizeHandOpenAngle(0.3);
      assertEquals(Math.toRadians(63.0), inBetweenAngle, EPSILON);
   }

   @Test
   public void testNormalizeHandOpenAngle()
   {
      double normalizedOpen = normalizeHandOpenAngle(Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES));
      assertEquals(1.0, normalizedOpen, EPSILON);

      double normalizedClosed = normalizeHandOpenAngle(0.0);
      assertEquals(0.0, normalizedClosed, EPSILON);

      double normalizedInBetween = normalizeHandOpenAngle(Math.toRadians(63.0));
      assertEquals(0.3, normalizedInBetween, EPSILON);
   }

   @Test
   public void testDenormalizeHandPosition()
   {
      double openHandPosition = denormalizeHandPosition(0.0, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(POSITION_LOWER_LIMIT, openHandPosition, EPSILON);

      double closedHandPosition = denormalizeHandPosition(1.0, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(POSITION_UPPER_LIMIT, closedHandPosition, EPSILON);

      double inBetweenHandPosition = denormalizeHandPosition(0.3, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      double trueValue = 0.3 * (POSITION_UPPER_LIMIT - POSITION_LOWER_LIMIT) + POSITION_LOWER_LIMIT;
      assertEquals(trueValue, inBetweenHandPosition, EPSILON);
   }

   @Test
   public void testNormalizeHandPosition()
   {
      double normalizedOpenPosition = normalizeHandPosition(POSITION_LOWER_LIMIT, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(0.0, normalizedOpenPosition, EPSILON);

      double normalizedClosedPosition = normalizeHandPosition(POSITION_UPPER_LIMIT, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(1.0, normalizedClosedPosition, EPSILON);

      double normalizedInBetweenPosition = normalizeHandPosition(3.597185, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(0.3, normalizedInBetweenPosition, EPSILON);
   }

   @Test
   public void testHandOpenAngleToPosition()
   {
      double openPosition = handOpenAngleToPosition(Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES), POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(POSITION_LOWER_LIMIT, openPosition, EPSILON);

      double closedPosition = handOpenAngleToPosition(0.0, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(POSITION_UPPER_LIMIT, closedPosition, EPSILON);

      double inBetweenPosition = handOpenAngleToPosition(Math.toRadians(63.0), POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      double trueValue = 0.7 * (POSITION_UPPER_LIMIT - POSITION_LOWER_LIMIT) + POSITION_LOWER_LIMIT;
      assertEquals(trueValue, inBetweenPosition, EPSILON);
   }

   @Test
   public void testHandPositionToOpenAngle()
   {
      double openAngle = handPositionToOpenAngle(POSITION_LOWER_LIMIT, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES), openAngle, EPSILON);

      double closedAngle = handPositionToOpenAngle(POSITION_UPPER_LIMIT, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(0.0, closedAngle, EPSILON);

      double position = 0.7 * (POSITION_UPPER_LIMIT - POSITION_LOWER_LIMIT) + POSITION_LOWER_LIMIT;
      double inBetweenAngle = handPositionToOpenAngle(position, POSITION_LOWER_LIMIT, POSITION_UPPER_LIMIT);
      assertEquals(Math.toRadians(63.0), inBetweenAngle, EPSILON);
   }
}
