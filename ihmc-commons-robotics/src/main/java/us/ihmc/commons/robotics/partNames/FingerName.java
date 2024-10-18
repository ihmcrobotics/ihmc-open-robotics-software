package us.ihmc.commons.robotics.partNames;

public enum FingerName
{
   THUMB, INDEX, MIDDLE, PINKY;
   
   public static final FingerName[] threeFingerValues = new FingerName[] {THUMB, INDEX, MIDDLE};
   
   public static final FingerName[] fourFingerValues = new FingerName[] {THUMB, INDEX, MIDDLE, PINKY};
}
