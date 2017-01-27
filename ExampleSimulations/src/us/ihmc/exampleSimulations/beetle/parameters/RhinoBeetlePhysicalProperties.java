package us.ihmc.exampleSimulations.beetle.parameters;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public class RhinoBeetlePhysicalProperties
{
   private static final SegmentDependentList<RobotSextant, Vector3d> offsetsFromKneeToFootInWorld = new SegmentDependentList<>(RobotSextant.class);
   
   static
   {
      double frontX = 0.0;
      double frontY = 0.140;
      double frontZ = 0.0;
      offsetsFromKneeToFootInWorld.set(RobotSextant.FRONT_LEFT, new Vector3d(frontX, frontY, frontZ));
      offsetsFromKneeToFootInWorld.set(RobotSextant.FRONT_RIGHT, new Vector3d(frontX, -frontY, frontZ));
      
      double middleX = 0.0;
      double middleY = 0.140;
      offsetsFromKneeToFootInWorld.set(RobotSextant.MIDDLE_LEFT, new Vector3d(middleX, middleY,  0.0));
      offsetsFromKneeToFootInWorld.set(RobotSextant.MIDDLE_RIGHT, new Vector3d(middleX, -middleY,  0.0));
      
      double hindX = 0.0;
      double hindY = 0.140;
      double hindZ = 0.0;
      offsetsFromKneeToFootInWorld.set(RobotSextant.HIND_LEFT, new Vector3d(hindX, hindY, hindZ));
      offsetsFromKneeToFootInWorld.set(RobotSextant.HIND_RIGHT, new Vector3d(hindX, -hindY, hindZ));
   }
   
   public static Vector3d getOffsetFromJointBeforeFootToSoleAlignedWithWorld(RobotSextant robotSextant)
   {
      return offsetsFromKneeToFootInWorld.get(robotSextant);
   }
   
   public static SegmentDependentList<RobotSextant, Vector3d> getOffsetsFromJointBeforeFootToSoleAlignedWithWorld()
   {
      return offsetsFromKneeToFootInWorld;
   }
}
