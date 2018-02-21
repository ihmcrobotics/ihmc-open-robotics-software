package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class HumanoidBehaviorTypePacket extends Packet<HumanoidBehaviorTypePacket>
{
   public static final byte STOP = 0;
   public static final byte TEST = 1;
   public static final byte WALK_TO_LOCATION = 2;
   public static final byte WALK_TO_GOAL = 3;
   public static final byte DIAGNOSTIC = 4;
   public static final byte PICK_UP_BALL = 5;
   public static final byte RESET_ROBOT = 6;
   public static final byte TURN_VALVE = 7;
   public static final byte WALK_THROUGH_DOOR = 8;
   public static final byte EXAMPLE_BEHAVIOR = 9;
   public static final byte BALL_DETECTION = 10;
   public static final byte TEST_PIPELINE = 11;
   public static final byte TEST_STATEMACHINE = 12;
   public static final byte FOLLOW_FIDUCIAL_50 = 13;
   public static final byte LOCATE_FIDUCIAL = 14;
   public static final byte WAlK_OVER_TERRAIN = 15;
   public static final byte FOLLOW_VALVE = 16;
   public static final byte LOCATE_VALVE = 17;
   public static final byte WALK_OVER_TERRAIN_TO_VALVE = 18;
   public static final byte DEBUG_PARTIAL_FOOTHOLDS = 19;
   public static final byte WALK_TO_GOAL_ANYTIME_PLANNER = 20;
   public static final byte TEST_ICP_OPTIMIZATION = 21;
   public static final byte TEST_GC_GENERATION = 22;
   public static final byte TEST_SMOOTH_ICP_PLANNER = 23;
   public static final byte PUSH_AND_WALK = 24;
   public static final byte COLLABORATIVE_TASK = 25;
   public static final byte FIRE_FIGHTING = 26;
   public static final byte CUTTING_WALL = 27;
   public static final byte REPEATEDLY_WALK_FOOTSTEP_LIST = 28;

   public byte humanoidBehaviorType;

   // empty constructor for deserialization
   public HumanoidBehaviorTypePacket()
   {
   }

   @Override
   public void set(HumanoidBehaviorTypePacket other)
   {
      humanoidBehaviorType = other.humanoidBehaviorType;
      setPacketInformation(other);
   }

   public byte getHumanoidBehaviorType()
   {
      return humanoidBehaviorType;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorTypePacket other, double epsilon)
   {
      return humanoidBehaviorType == other.humanoidBehaviorType;
   }
}
