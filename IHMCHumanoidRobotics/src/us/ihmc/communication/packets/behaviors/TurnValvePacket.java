package us.ihmc.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TurnValvePacket extends Packet<TurnValvePacket>
{
   public RigidBodyTransform valveTransformToWorld;
   public double graspApproachConeAngle;
   public double valveRadius;
   public double turnValveAngle;
   public double valveRotationRateRadPerSec;

   public TurnValvePacket(Random random)
   {
      valveTransformToWorld = RigidBodyTransform.generateRandomTransform(random);
      graspApproachConeAngle = random.nextDouble();
      valveRadius = random.nextDouble();
      turnValveAngle = random.nextDouble();
      valveRotationRateRadPerSec = random.nextDouble();
   }
   
   public TurnValvePacket()
   {
   }

   public TurnValvePacket(RigidBodyTransform valveTransformToWorld, double graspApproachConeAngle, double valveRadius, double turnValveAngle, double valveRotationRateRadPerSec)
   {
      this.valveTransformToWorld = valveTransformToWorld;
      this.graspApproachConeAngle = graspApproachConeAngle;
      this.valveRadius = valveRadius;
      this.turnValveAngle = turnValveAngle;
      this.valveRotationRateRadPerSec = valveRotationRateRadPerSec;
   }

   public RigidBodyTransform getValveTransformToWorld()
   {
      return valveTransformToWorld;
   }

   public double getGraspApproachConeAngle()
   {
      return graspApproachConeAngle;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }

   public double getTurnValveAngle()
   {
      return turnValveAngle;
   }
   
   public double getValveRotationRate()
   {
      return valveRotationRateRadPerSec;
   }
   
   public boolean epsilonEquals(TurnValvePacket turnValvePacket, double epsilon)
   {
      boolean transformEquals = valveTransformToWorld.epsilonEquals(turnValvePacket.getValveTransformToWorld(), epsilon);
      boolean coneAngleEquals = graspApproachConeAngle == turnValvePacket.getGraspApproachConeAngle();
      boolean radiusEquals = valveRadius == turnValvePacket.getValveRadius();
      boolean turnAngleEquals = turnValveAngle == turnValvePacket.getTurnValveAngle();
      boolean rotatationRateEquals = valveRotationRateRadPerSec == turnValvePacket.valveRotationRateRadPerSec;

      return transformEquals && coneAngleEquals && radiusEquals && turnAngleEquals && rotatationRateEquals;
   }
}
