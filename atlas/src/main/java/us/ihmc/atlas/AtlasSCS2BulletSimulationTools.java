package us.ihmc.atlas;

import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasSCS2BulletSimulationTools
{
   public static void fixHumanoidCollisionGroupsMasksToPreventSelfCollision(RobotDefinition robotDefinition)
   {
      String defaultFilter = "DefaultFilter";
      String staticFilter = "StaticFilter";
      String kinematicFilter = "KinematicFilter";
      String debrisFilter = "DebrisFilter";
      String sensorTrigger = "SensorTrigger";
      String characterFilter = "CharacterFilter";
      String body = "Body";
      String pelvis = "Pelvis";
      String rightLeg = "RightLeg";
      String leftLeg = "LeftLeg";
      String rightArm = "RightArm";
      String leftArm = "LeftArm";
      String rightHand = "RightHand";
      String leftHand = "LeftHand";
      long bulletCollisionGroup;
      long bulletCollideMask;

      CollidableHelper helper = new CollidableHelper();

      // Set default Bullet collision groups/masks
      bulletCollisionGroup = helper.getCollisionMask(defaultFilter);
      bulletCollisionGroup = helper.getCollisionMask(staticFilter);
      bulletCollisionGroup = helper.getCollisionMask(kinematicFilter);
      bulletCollisionGroup = helper.getCollisionMask(debrisFilter);
      bulletCollisionGroup = helper.getCollisionMask(sensorTrigger);
      bulletCollisionGroup = helper.getCollisionMask(characterFilter);

      for (RigidBodyDefinition rigidBodyDefinition : robotDefinition.getAllRigidBodies())
      {
         for (CollisionShapeDefinition shapeDefinition : rigidBodyDefinition.getCollisionShapeDefinitions())
         {
            if (shapeDefinition.getName().contains("pelvis")
                || shapeDefinition.getName().contains("uglut")
                || shapeDefinition.getName().contains("lglut")
            )
            {
               bulletCollisionGroup = helper.getCollisionMask(pelvis);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, leftArm, rightArm);

            }
            else if (shapeDefinition.getName().contains("utorso")
                     || shapeDefinition.getName().contains("hokuyo")
                     || shapeDefinition.getName().contains("head"))
            {
               bulletCollisionGroup = helper.getCollisionMask(body);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, rightLeg, leftLeg, rightArm, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_uleg")
                     || shapeDefinition.getName().contains("l_lleg")
                     || shapeDefinition.getName().contains("l_talus")
                     || shapeDefinition.getName().contains("l_foot"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftLeg);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, rightLeg, leftArm);

            }
            else if (shapeDefinition.getName().contains("r_uleg")
                     || shapeDefinition.getName().contains("r_lleg")
                     || shapeDefinition.getName().contains("r_talus")
                     || shapeDefinition.getName().contains("r_foot"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightLeg);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, leftLeg, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_uarm")
                     || shapeDefinition.getName().contains("l_clav")
                     || shapeDefinition.getName().contains("l_scap")
                     || shapeDefinition.getName().contains("l_larm")
                     || shapeDefinition.getName().contains("l_ufarm")
                     || shapeDefinition.getName().contains("l_lfarm"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftArm);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, pelvis, leftLeg, rightLeg, rightArm);
            }
            else if (shapeDefinition.getName().contains("r_uarm")
                     || shapeDefinition.getName().contains("r_clav")
                     || shapeDefinition.getName().contains("r_scap")
                     || shapeDefinition.getName().contains("r_larm")
                     || shapeDefinition.getName().contains("r_ufarm")
                     || shapeDefinition.getName().contains("r_lfarm"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightArm);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, pelvis, leftLeg, rightLeg, leftArm);
            }
            else if (shapeDefinition.getName().contains("r_finger")
                  || shapeDefinition.getName().contains("r_palm")
                  || shapeDefinition.getName().contains("r_hand"))
            {
               bulletCollisionGroup = helper.getCollisionMask(rightHand);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, pelvis, leftLeg, rightLeg, leftArm);
            }
            else if (shapeDefinition.getName().contains("l_finger")
                  || shapeDefinition.getName().contains("l_palm")
                  || shapeDefinition.getName().contains("l_hand"))
            {
               bulletCollisionGroup = helper.getCollisionMask(leftHand);
               bulletCollideMask = helper.createCollisionGroup(defaultFilter, staticFilter, body, pelvis, leftLeg, rightLeg, rightArm);
            }
            else
            {
               bulletCollisionGroup = 1;
               bulletCollideMask = 1 + 2;
            }

            //bullet has it the opposite names as CollidableHelper e.g. a collisionMask is a collisionGroup in bullet
            shapeDefinition.setCollisionMask(bulletCollisionGroup);
            shapeDefinition.setCollisionGroup(bulletCollideMask);
         }
      }
   }
}
