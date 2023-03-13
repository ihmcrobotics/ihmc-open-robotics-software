package us.ihmc.simulationConstructionSetTools.tools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.PhysicsEngineTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.ArrayList;
import java.util.List;

public class CollidableTools
{
   public static List<Collidable> toCollidables(long collisionMask, long collisionGroup, TerrainObject3D terrainObject3D)
   {
      List<Collidable> collidables = new ArrayList<>();

      for (Shape3DReadOnly terrainShape : terrainObject3D.getTerrainCollisionShapes())
      {
         FrameShape3DBasics frameShape3DBasics = PhysicsEngineTools.toFrameShape3DBasics(ReferenceFrame.getWorldFrame(), terrainShape);
         if (frameShape3DBasics instanceof FrameRamp3DBasics)
         { // FIXME: Workaround the RampTerrainObject that doesn't initialize the Ramp3D shape properly.
            ((FrameRamp3DBasics) frameShape3DBasics).getPose().appendTranslation(-0.5 * ((Ramp3DReadOnly) frameShape3DBasics).getSizeX(), 0.0, 0.0);
         }
         collidables.add(new Collidable(null, collisionMask, collisionGroup, frameShape3DBasics));
      }

      return collidables;
   }

   public static List<Collidable> toCollidables(long collisionMask, long collisionGroup, CommonAvatarEnvironmentInterface environment)
   {
      return toCollidables(collisionMask, collisionGroup, environment.getTerrainObject3D());
   }
}
