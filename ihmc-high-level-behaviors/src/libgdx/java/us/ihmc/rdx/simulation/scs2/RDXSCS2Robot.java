package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class RDXSCS2Robot
{
   private final RobotDefinition robotDefinition;
   private final YoRegistry mirroredRobotRegistry;
   private final RigidBodyBasics originalRigidBody;
   private RDXRigidBody rootBody;
   private LinkedYoRegistry robotLinkedYoRegistry;
   private boolean initialize = true;

   public RDXSCS2Robot(RobotDefinition robotDefinition)
   {
      this.robotDefinition = robotDefinition;
      mirroredRobotRegistry = SharedMemoryTools.newRegistryFromNamespace(SimulationSession.ROOT_REGISTRY_NAME, robotDefinition.getName());
      originalRigidBody = robotDefinition.newInstance(ReferenceFrameTools.constructARootFrame("dummy"));
   }

   public void create(RDXYoManager yoManager)
   {
      rootBody = RDXMultiBodySystemFactories.toYoRDXMultiBodySystem(originalRigidBody,
                                                                    ReferenceFrame.getWorldFrame(),
                                                                    robotDefinition,
                                                                    mirroredRobotRegistry,
                                                                    Gdx.app::postRunnable);
      RDXMultiBodySystemFactories.setupFourbars(rootBody, robotDefinition, RDXVisualTools.NO_SCALING, false);
      robotLinkedYoRegistry = yoManager.newLinkedYoRegistry(mirroredRobotRegistry);
      mirroredRobotRegistry.getVariables().forEach(yoVariable ->
      {
         LinkedYoVariable<YoVariable> linkYoVariable = robotLinkedYoRegistry.linkYoVariable(yoVariable);
         linkYoVariable.addUser(this);
      });
   }

   public void update()
   {
      if (robotLinkedYoRegistry.pull() || initialize)
      {
         rootBody.updateFramesRecursively();
         rootBody.updateSubtreeGraphics();
         initialize = false;
      }
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      rootBody.getVisualRenderables(renderables, pool);
   }

   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      rootBody.getCollisionMeshRenderables(renderables, pool);
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public RDXRigidBody getRootBody()
   {
      return rootBody;
   }
}
