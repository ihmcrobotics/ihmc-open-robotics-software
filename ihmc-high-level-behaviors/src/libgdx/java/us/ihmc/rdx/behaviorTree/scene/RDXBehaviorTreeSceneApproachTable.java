package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

public class RDXBehaviorTreeSceneApproachTable extends RDXBehaviorTreeSceneObject
{
   private long leftTablePoints = 0;
   private long rightTablePoints = 0;
   private final SideDependentList<ModelInstance> capsules = new SideDependentList<>();

   public RDXBehaviorTreeSceneApproachTable(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);

      for (RobotSide side : RobotSide.values)
      {
         ModelInstance modelInstance = RDXModelBuilder.buildModelInstance(modelBuilder ->
                            modelBuilder.addCapsule(0.6, 0.05, new Vector3D(), LibGDXTools.toLibGDX(ColorDefinitions.GreenYellow())));
         LibGDXTools.setOpacity(modelInstance, 0.3f);
         capsules.put(side, modelInstance);
      }
   }

   @Override
   protected void renderDetectionInfo()
   {
      ImGui.beginDisabled(!getPersistentDetection().getIsStable());
      ImGui.text("Left table points: %d".formatted(leftTablePoints));
      ImGui.text("Right table points: %d".formatted(rightTablePoints));
      ImGui.endDisabled();
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      leftTablePoints = message.getLeftTablePoints();
      rightTablePoints = message.getRightTablePoints();

      LibGDXTools.toLibGDX(message.getLeftCapsuleCenter(), capsules.get(RobotSide.LEFT).transform);
      LibGDXTools.toLibGDX(message.getRightCapsuleCenter(), capsules.get(RobotSide.RIGHT).transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRenderables(renderables, pool);

      for (RobotSide side : RobotSide.values)
         capsules.get(side).getRenderables(renderables, pool);
   }
}
