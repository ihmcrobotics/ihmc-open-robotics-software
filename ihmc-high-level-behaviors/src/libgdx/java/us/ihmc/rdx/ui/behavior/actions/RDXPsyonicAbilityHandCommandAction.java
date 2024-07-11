package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripSpeed;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripType;
import us.ihmc.behaviors.sequence.actions.PsyonicAbilityHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.PsyonicAbilityHandCommandActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXPsyonicAbilityHandCommandAction extends RDXActionNode<PsyonicAbilityHandCommandActionState, PsyonicAbilityHandCommandActionDefinition>
{
   private final PsyonicAbilityHandCommandActionDefinition definition;

   private final ImInt currentLegacyGripCommandIndex = new ImInt();
   private final ImInt currentLegacyGripSpeedIndex = new ImInt();

   private final String[] legacyGripCommandNames;
   private final String[] legacyGripSpeedNames;

   public RDXPsyonicAbilityHandCommandAction(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new PsyonicAbilityHandCommandActionState(id, crdtInfo, saveFileDirectory));

      definition = getDefinition();

      definition.setName(getActionTypeTitle());

      legacyGripCommandNames = new String[LegacyGripType.values().length];
      for (int i = 0; i < LegacyGripType.values().length; i++)
         legacyGripCommandNames[i] = LegacyGripType.values()[i].name();

      legacyGripSpeedNames = new String[LegacyGripSpeed.values().length];
      for (int i = 0; i < LegacyGripSpeed.values().length; i++)
         legacyGripSpeedNames[i] = LegacyGripSpeed.values()[i].name();
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(100.0f);

      ImGui.text("(Legacy) Grip commands");
      ImGui.text(definition.getSide().getCamelCaseNameForMiddleOfExpression());

      String currentGripCommand = definition.getLegacyGripType().name();
      String currentSpeed = definition.getLegacyGripSpeed().name();

      for (int i = 0; i < legacyGripCommandNames.length; i++)
      {
         if (legacyGripCommandNames[i].equals(currentGripCommand))
            currentLegacyGripCommandIndex.set(i);
      }

      for (int i = 0; i < legacyGripSpeedNames.length; i++)
      {
         if (legacyGripSpeedNames[i].equals(currentSpeed))
            currentLegacyGripSpeedIndex.set(i);
      }

      if (ImGui.combo("Grip command##AbilityHand" + definition.getSide().getLowerCaseName(), currentLegacyGripCommandIndex, legacyGripCommandNames))
      {
         LegacyGripType gripType = LegacyGripType.valueOf(legacyGripCommandNames[currentLegacyGripCommandIndex.get()]);
         definition.setLegacyGripType(gripType);
      }
      if (ImGui.combo("Grip speed##AbilityHand" + definition.getSide().getLowerCaseName(), currentLegacyGripSpeedIndex, legacyGripSpeedNames))
      {
         LegacyGripSpeed gripSpeed = LegacyGripSpeed.valueOf(legacyGripSpeedNames[currentLegacyGripSpeedIndex.get()]);
         definition.setLegacyGripSpeed(gripSpeed);
      }

      ImGui.separator();
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();
      ImGui.sameLine();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration (PSYONIC)";
   }
}
