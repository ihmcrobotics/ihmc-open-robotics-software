package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

import us.ihmc.simulationconstructionset.commands.ExportVideoCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.videos.VideoSaveDialog;

public class MediaCaptureDialogGenerator implements MediaCaptureDialogConstructor
{
   private ViewportSelectorCommandExecutor viewportSelector;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   private StopCommandExecutor stopCommandExecutor;
   private ExportVideoCommandExecutor exportVideoCommandExecutor;
   private StandardSimulationGUI myGUI;
   
   private StandardGUIActions standardGUIActions;
   private ActiveCanvas3DHolder activeCanvas3DHolder;
   
   
   public MediaCaptureDialogGenerator(ExportVideoCommandExecutor exportVideoCommandExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler,
         StopCommandExecutor stopCommandExecutor, ViewportSelectorCommandExecutor viewportSelector, StandardSimulationGUI myGUI, StandardGUIActions standardGUIActions, ActiveCanvas3DHolder activeCanvas3DHolder)
   {
      this.exportVideoCommandExecutor = exportVideoCommandExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
      this.stopCommandExecutor = stopCommandExecutor;
      this.viewportSelector = viewportSelector;
      
      this.myGUI = myGUI;
      this.standardGUIActions = standardGUIActions;
      this.activeCanvas3DHolder = activeCanvas3DHolder;
   }

   public void createVideo(File file)
   {
      exportVideoCommandExecutor.createVideo(file);
   }

   public void constructDialog()
   {
      stopCommandExecutor.stop();
      new VideoSaveDialog(null, myGUI, standardGUIActions, activeCanvas3DHolder, exportVideoCommandExecutor, guiEnablerAndDisabler);    

   }

   public void closeAndDispose()
   {
      viewportSelector = null;
      guiEnablerAndDisabler = null;
      stopCommandExecutor = null;
      exportVideoCommandExecutor = null;
      myGUI = null;

      standardGUIActions = null;
      activeCanvas3DHolder = null;
   }
}

