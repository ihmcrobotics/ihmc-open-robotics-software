package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

import us.ihmc.simulationconstructionset.commands.ExportMovieCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.movies.MovieSaveDialog;

public class MediaCaptureDialogGenerator implements MediaCaptureDialogConstructor
{
   private ViewportSelectorCommandExecutor viewportSelector;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   private StopCommandExecutor stopCommandExecutor;
   private ExportMovieCommandExecutor exportMovieCommandExecutor;
   private StandardSimulationGUI myGUI;
   
   private StandardGUIActions standardGUIActions;
   private ActiveCanvas3DHolder activeCanvas3DHolder;
   
   
   public MediaCaptureDialogGenerator(ExportMovieCommandExecutor exportMovieCommandExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler,
         StopCommandExecutor stopCommandExecutor, ViewportSelectorCommandExecutor viewportSelector, StandardSimulationGUI myGUI, StandardGUIActions standardGUIActions, ActiveCanvas3DHolder activeCanvas3DHolder)
   {
      this.exportMovieCommandExecutor = exportMovieCommandExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
      this.stopCommandExecutor = stopCommandExecutor;
      this.viewportSelector = viewportSelector;
      
      this.myGUI = myGUI;
      this.standardGUIActions = standardGUIActions;
      this.activeCanvas3DHolder = activeCanvas3DHolder;
   }

   public void createMovie(File file)
   {
      exportMovieCommandExecutor.createMovie(file);
   }

   public void constructMediaCaptureDialog()
   {
      stopCommandExecutor.stop();
      new MovieSaveDialog(null, myGUI, standardGUIActions, activeCanvas3DHolder, exportMovieCommandExecutor, guiEnablerAndDisabler);    

   }

   public void closeAndDispose()
   {
      viewportSelector = null;
      guiEnablerAndDisabler = null;
      stopCommandExecutor = null;
      exportMovieCommandExecutor = null;
      myGUI = null;

      standardGUIActions = null;
      activeCanvas3DHolder = null;
   }
}

