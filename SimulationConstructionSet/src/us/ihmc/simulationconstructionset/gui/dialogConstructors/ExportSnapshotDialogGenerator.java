package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.commands.ExportSnapshotCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.SwingWorker;
import us.ihmc.tools.gui.MyFileFilter;

public class ExportSnapshotDialogGenerator implements ExportSnapshotDialogConstructor
{
   private ExportSnapshotCommandExecutor exportSnapshotCommandExecutor;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   
   @SuppressWarnings("unused")
   private ActiveCanvas3DHolder activeCanvas3DHolder;
   private JFrame frame;
   private JFileChooser fileChooser;

   // private javax.swing.filechooser.FileFilter stateFileFilter = new MyFileFilter(".state", "State (.state)");
   // private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(".data", "Data (.data)");
   private javax.swing.filechooser.FileFilter jpegFileFilter = new MyFileFilter(new String[] {".jpg", ".jpeg"}, "JPEG (.jpg, .jpeg)");

   // private javax.swing.filechooser.FileFilter configFileFilter = new MyFileFilter(".config", "Configuration (.config)");

   public ExportSnapshotDialogGenerator(ExportSnapshotCommandExecutor exportSnapshotCommandExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler, Robot[] robots, ActiveCanvas3DHolder activeCanvas3DHolder, JFrame frame)
   {
      this.exportSnapshotCommandExecutor = exportSnapshotCommandExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
      
      // this.canvas3D = canvas3D;
      this.activeCanvas3DHolder = activeCanvas3DHolder;
      this.frame = frame;

      try    // +++++++JEP: Applet Stuff
      {
         fileChooser = new JFileChooser();

         if (robots != null)
         {
            URL defaultDirURL = robots.getClass().getResource(".");
            if (defaultDirURL != null)
            {
               String defaultDirString = defaultDirURL.getPath();
               if (defaultDirString != null)
               {
                  int idx = defaultDirString.indexOf("classes");
                  if (idx > 0)
                     defaultDirString = defaultDirString.substring(0, idx);

                  setCurrentDirectory(defaultDirString);
               }
            }
         }

         fileChooser.addChoosableFileFilter(jpegFileFilter);
      }
      catch (Exception e)
      {
      }
   }

   @Override
   public void setCurrentDirectory(File dir)
   {
      fileChooser.setCurrentDirectory(dir);
   }

   @Override
   public void setCurrentDirectory(String dir)
   {
      fileChooser.setCurrentDirectory(new File(dir));
   }


   @Override
   public void constructDialog()
   {
      guiEnablerAndDisabler.disableGUIComponents();

      // Do Snapshot
      @SuppressWarnings("unused")
      SwingWorker worker = new SwingWorker()
      {
         @Override
         public Object construct()
         {
            // synchronized(BusyLock.lock)
            // {
            try
            {
               if (fileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
               {
                  File chosenFile = fileChooser.getSelectedFile();

                  exportSnapshotCommandExecutor.exportSnapshot(chosenFile);

//                //if (chosenFile.canWrite())
//                {
//                  YoCanvas3D canvas3D = activeCanvas3DHolder.getActiveCanvas3D();
//
//                  canvas3D.screenShot();
//                  // Wait for the screen shot to finish:
//                  while (!canvas3D.screenShotFinished())
//                  {
//                    try {Thread.sleep(10);}
//                    catch(InterruptedException exp){}
//                  }
//
//                  BufferedImage img = canvas3D.getBufferedImage();
//                  //save out to a file
//
//                  try
//                  {
//
//                    FileOutputStream out = new FileOutputStream(chosenFile);
//                    JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(out);
//                    JPEGEncodeParam param = encoder.getDefaultJPEGEncodeParam(img);
//                    //this produces a JPEG with .1 lossy
//
//                    param.setQuality(0.9f,false);
//                    encoder.setJPEGEncodeParam(param);
//                    encoder.encode(img);
//                    out.close();
//                  }
//                  catch ( IOException exp )
//                  {
//                    System.out.println("I/O exception!"+exp.toString());
//                  }
//
//                }

               }

            }
            catch (Exception exp)
            {
               exp.printStackTrace();
            }

            return new Object();

            // }
         }

         @Override
         public void finished()
         {
            // Runs in Swing dispatching thread when done...

            // Enable buttons:
            guiEnablerAndDisabler.enableGUIComponents();
         }

      };
   }

   public void closeAndDispose()
   {
      exportSnapshotCommandExecutor = null;
      guiEnablerAndDisabler = null;

      activeCanvas3DHolder = null;
      frame = null;
      fileChooser = null;

      jpegFileFilter = null;
   }

}

