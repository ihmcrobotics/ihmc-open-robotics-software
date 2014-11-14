package us.ihmc.simulationconstructionset.movies;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.camera.CaptureDevice;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;

import us.ihmc.simulationconstructionset.commands.ExportMovieCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.TickUpdateListener;
import us.ihmc.simulationconstructionset.gui.ViewportPanel;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;

public class MovieSaveDialog implements TickUpdateListener
{
   private final Dimension[] dimensions = {new Dimension(640, 360), new Dimension(1280, 720), new Dimension(1920, 1080), new Dimension(640, 480),
           new Dimension(800, 600)};

   private final int frameWidth = 720;
   private final int frameHeight = 600;
   private final int previewWidth = frameWidth;
   private final int previewHeight = 360;

   private final Graphics3DAdapter graphics3dAdapter;
   private final ExportMovieCommandExecutor exportMovieCommandExecutor;
   private final GUIEnablerAndDisabler guiEnablerAndDisabler;
   private final StandardSimulationGUI myGUI;

   private JDialog exportDialog;
   private ViewportPanel viewportPanel;
   private JTextField frameRateTextField;
   private JTextField playbackRateTextField;
   
   private Dimension dimension;

   public MovieSaveDialog(Window owner, StandardSimulationGUI myGUI, StandardGUIActions standardGUIActions, ActiveCanvas3DHolder activeCanvas3DHolder,
                          ExportMovieCommandExecutor exportMovieCommandExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler)
   {
      exportDialog = new JDialog(owner, "Export Movie");
      exportDialog.setName("Export Movie");
      
      graphics3dAdapter = myGUI.getGraphics3dAdapter();
      this.exportMovieCommandExecutor = exportMovieCommandExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
      this.myGUI = myGUI;

      exportDialog.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            close();
         }
      });
      exportDialog.setSize(frameWidth, frameHeight);

      Container container = exportDialog.getContentPane();
      container.setLayout(null);

      viewportPanel = myGUI.createViewportPanel();

      viewportPanel.getCamera().copyPositionTrackingDollyConfiguration(activeCanvas3DHolder.getCamera());

      setResolution(dimensions[0]);

      JPanel previewPanel = new JPanel();
      previewPanel.setLayout(null);
      previewPanel.setBounds(0, 0, previewWidth, previewHeight);
      previewPanel.add(viewportPanel);
      container.add(previewPanel);

      JPanel panel = new JPanel();
      panel.setBounds(0, previewHeight, frameWidth, frameHeight - previewHeight);
      container.add(panel);

      StandardGUIActions movieActions = new StandardGUIActions();
      movieActions.createMovieExportActions(standardGUIActions, viewportPanel);
      JPanel toolbar = movieActions.createMovieExportPanelButtons();
      panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
      panel.add(toolbar);

      JPanel settings = new JPanel();
      JComboBox resolutions = new JComboBox(dimensions);
      resolutions.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            setResolution((Dimension) ((JComboBox) e.getSource()).getSelectedItem());
         }
      });

      JPanel frPanel = new JPanel();
      JLabel frameRateLabel = new JLabel("Frame Rate");
      frameRateTextField = new JTextField("30",3);
      frPanel.add(frameRateLabel);
      frPanel.add(frameRateTextField);

      JPanel pbPanel = new JPanel();

      JLabel playbackRateLabel = new JLabel("Playback Rate");
      playbackRateTextField = new JTextField("1",3);
      pbPanel.add(playbackRateLabel);
      pbPanel.add(playbackRateTextField);

      JButton export = new JButton("Export");
      export.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            selectMovieLoction();
         }
      });

      JButton close = new JButton("Close");
      close.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            close();
         }
      });

      settings.add(resolutions, BorderLayout.CENTER);
      settings.add(frPanel, BorderLayout.CENTER);
      settings.add(pbPanel, BorderLayout.CENTER);
      settings.add(export, BorderLayout.CENTER);
      settings.add(close, BorderLayout.CENTER);

      panel.add(settings);


      myGUI.addTickUpdateListener(this);

      exportDialog.setVisible(true);
   }

   private void selectMovieLoction()
   {
      JFileChooser saveDialog = new JFileChooser(System.getProperty("user.home"));
      saveDialog.setFileFilter(new MovieFileFilter());
      File selectedFile = null;
      if (JFileChooser.APPROVE_OPTION == saveDialog.showSaveDialog(exportDialog))
      {
         selectedFile = saveDialog.getSelectedFile();
      }

      if (selectedFile != null)
      {
         float frameRate = new Float(frameRateTextField.getText());
         float playRate = new Float(playbackRateTextField.getText());

         exportMovie(selectedFile,frameRate,playRate);
      }
   }

   private void exportMovie(File selectedFile, float frameRate, float playbackspeed)
   {
      guiEnablerAndDisabler.disableGUIComponents();

      ViewportAdapter adapter = graphics3dAdapter.createNewViewport(null, false, true);
      adapter.setupOffscreenView((int) dimension.getWidth(), (int) dimension.getHeight());
      adapter.setCameraController(viewportPanel.getCamera());

      CaptureDevice captureDevice = adapter.getCaptureDevice();
      exportMovieCommandExecutor.createMovie(captureDevice, selectedFile, false, playbackspeed, frameRate);

      graphics3dAdapter.closeViewport(adapter);
      guiEnablerAndDisabler.enableGUIComponents();
   }

   private void close()
   {
      exportDialog.setVisible(false);
      viewportPanel.closeAndDispose();
      viewportPanel = null;
      exportDialog.dispose();
      exportDialog = null;
      guiEnablerAndDisabler.enableGUIComponents();
      myGUI.removeTickUpdateListener(this);
   }

   private void setResolution(Dimension d)
   {
      double aspectRatio = ((double) d.getWidth()) / ((double) d.getHeight());

      double screenAspect = ((double) previewWidth) / ((double) previewHeight);

      int width, height;
      if (aspectRatio > screenAspect)
      {
         width = previewWidth;
         height = (int) (1.0 / aspectRatio * previewWidth);
      }
      else
      {
         width = (int) (aspectRatio * previewHeight);
         height = previewHeight;
      }

      viewportPanel.setBounds((previewWidth - width) / 2, (previewHeight - height) / 2, width, height);
      viewportPanel.repaint();
      viewportPanel.revalidate();
      dimension = d;
   }

   public void update(int tick)
   {
      if (viewportPanel != null)
      {
         if (viewportPanel.getCamera().useKeyCameraPoints())
         {
            viewportPanel.getCamera().setKeyFrameTime(tick);
         }
      }
   }

}
