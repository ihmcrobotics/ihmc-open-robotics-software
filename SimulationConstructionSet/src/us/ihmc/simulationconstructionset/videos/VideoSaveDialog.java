package us.ihmc.simulationconstructionset.videos;

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

import us.ihmc.simulationconstructionset.commands.ExportVideoCommandExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.TickUpdateListener;
import us.ihmc.simulationconstructionset.gui.ViewportPanel;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;

public class VideoSaveDialog implements TickUpdateListener
{
   private final Resolution[] dimensions = {new Resolution(640, 360), new Resolution(1280, 720), new Resolution(1920, 1080), new Resolution(640, 480),
           new Resolution(800, 600)};

   private final int frameWidth = 720;
   private final int frameHeight = 600;
   private final int previewWidth = frameWidth;
   private final int previewHeight = 360;

   private final GUIEnablerAndDisabler guiEnablerAndDisabler;
   private final StandardSimulationGUI myGUI;

   private JDialog exportDialog;
   private ViewportPanel viewportPanel;
   private JTextField frameRateTextField;
   private JTextField playbackRateTextField;
   
   private final Dimension dimension = new Dimension(1900, 1080);
   private final ExportVideoCommandExecutor exportVideoCommandExecutor;
      
   public VideoSaveDialog(Window owner, StandardSimulationGUI myGUI, StandardGUIActions standardGUIActions, ActiveCanvas3DHolder activeCanvas3DHolder,
                          ExportVideoCommandExecutor exportVideoCommandExecutor, GUIEnablerAndDisabler guiEnablerAndDisabler)
   {      
      this.exportVideoCommandExecutor = exportVideoCommandExecutor;

      exportDialog = new JDialog(owner, "Export Video");
      exportDialog.setName("Export Video");
      
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

      StandardGUIActions videoActions = new StandardGUIActions();
      videoActions.createVideoExportActions(standardGUIActions, viewportPanel);
      JPanel toolbar = videoActions.createVideoExportPanelButtons();
      panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
      panel.add(toolbar);

      JPanel settings = new JPanel();
      JComboBox <Dimension> resolutions = new JComboBox<Dimension>(dimensions);

      resolutions.addActionListener(new ActionListener()
      {
         @Override
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
         @Override
         public void actionPerformed(ActionEvent e)
         {
            selectVideoLoction();
         }
      });

      JButton close = new JButton("Close");
      close.addActionListener(new ActionListener()
      {
         @Override
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

   private void selectVideoLoction()
   {
      JFileChooser saveDialog = new JFileChooser(System.getProperty("user.home"));
      saveDialog.setFileFilter(new VideoFileFilter());
      File selectedFile = null;
      if (JFileChooser.APPROVE_OPTION == saveDialog.showSaveDialog(exportDialog))
      {
         selectedFile = saveDialog.getSelectedFile();
      }

      if (selectedFile != null)
      {
         float frameRate = new Float(frameRateTextField.getText());
         float playBackRate = new Float(playbackRateTextField.getText());

         Boolean isSequanceSelected = false;
         exportVideoCommandExecutor.createVideo(viewportPanel.getCamera(), selectedFile, dimension, isSequanceSelected , playBackRate, frameRate);
      }
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

   private void setResolution(Dimension dimension)
   {
      double aspectRatio = ((double) dimension.getWidth()) / ((double) dimension.getHeight());
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
      
      this.dimension.setSize(dimension);
   }

   @Override
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

   private class Resolution extends Dimension
   {
      public Resolution() {
      }

      public Resolution(Dimension d) {
         super(d);
      }

      public Resolution(int width, int height) {
         super(width, height);
      }

      @Override
      public String toString() {
         return width + " x " + height;
      }
   }
}
