package us.ihmc.robotDataCommunication.logger;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import com.yobotics.simulationconstructionset.LongYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;

public class YoVariableLogVisualizerGUI extends JFrame
{
   private static final long serialVersionUID = 6414551517161321596L;
   
   private final VideoDataPlayer player;
   private final YoVariableLogPlaybackRobot robot;
   private final SimulationConstructionSet scs;

   public YoVariableLogVisualizerGUI(VideoDataPlayer player, YoVariableLogPlaybackRobot robot, SimulationConstructionSet scs)
   {
      super();
      
      this.player = player;
      this.robot = robot;
      this.scs = scs;
      
      setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      setTitle("YoVariable Log playback");
      setSize(600, 100);
      
      setLocationByPlatform(true);
      setLocationRelativeTo(null);
      
      getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
      
      addGUIElements();
      
      setVisible(true);
   }
   
   private void switchVideoUpdate(boolean update)
   {
       player.updateVideo(update);
   }

   private void seek(int newValue)
   {
      if(!scs.isSimulating())
      {
         robot.seek(newValue);
         try
         {
            scs.simulateOneRecordStepNow();
         }
         catch (UnreasonableAccelerationException e)
         {
            e.printStackTrace();
         }
         player.indexChanged(0, 0);
      }
   }
   
   private void addGUIElements()
   {
      final JCheckBox showVideo = new JCheckBox("Update video", true);
      showVideo.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            switchVideoUpdate(showVideo.isSelected());
         }
      });
      
      getContentPane().add(showVideo);
      
      JPanel timePanel = new JPanel();
      timePanel.setLayout(new BoxLayout(timePanel, BoxLayout.X_AXIS));
      timePanel.add(new JLabel("Position: "));
      timePanel.add(new JLabel(String.valueOf(robot.getInitialTimestamp())));
      final JSlider slider = new JSlider(0, robot.getNumberOfEntries(), 0);
      slider.addChangeListener(new ChangeListener()
      {
         
         @Override
         public void stateChanged(ChangeEvent e)
         {
            seek(slider.getValue());
         }
      });
      
      robot.addCurrentRecordTickListener(new VariableChangedListener()
      {
         
         @Override
         public void variableChanged(YoVariable v)
         {
            if(scs.isSimulating())
            {
               slider.setValue((int) ((LongYoVariable) v).getLongValue());
            }
         }
      });
      timePanel.add(slider);
      timePanel.add(new JLabel(String.valueOf(robot.getFinalTimestamp())));
      getContentPane().add(timePanel);
   }

   
}
