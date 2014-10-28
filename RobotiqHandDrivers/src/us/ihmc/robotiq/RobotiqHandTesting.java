package us.ihmc.robotiq;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class RobotiqHandTesting
{
	private JFrame frame = new JFrame();
	private JPanel panel = new JPanel(new GridBagLayout());
	private GridBagConstraints c = new GridBagConstraints();
	
	private SliderWithTwoButtonsPanel finger1Panel = new SliderWithTwoButtonsPanel("Index Finger");
	private SliderWithTwoButtonsPanel finger2Panel = new SliderWithTwoButtonsPanel("Middle Finger");
	private SliderWithTwoButtonsPanel finger3Panel = new SliderWithTwoButtonsPanel("Thumb");
	
//	private RobotiqHandInterface hand = new RobotiqHandInterface();

	public static void main(String[] args) throws Exception
	{
		new RobotiqHandTesting().init();
	}
	
	private void init()
	{
//		setupSliderListeners();
		
		setupButtonListeners();
		
		setupUI();
	}
	
	private void setupUI()
	{
		frame.add(panel);
		
		c.gridx = 0;
		c.gridy = 0;
		
		panel.add(finger1Panel, c);
		
		c.gridy++;
		panel.add(finger2Panel, c);
		
		c.gridy++;
		panel.add(finger3Panel, c);
		
		frame.pack();
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setVisible(true);
	}
	
	private void setupSliderListeners()
	{
		finger1Panel.attachSliderActionListener(new ChangeListener()
		{
			@Override
			public void stateChanged(ChangeEvent e)
			{
				JSlider slider = (JSlider) e.getSource();
				System.out.println(slider.getValue());
			}
		});
		
		finger2Panel.attachSliderActionListener(new ChangeListener()
		{
			@Override
			public void stateChanged(ChangeEvent e)
			{
				// TODO Auto-generated method stub
				JSlider slider = (JSlider) e.getSource();
				System.out.println(slider.getValue());
			}
		});
		
		finger3Panel.attachSliderActionListener(new ChangeListener()
		{
			@Override
			public void stateChanged(ChangeEvent e)
			{
				// TODO Auto-generated method stub
				JSlider slider = (JSlider) e.getSource();
				System.out.println(slider.getValue());
			}
		});
	}
	
	private void setupButtonListeners()
	{
		finger1Panel.attachTopButtonActionListener(new ActionListener()
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				JSlider slider = finger1Panel.getSlider();
				System.out.println(slider.getValue());
			}
		});
		
		finger1Panel.attachBottomButtonActionListener(new ResetSliderActionListener(finger1Panel.getSlider()));
		
		finger2Panel.attachTopButtonActionListener(new ActionListener()
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				// TODO Auto-generated method stub
				
			}
		});
		
		finger2Panel.attachBottomButtonActionListener(new ResetSliderActionListener(finger2Panel.getSlider()));
		
		finger3Panel.attachTopButtonActionListener(new ActionListener()
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				// TODO Auto-generated method stub
				
			}
		});
		
		finger3Panel.attachBottomButtonActionListener(new ResetSliderActionListener(finger3Panel.getSlider()));
		
	}
	
	class ResetSliderActionListener implements ActionListener
	{
		private final JSlider slider;
		
		public ResetSliderActionListener(JSlider slider)
		{
			this.slider = slider;
		}
		
		@Override
		public void actionPerformed(ActionEvent e)
		{
			slider.setValue(0);
		}
	}
}
