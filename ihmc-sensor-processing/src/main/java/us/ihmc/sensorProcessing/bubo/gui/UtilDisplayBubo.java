/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.ihmc.sensorProcessing.bubo.gui;

import java.awt.BorderLayout;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.SwingUtilities;

import us.ihmc.sensorProcessing.bubo.gui.d3.PointCloudPanel;

/**
 * @author Peter Abeles
 */
public class UtilDisplayBubo {

	public static FactoryVisualization3D createVisualize3D() {
		try {
			Class factory = Class.forName("bubo.gui.jme.JmeFactoryVisualization3D");
			return (FactoryVisualization3D)factory.newInstance();
		} catch (ClassNotFoundException e) {
			throw new RuntimeException(e);
		} catch (InstantiationException e) {
			throw new RuntimeException(e);
		} catch (IllegalAccessException e) {
			throw new RuntimeException(e);
		}
	}

	public static JFrame createWindow(final JPanel gui, final PointCloudPanel panel, String name ) {
		final JFrame frame = new JFrame(name);

		// TODO make this part of the API
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosed(WindowEvent e) {
				panel.shutdownVisualize();
			}
		});

		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				frame.add(gui, BorderLayout.CENTER);
				frame.pack();
				frame.setVisible(true);
			}
		});
		return frame;
	}

	public static JFrame show(JComponent comp, String windowName, boolean hasScrollBars, int x, int y, int width, int height) {
		JFrame frame = new JFrame(windowName);

		frame.setLocation(x, y);
		if (hasScrollBars) {
			JScrollPane scroll = new JScrollPane(comp);
			frame.add(scroll, BorderLayout.CENTER);

		} else {
			frame.add(comp, BorderLayout.CENTER);
		}

		frame.setSize(width, height);
		frame.setVisible(true);

		return frame;
	}

	public static JFrame show(JComponent comp, String windowName) {
		JFrame frame = new JFrame(windowName);

		frame.add(comp, BorderLayout.CENTER);

		frame.pack();
		frame.setVisible(true);

		return frame;
	}

	/**
	 * Pauses the program for the specified number of milliseconds.
	 *
	 * @param milliseconds Length of the pause.
	 */
	public static void pause(long milliseconds) {
		if (milliseconds == 0)
			return;

		synchronized (Thread.currentThread()) {
			try {
				Thread.currentThread().wait(milliseconds);
			} catch (InterruptedException e) {

			}
		}
	}

	/**
	 * <p>
	 * Adds a ComponentListener which will automatically change the display's scale factor so that it
	 * shows the same physical area as the component is resized.
	 * </p>
	 *
	 * @param display         The SpacialDisplay which is to be rescalled.
	 * @param lengthWordUnits The desired physical length of the display area.
	 */
	public static void autoRescaleLength(final SpacialDisplay display, final double lengthWordUnits) {
		display.addComponentListener(new ComponentListener() {

			@Override
			public void componentResized(ComponentEvent e) {
				int length = Math.min(display.getWidth(), display.getHeight());
				display.setPixelsPerMeter(length / lengthWordUnits);
			}

			@Override
			public void componentMoved(ComponentEvent e) {
			}

			@Override
			public void componentShown(ComponentEvent e) {
			}

			@Override
			public void componentHidden(ComponentEvent e) {
			}
		});
	}
}
