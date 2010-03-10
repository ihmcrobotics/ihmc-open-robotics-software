package com.yobotics.simulationconstructionset.gui.config;

import com.yobotics.simulationconstructionset.*;
import java.util.ArrayList;

public class CameraConfigurationList
{
  private ArrayList configs = new ArrayList();

  public CameraConfigurationList()
  {
  }

  public void addCameraConfiguration(CameraConfiguration config)
  {
    configs.add(config);
  }

  public String[] getCameraConfigurationNames()
  {
    int n = configs.size();
    String[] ret = new String[n];

    for(int i=0; i<n; i++)
    {
      ret[i] = ((CameraConfiguration) configs.get(i)).getName();
    }

    return ret;
  }

  public CameraConfiguration getCameraConfiguration(String name)
  {
    int n = configs.size();

    for(int i=0; i<n; i++)
    {
      CameraConfiguration config = ((CameraConfiguration) configs.get(i));

      if (config.getName().equals(name)) return config;
    }

    return null;
  }
}
