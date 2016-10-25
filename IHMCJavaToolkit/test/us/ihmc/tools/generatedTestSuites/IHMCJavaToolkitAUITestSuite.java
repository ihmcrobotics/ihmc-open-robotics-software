package us.ihmc.tools.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.UI)
@SuiteClasses
({
   us.ihmc.tools.gui.GUIMessagePanelTest.class,
   us.ihmc.tools.inputDevices.ghostMouse.GhostMousePlaybackTest.class
})

public class IHMCJavaToolkitAUITestSuite
{
   public static void main(String[] args)
   {

   }
}
