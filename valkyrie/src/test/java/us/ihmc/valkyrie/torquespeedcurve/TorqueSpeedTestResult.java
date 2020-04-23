package us.ihmc.valkyrie.torquespeedcurve;

import us.ihmc.avatar.testTools.DRCSimulationTestHelper;

public class TorqueSpeedTestResult {
    public String testInfo;
    public String testSuffix;
    public boolean testSucceeded;
    DRCSimulationTestHelper testHelper;
    
    public TorqueSpeedTestResult(DRCSimulationTestHelper helper, String info, String suffix, boolean result) {
    	this.testHelper = helper;
    	this.testInfo = info;
    	this.testSuffix = suffix;
    	this.testSucceeded = result;
    }
}
