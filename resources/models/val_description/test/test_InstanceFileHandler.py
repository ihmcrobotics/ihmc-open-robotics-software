#!/usr/bin/env python

import unittest
import os
from val_description.InstanceFileHandler import InstanceFileHandler
import xml.etree.ElementTree as xmlParser


class instanceFileHandlerTests(unittest.TestCase):

    def setUp(self):
        self.testDirectory = os.path.dirname(os.path.abspath(__file__))

    def tearDown(self):
        pass

    def testGetXmlRoot(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        assert instanceFileHandler.getInstanceRoot().tag == 'robot'

    def testGetMechanisms(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        mechanisms = instanceFileHandler.getMechanisms()
        mechanism_ids = ['leftHipYaw',
                         'leftHipRoll',
                         'leftHipPitch',
                         'leftKneePitch',
                         'leftAnkle',
                         'rightHipYaw',
                         'rightHipRoll',
                         'rightHipPitch',
                         'rightKneePitch',
                         'rightAnkle',
                         'leftShoulderPitch',
                         'leftShoulderRoll',
                         'leftShoulderYaw',
                         'leftElbowPitch',
                         'leftForearmYaw',
                         'leftWrist',
                         'rightShoulderPitch',
                         'rightShoulderRoll',
                         'rightShoulderYaw',
                         'rightElbowPitch',
                         'rightForearmYaw',
                         'rightWrist',
                         'lowerNeckPitch',
                         'neckYaw',
                         'upperNeckPitch',
                         'torso_yaw',
                         'waist']

        for mechanism in mechanisms:
            assert mechanism.tag == 'Mechanism'
            assert mechanism.get('id') in mechanism_ids

    def testGetChannels(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        channels = instanceFileHandler.getChannels()
        channelsToCheck = [
            '/right_arm', '/left_arm', '/right_leg', '/left_leg', '/neck', '/trunk']
        for channel in channels:
            assert channel.tag == 'Channel'
            assert channel.get('id') in channelsToCheck

    def testGetDevices(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        devices = instanceFileHandler.getDevices()
        devicesToCheck = ['pelvis_imu1', 'pelvis_imu2', 'torso_imu1',
                          'torso_imu2', 'left_foot_force_torque', 'right_foot_force_torque']
        for device in devices:
            assert device.tag == 'Device'
            assert device.get('id') in devicesToCheck

    def testGetActuatorSerialNumbers(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        serialNumbers = instanceFileHandler.getSerialNumbers()
        serialNumbersToCheck = ['v_a_006',
                                'v_b_002',
                                'v_c_002',
                                'v_d_001',
                                'v_e_001',
                                'v_e_002',
                                'v_a_003',
                                'v_b_001',
                                'v_c_001',
                                'v_d_004',
                                'v_e_003',
                                'v_e_004',
                                'v_a_005',
                                'v_b_004',
                                'v_f_005',
                                'v_f_002',
                                'v_g_005',
                                'UNKNOWN',
                                'UNKNOWN',
                                'v_a_001',
                                'v_b_003',
                                'v_f_004',
                                'v_f_003',
                                'v_g_003',
                                'UNKNOWN',
                                'UNKNOWN',
                                'v_g_006',
                                'v_g_006',
                                'v_g_006',
                                'v_a_004',
                                'v_e_006',
                                'v_e_005']

        for serialNumber in serialNumbers:
            assert serialNumber in serialNumbersToCheck

    def testGetActuatorCoeffFiles(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        coeffFiles = instanceFileHandler.getActuatorCoeffFiles()
        coeffFilesToCheck = ['v_a_006.xml',
                             'v_b_002.xml',
                             'v_c_002.xml',
                             'v_d_001.xml',
                             'v_e_001.xml',
                             'v_e_002.xml',
                             'v_a_003.xml',
                             'v_b_001.xml',
                             'v_c_001.xml',
                             'v_d_004.xml',
                             'v_e_003.xml',
                             'v_e_004.xml',
                             'v_a_005.xml',
                             'v_b_004.xml',
                             'v_f_005.xml',
                             'v_f_002.xml',
                             'v_g_005.xml',
                             'UNKNOWN.xml',
                             'UNKNOWN.xml',
                             'v_a_001.xml',
                             'v_b_003.xml',
                             'v_f_004.xml',
                             'v_f_003.xml',
                             'v_g_003.xml',
                             'UNKNOWN.xml',
                             'UNKNOWN.xml',
                             'v_g_006.xml',
                             'v_g_006.xml',
                             'v_g_006.xml',
                             'v_a_004.xml',
                             'v_e_006.xml',
                             'v_e_005.xml']
        for coeffFile in coeffFiles:
            assert coeffFile in coeffFilesToCheck

    def testGetActuatorSerialNumberByNode(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        node = '/left_leg/j4'
        serialNumberToCheck = 'v_d_001'
        serialNumber = instanceFileHandler.getActuatorSerialNumberByNode(node)

        assert serialNumber == serialNumberToCheck

        node = '/left_leg/ankle/left_actuator'
        serialNumberToCheck = 'v_e_001'
        serialNumber = instanceFileHandler.getActuatorSerialNumberByNode(node)

        assert serialNumber == serialNumberToCheck

    def testGetNodes(self):
        sampleInstanceFile = self.testDirectory + '/test_files/valkyrie_A.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)
        nodes = instanceFileHandler.getNodeNames()
        nodesToCheck = ['/left_leg/j1',
                        '/left_leg/j2',
                        '/left_leg/j3',
                        '/left_leg/j4',
                        '/left_leg/ankle/left_actuator',
                        '/left_leg/ankle/right_actuator',
                        '/right_leg/j1',
                        '/right_leg/j2',
                        '/right_leg/j3',
                        '/right_leg/j4',
                        '/right_leg/ankle/left_actuator',
                        '/right_leg/ankle/right_actuator',
                        '/left_arm/j3',
                        '/left_arm/j2',
                        '/left_arm/j3',
                        '/left_arm/j4',
                        '/left_arm/j5',
                        '/left_arm/wrist/top_actautor',
                        '/left_arm/wrist/bottom_actuator',
                        '/right_arm/j3',
                        '/right_arm/j2',
                        '/right_arm/j3',
                        '/right_arm/j4',
                        '/right_arm/j5',
                        '/right_arm/wrist/top_actautor',
                        '/right_arm/wrist/bottom_actuator',
                        '/neck/j1',
                        '/neck/j2',
                        '/neck/j3',
                        '/trunk/j1',
                        '/trunk/waist/left_actuator',
                        '/trunk/waist/right_actuator']

        for node in nodes:
            assert node in nodesToCheck

    def testInstanceConfigDictionary(self):
        sampleInstanceFile = self.testDirectory + \
            '/test_files/sample_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

        expectedConfigDictionary = {'/left_leg/ankle/actuator0': {'configFiles': ['v_e_001.xml', 'e.xml', 'e_sv.xml', 'testbench.xml',
                                                                                  'sensors.xml', 'safety.xml', 'mode.xml'], 'firmware': 'linear/turbo_bootloader.bin', 'type': 'turbodriver', 'location': '/left_leg/ankle/actuator0'}, '/left_leg/j1':
                                    {'configFiles': ['v_a_001.xml', 'a.xml', 'a_sv.xml', 'testbench.xml', 'sensors.xml', 'safety.xml', 'mode.xml'], 'firmware':
                                     'rotary/turbo_bootloader.bin', 'type': 'turbodriver', 'location': '/left_leg/j1'}, '/left_leg/ankle/actuator1': {'configFiles':
                                                                                                                                                      ['v_e_002.xml', 'e.xml', 'e_sv.xml', 'testbench.xml',
                                                                                                                                                          'sensors.xml', 'safety.xml', 'mode.xml'],
                                                                                                                                                      'firmware': 'linear/turbo_bootloader.bin', 'type': 'turbodriver', 'location': '/left_leg/ankle/actuator1'}}

        instanceConfig = instanceFileHandler.getInstanceConfig()
        assert cmp(instanceConfig, expectedConfigDictionary) == 0

    def testGetFirmwareType(self):
        sampleInstanceFile = self.testDirectory + \
            '/test_files/sample_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

        expectedFirmwareType = 'rotary/turbo_bootloader.bin'
        firmwareType = instanceFileHandler.getFirmware('/left_leg/j1')

        assert firmwareType == expectedFirmwareType

    def testGetNodeType(self):
        sampleInstanceFile = self.testDirectory + \
            '/test_files/sample_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

        expectedNodeType = 'turbodriver'
        nodeType = instanceFileHandler.getNodeType('/left_leg/j1')

        assert nodeType == expectedNodeType

    def testGatherCoeffs(self):
        sampleInstanceFile = self.testDirectory + \
            '/test_files/sample_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

        coeffs = instanceFileHandler.gatherCoeffs("/left_leg/j1")
        print coeffs
        expectedCoeffs = {'TemperatureSensor_SensorLoc2': 2.0, 'TemperatureSensor_SensorLoc1': 1.0, 'JointSensors_OutputPosition': 2.0,
                          'IGainAmpsPerBit': 0.018928, 'DeltaAPSSafeLimit': 9999.0, 'TorqueControl_enablePID': 1.0,
                          'TorqueControl_FFd_fc_Hz': 25.0, 'APS1DriftSafeLimit': 9999.0, 'PositionControl_MotorTorqueDirection': 1.0,
                          'TorqueControl_Kd_fc_Hz': 50.0, 'VelocitySafeLimit': 9999.0, 'WindingResistance': 2.112, 'JointOutputAPS_MountingGain': 1.0,
                          'JointSafety_LowerLimit_Rad': -3.15, 'CommTimeoutMs': 80.0, 'PhaseACurOffset': 2048.0, 'JerkSafeLimit': 9999.0,
                          'BusVoltage_SensorGain': 0.163412, 'PositionOffset_Rad': -1.6651, 'EncMountingDir': 1.0, 'TorqueControl_TdobWindupLimit_Nm': 80.0,
                          'MotorAccFilter_fc_Hz': 50.0, 'JointOutputAPS_CountsToRad': 0.00076699038, 'PhaseCCurOffset': 2045.0, 'SpaceVector_MaxNormVoltage': 0.666,
                          'MotorWindingType': 0.0, 'TorqueControl_Tdob_fc_Hz': 50.0, 'JointSensors_OutputVelocity': 1.0, 'Renishaw_CountsToRad': 5.8516723e-09,
                          'TorqueControl_m': 1.2, 'EncoderIndexOffset': 1.16973095726, 'BusVoltage_BitOffset': 2048.0, 'SpringStiffness': 2750.0,
                          'Inductance_DAxis': 0.0009, 'MotorVelFilter_fc_Hz': 800.0, 'JointKinematicDir': -1.0, 'TorqueOffset_Nm': -9.39, 'TemperatureSensor_MaxTemp1': 125.0,
                          'TorqueControl_MotorTorqueDirection': 1.0, 'TemperatureSensor_MaxTemp2': 110.0, 'PositionControl_Kd': 1.0, 'TorqueControl_Current2MotorTorque': 0.0375,
                          'PhaseBCurOffset': 2048.0, 'TorqueControl_PD_damp': 0.95, 'EncDriftSafeLimit': 9999.0, 'DeadTimeCompensation': 0.02, 'TorqueControl_b': 70.0, 'TorqueControl_enableDOB': 0.0,
                          'SpringAPS_MountingGain': -1.0, 'PositionControl_Kd_fc_Hz': 50.0, 'Inductance_QAxis': 0.00139, 'JointVelFilter_fc_Hz': 30.0,
                          'PositionControl_Kp': 500.0, 'TorqueControl_enableFF': 1.0, 'JointGearRatio': 160.0, 'NumberOfPoles': 8.0, 'PositionControl_SensorFeedback': 4.0,
                          'PositionControl_Input_fc_Hz': 30.0, 'JointMinValue': -3.14159265359, 'FluxLinkage': 0.0444, 'TorqueControl_Kd': 0.03, 'JointTorqueLimit_Nm': 10.0,
                          'TorqueControl_Kp': 3.351, 'SpringAPS_BitOffset': 115108200.0, 'JointSensors_MotorPosition': 1.0, 'JointSafety_LimitZone_Rad': 0.07,
                          'TorqueControl_enableDynFF': 0.0, 'TorqueControl_autoKd': 0.0, 'JointSensors_OutputForce': 2.0, 'PositionControl_enableInLPF': 1.0,
                          'Commutation_Select': 2.0, 'JointMaxValue': 3.14159265359, 'CurrVelFilter_fc_Hz': 200.0, 'TorqueControl_ParallelDamping': 0.0, 'JointSafety_UpperLimit_Rad': 3.15,
                          'EncoderCPR': 544.0, 'SpaceVector_CurrentToSV': 1.0, 'CurrentSafeLimit': 13.0, 'EffortControl_Alpha': 0.0, 'EffortControl_AlphaDot': 0.5}
        import difflib
        a = '\n'.join(['%s:%s' % (key, value) for (key, value) in sorted(coeffs.items())])
        b = '\n'.join(['%s:%s' % (key, value) for (key, value) in sorted(expectedCoeffs.items())])
        for diffs in difflib.unified_diff(a.splitlines(), b.splitlines()):
            print diffs
        assert cmp(coeffs, expectedCoeffs) == 0

    def testGatherCoeffsHandleKeyError(self):
        sampleInstanceFile = self.testDirectory + \
            '/test_files/sample_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

    # This next line should NOT raise a key error!
        coeffs = instanceFileHandler.gatherCoeffs("/bum_leg/j1")

    def testLoadXMLCoeffs(self):
        sampleInstanceFile = self.testDirectory + \
            '/test_files/sample_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

        expectedCoeffs = {'JointSensors_OutputPosition': {'source': 'v_a_001.xml', 'value': 2.0},
                          'PositionControl_MotorTorqueDirection': {'source': 'v_a_001.xml', 'value': 1.0},
                          'JointOutputAPS_MountingGain': {'source': 'v_a_001.xml', 'value': 1.0},
                          'SpringAPS_MountingGain': {'source': 'v_a_001.xml', 'value': -1.0},
                          'PositionControl_enableInLPF': {'source': 'v_a_001.xml', 'value': 1.0},
                          'PositionOffset_Rad': {'source': 'v_a_001.xml', 'value': -1.6651},
                          'JointSensors_OutputVelocity': {'source': 'v_a_001.xml', 'value': 1.0},
                          'TorqueOffset_Nm': {'source': 'v_a_001.xml', 'value': -9.39},
                          'EncoderIndexOffset': {'source': 'v_a_001.xml', 'value': 1.16973095726},
                          'JointKinematicDir': {'source': 'v_a_001.xml', 'value': -1.0},
                          'TorqueControl_MotorTorqueDirection': {'source': 'v_a_001.xml', 'value': 1.0},
                          'EncMountingDir': {'source': 'v_a_001.xml', 'value': 1.0},
                          'JointMaxValue': {'source': 'v_a_001.xml', 'value': 3.14159265359},
                          'JointSensors_OutputForce': {'source': 'v_a_001.xml', 'value': 2.0},
                          'JointMinValue': {'source': 'v_a_001.xml', 'value': -3.14159265359},
                          'SpringAPS_BitOffset': {'source': 'v_a_001.xml', 'value': 115108200.0},
                          'JointSensors_MotorPosition': {'source': 'v_a_001.xml', 'value': 1.0},
                          'JointSafety_LimitZone_Rad': {'source': 'v_a_001.xml', 'value': 0.07},
                          'PositionControl_Input_fc_Hz': {'source': 'v_a_001.xml', 'value': 30.0}}

        assert cmp(instanceFileHandler.loadXMLCoeffs(
            'v_a_001.xml'), expectedCoeffs) == 0

    def testLoadAnkleInstanceFile(self):
        sampleInstanceFile = self.testDirectory + '/test_files/ankle_instance.xml'
        instanceFileHandler = InstanceFileHandler(sampleInstanceFile)

        nodesToCheck = instanceFileHandler.getNodeNames()

        assert '/left_leg/ankle/left_actuator' in nodesToCheck
        assert '/left_leg/ankle/right_actuator' in nodesToCheck

if __name__ == '__main__':
    unittest.main()
